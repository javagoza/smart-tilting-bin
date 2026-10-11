"""Hardware-free tests: simulated ADXL372 register file, FIFO and SPI framing.
Run with:  python3 -m unittest -v test_adxl372
"""
import unittest
import warnings

import adxl372 as A
from adxl372 import ADXL372, FifoFormat, FifoMode, OpMode, signed12


def enc12(v):
    """Encode a signed 12-bit value as the two left-justified bytes the chip returns."""
    u = (v & 0xFFF) << 4
    return [(u >> 8) & 0xFF, u & 0xFF]


class FakeBus:
    kind = "fake"

    def __init__(self, revid=3):
        self.regs = {A.REG_DEVID: 0xAD, A.REG_DEVID_MST: 0x1D, A.REG_PARTID: 0xFA,
                     A.REG_REVID: revid}
        self.fifo = []           # list of bytes
        self.sample = (0, 0, 0)
        self.data_rdy = True
        self.status2 = 0
        self.writes = []

    def _status_bytes(self):
        n = len(self.fifo) // 2
        return [0x01 if self.data_rdy else 0x00, self.status2, (n >> 8) & 3, n & 0xFF]

    def read(self, reg, n=1):
        if reg == A.REG_FIFO_DATA:  # non-incrementing port
            out, self.fifo = self.fifo[:n], self.fifo[n:]
            return out
        out = []
        for i in range(n):
            r = reg + i
            if A.REG_STATUS_1 <= r <= A.REG_FIFO_ENTRIES_1:
                out.append(self._status_bytes()[r - A.REG_STATUS_1])
            elif A.REG_X_DATA_H <= r <= A.REG_X_DATA_H + 5:
                bytes6 = sum((enc12(v) for v in self.sample), [])
                out.append(bytes6[r - A.REG_X_DATA_H])
            else:
                out.append(self.regs.get(r, 0))
        return out

    def write(self, reg, value):
        self.writes.append((reg, value))
        self.regs[reg] = value

    def push_sets(self, sets):
        for s in sets:
            for v in s:
                self.fifo += enc12(v)

    def close(self):
        pass


def make(**kw):
    bus = FakeBus()
    dev = ADXL372(bus, **kw)
    dev.init()
    return bus, dev


class InitTests(unittest.TestCase):
    def test_init_defaults(self):
        bus, dev = make()
        r = bus.regs
        self.assertIn((A.REG_RESET, 0x52), bus.writes)
        self.assertEqual((r[A.REG_TIMING] >> 5) & 7, 3)        # 3200 Hz
        self.assertEqual(r[A.REG_MEASURE] & 7, 3)              # 1600 Hz bandwidth
        self.assertEqual(r[A.REG_POWER_CTL] & 3, OpMode.STANDBY)
        self.assertEqual(r[A.REG_FIFO_CTL], 0)                 # bypassed
        self.assertEqual(r[A.REG_INT1_MAP], 0)
        self.assertEqual(r[A.REG_INT2_MAP], 0)
        self.assertEqual(dev.odr_hz, 3200)
        self.assertEqual(dev.bandwidth_hz, 1600)

    def test_bad_id(self):
        bus = FakeBus()
        bus.regs[A.REG_PARTID] = 0x00
        with self.assertRaises(RuntimeError):
            ADXL372(bus).init()

    def test_i2c_old_revision_warns(self):
        bus = FakeBus(revid=2)
        bus.kind = "i2c"
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            ADXL372(bus).init()
        self.assertTrue(any("REVID" in str(x.message) for x in w))

    def test_invalid_constructor_args(self):
        with self.assertRaises(ValueError):
            ADXL372(FakeBus(), odr_hz=1234)
        with self.assertRaises(ValueError):
            ADXL372(FakeBus(), odr_hz=400, bandwidth_hz=800)

    def test_start_stop(self):
        bus, dev = make()
        dev.start()
        self.assertEqual(dev.get_mode(), OpMode.FULL_BW)
        dev.stop()
        self.assertEqual(dev.get_mode(), OpMode.STANDBY)


class ConfigTests(unittest.TestCase):
    def test_odr_lowers_bandwidth(self):
        bus, dev = make(odr_hz=6400, bandwidth_hz=3200)
        dev.set_odr(800)
        self.assertEqual(dev.odr_hz, 800)
        self.assertEqual(dev.bandwidth_hz, 400)
        self.assertEqual(bus.regs[A.REG_MEASURE] & 7, 1)

    def test_bandwidth_validation(self):
        _, dev = make(odr_hz=800, bandwidth_hz=400)
        with self.assertRaises(ValueError):
            dev.set_bandwidth(1600)
        dev.set_bandwidth(200)
        self.assertEqual(dev.bandwidth_hz, 200)

    def test_config_restores_running_mode(self):
        _, dev = make()
        dev.start()
        dev.set_odr(1600)
        self.assertEqual(dev.get_mode(), OpMode.FULL_BW)

    def test_low_noise(self):
        bus, dev = make()
        dev.set_low_noise(True)
        self.assertTrue(bus.regs[A.REG_MEASURE] & 0x08)

    def test_activity_threshold_encoding(self):
        bus, dev = make()
        dev.set_activity_threshold("activity", 1000, referenced=True, enable=True)
        for h in (0x23, 0x25, 0x27):
            self.assertEqual(bus.regs[h], 1)        # 10 >> 3
            self.assertEqual(bus.regs[h + 1], 0x43)  # (10 << 5) | ref<<1 | en
        dev.set_activity_threshold("inactivity", 100, referenced=False, enable=False)
        self.assertEqual(bus.regs[0x2A], 0)
        self.assertEqual(bus.regs[0x2B], 0x20)       # code 1 << 5
        with self.assertRaises(ValueError):
            dev.set_activity_threshold("nope", 100)

    def test_timers_depend_on_odr(self):
        bus, dev = make(odr_hz=3200)
        dev.set_activity_time_ms(1000)
        self.assertEqual(bus.regs[A.REG_TIME_ACT], 152)      # 1 s / 6.6 ms
        dev.set_inactivity_time_ms(10000)
        self.assertEqual((bus.regs[A.REG_TIME_INACT_H] << 8) | bus.regs[A.REG_TIME_INACT_L], 385)
        dev.set_odr(6400)                                    # timers are re-applied
        self.assertEqual(bus.regs[A.REG_TIME_ACT], 0xFF)     # 303 clamped to 8 bits
        self.assertEqual((bus.regs[A.REG_TIME_INACT_H] << 8) | bus.regs[A.REG_TIME_INACT_L], 769)

    def test_interrupt_map(self):
        bus, dev = make()
        dev.set_interrupts(int1=("act", "inact"), int2=("fifo_full",))
        self.assertEqual(bus.regs[A.REG_INT1_MAP], 0x30)
        self.assertEqual(bus.regs[A.REG_INT2_MAP], 0x04)
        with self.assertRaises(ValueError):
            dev.set_interrupts(int1=("bogus",))


class ReadTests(unittest.TestCase):
    def test_signed12(self):
        for v in (0, 1, -1, 100, -100, 2047, -2048):
            self.assertEqual(signed12(*enc12(v)), v)

    def test_read_xyz(self):
        bus, dev = make()
        bus.sample = (100, -100, 2047)
        self.assertEqual(dev.read_xyz_raw(), (100, -100, 2047))
        g = dev.read_xyz_g()
        self.assertAlmostEqual(g[2], 2047 * 400 / 4095)
        self.assertAlmostEqual(g[2], 199.95, places=1)

    def test_read_timeout(self):
        bus, dev = make()
        bus.data_rdy = False
        with self.assertRaises(TimeoutError):
            dev.read_xyz_raw(timeout=0.01)

    def test_status_decode(self):
        bus, dev = make()
        bus.status2 = 0x20 | 0x10
        bus.push_sets([(1, 2, 3)] * 5)
        st = dev.get_status()
        self.assertTrue(st.act and st.inact and not st.act2)
        self.assertEqual(st.fifo_entries, 15)
        self.assertTrue(st.data_rdy)


class FifoTests(unittest.TestCase):
    def test_configure_fifo_registers(self):
        bus, dev = make()
        dev.configure_fifo(FifoMode.STREAMED, FifoFormat.XYZ, 300)
        self.assertEqual(bus.regs[A.REG_FIFO_SAMPLES], 300 & 0xFF)
        self.assertEqual(bus.regs[A.REG_FIFO_CTL], (0 << 3) | (1 << 1) | 1)
        dev.configure_fifo(FifoMode.TRIGGERED, FifoFormat.XY, 60)
        self.assertEqual(bus.regs[A.REG_FIFO_CTL], (3 << 3) | (2 << 1) | 0)
        self.assertEqual(dev.fifo_axes, "xy")
        with self.assertRaises(ValueError):
            dev.configure_fifo(FifoMode.STREAMED, FifoFormat.XYZ, 513)

    def test_read_fifo_leaves_one_set(self):
        bus, dev = make()
        dev.configure_fifo(FifoMode.STREAMED, FifoFormat.XYZ, 30)
        sets = [(i, -i, 2 * i) for i in range(5)]
        bus.push_sets(sets)
        got = dev.read_fifo()
        self.assertEqual(got, sets[:4])
        self.assertEqual(len(bus.fifo), 6)                  # one set left behind
        bus.push_sets([(7, 8, 9)])
        self.assertEqual(dev.read_fifo(), [sets[4]])

    def test_read_fifo_all(self):
        bus, dev = make()
        dev.configure_fifo(FifoMode.STREAMED, FifoFormat.XYZ, 30)
        sets = [(i, i, i) for i in range(5)]
        bus.push_sets(sets)
        self.assertEqual(dev.read_fifo(leave_one_set=False), sets)
        self.assertEqual(bus.fifo, [])

    def test_read_fifo_chunks_large_buffers(self):
        bus, dev = make()
        dev.configure_fifo(FifoMode.STREAMED, FifoFormat.XYZ, 510)
        sets = [((i % 200) - 100, i % 50, -(i % 70)) for i in range(170)]
        bus.push_sets(sets)
        self.assertEqual(dev.read_fifo(leave_one_set=False), sets)

    def test_single_axis_format(self):
        bus, dev = make()
        dev.configure_fifo(FifoMode.STREAMED, FifoFormat.Z, 10)
        bus.push_sets([(5,), (-6,), (7,)])
        self.assertEqual(dev.read_fifo(leave_one_set=False), [(5,), (-6,), (7,)])

    def test_overrun_flag(self):
        bus, dev = make()
        dev.get_status()
        self.assertFalse(dev.fifo_overrun)
        bus._status_bytes = lambda: [0x08, 0, 0, 0]
        dev.get_status()
        self.assertTrue(dev.fifo_overrun)


class SpiFramingTests(unittest.TestCase):
    class FakeSpi:
        def __init__(self):
            self.frames = []

        def xfer2(self, data):
            self.frames.append(list(data))
            return [0] + [0xA0 + i for i in range(len(data) - 1)]

        def close(self):
            pass

    def test_read_write_frames(self):
        spi = self.FakeSpi()
        bus = A.SPIBus(spi=spi)
        self.assertEqual(bus.read(A.REG_DEVID, 1), [0xA0])
        self.assertEqual(spi.frames[-1], [(0x00 << 1) | 1, 0])
        bus.read(A.REG_X_DATA_H, 6)
        self.assertEqual(spi.frames[-1], [(0x08 << 1) | 1] + [0] * 6)
        bus.write(A.REG_RESET, 0x52)
        self.assertEqual(spi.frames[-1], [0x41 << 1, 0x52])


if __name__ == "__main__":
    unittest.main()
