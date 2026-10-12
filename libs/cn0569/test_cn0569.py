"""Hardware-free tests: simulated I2C bus + synthetic hand movements.
Run with:  python3 -m unittest -v test_cn0569
"""
import unittest
from types import SimpleNamespace

import adpd1080 as A
from irq import InterruptLine
from adpd1080 import ADPD1080
from gesture import Gesture, GestureEngine


class FakeBus:
    def __init__(self):
        self.regs = {A.REG_DEVID: A.DEVICE_ID_ADPD108X, A.REG_CLK_RATIO: 2000}
        self.fifo = []
        self.writes = []
        self.status_flags = 0

    def push_sample(self, ch8):
        for v in ch8:
            self.fifo += [v & 0xFFFF, (v >> 16) & 0xFFFF]

    def read_words(self, reg, count=1):
        if reg == A.REG_STATUS:
            return [(min(len(self.fifo) * 2, 255) << 8) | self.status_flags]
        if reg == A.REG_FIFO_ACCESS:
            out, self.fifo = self.fifo[:count], self.fifo[count:]
            return out
        return [self.regs.get(reg, 0)]

    def write_word(self, reg, val):
        self.writes.append((reg, val))
        self.regs[reg] = val


def sensor_channels(x, y, intensity):
    """4 channels of one sensor such that (c1-c0)/(c1+c0)=x and (c3-c2)/(c3+c2)=y."""
    return [intensity * (1 - x) / 2, intensity * (1 + x) / 2,
            intensity * (1 - y) / 2, intensity * (1 + y) / 2]


def swipe(p0, p1, n=14, peak=3000, base=100):
    """8-channel frames: the hand moves from p0 to p1 with a bell-shaped intensity."""
    frames = []
    for i in range(n):
        s = i / (n - 1)
        x = p0[0] + (p1[0] - p0[0]) * s
        y = p0[1] + (p1[1] - p0[1]) * s
        inten = peak * (1 - (2 * s - 1) ** 2) + 400  # >0 at the ends
        ch = sensor_channels(x, y, inten)
        frames.append([int(c) + base for c in ch] * 2)
    return frames + [[base] * 8] * 12  # hand removed


def run_engine(frames, **kw):
    it = iter(frames)
    eng = GestureEngine(lambda: next(it), **kw)
    eng.baseline = [100] * 8
    events = []
    for f in frames:
        events += eng.process_frame(f)
    return events


def gestures(events):
    return [(e.sensor, e.gesture) for e in events if e.kind == "gesture"]


class DriverTests(unittest.TestCase):
    def test_init_sequence(self):
        bus = FakeBus()
        ADPD1080(bus, fs_hz=512).init()
        r = bus.regs
        self.assertEqual(r[A.REG_PD_LED_SELECT], 0x0455)
        self.assertEqual(r[A.REG_FSAMPLE], 15)
        self.assertEqual(r[A.REG_SLOTA_NUMPULSES], 0x040E)
        self.assertEqual(r[A.REG_SLOTB_AFE_WINDOW], 0x22F0)
        self.assertEqual(r[A.REG_MATH], 0x0544)
        self.assertEqual(r[A.REG_ILED1_COARSE], 0x2036)
        self.assertEqual(r[A.REG_INT_SEQ_A], 0x9)
        slot = r[A.REG_SLOT_EN]
        self.assertEqual((slot >> 2) & 7, 6)
        self.assertEqual((slot >> 6) & 7, 6)
        self.assertTrue(slot & 0x21 == 0x21)

    def test_bad_device_id(self):
        bus = FakeBus()
        bus.regs[A.REG_DEVID] = 0x1234
        with self.assertRaises(RuntimeError):
            ADPD1080(bus).init()

    def test_fifo_parse_and_frame(self):
        bus = FakeBus()
        dev = ADPD1080(bus)
        for k in range(8):
            bus.push_sample([70000 * (c + 1) + k for c in range(8)])
        frame = dev.next_frame(8)
        self.assertEqual(frame[0], sum(70000 + k for k in range(8)))
        self.assertEqual(frame[7], sum(70000 * 8 + k for k in range(8)))

    def test_fifo_no_burst(self):
        bus = FakeBus()
        dev = ADPD1080(bus, fifo_burst=False)
        bus.push_sample(list(range(100000, 100008)))
        self.assertEqual(dev.read_samples(1)[0], tuple(range(100000, 100008)))

    def test_timeout(self):
        with self.assertRaises(TimeoutError):
            ADPD1080(FakeBus()).read_samples(1, timeout=0.01)


class GestureTests(unittest.TestCase):
    def check(self, p0, p1, expected):
        ev = run_engine(swipe(p0, p1))
        self.assertEqual(gestures(ev), [(0, expected), (1, expected)])

    def test_right(self): self.check((-0.7, 0), (0.7, 0), Gesture.RIGHT)
    def test_left(self): self.check((0.7, 0), (-0.7, 0), Gesture.LEFT)
    def test_up(self): self.check((0, -0.7), (0, 0.7), Gesture.UP)
    def test_down(self): self.check((0, 0.7), (0, -0.7), Gesture.DOWN)
    def test_click(self): self.check((0.0, 0.0), (0.02, 0.0), Gesture.CLICK)

    def test_short_blip_discarded(self):
        self.assertEqual(gestures(run_engine(swipe((-0.7, 0), (0.7, 0), n=4))), [])

    def test_no_signal_no_events(self):
        self.assertEqual(run_engine([[100] * 8] * 50), [])

    def test_presence_hysteresis(self):
        ev = run_engine(swipe((-0.7, 0), (0.7, 0)))
        kinds = [e.kind for e in ev]
        self.assertEqual(kinds[0], "presence")
        self.assertEqual(kinds[-1], "absence")
        self.assertEqual(kinds.count("presence"), 1)

    def test_trajectory_recorded(self):
        ev = [e for e in run_engine(swipe((-0.7, 0), (0.7, 0))) if e.kind == "gesture"]
        traj = ev[0].trajectory
        self.assertGreater(len(traj), 5)
        self.assertLess(traj[0][0], traj[-1][0])

    def test_two_swipes_in_a_row(self):
        fr = swipe((-0.7, 0), (0.7, 0)) + swipe((0, -0.7), (0, 0.7))
        got = [g for _, g in gestures(run_engine(fr))]
        self.assertEqual(got, [Gesture.RIGHT, Gesture.RIGHT, Gesture.UP, Gesture.UP])


def make_dev(**kw):
    bus = FakeBus()
    dev = ADPD1080(bus, **kw)
    dev.init()
    return bus, dev


class InterruptRegisterTests(unittest.TestCase):
    def test_pin0_defaults_active_high_push_pull(self):
        bus, dev = make_dev()
        dev.configure_interrupt_pin(0)
        self.assertEqual(bus.regs[A.REG_GPIO_DRV] & 0x7, 0x4)      # enabled, active high, always driven
        self.assertEqual(bus.regs[A.REG_GPIO_CTRL] & 0x1F, A.GPIO_ALT_INTERRUPT)

    def test_pin0_polarity_and_drive_bits(self):
        bus, dev = make_dev()
        dev.configure_interrupt_pin(0, active_low=True)
        self.assertEqual(bus.regs[A.REG_GPIO_DRV] & 0x7, 0x5)
        dev.configure_interrupt_pin(0, open_drain=True)
        self.assertEqual(bus.regs[A.REG_GPIO_DRV] & 0x7, 0x6)
        dev.configure_interrupt_pin(0, active_low=True, open_drain=True)
        self.assertEqual(bus.regs[A.REG_GPIO_DRV] & 0x7, 0x7)

    def test_pin1_uses_its_own_bits_and_enable(self):
        bus, dev = make_dev()
        dev.configure_interrupt_pin(1)
        self.assertEqual(bus.regs[A.REG_GPIO_DRV] & 0x300, 0)
        self.assertTrue(bus.regs[A.REG_EXT_SYNC_SEL] & 0x0040)
        self.assertEqual(bus.regs[A.REG_GPIO_CTRL] & 0x1F00, 0x0100)
        dev.configure_interrupt_pin(1, active_low=True, open_drain=True)
        self.assertEqual(bus.regs[A.REG_GPIO_DRV] & 0x300, 0x300)

    def test_other_bits_preserved(self):
        bus, dev = make_dev()
        bus.regs[A.REG_GPIO_DRV] = 0xF000
        bus.regs[A.REG_GPIO_CTRL] = 0x0100          # GPIO1 alt config must survive a GPIO0 change
        dev.configure_interrupt_pin(0)
        self.assertEqual(bus.regs[A.REG_GPIO_DRV] & 0xF000, 0xF000)
        self.assertEqual(bus.regs[A.REG_GPIO_CTRL] & 0x1F00, 0x0100)

    def test_invalid_pin(self):
        _, dev = make_dev()
        with self.assertRaises(ValueError):
            dev.configure_interrupt_pin(2)

    def test_fifo_threshold_and_unmask(self):
        bus, dev = make_dev()
        bus.regs[A.REG_INT_MASK] = 0x01FF           # everything masked
        for packets, words in ((1, 15), (2, 31), (3, 47)):
            dev.enable_fifo_interrupt(packets)
            self.assertEqual((bus.regs[A.REG_FIFO_THRESH] >> 8) & 0x3F, words)
            self.assertEqual(bus.regs[A.REG_INT_MASK], 0x00FF)  # bit 8 cleared = enabled
        for bad in (0, 4):
            with self.assertRaises(ValueError):
                dev.enable_fifo_interrupt(bad)

    def test_slot_interrupts(self):
        bus, dev = make_dev()
        bus.regs[A.REG_INT_MASK] = 0x01FF
        dev.enable_slot_interrupt("a")
        self.assertEqual(bus.regs[A.REG_INT_MASK], 0x01DF)      # bit 5 cleared
        dev.enable_slot_interrupt("B")
        self.assertEqual(bus.regs[A.REG_INT_MASK], 0x019F)      # bit 6 cleared
        with self.assertRaises(ValueError):
            dev.enable_slot_interrupt("c")

    def test_disable_interrupts(self):
        bus, dev = make_dev()
        bus.regs[A.REG_INT_MASK] = 0x0000
        dev.disable_interrupts()
        self.assertEqual(bus.regs[A.REG_INT_MASK], 0x0160)

    def test_mode_is_restored_after_reconfiguring(self):
        bus, dev = make_dev()
        self.assertEqual(dev.get_mode(), A.MODE_PROGRAM)
        dev.enable_fifo_interrupt(2)
        self.assertEqual(dev.get_mode(), A.MODE_PROGRAM)
        dev.start()
        dev.enable_fifo_interrupt(2)                  # while sampling
        self.assertEqual(dev.get_mode(), A.MODE_NORMAL)

    def test_flags_and_fifo_level(self):
        bus, dev = make_dev()
        bus.status_flags = 0x0060
        bus.push_sample(list(range(8)))
        f = dev.get_interrupt_flags()
        self.assertTrue(f.slot_a and f.slot_b)
        self.assertEqual(f.fifo_bytes, 32)
        bus.status_flags = 0x0040
        f = dev.get_interrupt_flags()
        self.assertFalse(f.slot_a)
        self.assertTrue(f.slot_b)

    def test_clear_flags_writes_only_flag_bits(self):
        bus, dev = make_dev()
        for _ in range(4):                            # full FIFO: STATUS bit 15 reads as 1
            bus.push_sample(list(range(8)))
        self.assertTrue(dev.reg_read(A.REG_STATUS) & 0x8000)
        dev.clear_interrupt_flags(slot_a=True, slot_b=False)
        self.assertEqual(bus.writes[-1], (A.REG_STATUS, 0x0020))   # never 0x8000: FIFO untouched
        dev.clear_interrupt_flags(slot_a=False, slot_b=True)
        self.assertEqual(bus.writes[-1], (A.REG_STATUS, 0x0040))
        n = len(bus.writes)
        dev.clear_interrupt_flags(False, False)
        self.assertEqual(len(bus.writes), n)

    def test_stop_follows_datasheet_sequence(self):
        bus, dev = make_dev()
        dev.start()
        bus.writes.clear()
        dev.stop()
        seq = [(r, v) for r, v in bus.writes if r in (A.REG_MODE, A.REG_STATUS)]
        self.assertEqual(seq, [(A.REG_MODE, A.MODE_PROGRAM), (A.REG_STATUS, 0x80FF),
                               (A.REG_MODE, A.MODE_STANDBY)])


class InterruptReadTests(unittest.TestCase):
    def test_use_interrupt_configures_chip_and_enables_waiting(self):
        bus, dev = make_dev()
        dev.use_interrupt(lambda t: True, packets=2, active_low=False)
        self.assertEqual(bus.regs[A.REG_GPIO_DRV] & 0x7, 0x4)
        self.assertEqual((bus.regs[A.REG_FIFO_THRESH] >> 8) & 0x3F, 31)
        self.assertEqual(bus.regs[A.REG_INT_MASK] & A.INT_MASK_FIFO, 0)

    def test_read_samples_sleeps_on_the_interrupt(self):
        bus, dev = make_dev()
        calls = []

        def wait(timeout):
            calls.append(timeout)
            bus.push_sample([10 * (len(calls)) + c for c in range(8)])
            bus.push_sample([10 * (len(calls)) + c + 1 for c in range(8)])
            return True

        dev.use_interrupt(wait)
        samples = dev.read_samples(4)
        self.assertEqual(len(samples), 4)
        self.assertEqual(len(calls), 2)                      # one wake-up per 2 packets
        self.assertTrue(all(0 < t <= dev.interrupt_poll_s for t in calls))
        self.assertEqual(dev.irq_timeouts, 0)
        self.assertEqual(samples[0][0], 10)
        self.assertEqual(samples[2][0], 20)

    def test_no_wait_when_data_already_in_fifo(self):
        bus, dev = make_dev()
        dev.use_interrupt(lambda t: self.fail("must not wait: the FIFO already has data"))
        for _ in range(3):
            bus.push_sample(list(range(8)))
        self.assertEqual(len(dev.read_samples(3)), 3)

    def test_missed_edges_are_counted_but_data_still_read(self):
        bus, dev = make_dev()
        state = {"n": 0}

        def wait(timeout):
            state["n"] += 1
            if state["n"] == 2:
                bus.push_sample(list(range(8)))
            return False                                      # no edge ever arrives

        dev.use_interrupt(wait)
        self.assertEqual(len(dev.read_samples(1, timeout=1.0)), 1)
        self.assertEqual(dev.irq_timeouts, 2)

    def test_timeout_still_raises(self):
        _, dev = make_dev()
        dev.use_interrupt(lambda t: False)
        with self.assertRaises(TimeoutError):
            dev.read_samples(1, timeout=0.05)
        self.assertGreater(dev.irq_timeouts, 0)

    def test_disable_interrupt_goes_back_to_polling(self):
        bus, dev = make_dev()
        dev.use_interrupt(lambda t: self.fail("must not be used any more"))
        dev.disable_interrupt()
        bus.push_sample(list(range(8)))
        self.assertEqual(len(dev.read_samples(1)), 1)
        self.assertEqual(bus.regs[A.REG_INT_MASK] & 0x0160, 0x0160)

    def test_frames_work_with_interrupts(self):
        bus, dev = make_dev()

        def wait(timeout):
            for _ in range(2):
                bus.push_sample([1000] * 8)
            return True

        dev.use_interrupt(wait)
        self.assertEqual(dev.next_frame(8), [8000] * 8)


class FakeRequest:
    def __init__(self, on_wait=None):
        self.edges = 0
        self.released = False
        self.on_wait = on_wait
        self.drained = 0
        self.timeouts = []

    def wait_edge_events(self, timeout=None):
        self.timeouts.append(timeout)
        if self.on_wait:
            self.on_wait(self)
        return self.edges > 0

    def read_edge_events(self):
        n, self.edges = self.edges, 0
        self.drained += n
        return [object()] * n

    def release(self):
        self.released = True


class FakeGpiod:
    class LineSettings:
        def __init__(self, edge_detection=None):
            self.edge_detection = edge_detection

    line = SimpleNamespace(Edge=SimpleNamespace(RISING="RISING", FALLING="FALLING"))

    def __init__(self, request=None):
        self.request = request or FakeRequest()
        self.args = None

    def request_lines(self, chip, consumer=None, config=None):
        self.args = (chip, consumer, config)
        return self.request


class InterruptLineTests(unittest.TestCase):
    def test_rising_edge_by_default(self):
        g = FakeGpiod()
        InterruptLine(25, _gpiod=g)
        chip, consumer, cfg = g.args
        self.assertEqual(chip, "/dev/gpiochip0")
        self.assertEqual(consumer, "adpd1080")
        self.assertEqual(list(cfg), [25])
        self.assertEqual(cfg[25].edge_detection, "RISING")

    def test_falling_edge_for_active_low(self):
        g = FakeGpiod()
        InterruptLine(24, chip="/dev/gpiochip4", active_low=True, _gpiod=g)
        self.assertEqual(g.args[0], "/dev/gpiochip4")
        self.assertEqual(g.args[2][24].edge_detection, "FALLING")

    def test_wait_true_drains_all_queued_edges(self):
        g = FakeGpiod()
        line = InterruptLine(25, _gpiod=g)
        g.request.edges = 3
        self.assertTrue(line.wait(0.5))
        self.assertEqual(g.request.drained, 3)
        self.assertEqual(g.request.timeouts, [0.5])

    def test_wait_false_on_timeout(self):
        g = FakeGpiod()
        line = InterruptLine(25, _gpiod=g)
        self.assertFalse(line.wait(0.01))
        self.assertEqual(g.request.drained, 0)

    def test_close_and_context_manager(self):
        g = FakeGpiod()
        with InterruptLine(25, _gpiod=g):
            pass
        self.assertTrue(g.request.released)

    def test_driver_end_to_end_with_fake_line(self):
        bus, dev = make_dev()

        def on_wait(req):                    # the "chip" raises INT once 2 samples are in the FIFO
            for _ in range(2):
                bus.push_sample([500] * 8)
            req.edges += 1

        g = FakeGpiod(FakeRequest(on_wait))
        line = InterruptLine(25, _gpiod=g)
        dev.use_interrupt(line.wait, packets=2)
        dev.start()
        frame = dev.next_frame(4)
        self.assertEqual(frame, [2000] * 8)
        self.assertEqual(dev.irq_timeouts, 0)


if __name__ == "__main__":
    unittest.main()
