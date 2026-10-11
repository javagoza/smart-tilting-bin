"""Python driver for the Analog Devices ADXL372 (EVAL-ADXL372Z) on Linux / Raspberry Pi.

Register map, enums and the configuration sequence are ported from ADI's
no-OS driver (drivers/accel/adxl372, BSD-3-Clause). Scale factor, activity /
inactivity timer scaling and the FIFO read-back rule follow the behaviour of
ADI's Linux IIO driver (drivers/iio/accel/adxl372*.c, GPL-2.0+).

Both serial interfaces of the chip are supported:
  * I2C  (SMBus2 / i2c-dev)   -> I2CBus
  * SPI  (spidev, mode 0)     -> SPIBus

Data format: 12-bit two's complement, left-justified in 16 bits
(value = signed12((hi << 4) | (lo >> 4))); +/-200 g full scale, ~97.7 mg/LSB.
"""
import contextlib
import time
import warnings
from collections import namedtuple
from enum import IntEnum

I2C_ADDR_DEFAULT = 0x53  # address used in ADI's Linux example; check your board's ASEL setting

DEVID_VAL = 0xAD
DEVID_MST_VAL = 0x1D
PARTID_VAL = 0xFA
RESET_CODE = 0x52

# --- Registers ---
REG_DEVID = 0x00
REG_DEVID_MST = 0x01
REG_PARTID = 0x02
REG_REVID = 0x03
REG_STATUS_1 = 0x04
REG_STATUS_2 = 0x05
REG_FIFO_ENTRIES_2 = 0x06
REG_FIFO_ENTRIES_1 = 0x07
REG_X_DATA_H = 0x08
REG_OFFSET_X = 0x20
REG_OFFSET_Y = 0x21
REG_OFFSET_Z = 0x22
REG_TIME_ACT = 0x29
REG_TIME_INACT_H = 0x30
REG_TIME_INACT_L = 0x31
REG_HPF = 0x38
REG_FIFO_SAMPLES = 0x39
REG_FIFO_CTL = 0x3A
REG_INT1_MAP = 0x3B
REG_INT2_MAP = 0x3C
REG_TIMING = 0x3D
REG_MEASURE = 0x3E
REG_POWER_CTL = 0x3F
REG_SELF_TEST = 0x40
REG_RESET = 0x41
REG_FIFO_DATA = 0x42

# High-byte address of the X/Y/Z threshold registers (low byte = high + 1)
_THRESH_H = {
    "activity": (0x23, 0x25, 0x27),
    "activity2": (0x32, 0x34, 0x36),
    "inactivity": (0x2A, 0x2C, 0x2E),
}

# --- Bit fields ---
POWER_CTL_MODE_MSK = 0x03
POWER_CTL_HPF_DIS_MSK = 0x04
POWER_CTL_LPF_DIS_MSK = 0x08
POWER_CTL_FIL_SETTLE_MSK = 0x10
POWER_CTL_INSTANT_ON_TH_MSK = 0x20
MEASURE_BANDWIDTH_MSK = 0x07
MEASURE_LOW_NOISE_MSK = 0x08
MEASURE_LINKLOOP_MSK = 0x30
MEASURE_AUTOSLEEP_MSK = 0x40
TIMING_ODR_MSK = 0xE0
TIMING_WAKE_UP_RATE_MSK = 0x1C
FIFO_CTL_SAMPLES_MSK = 0x01
FIFO_CTL_MODE_MSK = 0x06
FIFO_CTL_FORMAT_MSK = 0x38

INT_BITS = {
    "data_rdy": 0, "fifo_rdy": 1, "fifo_full": 2, "fifo_ovr": 3,
    "inact": 4, "act": 5, "awake": 6, "low": 7,
}

ODR_HZ = (400, 800, 1600, 3200, 6400)
BANDWIDTH_HZ = (200, 400, 800, 1600, 3200)

G_PER_LSB = 400.0 / 4095       # +/-200 g over 12 bits (Linux driver: 0.958241 m/s^2 per LSB)
STANDARD_GRAVITY = 9.80665
MAX_FIFO_SAMPLES = 512


class OpMode(IntEnum):
    STANDBY = 0
    WAKE_UP = 1
    INSTANT_ON = 2
    FULL_BW = 3


class ActProcMode(IntEnum):
    DEFAULT = 0
    LINKED = 1
    LOOPED = 2


class FifoMode(IntEnum):
    BYPASSED = 0
    STREAMED = 1
    TRIGGERED = 2
    OLD_SAVED = 3


class FifoFormat(IntEnum):
    XYZ = 0
    X = 1
    Y = 2
    XY = 3
    Z = 4
    XZ = 5
    YZ = 6


FIFO_AXES = {
    FifoFormat.XYZ: "xyz", FifoFormat.X: "x", FifoFormat.Y: "y", FifoFormat.XY: "xy",
    FifoFormat.Z: "z", FifoFormat.XZ: "xz", FifoFormat.YZ: "yz",
}

Status = namedtuple(
    "Status",
    "data_rdy fifo_rdy fifo_full fifo_overrun awake inact act act2 fifo_entries raw1 raw2",
)


def signed12(hi, lo):
    """Sign-extend the left-justified 12-bit sample stored in two bytes."""
    v = (hi << 4) | (lo >> 4)
    return v - 4096 if v & 0x800 else v


# ---------------------------------------------------------------- buses
class I2CBus:
    """I2C access through smbus2. Multi-byte reads auto-increment (FIFO_DATA does not)."""

    kind = "i2c"

    def __init__(self, bus=1, addr=I2C_ADDR_DEFAULT):
        from smbus2 import SMBus, i2c_msg  # lazy import: unit tests do not need it

        self._msg = i2c_msg
        self._bus = SMBus(bus)
        self.addr = addr

    def read(self, reg, n=1):
        w = self._msg.write(self.addr, [reg])
        r = self._msg.read(self.addr, n)
        self._bus.i2c_rdwr(w, r)  # repeated start
        return list(r)

    def write(self, reg, value):
        self._bus.write_i2c_block_data(self.addr, reg, [value & 0xFF])

    def close(self):
        self._bus.close()


class SPIBus:
    """SPI access through spidev, mode 0. Address byte = (reg << 1) | R/W."""

    kind = "spi"

    def __init__(self, bus=0, device=0, speed_hz=1000000, spi=None):
        if spi is None:
            import spidev  # lazy import

            spi = spidev.SpiDev()
            spi.open(bus, device)
            spi.max_speed_hz = speed_hz
            spi.mode = 0
        self._spi = spi

    def read(self, reg, n=1):
        out = self._spi.xfer2([((reg & 0xFF) << 1) | 1] + [0] * n)
        return list(out[1:])

    def write(self, reg, value):
        self._spi.xfer2([(reg & 0xFF) << 1, value & 0xFF])

    def close(self):
        self._spi.close()


# --------------------------------------------------------------- driver
class ADXL372:
    def __init__(self, bus, odr_hz=3200, bandwidth_hz=1600, low_noise=False):
        """bus: I2CBus / SPIBus, or any object with read(reg, n) -> list[int]
        and write(reg, value)."""
        if odr_hz not in ODR_HZ:
            raise ValueError("odr_hz must be one of %s" % (ODR_HZ,))
        if bandwidth_hz not in BANDWIDTH_HZ:
            raise ValueError("bandwidth_hz must be one of %s" % (BANDWIDTH_HZ,))
        if BANDWIDTH_HZ.index(bandwidth_hz) > ODR_HZ.index(odr_hz):
            raise ValueError("bandwidth must not exceed ODR/2")
        self.bus = bus
        self._odr_idx = ODR_HZ.index(odr_hz)
        self._bw_idx = BANDWIDTH_HZ.index(bandwidth_hz)
        self._low_noise = low_noise
        self._fifo_mode = FifoMode.BYPASSED
        self._fifo_fmt = FifoFormat.XYZ
        self._act_time_ms = None
        self._inact_time_ms = None
        self.revid = None
        self.fifo_overrun = False

    # --- register access ---
    def reg_read(self, reg):
        return self.bus.read(reg, 1)[0]

    def reg_write(self, reg, value):
        self.bus.write(reg, value & 0xFF)

    def set_field(self, reg, mask, value):
        shift = (mask & -mask).bit_length() - 1
        v = self.reg_read(reg)
        v = (v & ~mask) | ((value << shift) & mask)
        self.reg_write(reg, v)

    # --- initialisation / modes ---
    def init(self):
        devid = self.reg_read(REG_DEVID)
        partid = self.reg_read(REG_PARTID)
        if devid != DEVID_VAL or partid != PARTID_VAL:
            raise RuntimeError(
                "Unexpected ID (DEVID=0x%02X, PARTID=0x%02X; expected 0xAD / 0xFA). "
                "Check wiring/address/interface." % (devid, partid)
            )
        self.revid = self.reg_read(REG_REVID)
        if getattr(self.bus, "kind", None) == "i2c" and self.revid < 3:
            warnings.warn(
                "ADXL372 REVID %d: I2C might not work properly with other devices "
                "on the bus (fixed from revision 3)" % self.revid
            )

        self.set_mode(OpMode.STANDBY)
        self.reg_write(REG_RESET, RESET_CODE)
        time.sleep(0.005)
        self.set_mode(OpMode.STANDBY)

        self.set_field(REG_TIMING, TIMING_ODR_MSK, self._odr_idx)
        self.set_field(REG_MEASURE, MEASURE_BANDWIDTH_MSK, self._bw_idx)
        self.set_field(REG_MEASURE, MEASURE_LOW_NOISE_MSK, int(self._low_noise))
        self.configure_fifo(FifoMode.BYPASSED)
        self.set_interrupts()  # nothing mapped to INT1/INT2

    def get_mode(self):
        return OpMode(self.reg_read(REG_POWER_CTL) & POWER_CTL_MODE_MSK)

    def set_mode(self, mode):
        self.set_field(REG_POWER_CTL, POWER_CTL_MODE_MSK, int(mode))

    def start(self, mode=OpMode.FULL_BW):
        self.set_mode(mode)

    def stop(self):
        self.set_mode(OpMode.STANDBY)

    def close(self):
        self.bus.close()

    @contextlib.contextmanager
    def _standby(self):
        prev = self.get_mode()
        if prev != OpMode.STANDBY:
            self.set_mode(OpMode.STANDBY)
        try:
            yield
        finally:
            if prev != OpMode.STANDBY:
                self.set_mode(prev)

    # --- measurement configuration ---
    @property
    def odr_hz(self):
        return ODR_HZ[self._odr_idx]

    @property
    def bandwidth_hz(self):
        return BANDWIDTH_HZ[self._bw_idx]

    def set_odr(self, hz):
        if hz not in ODR_HZ:
            raise ValueError("odr_hz must be one of %s" % (ODR_HZ,))
        idx = ODR_HZ.index(hz)
        with self._standby():
            self.set_field(REG_TIMING, TIMING_ODR_MSK, idx)
            self._odr_idx = idx
            if self._bw_idx > idx:  # bandwidth <= ODR/2 (Nyquist), as in the Linux driver
                self._bw_idx = idx
                self.set_field(REG_MEASURE, MEASURE_BANDWIDTH_MSK, idx)
        # the activity/inactivity timer scale depends on the ODR
        if self._act_time_ms is not None:
            self.set_activity_time_ms(self._act_time_ms)
        if self._inact_time_ms is not None:
            self.set_inactivity_time_ms(self._inact_time_ms)

    def set_bandwidth(self, hz):
        if hz not in BANDWIDTH_HZ:
            raise ValueError("bandwidth_hz must be one of %s" % (BANDWIDTH_HZ,))
        idx = BANDWIDTH_HZ.index(hz)
        if idx > self._odr_idx:
            raise ValueError("bandwidth must not exceed ODR/2 (current ODR %d Hz)" % self.odr_hz)
        with self._standby():
            self.set_field(REG_MEASURE, MEASURE_BANDWIDTH_MSK, idx)
        self._bw_idx = idx

    def set_low_noise(self, enable):
        with self._standby():
            self.set_field(REG_MEASURE, MEASURE_LOW_NOISE_MSK, int(bool(enable)))
        self._low_noise = bool(enable)

    # --- status ---
    def get_status(self):
        b = self.bus.read(REG_STATUS_1, 4)
        s1, s2 = b[0], b[1]
        entries = ((b[2] & 0x3) << 8) | b[3]
        self.fifo_overrun = bool((s1 >> 3) & 1)
        return Status(
            data_rdy=bool(s1 & 1), fifo_rdy=bool((s1 >> 1) & 1),
            fifo_full=bool((s1 >> 2) & 1), fifo_overrun=self.fifo_overrun,
            awake=bool((s1 >> 6) & 1),
            inact=bool((s2 >> 4) & 1), act=bool((s2 >> 5) & 1), act2=bool((s2 >> 6) & 1),
            fifo_entries=entries, raw1=s1, raw2=s2,
        )

    # --- single-sample reads ---
    def read_xyz_raw(self, timeout=0.5):
        """Wait for DATA_RDY and return (x, y, z) as signed 12-bit counts."""
        deadline = time.monotonic() + timeout
        while not (self.reg_read(REG_STATUS_1) & 1):
            if time.monotonic() > deadline:
                raise TimeoutError("DATA_RDY never set: is the device in measurement mode?")
            time.sleep(0.0002)
        b = self.bus.read(REG_X_DATA_H, 6)
        return (signed12(b[0], b[1]), signed12(b[2], b[3]), signed12(b[4], b[5]))

    def read_xyz_g(self, timeout=0.5):
        return tuple(c * G_PER_LSB for c in self.read_xyz_raw(timeout))

    # --- activity / inactivity detection ---
    def set_activity_threshold(self, kind, mg, referenced=True, enable=True):
        """kind: 'activity' | 'activity2' | 'inactivity'. Threshold in mg (100 mg/code)."""
        if kind not in _THRESH_H:
            raise ValueError("kind must be one of %s" % (sorted(_THRESH_H),))
        code = int(mg) // 100
        if not 0 <= code <= 0x7FF:
            raise ValueError("threshold out of range (0..204700 mg)")
        hi = (code >> 3) & 0xFF
        lo = ((code << 5) | (int(bool(referenced)) << 1) | int(bool(enable))) & 0xFF
        for addr in _THRESH_H[kind]:
            self.reg_write(addr, hi)
            self.reg_write(addr + 1, lo)

    def set_activity_time_ms(self, ms):
        scale_us = 3300 if self.odr_hz == 6400 else 6600
        code = min(0xFF, (int(ms) * 1000 + scale_us // 2) // scale_us)
        self.reg_write(REG_TIME_ACT, code)
        self._act_time_ms = int(ms)

    def set_inactivity_time_ms(self, ms):
        scale_ms = 13 if self.odr_hz == 6400 else 26
        code = min(0xFFFF, (int(ms) + scale_ms // 2) // scale_ms)
        self.reg_write(REG_TIME_INACT_H, code >> 8)
        self.reg_write(REG_TIME_INACT_L, code & 0xFF)
        self._inact_time_ms = int(ms)

    def set_activity_processing(self, mode):
        with self._standby():
            self.set_field(REG_MEASURE, MEASURE_LINKLOOP_MSK, int(mode))

    # --- interrupts ---
    def set_interrupts(self, int1=(), int2=()):
        """Map status events to the INT1 / INT2 pins.
        Names: data_rdy, fifo_rdy, fifo_full, fifo_ovr, inact, act, awake, low."""
        for pin_reg, names in ((REG_INT1_MAP, int1), (REG_INT2_MAP, int2)):
            v = 0
            for n in names:
                if n not in INT_BITS:
                    raise ValueError("unknown interrupt source %r" % n)
                v |= 1 << INT_BITS[n]
            self.reg_write(pin_reg, v)

    # --- FIFO ---
    def configure_fifo(self, mode, fmt=FifoFormat.XYZ, samples=0):
        """samples: FIFO watermark in 16-bit samples (max 512); use a multiple of the
        number of axes in `fmt`. Must be configured in standby (handled here)."""
        mode, fmt = FifoMode(mode), FifoFormat(fmt)
        if not 0 <= samples <= MAX_FIFO_SAMPLES:
            raise ValueError("samples must be between 0 and %d" % MAX_FIFO_SAMPLES)
        ctl = (int(fmt) << 3) | (int(mode) << 1) | (1 if samples > 0xFF else 0)
        with self._standby():
            self.reg_write(REG_FIFO_SAMPLES, samples & 0xFF)
            self.reg_write(REG_FIFO_CTL, ctl)
        self._fifo_mode, self._fifo_fmt = mode, fmt

    @property
    def fifo_axes(self):
        return FIFO_AXES[self._fifo_fmt]

    def read_fifo(self, leave_one_set=True, max_sets=None):
        """Drain the FIFO. Returns a list of tuples (one per sample set, axes in
        `fifo_axes` order) in signed 12-bit counts.

        leave_one_set: leave one sample set in the FIFO, as ADI's Linux driver does,
        so multi-axis data is never overwritten or stored out of order."""
        st = self.get_status()
        set_size = len(self.fifo_axes)
        sets = st.fifo_entries // set_size
        if leave_one_set and sets > 0:
            sets -= 1
        if max_sets is not None:
            sets = min(sets, max_sets)
        out = []
        chunk_sets = max(1, 480 // (2 * set_size))
        while sets > 0:
            n = min(sets, chunk_sets)
            raw = self.bus.read(REG_FIFO_DATA, n * set_size * 2)  # FIFO_DATA does not auto-increment
            for i in range(0, len(raw), 2 * set_size):
                out.append(
                    tuple(signed12(raw[i + 2 * a], raw[i + 2 * a + 1]) for a in range(set_size))
                )
            sets -= n
        return out
