"""I2C driver for the ADPD1080 (EVAL-CN0569-PMDZ board) on Linux / Raspberry Pi.

The initialisation sequence and the register map are ported from ADI's
firmware (no-OS: drivers/photo-electronic/adpd188 and
projects/iio_adpd1080, BSD-3-Clause licence), which is what the official
gesture demo on the EVAL-ADICUP3029 uses.

Data format (same as ADI's demo):
  * Slot A -> PD1..4 (canales 0..3), Slot B -> PD5..8 (canales 4..7).
  * Each FIFO sample is 8 channels x 32 bit = 16 words of 16 bit
    (low word first) = 32 bytes.

Interrupts: GPIO0 of the ADPD1080 (the INT pin of the Pmod connector) can signal
"FIFO threshold reached" or "new sample in slot A/B". See use_interrupt() and
irq.py; behaviour follows the ADPD1080/ADPD1081 data sheet (Rev. C), section
"Reading Data". The FIFO is only 128 bytes, i.e. 4 samples of 32 bytes.

Requirements on the Pi:
  * `dtparam=i2c_arm=on` and, for 512 Hz, `dtparam=i2c_arm_baudrate=400000`
    in config.txt (at 100 kHz the FIFO cannot be drained fast enough at 512 Hz).
  * pip install smbus2
"""
import contextlib
import time
from collections import namedtuple

I2C_ADDR = 0x64
DEVICE_ID_ADPD108X = 0x0A16

# --- Registers ---
REG_STATUS = 0x00
REG_INT_MASK = 0x01
REG_GPIO_DRV = 0x02
REG_FIFO_THRESH = 0x06
REG_DEVID = 0x08
REG_CLK_RATIO = 0x0A
REG_GPIO_CTRL = 0x0B
REG_SW_RESET = 0x0F
REG_MODE = 0x10
REG_SLOT_EN = 0x11
REG_FSAMPLE = 0x12
REG_PD_LED_SELECT = 0x14
REG_NUM_AVG = 0x15
REG_INT_SEQ_A = 0x17
REG_INT_SEQ_B = 0x1D
REG_ILED1_COARSE = 0x23
REG_SLOTA_NUMPULSES = 0x31
REG_SLOTB_NUMPULSES = 0x36
REG_SLOTA_AFE_WINDOW = 0x39
REG_SLOTB_AFE_WINDOW = 0x3B
REG_SAMPLE_CLK = 0x4B
REG_CLK32M_ADJUST = 0x4D
REG_EXT_SYNC_SEL = 0x4F
REG_CLK32M_CAL_EN = 0x50
REG_MATH = 0x58
REG_DATA_ACCESS_CTL = 0x5F
REG_FIFO_ACCESS = 0x60

MODE_STANDBY, MODE_PROGRAM, MODE_NORMAL = 0, 1, 2
FIFO_32BIT_4CHAN = 0x6

NUM_CHANNELS = 8
WORDS_PER_SAMPLE = 16
BYTES_PER_SAMPLE = 32
FIFO_DEPTH_BYTES = 128                 # data sheet: 128-byte FIFO = 4 samples of 32 bytes

# --- Interrupt-related fields ---
GPIO_ALT_INTERRUPT = 0x01              # GPIO_CTRL alt. function "interrupt" (INT_MASK-controlled)
STATUS_SLOT_A_INT = 0x0020             # STATUS flags, write 1 to clear
STATUS_SLOT_B_INT = 0x0040
INT_MASK_FIFO = 0x0100                 # INT_MASK bits: 1 = masked, 0 = enabled
INT_MASK_SLOT_B = 0x0040
INT_MASK_SLOT_A = 0x0020
FIFO_THRESH_MASK = 0x3F00
MAX_FIFO_INT_PACKETS = 3               # a 4th packet would leave no headroom in the 128-byte FIFO

InterruptFlags = namedtuple("InterruptFlags", "slot_a slot_b fifo_bytes")


class SMBus2Bus:
    """Real I2C access through smbus2. 8-bit register addresses, 16-bit data, MSB first."""

    def __init__(self, bus=1, addr=I2C_ADDR):
        from smbus2 import SMBus, i2c_msg  # lazy import: the unit tests do not need it

        self._msg = i2c_msg
        self._bus = SMBus(bus)
        self.addr = addr

    def read_words(self, reg, count=1):
        w = self._msg.write(self.addr, [reg])
        r = self._msg.read(self.addr, 2 * count)
        self._bus.i2c_rdwr(w, r)  # repeated start, as in ADI's driver
        d = list(r)
        return [(d[i] << 8) | d[i + 1] for i in range(0, len(d), 2)]

    def write_word(self, reg, value):
        self._bus.write_i2c_block_data(
            self.addr, reg, [(value >> 8) & 0xFF, value & 0xFF]
        )

    def close(self):
        self._bus.close()


def _shift(mask):
    return (mask & -mask).bit_length() - 1


class ADPD1080:
    def __init__(self, bus, fs_hz=512, fifo_burst=True):
        """bus: object providing read_words(reg, count) and write_word(reg, val).
        fifo_burst: read the FIFO with a single long transaction on register 0x60.
        If the data looks inconsistent, set it to False (one word per read, slower).
        """
        self.bus = bus
        self.fs_hz = fs_hz
        self.fifo_burst = fifo_burst
        self._wait = None              # set by use_interrupt()
        self.interrupt_poll_s = 0.05   # safety-net timeout for each interrupt wait
        self.irq_timeouts = 0          # waits that expired without an edge

    # --- register access ---
    def reg_read(self, reg):
        return self.bus.read_words(reg, 1)[0]

    def reg_write(self, reg, value):
        self.bus.write_word(reg, value & 0xFFFF)

    def set_field(self, reg, mask, value):
        v = self.reg_read(reg)
        v = (v & ~mask) | ((value << _shift(mask)) & mask)
        self.reg_write(reg, v)

    def set_mode(self, mode):
        self.reg_write(REG_MODE, mode & 0x3)

    def get_mode(self):
        return self.reg_read(REG_MODE) & 0x3

    @contextlib.contextmanager
    def _programming(self):
        """Registers must be written in program mode; restore the previous mode after."""
        prev = self.get_mode()
        if prev != MODE_PROGRAM:
            self.set_mode(MODE_PROGRAM)
        try:
            yield
        finally:
            if prev != MODE_PROGRAM:
                self.set_mode(prev)

    # --- initialisation ---
    def init(self):
        devid = self.reg_read(REG_DEVID)
        if devid != DEVICE_ID_ADPD108X:
            raise RuntimeError(
                "Unexpected DEVID 0x%04X (expected 0x%04X). Check wiring/address."
                % (devid, DEVICE_ID_ADPD108X)
            )
        self.reg_write(REG_SW_RESET, 1)
        time.sleep(0.01)

        # State-machine clock
        self.set_field(REG_SAMPLE_CLK, 0x0080, 1)
        self.set_mode(MODE_PROGRAM)

        # Slots A and B enabled, 4 channels of 32 bit each in the FIFO.
        # bit13 RDOUT_MODE and bit12 FIFO_OVRN_PREVENT as in ADI's example.
        v = self.reg_read(REG_SLOT_EN)
        v |= 0x2000 | 0x1000
        v = (v & ~0x0001) | 0x0001                               # slot A enable
        v = (v & ~0x001C) | (FIFO_32BIT_4CHAN << 2)              # slot A FIFO mode
        v = (v & ~0x0020) | 0x0020                               # slot B enable
        v = (v & ~0x01C0) | (FIFO_32BIT_4CHAN << 6)              # slot B FIFO mode
        self.reg_write(REG_SLOT_EN, v)

        # LED1 in both slots; slot A -> PD1-4 (sel 5), slot B -> PD5-8 (sel 4)
        v = self.reg_read(REG_PD_LED_SELECT)
        v &= ~(0x0003 | 0x000C | 0x00F0 | 0x0F00)
        v |= (1 << 0) | (1 << 2) | (5 << 4) | (4 << 8)
        self.reg_write(REG_PD_LED_SELECT, v)

        self.reg_write(REG_NUM_AVG, 0)  # no averaging

        # Chop order: inverted, non-inverted, non-inverted, inverted
        self.set_field(REG_INT_SEQ_A, 0x000F, 0x9)
        self.set_field(REG_INT_SEQ_B, 0x000F, 0x9)

        # IR LED current (LED1): coarse=6, slew=3, scale=1
        v = self.reg_read(REG_ILED1_COARSE)
        v &= ~(0x000F | 0x0070)
        v |= 6 | (3 << 4) | 0x2000
        self.reg_write(REG_ILED1_COARSE, v)

        # 4 pulses per slot, period 0xE (15 us)
        self.set_field(REG_SLOTA_NUMPULSES, 0xFF00, 4)
        self.set_field(REG_SLOTA_NUMPULSES, 0x00FF, 0xE)
        self.set_field(REG_SLOTB_NUMPULSES, 0xFF00, 4)
        self.set_field(REG_SLOTB_NUMPULSES, 0x00FF, 0xE)

        # Integrator window
        self.set_field(REG_SLOTA_AFE_WINDOW, 0xF800, 4)
        self.set_field(REG_SLOTA_AFE_WINDOW, 0x07FF, 0x2F0)
        self.set_field(REG_SLOTB_AFE_WINDOW, 0xF800, 4)
        self.set_field(REG_SLOTB_AFE_WINDOW, 0x07FF, 0x2F0)

        # Math: chop inverted, non-inverted, non-inverted, inverted
        self.set_field(REG_MATH, 0x0C00, 1)  # MATH34_B
        self.set_field(REG_MATH, 0x0300, 1)  # MATH34_A
        self.set_field(REG_MATH, 0x0060, 2)  # MATH12_B
        self.set_field(REG_MATH, 0x0006, 2)  # MATH12_A

        # NOTE: ADI's demo also trims the 32 kHz clock by counting GPIO0 pulses.
        # It is omitted here: it only affects the accuracy of fs, not the gestures.

        self.calibrate_clk32m()
        self.set_sample_rate(self.fs_hz)

    def calibrate_clk32m(self):
        self.set_field(REG_DATA_ACCESS_CTL, 0x0001, 1)
        self.set_field(REG_CLK32M_CAL_EN, 0x0020, 1)
        time.sleep(0.001)
        ratio = self.reg_read(REG_CLK_RATIO) & 0x0FFF
        clk_error = 32000000.0 * (1.0 - ratio / 2000.0)
        self.reg_write(REG_CLK32M_ADJUST, int(clk_error / 112000) & 0x00FF)
        self.set_field(REG_CLK32M_CAL_EN, 0x0020, 0)
        self.set_field(REG_DATA_ACCESS_CTL, 0x0001, 0)

    def set_sample_rate(self, fs_hz):
        if not 0 < fs_hz <= 2000:
            raise ValueError("fs_hz must be between 1 and 2000")
        self.fs_hz = fs_hz
        self.reg_write(REG_FSAMPLE, 32000 // (fs_hz * 4))

    # --- acquisition ---
    def fifo_bytes(self):
        return (self.reg_read(REG_STATUS) >> 8) & 0xFF

    def fifo_clear(self):
        v = self.reg_read(REG_STATUS)
        # bit15 = clear FIFO; write 0 to the slot flags so they are not cleared
        self.reg_write(REG_STATUS, (v | 0x8000) & ~0x0060)

    def start(self):
        self.set_mode(MODE_PROGRAM)
        self.fifo_clear()
        self.set_mode(MODE_NORMAL)

    def stop(self):
        # Data sheet shutdown sequence: program mode, clear all interrupts and the
        # FIFO (write 0x80FF to STATUS), then standby.
        self.set_mode(MODE_PROGRAM)
        self.reg_write(REG_STATUS, 0x80FF)
        self.set_mode(MODE_STANDBY)

    # --- interrupts ---
    def configure_interrupt_pin(self, pin=0, active_low=False, open_drain=False):
        """Route the interrupt function to GPIO0 (the Pmod INT pin) or GPIO1.

        active_low: assert the pin low (default: active high).
        open_drain: drive the pin only while asserted (default: always driven).
        """
        if pin not in (0, 1):
            raise ValueError("pin must be 0 or 1")
        drv = int(bool(active_low)) | (int(bool(open_drain)) << 1)
        with self._programming():
            if pin == 0:
                self.set_field(REG_GPIO_DRV, 0x0007, drv | 0x4)       # polarity, drive, enable
                self.set_field(REG_GPIO_CTRL, 0x001F, GPIO_ALT_INTERRUPT)
            else:
                self.set_field(REG_GPIO_DRV, 0x0300, drv)             # bits 8 (pol) and 9 (drive)
                self.set_field(REG_EXT_SYNC_SEL, 0x0040, 1)           # GPIO1 output enable
                self.set_field(REG_GPIO_CTRL, 0x1F00, GPIO_ALT_INTERRUPT)

    def enable_fifo_interrupt(self, packets=2):
        """Assert INT when at least `packets` complete samples are in the FIFO.

        packets: 1..3 (one sample = 32 bytes = 16 words; the FIFO holds 4 samples).
        FIFO_THRESH is set to packets * 16 - 1 words, as the data sheet recommends.
        """
        if not 1 <= packets <= MAX_FIFO_INT_PACKETS:
            raise ValueError("packets must be between 1 and %d" % MAX_FIFO_INT_PACKETS)
        with self._programming():
            self.set_field(REG_FIFO_THRESH, FIFO_THRESH_MASK, packets * WORDS_PER_SAMPLE - 1)
            self.set_field(REG_INT_MASK, INT_MASK_FIFO, 0)            # 0 = enabled

    def enable_slot_interrupt(self, slot="b"):
        """Assert INT every time the data registers of slot 'a' or 'b' are updated.
        Clear the flag with clear_interrupt_flags() after reading the data."""
        mask = {"a": INT_MASK_SLOT_A, "b": INT_MASK_SLOT_B}.get(str(slot).lower())
        if mask is None:
            raise ValueError("slot must be 'a' or 'b'")
        with self._programming():
            self.set_field(REG_INT_MASK, mask, 0)

    def disable_interrupts(self):
        with self._programming():
            for mask in (INT_MASK_FIFO, INT_MASK_SLOT_A, INT_MASK_SLOT_B):
                self.set_field(REG_INT_MASK, mask, 1)                 # 1 = masked

    def get_interrupt_flags(self):
        v = self.reg_read(REG_STATUS)
        return InterruptFlags(
            slot_a=bool(v & STATUS_SLOT_A_INT),
            slot_b=bool(v & STATUS_SLOT_B_INT),
            fifo_bytes=(v >> 8) & 0xFF,
        )

    def clear_interrupt_flags(self, slot_a=True, slot_b=True):
        """Write 1 to the slot flags to clear them. Only those bits are written, so
        the FIFO is never cleared by accident. (The FIFO interrupt clears by itself
        when the FIFO is read.)"""
        v = (STATUS_SLOT_A_INT if slot_a else 0) | (STATUS_SLOT_B_INT if slot_b else 0)
        if v:
            self.reg_write(REG_STATUS, v)

    def use_interrupt(self, wait, packets=2, pin=0, active_low=False, open_drain=False):
        """Make read_samples()/next_frame() sleep on the INT line instead of polling.

        wait: callable wait(timeout_seconds) -> bool, True if an edge arrived
              (e.g. irq.InterruptLine(...).wait). Call after init() and before start().
        """
        self.configure_interrupt_pin(pin, active_low, open_drain)
        self.enable_fifo_interrupt(packets)
        self._wait = wait

    def disable_interrupt(self):
        self._wait = None
        self.disable_interrupts()

    def _read_fifo_words(self, n):
        if self.fifo_burst:
            return self.bus.read_words(REG_FIFO_ACCESS, n)
        return [self.reg_read(REG_FIFO_ACCESS) for _ in range(n)]

    @staticmethod
    def parse_sample(words):
        return tuple(
            words[2 * c] | (words[2 * c + 1] << 16) for c in range(NUM_CHANNELS)
        )

    def read_samples(self, n, timeout=1.0):
        """Return n samples, each a tuple of 8 integers (channels 0..7)."""
        out = []
        deadline = time.monotonic() + timeout
        while len(out) < n:
            avail = self.fifo_bytes() // BYTES_PER_SAMPLE
            if avail == 0:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("The FIFO is not filling: is the device in NORMAL mode?")
                if self._wait is not None:
                    # sleep until the INT edge; the short timeout is a safety net
                    if not self._wait(min(remaining, self.interrupt_poll_s)):
                        self.irq_timeouts += 1
                else:
                    time.sleep(0.0005)
                continue
            take = min(avail, n - len(out))
            words = self._read_fifo_words(take * WORDS_PER_SAMPLE)
            for i in range(take):
                out.append(
                    self.parse_sample(
                        words[i * WORDS_PER_SAMPLE:(i + 1) * WORDS_PER_SAMPLE]
                    )
                )
            deadline = time.monotonic() + timeout
        return out

    def next_frame(self, n_avg=8):
        """Sum n_avg samples per channel (equivalent to rx() with an 8-sample buffer in pyadi-iio)."""
        acc = [0] * NUM_CHANNELS
        for s in self.read_samples(n_avg):
            for i, v in enumerate(s):
                acc[i] += v
        return acc
