# EVAL-CN0569-PMDZ Gesture Sensor Driver for Raspberry Pi

A pure-Python I²C driver and gesture/presence engine for the Analog Devices
**EVAL-CN0569-PMDZ** optical gesture board (ADPD1080 photometric front end +
2 × ADPD2140 angle-sensitive photodiodes), targeting **Linux on a Raspberry Pi
(tested design target: Compute Module 4)**.

The board does **not** recognise gestures by itself. The ADPD1080 only drives
the IR LED and digitises the photodiode currents. Everything above that
(offset removal, hand position, swipe classification, presence) runs on the host.
Analog Devices ships a reference implementation for the EVAL-ADICUP3029 that
streams samples over a serial IIO link to a PC running `pyadi-iio`. This
repository removes that dependency: the Raspberry Pi talks to the ADPD1080
directly over I²C.

> **Status.** The register sequence is ported from ADI's official firmware and
> the gesture logic is covered by unit tests using a simulated I²C bus and
> synthetic hand movements. It has **not yet been validated against real
> hardware**. See [Known limitations](#known-limitations) before relying on it.

---

## Table of contents

1. [Features](#features)
2. [Hardware](#hardware)
3. [Installation](#installation)
4. [Quick start](#quick-start)
5. [Repository layout](#repository-layout)
6. [How it works](#how-it-works)
7. [Driver reference (`adpd1080.py`)](#driver-reference-adpd1080py)
8. [Register map](#register-map)
9. [Gesture engine reference (`gesture.py`)](#gesture-engine-reference-gesturepy)
10. [Using the gestures](#using-the-gestures)
11. [Interrupts (INT / GPIO0)](#interrupts-int--gpio0)
12. [Tuning](#tuning)
13. [Recording data and rotation gestures](#recording-data-and-rotation-gestures)
14. [Testing](#testing)
15. [Troubleshooting](#troubleshooting)
16. [Known limitations](#known-limitations)
17. [Credits and licence](#credits-and-licence)

---

## Features

- Direct I²C access to the ADPD1080 through `smbus2` (no IIO server, no serial bridge).
- Initialisation sequence ported from ADI's no-OS driver, with every register
  and bit field documented below.
- Bulk FIFO readout of all 8 photodiode channels (2 sensors × 4 channels).
- **Interrupt support:** the INT pin of the Pmod connector (ADPD1080 GPIO0) can
  signal "FIFO threshold reached" or "slot data updated", and the driver can sleep
  on that line with libgpiod instead of polling (see [Interrupts](#interrupts-int--gpio0)).
- Swipe detection: `UP`, `DOWN`, `LEFT`, `RIGHT` and `CLICK`, per sensor.
- Presence detection with hysteresis (`presence` / `absence` events), meant to
  wake the application before gestures are used.
- Full (x, y, intensity) trajectory delivered with every gesture, which allows
  custom recognisers such as circular (rotate left/right) gestures or a neural network.
- Hardware-independent algorithm: the gesture engine only needs a function that
  returns a 8-integer frame, so it can be tested and replayed from CSV files.
- Unit tests that run without hardware.

---

## Hardware

### Board

| Item | Detail |
|------|--------|
| Board | Analog Devices EVAL-CN0569-PMDZ (Circuit from the Lab CN0569) |
| Front end | ADPD1080, I²C slave address **0x64**, device ID register reads **0x0A16** |
| Sensors | 2 × ADPD2140 four-channel angle-sensitive photodiodes, 25.4 mm (1 in) apart |
| Light source | IR LED driven by the ADPD1080 (LED1 output, configured by this driver) |
| Host connector | Pmod I²C (P1), 1 × 6 |

Pmod P1 signals, in pin order: 1 `INT`, 2 `RESET` (not connected), 3 `SCL`,
4 `SDA`, 5 `GND`, 6 `VCC`. `INT` carries the ADPD1080's **GPIO0** signal; on this
pin it uses the Pmod `VCC` as its logic level, like the I²C lines.

### Wiring to a Raspberry Pi / CM4 IO board

| Pmod pin | Raspberry Pi |
|----------|--------------|
| SDA | GPIO2 (pin 3), I²C1 SDA |
| SCL | GPIO3 (pin 5), I²C1 SCL |
| VCC | 3.3 V |
| GND | GND |
| INT (Pmod pin 1) | Optional: GPIO25 (pin 22) or any free GPIO, 3.3 V logic. Needed only for [interrupt-driven reads](#interrupts-int--gpio0) |
| RESET | Not connected |

### Channel mapping

The driver configures the ADPD1080 exactly like ADI's IIO firmware:

| Channels in a frame | ADPD1080 slot | Photodiodes | Sensor |
|---------------------|---------------|-------------|--------|
| 0 – 3 | Slot A | PD1 – PD4 | Sensor 0 |
| 4 – 7 | Slot B | PD5 – PD8 | Sensor 1 |

Inside each group of four channels `c[0..3]` the hand position is computed as:

```
x = (c[1] - c[0]) / (c[1] + c[0])
y = (c[3] - c[2]) / (c[3] + c[2])
L = c[0] + c[1] + c[2] + c[3]        # total reflected light, grows as the hand gets closer
```

`x` and `y` lie in [-1, 1] and `L` is the intensity used for both presence and
gesture start/end detection.

---

## Installation

### 1. Enable I²C at 400 kHz

Add to `/boot/firmware/config.txt` (or `/boot/config.txt` on older images) and reboot:

```
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

> **Why 400 kHz?** At the default 512 Hz sample rate every sample is 32 bytes in
> the FIFO, i.e. about 16 kB/s of payload, which is roughly 147 kbit/s on the
> wire once the 9th (ACK) bit of each byte is counted. That does not fit in a
> 100 kHz bus. If you must stay at 100 kHz, run at 128–256 Hz (`--fs 256`).

On a CM4, make sure I²C1 is exposed on your carrier board (GPIO2/GPIO3).

### 2. Install dependencies

```bash
sudo apt install -y i2c-tools python3-pip
pip install smbus2
pip install gpiod        # only for interrupt-driven reads (libgpiod Python bindings, API v2)
```

### 3. Check the bus

```bash
i2cdetect -y 1
```

You should see `64` in the table. Without it, check wiring, power and pull-ups.

---

## Quick start

1. **Raw check.** Move your hand in front of the sensors and watch the eight
   channels react:

   ```bash
   python3 demo.py --raw
   ```

2. **Gestures and presence:**

   ```bash
   python3 demo.py
   ```

   Keep your hand away during the 16-frame calibration, then swipe. Example output:

   ```
   Calibrating (keep your hand away)...
   Ready.
   PRESENCE
   GESTURE sensor 0: RIGHT
   GESTURE sensor 1: RIGHT
   ABSENCE
   ```

3. **Record raw frames for later analysis or training:**

   ```bash
   python3 demo.py --record session1.csv
   ```

4. **Interrupt-driven reads** (INT wired to GPIO25, see [Interrupts](#interrupts-int--gpio0)):

   ```bash
   python3 demo.py --int-gpio 25
   ```

`demo.py` options:

| Option | Default | Meaning |
|--------|---------|---------|
| `--bus` | `1` | I²C bus number (`/dev/i2c-N`) |
| `--addr` | `0x64` | ADPD1080 I²C address |
| `--fs` | `512` | ADC sample rate in Hz (1–2000) |
| `--l-thresh` | `1000` | Intensity threshold that starts/ends a gesture |
| `--d-thresh` | `0.07` | Minimum displacement to count as a swipe (else `CLICK`) |
| `--no-burst` | off | Read the FIFO one word at a time (slower, see [Troubleshooting](#troubleshooting)) |
| `--int-gpio BCM` | none | BCM GPIO wired to the Pmod INT pin: sleep on the FIFO interrupt instead of polling |
| `--int-active-low` | off | Configure INT as active low (falling edge) |
| `--int-open-drain` | off | Drive INT only while asserted (needs a pull resistor on the line) |
| `--int-packets N` | `2` | Samples in the FIFO (1–3) that raise the interrupt |
| `--gpio-chip PATH` | `/dev/gpiochip0` | GPIO chip, see `gpiodetect` |
| `--raw` | off | Print raw frames only |
| `--record CSV` | none | Save raw frames (timestamp + 8 channels) |

---

## Repository layout

```
.
├── adpd1080.py        # Hardware layer: I²C bus wrapper + ADPD1080 driver
├── gesture.py         # Hardware-independent gesture and presence engine
├── irq.py             # INT line edge waiting with libgpiod (optional)
├── demo.py            # Command-line demo / data recorder
├── test_cn0569.py     # Unit tests (fake I²C bus, fake GPIO line, synthetic hand movements)
└── README.md
```

---

## How it works

```
 IR LED ──► hand ──► ADPD2140 ×2 ──► ADPD1080 (pulses LED, integrates, ADC)
                                        │  I²C (0x64)
                                        ▼
                         adpd1080.py : FIFO → samples (8 × 32 bit)
                                        │  sum of N samples = one "frame"
                                        ▼
                    gesture.py : baseline removal → (x, y, L) per sensor
                                 → start/end detection → classification
                                        │
                                        ▼
                     Events: presence, absence, gesture (+ trajectory)
```

### Data format

Each ADC sample is stored in the FIFO as 8 channels × 32 bit, sent as 16-bit
words, low word first:

```
word[2*ch] | (word[2*ch + 1] << 16)    for ch = 0 … 7
```

One sample is therefore 16 words = 32 bytes. The `STATUS` register reports the
number of bytes in the FIFO in bits 15:8, so the driver reads
`bytes // 32` complete samples at a time.

**The FIFO is only 128 bytes: 4 samples.** The data sheet states that a new
sample is written only if a complete packet fits, and is lost otherwise. At 512 Hz
a new sample arrives every 1.95 ms, so a full FIFO represents just 7.8 ms of data.
Keep the read loop light (or use [interrupts](#interrupts-int--gpio0) to be woken
as soon as data is ready), or lower the sample rate. Reading `FIFO_ACCESS` (`0x60`)
advances through the FIFO automatically and frees the space, which is why one long
burst read returns successive words.

### Frames

`ADPD1080.next_frame(n_avg=8)` adds `n_avg` consecutive samples per channel. This
reproduces `rx()` with an 8-sample buffer in ADI's `pyadi-iio` demo. At 512 Hz
it produces ~64 frames per second, and the gesture engine runs once per frame.

### Gesture algorithm

Per sensor, for every frame:

1. Subtract the calibrated baseline from each channel (clamped at zero).
2. Compute `L`, `x`, `y`.
3. **Start:** `L > l_thresh` while idle. Record the first `(x, y)`.
4. **Track:** while `L > l_thresh`, append `(x, y, L)` to the trajectory.
5. **End:** `L <= l_thresh` while active. If the gesture lasted at least
   `min_frames` frames after the first one, classify it; otherwise discard it.
6. **Classify** using the first and last trajectory points `(sx, sy)` → `(ex, ey)`:

| Condition | Result |
|-----------|--------|
| distance `< d_thresh` | `CLICK` |
| slope `m = (sy-ey)/(sx-ex)`, `abs(m) > 1`, `sy < ey` | `UP` |
| `abs(m) > 1`, otherwise | `DOWN` |
| `abs(m) < 1`, `sx > ex` | `LEFT` |
| `abs(m) < 1`, otherwise | `RIGHT` |
| `abs(m) == 1` | `CLICK` |

### Differences from ADI's reference implementation

| Topic | ADI `adpd2140_gesture_sensor` | This repository |
|-------|-------------------------------|-----------------|
| Transport | `pyadi-iio` over a serial IIO server on a microcontroller | Direct I²C from the host |
| Frame handling | `rx()` is called once per sensor, so each sensor consumes different data | One frame feeds both sensors |
| Baseline | `avg` is a class attribute shared by all instances | Per-instance baseline |
| Short gestures | Leave the "active" flag set with a stale start point | Discarded |
| End position | Taken on the frame where the signal already dropped below the threshold, where `x`/`y` are dominated by noise | Last frame **with** signal |
| Trajectory | Not kept | Returned with every gesture |
| Presence | Not available | Built in, with hysteresis |

---

## Driver reference (`adpd1080.py`)

### `SMBus2Bus(bus=1, addr=0x64)`

Thin wrapper over `smbus2`. Any object that implements the following two
methods can be used instead, which is how the unit tests simulate the chip:

| Method | Description |
|--------|-------------|
| `read_words(reg, count=1) -> list[int]` | Write the 8-bit register address, then read `count` 16-bit words (MSB first) using a repeated start |
| `write_word(reg, value)` | Write `reg`, then the 16-bit value MSB first, in a single transaction |

### `ADPD1080(bus, fs_hz=512, fifo_burst=True)`

| Method | Description |
|--------|-------------|
| `init()` | Verify `DEVID == 0x0A16`, software reset, configure slots, LED, timing, FIFO format, calibrate the 32 MHz clock and set the sample rate. Leaves the chip in **program** mode |
| `start()` | Program mode → clear FIFO → **normal** mode (sampling) |
| `stop()` | Data-sheet shutdown sequence: program mode → clear all interrupt flags and the FIFO (`STATUS ← 0x80FF`) → standby |
| `set_sample_rate(fs_hz)` | Writes `FSAMPLE = 32000 // (fs_hz * 4)`; valid range 1–2000 Hz |
| `calibrate_clk32m()` | 32 MHz oscillator trim, as in ADI's `adpd188_clk32mhz_cal` |
| `fifo_bytes() -> int` | Bytes currently in the FIFO (`STATUS[15:8]`) |
| `fifo_clear()` | Flush the FIFO |
| `read_samples(n, timeout=1.0)` | Return `n` samples, each a tuple of 8 integers. Raises `TimeoutError` if the FIFO stays empty |
| `next_frame(n_avg=8)` | Sum of `n_avg` samples per channel → list of 8 integers |
| `use_interrupt(wait, packets=2, pin=0, active_low=False, open_drain=False)` | Configure the INT pin and the FIFO interrupt, and make `read_samples()` / `next_frame()` call `wait(timeout)` instead of polling. Call after `init()`, before `start()` |
| `disable_interrupt()` | Back to polling; masks all interrupt sources |
| `configure_interrupt_pin(pin=0, active_low=False, open_drain=False)` | Route the interrupt function to GPIO0 (Pmod INT) or GPIO1 and set polarity / drive |
| `enable_fifo_interrupt(packets=2)` | Assert INT when at least `packets` (1–3) complete samples are in the FIFO |
| `enable_slot_interrupt(slot)` | Assert INT when the data registers of slot `"a"` or `"b"` are updated |
| `disable_interrupts()` | Mask the FIFO and both slot interrupts |
| `get_interrupt_flags()` | `InterruptFlags(slot_a, slot_b, fifo_bytes)` read from `STATUS` |
| `clear_interrupt_flags(slot_a=True, slot_b=True)` | Write 1 to the slot flags (only those bits, so the FIFO is never cleared by accident) |
| `irq_timeouts` | Counter of interrupt waits that expired without an edge (a diagnostic) |
| `interrupt_poll_s` | Maximum time of one interrupt wait, 0.05 s by default (safety net) |
| `get_mode()` | Current `MODE[1:0]` |
| `reg_read(reg)` / `reg_write(reg, value)` | Raw 16-bit register access |
| `set_field(reg, mask, value)` | Read-modify-write of a bit field |
| `set_mode(mode)` | `MODE_STANDBY` (0), `MODE_PROGRAM` (1), `MODE_NORMAL` (2) |

`fifo_burst=True` reads the whole FIFO payload with one long I²C read on
register `0x60`. With `False` it reads one word per transaction.

### Initialisation sequence

`init()` performs these steps, in this order (all values from ADI's
`adpd188_iio_init` and `iio_adpd1080` example):

1. Read `DEVID`; abort if it is not `0x0A16`.
2. `SW_RESET ← 1`, wait 10 ms.
3. `SAMPLE_CLK[7] ← 1` (enable the state-machine clock).
4. `MODE ← PROGRAM`.
5. `SLOT_EN`: enable slot A and B, FIFO mode `32-bit × 4 channels` for both,
   set `FIFO_OVRN_PREVENT` and `RDOUT_MODE`.
6. `PD_LED_SELECT ← 0x0455`: LED1 in both slots, slot A on PD1–4, slot B on PD5–8.
7. `NUM_AVG ← 0` (no averaging).
8. `INT_SEQ_A`, `INT_SEQ_B` `[3:0] ← 0x9` (chop order inverted / non-inverted / non-inverted / inverted).
9. `ILED1_COARSE ← 0x2036` (coarse 6, slew 3, scale bit set).
10. `SLOTA_NUMPULSES`, `SLOTB_NUMPULSES ← 0x040E` (4 pulses, period `0xE`, ~15 µs).
11. `SLOTA_AFE_WINDOW`, `SLOTB_AFE_WINDOW ← 0x22F0` (width 4, offset `0x2F0`).
12. `MATH ← 0x0544` (chop filter configuration matching the integrator order).
13. 32 MHz clock calibration.
14. `FSAMPLE` from the requested sample rate.

The 32 kHz clock trim that ADI's firmware performs by counting GPIO0 pulses is
**not** implemented; it only affects the absolute accuracy of the sample rate.

---

## Register map

Only registers touched by this driver are listed. Names and bit fields follow
ADI's `adpd188.h`; consult the ADPD1080 datasheet for the electrical meaning of
each field. All registers are 16 bits wide and are transferred MSB first.

| Addr | Name | Bits / fields used | Value written by `init()` |
|------|------|--------------------|---------------------------|
| `0x00` | `STATUS` | `[15:8]` FIFO bytes (read); bit 15 = clear FIFO (write 1, in program mode); bit 6 slot B int flag, bit 5 slot A int flag (write 1 to clear) | `fifo_clear()` writes `status \| 0x8000` with the slot flags masked to 0; `clear_interrupt_flags()` writes only the flag bits; `stop()` writes `0x80FF` |
| `0x01` | `INT_MASK` | bit 8 FIFO, bit 6 slot B, bit 5 slot A. **1 = masked, 0 = enabled** | `enable_*_interrupt()` clear the bit, `disable_interrupts()` sets them |
| `0x02` | `GPIO_DRV` | GPIO0: bit 0 polarity (1 = active low), bit 1 drive (1 = drive only when asserted), bit 2 pin enable. GPIO1: bit 8 polarity, bit 9 drive | `configure_interrupt_pin()`: active high, always driven, enabled |
| `0x06` | `FIFO_THRESH` | `[13:8]` FIFO interrupt threshold in 16-bit words (max 63) | `enable_fifo_interrupt()`: `packets * 16 - 1` |
| `0x0B` | `GPIO_CTRL` | GPIO0 function `[4:0]`, GPIO1 function `[12:8]`; `0` = ADPD103-compatible INT, `1` = interrupt function from `INT_MASK` | `1` for the selected pin |
| `0x08` | `DEVID` | Device ID | Read only; expected `0x0A16` |
| `0x0A` | `CLK_RATIO` | `[11:0]` 32 MHz clock ratio measurement | Read during clock calibration |
| `0x0F` | `SW_RESET` | bit 0 | `1` |
| `0x10` | `MODE` | `[1:0]`: 0 standby, 1 program, 2 normal | Changed by `set_mode()` |
| `0x11` | `SLOT_EN` | bit 0 slot A enable; `[4:2]` slot A FIFO mode; bit 5 slot B enable; `[8:6]` slot B FIFO mode; bit 12 `FIFO_OVRN_PREVENT`; bit 13 `RDOUT_MODE` | Slots on, FIFO mode `6` (32-bit × 4 ch) for both, bits 12 and 13 set |
| `0x12` | `FSAMPLE` | Sample period divider | `32000 // (fs_hz * 4)` → `15` at 512 Hz |
| `0x14` | `PD_LED_SELECT` | `[1:0]` slot A LED; `[3:2]` slot B LED; `[7:4]` slot A PD; `[11:8]` slot B PD | `0x0455` (A: LED1, PD sel 5; B: LED1, PD sel 4) |
| `0x15` | `NUM_AVG` | Averaging per slot | `0` |
| `0x17` | `INT_SEQ_A` | `[3:0]` integration (chop) order, slot A | `0x9` |
| `0x1D` | `INT_SEQ_B` | `[3:0]` integration (chop) order, slot B | `0x9` |
| `0x23` | `ILED1_COARSE` | `[3:0]` coarse current; `[6:4]` slew; bit 13 scale | `0x2036` |
| `0x31` | `SLOTA_NUMPULSES` | `[15:8]` pulses; `[7:0]` pulse period | `0x040E` |
| `0x36` | `SLOTB_NUMPULSES` | `[15:8]` pulses; `[7:0]` pulse period | `0x040E` |
| `0x39` | `SLOTA_AFE_WINDOW` | `[15:11]` width; `[10:0]` offset | `0x22F0` |
| `0x3B` | `SLOTB_AFE_WINDOW` | `[15:11]` width; `[10:0]` offset | `0x22F0` |
| `0x4B` | `SAMPLE_CLK` | bit 7 `CLK32K_EN` | bit 7 set |
| `0x4D` | `CLK32M_ADJUST` | `[7:0]` 32 MHz trim | Computed from `CLK_RATIO` |
| `0x4F` | `EXT_SYNC_SEL` | bit 6 GPIO1 output enable | Set only when the interrupt is routed to GPIO1 |
| `0x50` | `CLK32M_CAL_EN` | bit 5 calibration enable | Set, then cleared during calibration |
| `0x58` | `MATH` | `[11:10]` MATH34_B; `[9:8]` MATH34_A; `[6:5]` MATH12_B; `[2:1]` MATH12_A | `0x0544` |
| `0x5F` | `DATA_ACCESS_CTL` | bit 0 digital clock enable | Set and cleared during clock calibration |
| `0x60` | `FIFO_ACCESS` | FIFO data port (16-bit words); the address does not increment, each word read advances through the FIFO | Read in bursts |

### I²C transaction format

```
Write register : [ADDR+W] [reg] [value_hi] [value_lo]
Read register  : [ADDR+W] [reg] [Sr] [ADDR+R] [value_hi] [value_lo]
Read FIFO      : [ADDR+W] [0x60] [Sr] [ADDR+R] [hi][lo] [hi][lo] …   (burst mode)
```

### Clock calibration formula

```
clk_error     = 32_000_000 * (1 - CLK_RATIO / 2000)
CLK32M_ADJUST = int(clk_error / 112000) & 0xFF        # ADPD1080
```

---

## Gesture engine reference (`gesture.py`)

### `Gesture` (IntEnum)

| Name | Value |
|------|-------|
| `CLICK` | 0 |
| `UP` | 1 |
| `DOWN` | 2 |
| `LEFT` | 3 |
| `RIGHT` | 4 |

Values match the numbering used in ADI's demo.

### `Event`

| Field | Description |
|-------|-------------|
| `kind` | `"gesture"`, `"presence"` or `"absence"` |
| `sensor` | `0` or `1` for gestures, `None` for presence/absence |
| `gesture` | A `Gesture` for gesture events, otherwise `None` |
| `timestamp` | `time.time()` when the event was generated |
| `trajectory` | List of `(x, y, L)` tuples for gesture events |

### `GestureEngine(read_frame, on_event=None, ...)`

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `read_frame` | required | Callable returning a list of 8 integers (e.g. `dev.next_frame`) |
| `on_event` | no-op | Callback receiving each `Event` |
| `l_thresh` | `1000` | Intensity above which a gesture is active (per sensor, after baseline removal) |
| `d_thresh` | `0.07` | Minimum start–end distance for a swipe, otherwise `CLICK` |
| `min_frames` | `5` | Minimum frames after the first one for a gesture to be accepted |
| `presence_on` | `600` | Intensity needed to count a frame towards presence |
| `presence_off` | `300` | Intensity below which a frame counts towards absence |
| `presence_on_frames` | `2` | Consecutive frames above `presence_on` to raise `presence` |
| `presence_off_frames` | `10` | Consecutive frames below `presence_off` to raise `absence` |

| Method | Description |
|--------|-------------|
| `calibrate(n=16)` | Average `n` frames with no hand present and store them as the baseline |
| `process_frame(frame, t=None) -> list[Event]` | Feed one frame manually (used by tests and CSV replay) |
| `run(should_stop=lambda: False)` | Read frames and process them until `should_stop()` returns `True` |

`SensorTracker`, `position()` and `classify()` are also public, in case you
want to reuse part of the algorithm.

---

## Using the gestures

### Basic: react to swipes and presence

```python
from adpd1080 import ADPD1080, SMBus2Bus
from gesture import GestureEngine, Gesture

dev = ADPD1080(SMBus2Bus(bus=1), fs_hz=512)
dev.init()
dev.start()

def on_event(e):
    if e.kind == "presence":
        print("Hand detected: wake the UI")
    elif e.kind == "absence":
        print("Hand gone: go back to idle")
    elif e.kind == "gesture" and e.sensor == 0:   # use one sensor to avoid duplicates
        actions = {
            Gesture.LEFT:  lambda: print("previous"),
            Gesture.RIGHT: lambda: print("next"),
            Gesture.UP:    lambda: print("volume up"),
            Gesture.DOWN:  lambda: print("volume down"),
            Gesture.CLICK: lambda: print("select"),
        }
        actions[e.gesture]()

engine = GestureEngine(dev.next_frame, on_event)
engine.calibrate()          # keep your hand away
try:
    engine.run()
finally:
    dev.stop()
```

### Each gesture produces two events

Every swipe is seen by both ADPD2140 sensors, so you normally get one event with
`sensor == 0` and one with `sensor == 1`. Options:

- Use only one sensor, as above.
- Require agreement between both sensors before acting:

```python
import time

class Combiner:
    def __init__(self, callback, window=0.3):
        self.cb, self.window, self.pending = callback, window, {}

    def __call__(self, e):
        if e.kind != "gesture":
            return
        self.pending[e.sensor] = (e.gesture, e.timestamp)
        if len(self.pending) == 2:
            (g0, t0), (g1, t1) = self.pending[0], self.pending[1]
            self.pending.clear()
            if g0 == g1 and abs(t0 - t1) < self.window:
                self.cb(g0)
```

### Remapping directions to your mounting

"Left" and "up" are defined in the sensor coordinate system, so they depend on
how the board is mounted. Test each direction once, then remap in your
application without touching the library:

```python
FLIP_X = {Gesture.LEFT: Gesture.RIGHT, Gesture.RIGHT: Gesture.LEFT}
FLIP_Y = {Gesture.UP: Gesture.DOWN, Gesture.DOWN: Gesture.UP}
SWAP_XY = {Gesture.LEFT: Gesture.UP, Gesture.UP: Gesture.LEFT,
           Gesture.RIGHT: Gesture.DOWN, Gesture.DOWN: Gesture.RIGHT}

def remap(g, table):
    return table.get(g, g)
```

### Presence only

```python
def on_event(e):
    if e.kind in ("presence", "absence"):
        gpio_or_app_signal(e.kind == "presence")
```

Presence is derived from the larger of the two sensors' intensity, with
hysteresis (`presence_on`/`presence_off`) and debouncing (`*_frames`) so brief
flicker does not toggle it.

### Replaying a recording (no hardware)

```python
import csv
from gesture import GestureEngine

frames = [[int(v) for v in row[1:]] for row in list(csv.reader(open("session1.csv")))[1:]]
it = iter(frames)
engine = GestureEngine(lambda: next(it), print)
engine.baseline = frames[0]            # or average the first frames
for f in frames:
    engine.process_frame(f)
```

---

## Interrupts (INT / GPIO0)

### What the chip offers

The ADPD1080 has two GPIO pins that, among other functions, can signal interrupts.
According to the data sheet (Rev. C, section *Reading Data*) three events exist:

| Event | Raised when | Cleared by |
|-------|-------------|------------|
| FIFO threshold | The FIFO holds more words than `FIFO_THRESH` | Automatically, as soon as any data is read from the FIFO. It is raised again on the next write that leaves the FIFO above the threshold |
| Slot A data | The slot A data registers were updated | Writing 1 to `STATUS` bit 5 |
| Slot B data | The slot B data registers were updated | Writing 1 to `STATUS` bit 6 |

There is **no hardware presence or threshold comparator**: the interrupt only says
that data is ready. Presence and gestures are still computed in software from the
samples (`gesture.py`). What the interrupt buys you is that the host can sleep
until data arrives instead of polling `STATUS` in a loop.

### On the board

- **Pmod pin 1, `INT`, is the ADPD1080 GPIO0.** On this pin the signal uses the
  Pmod `VCC` (3.3 V from the Pi) as its logic level, like the I²C lines.
- GPIO0 and GPIO1 are also available on test points, connected directly to the
  ADPD1080 pins, so **those are 1.8 V signals:** the data sheet only guarantees
  compatibility with a 1.8 V supply and rates the GPIO pins at −0.3 V to +2.2 V. Do not connect a test point to a 3.3 V input
  without a level translator. The driver can route the interrupt to GPIO1, but
  only GPIO0 reaches the Pmod connector.
- Some controller boards leave the Pmod `INT` pin unconnected; on the Pi you wire it
  yourself, as ADI does with a breadboard wire on the EVAL-ADICUP3029.

Wiring:

| Pmod P1 | Raspberry Pi / CM4 |
|---------|--------------------|
| Pin 1 INT | GPIO25 (pin 22), or any free GPIO |
| Pin 5 GND | GND (already connected for I²C) |

### Configuration

`use_interrupt()` does everything in one call (call it after `init()` and before
`start()`):

```python
from adpd1080 import ADPD1080, SMBus2Bus
from irq import InterruptLine
from gesture import GestureEngine

dev = ADPD1080(SMBus2Bus(bus=1), fs_hz=512)
dev.init()

irq = InterruptLine(25)                       # INT (Pmod pin 1) wired to GPIO25
dev.use_interrupt(irq.wait, packets=2)        # configure the chip + sleep on the line
dev.start()

engine = GestureEngine(dev.next_frame, print)
engine.calibrate()                            # keep your hand away
try:
    engine.run()
finally:
    dev.stop()
    irq.close()
```

Equivalent command line: `python3 demo.py --int-gpio 25`.

What `use_interrupt()` writes (always in program mode; the previous mode is
restored afterwards):

| Step | Register | Value |
|------|----------|-------|
| Pin function | `GPIO_CTRL` (`0x0B`) | `[4:0] = 1` for GPIO0: interrupt function |
| Pin drive | `GPIO_DRV` (`0x02`) | bit 2 = 1 (enable); bit 0 = polarity; bit 1 = drive |
| Threshold | `FIFO_THRESH` (`0x06`) | `[13:8] = packets * 16 - 1` words |
| Unmask | `INT_MASK` (`0x01`) | bit 8 cleared (FIFO interrupt enabled) |

The threshold follows the data sheet recommendation: one packet (one sample, 16
words) minus one word makes the interrupt fire when at least one complete packet is
available. With `packets=2` it fires at 32 words, i.e. two samples.

### Choosing `packets`

The FIFO holds 4 samples, so only 1, 2 or 3 are accepted (4 would leave no room
for new data once the interrupt fires).

| `packets` | Wake-ups at 512 Hz | Time left before the FIFO is full |
|-----------|--------------------|-----------------------------------|
| 1 | 512 / s | ~5.9 ms |
| 2 (default) | 256 / s | ~3.9 ms |
| 3 | 171 / s | ~2.0 ms |

Fewer wake-ups mean less overhead, but the host has less time to react. With
`packets=2` the I²C traffic is the same as with polling; the gain is that the
process sleeps until the data is there, instead of waking every 0.5 ms to poll
`STATUS`, and that it is woken as soon as the data is ready.

### Polarity and drive

| Option | Chip setting | Edge to wait for |
|--------|--------------|------------------|
| default | Active high, always driven (push-pull) | Rising |
| `active_low=True` | Active low | Falling |
| `open_drain=True` | Drives the line only while asserted | Needs a pull resistor on the line; the Pmod level translation on the board may also matter, so verify with a scope |

`InterruptLine(pin, active_low=...)` and `use_interrupt(..., active_low=...)` must
match. Check the actual behaviour of your board with `gpiomon` or a scope before
trusting the defaults.

### How the read path behaves

`read_samples()` first looks at the FIFO level. Only when there is no complete
sample does it call `wait(timeout)`. Because the FIFO is checked before every wait,
an edge that arrived while the program was busy is never missed. Each wait is
limited to `interrupt_poll_s` (50 ms) as a safety net: if a wait expires without an
edge, `irq_timeouts` is incremented and the FIFO is checked again. A counter that
keeps growing while data flows means the edge or the wiring is wrong (the driver
then degrades to a 20 Hz poll, which is too slow for 512 Hz and will drop data).

`InterruptLine.wait()` drains every queued edge event on return, so stale events
cannot cause a burst of empty wake-ups.

### Slot-data interrupts

For applications that read the data registers instead of the FIFO, the slot
interrupts are exposed as well:

```python
dev.enable_slot_interrupt("b")                # slot B is updated last when both are in use
dev.start()
while True:
    if irq.wait(1.0):
        flags = dev.get_interrupt_flags()
        if flags.slot_b:
            words = dev.bus.read_words(0x64, 8)   # 0x64-0x67 slot A, 0x68-0x6B slot B (16-bit, clipped)
            dev.clear_interrupt_flags()
```

The gesture engine does not use this path, and these calls are covered by register
tests only.

### Presence and idle power

Because presence is software-based, "wake on presence" is built as: run the
interrupt-driven loop at a low sample rate while idle and let the engine's
`presence` event decide when to switch to the gesture rate. Changing the sample
rate while sampling requires going through program mode, and that live switch is
**not implemented** in this driver yet.

### Precautions

- Call `use_interrupt()` before `start()`. If you reconfigure `INT_MASK`, `FIFO_THRESH`
  or the GPIO registers while sampling, the driver goes through program mode and
  restores the previous mode, as the data sheet requires, but samples are not collected
  in the meantime.
- `stop()` clears all interrupt flags and the FIFO before standby, as the data sheet
  recommends.
- Do not call `clear_interrupt_flags()` by writing back a full `STATUS` read: when the
  FIFO is full, bit 15 of that value is set and writing it back would clear the FIFO.
  The driver writes only the flag bits.
- Debug the line with `gpiomon` (the option syntax differs between libgpiod v1 and v2,
  see `gpiomon --help`) before involving the driver.

---

## Tuning

| Symptom | What to adjust |
|---------|----------------|
| Gestures never trigger | Lower `l_thresh`. Use `demo.py --raw` and compare channel values with and without a hand at your working distance |
| Constant false triggers | Raise `l_thresh` and/or `presence_on`; recalibrate with nothing in front of the sensor; check for strong sunlight or IR sources |
| Small movements classified as `CLICK` | Lower `d_thresh` |
| Wobbly gestures classified as `CLICK` | Raise `d_thresh` |
| Fast swipes are missed | Raise the sample rate (`--fs`) or lower `min_frames` |
| Slow hand movements split into two gestures | Lower `l_thresh` slightly or add hold time before declaring the end |
| Presence flickers | Increase `presence_off_frames` or widen the gap between `presence_on` and `presence_off` |

LED current (`ILED1_COARSE`) and the integrator window can be changed through
`set_field()` after `init()` while the chip is in program mode, to match your
working distance. Put the chip back in normal mode with `start()` afterwards.

---

## Recording data and rotation gestures

A full circle ends near where it started, so the swipe classifier reports it
as `CLICK`. The trajectory returned with every gesture, however, contains enough
information to tell the rotation direction. The signed area enclosed by the
`(x, y)` path (shoelace formula) is positive for one sense of rotation and
negative for the other:

```python
def signed_area(traj):
    pts = [(x, y) for x, y, _ in traj]
    return 0.5 * sum(x0 * y1 - x1 * y0
                     for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))

def rotation(traj, min_area=0.05):
    a = signed_area(traj)
    if abs(a) < min_area:
        return None
    return "ccw" if a > 0 else "cw"

def on_event(e):
    if e.kind == "gesture" and e.gesture == Gesture.CLICK:
        r = rotation(e.trajectory)
        if r:
            print("rotate", r)
```

On synthetic circular trajectories this returns `ccw` for one direction and
`cw` for the other, and `None` for straight swipes. Which physical direction is
"ccw" depends on the sensor orientation, so verify it with a real hand and
flip the labels if needed. The `min_area` threshold must also be tuned on real data.

For more gestures or better robustness (several users, distances, lighting),
record labelled data with `--record` and train a small classifier (1-D CNN, GRU
or a random forest on hand-crafted features) using the 8 baseline-subtracted
channels, normalised by their sum so the distance dependence is reduced. Always
include a "no gesture" class, and use the intensity-based start/end detection to
cut the input windows.

---

## Testing

The tests need no hardware:

```bash
python3 -m unittest -v test_cn0569
```

They cover:

- Register values written by `init()` (including `PD_LED_SELECT`, `FSAMPLE`, `MATH`).
- Rejection of a wrong device ID.
- FIFO parsing (low word first, 32-bit channels) in burst and word-by-word modes.
- Timeout when the FIFO stays empty.
- Swipes in four directions, `CLICK`, short-blip rejection, two consecutive gestures.
- Presence/absence hysteresis and trajectory recording.
- Interrupt configuration for GPIO0 and GPIO1: `GPIO_DRV` polarity/drive/enable bits,
  `GPIO_CTRL` function, `FIFO_THRESH` values, `INT_MASK` handling (mask polarity, other bits
  preserved), restoring the operating mode, the shutdown sequence and flag clearing
  that never touches the FIFO-clear bit.
- Interrupt-driven reads: sleeping only when the FIFO is empty, counting expired
  waits, timeouts, and returning to polling.
- `InterruptLine` with a fake `gpiod` module: rising/falling edge selection, draining
  queued events, release on close, and an end-to-end read with a simulated INT line.
  The real GPIO access needs hardware.

These tests use a simulated chip and synthetic hand movements; they prove that
the code is self-consistent, not that it matches the physical device.

---

## Troubleshooting

| Problem | Likely cause / fix |
|---------|--------------------|
| `i2cdetect` does not show `0x64` | Wiring, power (3.3 V), I²C not enabled, wrong bus number |
| `RuntimeError: unexpected DEVID` | A different device answers at the address, or the bus is corrupted. Check with `i2cget -y 1 0x64 0x08 w` (the byte order will be swapped by `i2cget`) |
| `TimeoutError` while reading samples | The chip is not in normal mode (`start()` not called), the sample clock is disabled, or the FIFO is not being filled |
| FIFO overruns / stuttering data | The FIFO holds only 4 samples (7.8 ms at 512 Hz). I²C too slow: enable 400 kHz or reduce `--fs`; avoid heavy work between frames; use interrupts so the read starts as soon as data is ready |
| `irq_timeouts` keeps growing / `--int-gpio` shows stutter | No edge reaches the GPIO: wrong BCM number or chip, INT not wired, polarity mismatch (try `--int-active-low` and check `gpiomon`), or `use_interrupt()` was not called |
| Interrupt line stays asserted | The FIFO is not being read, or the threshold is below the data you leave in the FIFO; read complete samples after every edge |
| `ModuleNotFoundError: gpiod`, or `AttributeError` on `gpiod.request_lines` | Install the v2 bindings with `pip install gpiod`; the older v1 API has different function names |
| Channels look like ramps or repeating register values | Not expected: the data sheet says `FIFO_ACCESS` advances through the FIFO on every word read. If it happens on your device, run with `--no-burst` (`fifo_burst=False`) and report it |
| All channels are near zero | LED current too low for your distance, board covered, or `init()` did not run |
| Channels saturate | Hand too close or LED current too high; lower `ILED1_COARSE` |
| Left/right or up/down inverted | Board orientation; remap in your application (see above) |
| `ModuleNotFoundError: smbus2` | `pip install smbus2` |

---

## Known limitations

- **No hardware validation yet.** Register values come from ADI's official
  firmware, but this exact Python port has only been exercised against a simulated device.
- **Burst FIFO reads** are consistent with the data sheet (`FIFO_ACCESS` advances
  through the FIFO), but have not been confirmed on hardware; `--no-burst` is the fallback.
- **Interrupts are implemented but untested on hardware.** The register
  behaviour follows the ADPD1080/ADPD1081 data sheet (Rev. C) and ADI's no-OS driver,
  which has no interrupt example to compare with. The edge polarity, the drive mode
  and the level translation on the board's INT pin should be verified with `gpiomon`
  or a scope. The GPIO code is exercised only with a fake `gpiod` module.
- **Small FIFO.** 128 bytes = 4 samples. A host that cannot service the FIFO within a few
  milliseconds loses samples silently (the chip drops data that does not fit).
- **No hardware presence interrupt.** The chip's interrupt signals data, not presence.
  A live switch of the sample rate for idle/active operation is not implemented.
- **No 32 kHz clock trim.** The sample rate is therefore only approximate.
- **Presence is software-based.** It is computed from the intensity of the
  samples, not from a hardware threshold in the ADPD1080.
- **Default thresholds are starting points.** They come from ADI's demo and
  depend on distance, LED current and ambient light.
- **Optical limits.** The sensor sees the reflected light centroid, not the shape of
  the hand. Very small circular motions, or a hand that fills the whole field
  of view, lose contrast.
- **Linux timing.** Python and the Linux scheduler are not real-time. At high
  sample rates, keep the main loop light, or run it in its own thread/process.

---

## Credits and licence

- Register map, initialisation sequence and clock calibration are ported from
  Analog Devices' [no-OS](https://github.com/analogdevicesinc/no-OS) repository
  (`drivers/photo-electronic/adpd188`, `projects/iio_adpd1080`), BSD-3-Clause.
- Interrupt behaviour (FIFO threshold, slot flags, GPIO configuration) follows the
  ADPD1080/ADPD1081 data sheet, Rev. C, and the EVAL-CN0569-PMDZ user guide.
- The gesture algorithm follows the `adpd2140_gesture_sensor` class from
  Analog Devices' `pyadi-iio` (ADIBSD), with the changes listed above.
- EVAL-CN0569-PMDZ, ADPD1080 and ADPD2140 are products of Analog Devices, Inc.
  This project is not affiliated with or endorsed by Analog Devices.

Licence for the code in this repository: **to be defined by the repository
owner**. Keep ADI's copyright and BSD-3-Clause notices for the parts derived
from their sources.
