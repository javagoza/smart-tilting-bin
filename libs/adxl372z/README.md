# ADXL372 Python Driver for Raspberry Pi

A pure-Python driver for the Analog Devices **ADXL372** (±200 g, 3-axis, 12-bit
MEMS accelerometer) and the **EVAL-ADXL372Z** evaluation board, targeting
**Linux on a Raspberry Pi (design target: Compute Module 4)**. It talks to the
chip directly over **SPI** (what the EVAL-ADXL372Z exposes) or **I²C** (supported by
the chip, for other boards) — no kernel driver and no IIO server needed.

ADI already ships a mainline Linux IIO driver for this part, and `pyadi-iio` has
classes for other ADXL parts (ADXL313/345/355/380) but **not** for the ADXL372.
This repository fills that gap for users who prefer a self-contained userspace
driver (or who want direct access to the FIFO and the activity/inactivity engine).

> **Status.** The register map and configuration sequence are ported from ADI's
> no-OS driver, and the behaviour is cross-checked against ADI's Linux driver.
> The code is covered by unit tests using a simulated chip, but it has **not yet
> been validated against real hardware**. See [Known limitations](#known-limitations).

---

## Table of contents

1. [Features](#features)
2. [Hardware](#hardware)
3. [Installation](#installation)
4. [Quick start](#quick-start)
5. [Repository layout](#repository-layout)
6. [How it works](#how-it-works)
7. [Driver reference](#driver-reference)
8. [Register map](#register-map)
9. [Usage examples](#usage-examples)
10. [Choosing ODR, bandwidth and bus](#choosing-odr-bandwidth-and-bus)
11. [Testing](#testing)
12. [Troubleshooting](#troubleshooting)
13. [Known limitations](#known-limitations)
14. [Credits and licence](#credits-and-licence)

---

## Features

- I²C (`smbus2`) and SPI (`spidev`) transports behind one small interface; any
  object with `read(reg, n)` and `write(reg, value)` can be used (this is how the tests work).
- Device identification, software reset and full configuration: ODR, bandwidth,
  low-noise mode, operating mode.
- Single-sample reads with 12-bit sign extension, in raw counts or in g.
- 512-sample FIFO support: bypass / streamed / triggered / oldest-saved modes and
  all single-, dual- and three-axis formats, with chunked burst reads.
- Hardware activity / activity2 / inactivity detection: thresholds, timers
  (with the ODR-dependent scaling) and activity processing mode.
- INT1/INT2 event mapping and decoded status registers.
- Unit tests that run without hardware.

---

## Hardware

### The chip

| Item | Detail |
|------|--------|
| Device | ADXL372, 3-axis MEMS accelerometer, ±200 g |
| Resolution | 12 bit, ~97.7 mg/LSB (400 g / 4095) |
| Output data rates | 400, 800, 1600, 3200, 6400 Hz |
| Bandwidths | 200, 400, 800, 1600, 3200 Hz (never more than ODR/2) |
| FIFO | 512 samples (16-bit entries) |
| Interfaces | I²C and SPI |
| IDs | `DEVID = 0xAD`, `DEVID_MST = 0x1D`, `PARTID = 0xFA` |
| I²C address | ADI's Linux example uses **0x53** (only relevant for boards that expose I²C; the EVAL-ADXL372Z does not) |

### The EVAL-ADXL372Z board

Per the board's user guide (UG-1113) and schematic:

- The board is **20 mm × 20 mm** with no regulator, no level shifter and no
  connector fitted. It has **two sets of 0.1 in. vias for 5-pin headers** (P1 and P2)
  that you must populate yourself, and four mounting holes on a 15 mm × 15 mm pattern.
- **Only SPI is wired out.** P2 carries SCLK, MOSI, MISO and CS; there are no
  SDA/SCL pins. Use SPI with this board.
- **Two separate supplies:** `VS` (analog) and `VDDIO` (digital I/O). Both come out on P1.
- Bypass capacitors are fitted: 10 µF + 0.1 µF on VS and 0.1 µF on VDDIO.
- The four mounting holes (H1–H4) are tied to GND.

| Header | Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 |
|--------|-------|-------|-------|-------|-------|
| **P1** (power / interrupts) | VS | VDDIO | GND | INT2 | INT1 |
| **P2** (SPI) | GND | SCLK | MOSI | MISO | CS |

### Wiring to a Raspberry Pi / CM4 IO board (SPI0)

| EVAL-ADXL372Z | Raspberry Pi |
|---------------|--------------|
| P2-2 SCLK | GPIO11 (pin 23), SPI0 SCLK |
| P2-3 MOSI | GPIO10 (pin 19), SPI0 MOSI |
| P2-4 MISO | GPIO9 (pin 21), SPI0 MISO |
| P2-5 CS | GPIO8 (pin 24, CE0) or GPIO7 (pin 26, CE1) |
| P2-1 / P1-3 GND | GND |
| P1-1 VS | 3.3 V (see supply note) |
| P1-2 VDDIO | 3.3 V, so the logic levels match the Pi |
| P1-5 INT1 / P1-4 INT2 | Optional, any free GPIO (not used by this driver yet; ADI's Linux overlay example uses GPIO25) |

**Supply note.** Check the allowed ranges of `VS` and `VDDIO` in the ADXL372
data sheet before powering the board; the Pi's 3.3 V rail is the natural choice
for VDDIO. **The board has no reverse-polarity protection**: swapping VS/VDDIO and
GND can damage the ADXL372. Double-check the pinout before applying power.

**Handling note.** Dropping the board on a hard surface can generate several
thousand g, which may exceed the absolute maximum rating of the ADXL372.

### Do not run two drivers on the same chip

ADI's kernel overlay (`rpi-adxl372-overlay.dts`) binds the Linux IIO driver to
the device. If it is loaded, remove it before using this driver (or vice versa).
Both talking to the same chip at once will corrupt each other's configuration.

---

## Installation

### SPI (EVAL-ADXL372Z)

```
dtparam=spi=on
```

```bash
pip install spidev
ls /dev/spidev0.*       # /dev/spidev0.0 (CE0) and /dev/spidev0.1 (CE1)
```

The driver uses SPI mode 0 and 1 MHz by default (the value of ADI's overlay
example). Raise `speed_hz` only after checking the datasheet limit.

### I²C (not available on the EVAL-ADXL372Z)

Only needed for other boards that expose the chip's I²C interface.

Add to `/boot/firmware/config.txt` (or `/boot/config.txt` on older images) and reboot:

```
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

```bash
sudo apt install -y i2c-tools python3-pip
pip install smbus2
i2cdetect -y 1          # the device should appear (0x53 in ADI's Linux example)
```

---

## Quick start

```bash
python3 demo.py sample                        # x/y/z in g at 10 Hz over SPI (CE0)
python3 demo.py sample --cs 1                 # chip select CE1
python3 demo.py sample --iface i2c            # only for boards that expose I²C
python3 demo.py stream --odr 3200 --record shock.csv
python3 demo.py activity --act-mg 2000        # print ACTIVITY / INACTIVITY events
```

Example output of `sample` with the board lying flat (≈ +1 g on the axis pointing up; your numbers will differ):

```
x=  +0.010 g  y=  -0.020 g  z=  +0.980 g
```

`demo.py` options:

| Option | Default | Meaning |
|--------|---------|---------|
| `mode` | required | `sample`, `stream` or `activity` |
| `--iface` | `spi` | `spi` or `i2c` |
| `--bus` | `0` (SPI) / `1` (I²C) | Bus number |
| `--addr` | `0x53` | I²C address (I²C boards only) |
| `--cs` | `0` | SPI chip select |
| `--spi-hz` | `1000000` | SPI clock |
| `--odr` | `3200` | Output data rate in Hz |
| `--bw` | `1600` | Bandwidth in Hz |
| `--record CSV` | none | (`stream`) save timestamp + x/y/z in g |
| `--act-mg`, `--act-ms` | `2000`, `1` | Activity threshold and time |
| `--inact-mg`, `--inact-ms` | `300`, `1000` | Inactivity threshold and time |

---

## Repository layout

```
.
├── adxl372.py         # Buses (I2C, SPI) + ADXL372 driver
├── demo.py            # Command-line demo: sample / stream / activity
├── test_adxl372.py    # Unit tests (fake chip + SPI framing)
└── README.md
```

---

## How it works

### Sample format

Each axis is 12-bit two's complement, **left-justified in 16 bits**:

```
value = signed12((hi << 4) | (lo >> 4))
```

One LSB is 400 g / 4095 ≈ 0.0977 g (ADI's Linux driver reports 0.958241 m/s²,
which uses 9.81 m/s² for g). The driver exposes `G_PER_LSB` and
`STANDARD_GRAVITY = 9.80665`.

### Bus protocol

| | I²C | SPI (mode 0) |
|-|-----|--------------|
| Write | `[reg][value]` | `[reg << 1][value]` |
| Read | `[reg]` → repeated start → read *n* bytes | `[(reg << 1) \| 1][0 × n]` → *n* bytes after the first |
| Multi-byte read | Address auto-increments | Address auto-increments |
| `FIFO_DATA` (0x42) | **Does not** auto-increment: one long read returns successive FIFO entries | same |

### Operating modes (`POWER_CTL[1:0]`)

| Mode | Value | Meaning |
|------|-------|---------|
| `STANDBY` | 0 | Configuration state. FIFO, ODR and bandwidth are changed here |
| `WAKE_UP` | 1 | Low-power wake-up mode |
| `INSTANT_ON` | 2 | Instant-on mode |
| `FULL_BW` | 3 | Full-bandwidth measurement mode (normal use) |

Methods that need standby (`set_odr`, `set_bandwidth`, `configure_fifo`, …) switch
to standby, apply the change and **restore the previous mode** automatically.

### Initialisation (`init()`)

1. Read `DEVID` and `PARTID`; abort unless they are `0xAD` / `0xFA`.
2. Read `REVID`; on I²C with revision < 3, emit a warning (ADI notes an I²C chip
   bug fixed from revision 3 that can affect other devices on the bus).
3. Standby → write `RESET = 0x52` → wait 5 ms → standby.
4. Write ODR, bandwidth and low-noise settings.
5. FIFO in bypass mode; INT1/INT2 map cleared.

The chip is left in **standby**. Call `start()` to begin measuring.

### FIFO read-back rule

When several axes are stored, ADI's Linux driver always leaves **one sample set**
in the FIFO after every read so that data is never overwritten or stored out of
order. `read_fifo()` does the same by default (`leave_one_set=True`).

---

## Driver reference

### Buses

| Class | Constructor | Notes |
|-------|-------------|-------|
| `I2CBus` | `I2CBus(bus=1, addr=0x53)` | Uses `smbus2` |
| `SPIBus` | `SPIBus(bus=0, device=0, speed_hz=1000000)` | Uses `spidev`, mode 0 |

Both provide `read(reg, n) -> list[int]`, `write(reg, value)` and `close()`.

### `ADXL372(bus, odr_hz=3200, bandwidth_hz=1600, low_noise=False)`

| Method / property | Description |
|-------------------|-------------|
| `init()` | Identify, reset and configure (leaves standby mode) |
| `start(mode=OpMode.FULL_BW)` / `stop()` | Enter measurement / standby |
| `get_mode()` / `set_mode(mode)` | Read / write `POWER_CTL[1:0]` |
| `odr_hz`, `bandwidth_hz` | Current settings |
| `set_odr(hz)` | 400 … 6400. Lowers the bandwidth if it would exceed ODR/2 and re-applies activity timers |
| `set_bandwidth(hz)` | 200 … 3200; raises `ValueError` if above ODR/2 |
| `set_low_noise(enable)` | `MEASURE[3]` |
| `get_status()` | Returns `Status` (see below) |
| `read_xyz_raw(timeout=0.5)` | Wait for `DATA_RDY`, return `(x, y, z)` in signed counts |
| `read_xyz_g(timeout=0.5)` | Same, in g |
| `set_activity_threshold(kind, mg, referenced=True, enable=True)` | `kind`: `"activity"`, `"activity2"`, `"inactivity"`; 100 mg/code, 11-bit |
| `set_activity_time_ms(ms)` | 6.6 ms/code (3.3 ms at 6400 Hz), 8-bit |
| `set_inactivity_time_ms(ms)` | 26 ms/code (13 ms at 6400 Hz), 16-bit |
| `set_activity_processing(mode)` | `ActProcMode.DEFAULT / LINKED / LOOPED` |
| `set_interrupts(int1=(), int2=())` | Map events to pins: `data_rdy`, `fifo_rdy`, `fifo_full`, `fifo_ovr`, `inact`, `act`, `awake`, `low` |
| `configure_fifo(mode, fmt=XYZ, samples=0)` | `samples` ≤ 512 (watermark in 16-bit entries) |
| `fifo_axes` | Axis order of the current FIFO format, e.g. `"xyz"` |
| `read_fifo(leave_one_set=True, max_sets=None)` | Drain the FIFO → list of tuples in signed counts |
| `reg_read(reg)`, `reg_write(reg, value)`, `set_field(reg, mask, value)` | Raw register access for features not wrapped |
| `fifo_overrun` | Overrun flag from the last `get_status()` |

### `Status`

`data_rdy`, `fifo_rdy`, `fifo_full`, `fifo_overrun`, `awake`, `inact`, `act`,
`act2`, `fifo_entries` (number of 16-bit entries), plus the raw `raw1` / `raw2` bytes.

### Enums

| Enum | Values |
|------|--------|
| `OpMode` | `STANDBY`, `WAKE_UP`, `INSTANT_ON`, `FULL_BW` |
| `ActProcMode` | `DEFAULT`, `LINKED`, `LOOPED` |
| `FifoMode` | `BYPASSED`, `STREAMED`, `TRIGGERED`, `OLD_SAVED` |
| `FifoFormat` | `XYZ`, `X`, `Y`, `XY`, `Z`, `XZ`, `YZ` |

The peak-FIFO format of the chip is not exposed.

---

## Register map

Names and bit fields follow ADI's `adxl372.h`; see the ADXL372 datasheet for the
electrical meaning of each field. All registers are 8 bits wide.

| Addr | Name | Bits / fields | Used by this driver |
|------|------|---------------|---------------------|
| `0x00` | `DEVID` | `0xAD` | Identification |
| `0x01` | `DEVID_MST` | `0x1D` | Defined, not checked |
| `0x02` | `PARTID` | `0xFA` | Identification |
| `0x03` | `REVID` | Revision | I²C bug warning if < 3 |
| `0x04` | `STATUS_1` | b0 DATA_RDY, b1 FIFO_RDY, b2 FIFO_FULL, b3 FIFO_OVR, b5 USR_NVM_BUSY, b6 AWAKE, b7 ERR_USR_REGS | `get_status()` |
| `0x05` | `STATUS_2` | b4 INACT, b5 ACT, b6 ACT2 | `get_status()` |
| `0x06` | `FIFO_ENTRIES_2` | `[1:0]` entries [9:8] | `get_status()` |
| `0x07` | `FIFO_ENTRIES_1` | `[7:0]` entries [7:0] | `get_status()` |
| `0x08`–`0x0D` | `X/Y/Z_DATA_H/L` | 12-bit left-justified per axis | `read_xyz_*()` |
| `0x15`–`0x1A` | `X/Y/Z_MAXPEAK_H/L` | Peak acceleration | Not used |
| `0x20`–`0x22` | `OFFSET_X/Y/Z` | User offset trim | Not used |
| `0x23`/`0x24` | `X_THRESH_ACT_H/L` | H = code[10:3]; L = code[2:0]<<5 \| REF<<1 \| EN | `set_activity_threshold("activity")` (Y: `0x25/26`, Z: `0x27/28`) |
| `0x29` | `TIME_ACT` | Activity time | `set_activity_time_ms()` |
| `0x2A`/`0x2B` | `X_THRESH_INACT_H/L` | Same encoding | `set_activity_threshold("inactivity")` (Y: `0x2C/2D`, Z: `0x2E/2F`) |
| `0x30`/`0x31` | `TIME_INACT_H/L` | Inactivity time (16-bit) | `set_inactivity_time_ms()` |
| `0x32`/`0x33` | `X_THRESH_ACT2_H/L` | Same encoding | `set_activity_threshold("activity2")` (Y: `0x34/35`, Z: `0x36/37`) |
| `0x38` | `HPF` | High-pass filter corner | Not used |
| `0x39` | `FIFO_SAMPLES` | `[7:0]` watermark (bit 8 is in `FIFO_CTL[0]`) | `configure_fifo()` |
| `0x3A` | `FIFO_CTL` | `[5:3]` format, `[2:1]` mode, `[0]` samples bit 8 | `configure_fifo()` |
| `0x3B` | `INT1_MAP` | b0 DATA_RDY … b7 LOW (see `INT_BITS`) | `set_interrupts()` |
| `0x3C` | `INT2_MAP` | Same layout | `set_interrupts()` |
| `0x3D` | `TIMING` | `[7:5]` ODR, `[4:2]` wake-up rate, b1 EXT_CLK, b0 EXT_SYNC | ODR only |
| `0x3E` | `MEASURE` | b6 AUTOSLEEP, `[5:4]` LINKLOOP, b3 LOW_NOISE, `[2:0]` BANDWIDTH | Bandwidth, low-noise, linkloop |
| `0x3F` | `POWER_CTL` | b5 INSTANT_ON_TH, b4 FIL_SETTLE, b3 LPF_DIS, b2 HPF_DIS, `[1:0]` MODE | Mode |
| `0x40` | `SELF_TEST` | Self-test | Not used |
| `0x41` | `RESET` | Write `0x52` to reset | `init()` |
| `0x42` | `FIFO_DATA` | FIFO port, 16-bit entries, **no address increment** | `read_fifo()` |

Fields not wrapped by a method (wake-up rate, filter settling, HPF/LPF disable,
instant-on threshold, autosleep) can be set with `set_field()`; the masks are
available as constants in `adxl372.py`.

### Encodings

```
ODR index      : 0=400  1=800  2=1600  3=3200  4=6400 Hz
Bandwidth index: 0=200  1=400  2=800   3=1600  4=3200 Hz      (index <= ODR index)
Threshold code : mg // 100                                       (11 bit)
TIME_ACT       : round(ms * 1000 / 6600)  (6400 Hz: / 3300), clamped to 255
TIME_INACT     : round(ms / 26)           (6400 Hz: / 13),  clamped to 65535
FIFO_CTL       : (format << 3) | (mode << 1) | (samples > 255)
```

---

## Usage examples

### Read acceleration

```python
from adxl372 import ADXL372, SPIBus

dev = ADXL372(SPIBus(bus=0, device=0), odr_hz=3200, bandwidth_hz=1600)
dev.init()
dev.start()
x, y, z = dev.read_xyz_g()
print(x, y, z)
dev.stop()
```

On a board that exposes I²C, use `I2CBus(bus=1, addr=0x53)` instead.

### Stream with the FIFO

```python
import time
from adxl372 import ADXL372, SPIBus, FifoMode, FifoFormat, G_PER_LSB

dev = ADXL372(SPIBus(), odr_hz=3200, bandwidth_hz=1600)
dev.init()
dev.configure_fifo(FifoMode.STREAMED, FifoFormat.XYZ, 300)   # 100 sample sets
dev.start()
try:
    while True:
        for x, y, z in dev.read_fifo():
            print(x * G_PER_LSB, y * G_PER_LSB, z * G_PER_LSB)
        if dev.fifo_overrun:
            print("overrun: poll faster or reduce the ODR")
        time.sleep(0.02)
finally:
    dev.stop()
```

The FIFO holds 512 entries, i.e. ~170 three-axis sets. At 3200 Hz it fills in
about 53 ms, so poll at least every ~25 ms.

### Shock detection in software (FIFO magnitude)

```python
import math
THRESH_G = 20.0
for x, y, z in dev.read_fifo():
    g = math.sqrt(x * x + y * y + z * z) * G_PER_LSB
    if g > THRESH_G:
        print("impact: %.1f g" % g)
```

### Shock detection by the chip's activity engine

```python
from adxl372 import ActProcMode

dev.set_activity_threshold("activity", 2000, referenced=True, enable=True)      # 2 g
dev.set_activity_threshold("inactivity", 300, referenced=True, enable=True)     # 0.3 g
dev.set_activity_time_ms(1)
dev.set_inactivity_time_ms(1000)
dev.set_activity_processing(ActProcMode.LOOPED)       # as in ADI's Linux driver
dev.set_interrupts(int1=("act",), int2=("inact",))    # optional: route to the pins
dev.start()

while True:
    st = dev.get_status()
    if st.act:
        print("activity")
    if st.inact:
        print("inactivity")
```

With the INT pins wired to GPIOs you can wait for edges with `libgpiod` instead of polling.

### Offline analysis

`demo.py stream --record shock.csv` writes `t, x_g, y_g, z_g`. Timestamps within
one FIFO read are back-computed from the ODR; the sensor's own sample clock is
the reference for spacing.

---

## Choosing ODR, bandwidth and bus

| ODR | 3-axis payload | I²C @ 400 kHz | SPI @ 1 MHz |
|-----|----------------|---------------|-------------|
| 400 Hz | 2.4 kB/s | comfortable | comfortable |
| 800 Hz | 4.8 kB/s | comfortable | comfortable |
| 1600 Hz | 9.6 kB/s | fine | fine |
| 3200 Hz | 19.2 kB/s (~173 kbit/s on I²C) | fine, needs 400 kHz | fine |
| 6400 Hz | 38.4 kB/s (~346 kbit/s on I²C) | **close to the limit** | prefer SPI |

I²C figures count 9 bits per byte and ignore addressing overhead. At the default
100 kHz I²C speed only ODRs up to ~800 Hz are realistic. Use **SPI** for 6400 Hz
streaming; it is also the only interface the EVAL-ADXL372Z exposes. Python and the Linux scheduler are not real-time: keep the polling
loop light, and rely on the FIFO so that late reads do not lose data.

A bandwidth of ODR/2 gives the widest usable band. Lower bandwidths reduce noise
and aliasing at the cost of response speed; `set_low_noise(True)` trades range
headroom for noise (see the datasheet).

---

## Testing

The tests need no hardware:

```bash
python3 -m unittest -v test_adxl372
```

They cover:

- Identification, reset, default register values, wrong-ID and old-revision handling.
- ODR / bandwidth validation and the Nyquist coupling.
- Restoring the operating mode after configuration changes.
- Activity threshold and timer encodings, including re-scaling when the ODR changes.
- Interrupt map bits and status decoding.
- Sign extension of 12-bit samples, `DATA_RDY` timeout, unit conversion.
- FIFO configuration, drain with and without leaving one set behind, chunked reads
  of a large FIFO, single-axis formats and overrun reporting.
- SPI framing (`(reg << 1) | 1` for reads, `reg << 1` for writes).

These tests use a simulated chip: they show that the code is self-consistent, not
that it matches the physical device. The I²C transport is exercised only on real hardware.

---

## Troubleshooting

| Problem | Likely cause / fix |
|---------|--------------------|
| `RuntimeError: Unexpected ID` | Wrong interface or address, wiring, power, or a kernel driver holding the device |
| No response over SPI (ID read fails or returns 0x00/0xFF) | `dtparam=spi=on` missing, wrong chip select, MOSI/MISO swapped, CS not connected, or VS/VDDIO not powered |
| `i2cdetect` shows nothing | The EVAL-ADXL372Z has no I²C pins; this only applies to other boards |
| `TimeoutError: DATA_RDY never set` | Device still in standby: call `start()` |
| Values stuck at 0 or ±2047 | Measurement mode not active, saturation (shock above ±200 g), or wiring |
| FIFO overruns | Poll faster, lower the ODR, use SPI, or raise the I²C speed to 400 kHz |
| `ValueError: bandwidth must not exceed ODR/2` | Choose a bandwidth with an index ≤ the ODR index |
| Warning about REVID < 3 on I²C | Old silicon revision with an I²C bug; use SPI if other devices share the bus |
| `ModuleNotFoundError: smbus2 / spidev` | `pip install smbus2` or `pip install spidev` |
| `PermissionError` on `/dev/i2c-1` or `/dev/spidev*` | Add your user to the `i2c` / `spi` groups, or use `sudo` |

---

## Known limitations

- **No hardware validation yet.** Behaviour comes from ADI's drivers and the unit
  tests; the first real-hardware run should start with `demo.py sample`.
- **Board interface.** Per UG-1113, the EVAL-ADXL372Z exposes SPI only, so the
  I²C transport (and the 0x53 address taken from ADI's Linux example) is untested
  and relevant only to other boards. Supply voltage limits for `VS` / `VDDIO`
  are not covered here: check the ADXL372 data sheet.
- **SPI mode 0** is assumed (it is the default used by ADI's Linux driver, which
  does not request another mode).
- **Polling only.** INT1/INT2 can be mapped, but the driver does not wait on GPIO edges.
- **Status flag behaviour is not verified.** Whether `act` / `inact` latch until
  read, and how `fifo_rdy` / `fifo_full` relate to the watermark, should be checked
  in the datasheet. The driver therefore relies on `fifo_entries`, not on those flags.
- **Not wrapped:** peak registers and peak-FIFO format, offset trim, self-test,
  HPF corner, wake-up rate, instant-on threshold, filter settling, autosleep. Use
  `reg_read` / `reg_write` / `set_field` for these.
- **Scale factor** is nominal (400 g / 4095). For metrology, calibrate offsets and
  gain per axis.
- **Linux timing.** Python is not real-time; at high ODRs rely on the FIFO.

---

## Credits and licence

- Register map, enums and the configuration order are ported from Analog Devices'
  [no-OS](https://github.com/analogdevicesinc/no-OS) driver (`drivers/accel/adxl372`),
  BSD-3-Clause.
- The scale factor, activity/inactivity timer scaling, the Nyquist coupling between
  ODR and bandwidth, and the "leave one sample set in the FIFO" rule follow ADI's
  Linux IIO driver (`drivers/iio/accel/adxl372*.c`), licensed GPL-2.0+. Only
  behaviour and constants were reused, and the code here is an independent
  implementation.
- ADXL372 and EVAL-ADXL372Z are products of Analog Devices, Inc. This project is
  not affiliated with or endorsed by Analog Devices.

This library is distributed under the GNU General Public License v2.0 or later
