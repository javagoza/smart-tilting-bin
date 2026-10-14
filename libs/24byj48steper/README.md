# 28BYJ-48 Stepper Driver with Limit Switches for Raspberry Pi

A threaded Python driver for the **28BYJ-48** geared stepper motor (5 V, unipolar) on a
**ULN2003** driver board, with two limit switches (**home** and **end**), for the
**Raspberry Pi Compute Module 4** running Raspberry Pi OS.

It is built for one use case: **move between two working positions (start and end)
repeatedly, without having to touch the switches**. The switches are used to calibrate
(find zero and measure the travel), to re-reference the axis, and as a safety net that
stops the motor when something goes wrong. All motion runs in its own thread, so the
main application never blocks.

> **Status.** The logic is covered by 67 unit tests that run against a simulated carriage
> (virtual clock, no hardware needed) plus a few real-thread tests. It has **not yet been
> run on a physical motor**. Read [Known limitations](#known-limitations) and start with
> small speeds.

---

## Table of contents

1. [Features](#features)
2. [Hardware](#hardware)
3. [Installation](#installation)
4. [Quick start](#quick-start)
5. [Coordinate system](#coordinate-system)
6. [How it works](#how-it-works)
7. [Safety model](#safety-model)
8. [API reference](#api-reference)
9. [Usage patterns](#usage-patterns)
10. [Tuning](#tuning)
11. [Testing and the simulator](#testing-and-the-simulator)
12. [Troubleshooting](#troubleshooting)
13. [Known limitations](#known-limitations)
14. [Credits and licence](#credits-and-licence)

---

## Features

- **Non-blocking, thread-safe API.** Every motion call returns a `concurrent.futures.Future`
  immediately; one motion thread executes the queued commands in order.
- **Acceleration ramp.** Constant-acceleration trapezoidal profile (triangular for short
  moves), per-command speed and acceleration overrides.
- **Homing, ending and calibration.** `home()` references the HOME switch, `seek_end()`
  references the END switch, `calibrate()` does both and measures the usable travel.
- **Named work positions.** `go_start()` / `go_end()` move between positions you define
  (or teach), which stay clear of both switches.
- **Safety.** Unexpected switch hits, stuck or conflicting switches, search failures and
  GPIO errors stop the motor, de-energise the coils and put the driver in a FAULT state.
  `stop()` decelerates; `emergency_stop()` cuts the coils at once.
- **Debounced limit switches**, normally-open or normally-closed (fail-safe) wiring.
- **Persistent calibration.** Travel and work positions are saved to a JSON file, so a
  restart needs only `home()`, not a full calibration.
- **Events and diagnostics:** listeners for moves, homing and faults; timing statistics.
- **Pluggable hardware backend:** libgpiod v2 for the Pi, a simulator for tests and demos.
- Half-step (8 phases, ~4096 steps/rev) and full-step (4 phases, two coils on) modes.

---

## Hardware

### Motor and driver board

| Item | Value |
|------|-------|
| Motor | 28BYJ-48, 5 V unipolar, 5 wires (red = common +5 V), two centre-tapped coils |
| Wire colours | A blue, B pink, C yellow, D orange, E red |
| Step angle | Half step (8-phase sequence): 5.625°/64; full step (4-phase): 11.25°/64 |
| Steps per output revolution | ~4096 in half-step mode, ~2048 in full-step mode (nominal gear ratio 64:1) |
| Real gear ratio | 63.68395:1, i.e. ~4076 half steps (~2038 full steps) per revolution |
| Current | About 160–250 mA with two coils on (some sources quote up to 320 mA) |
| Speed | A retailer data sheet gives a no-load pull-in of 500 pulses/s and pull-out of 800 pulses/s |
| Driver board | ULN2003, inputs IN1–IN4, 5 V supply; its TTL/CMOS inputs are commonly driven directly from 3.3 V GPIO |

The driver counts **steps**, not degrees. Set `steps_per_rev=4076` (half) or `2038` (full)
if you want the exact angle conversion helpers (`steps_to_degrees`, `degrees_to_steps`).

> Use the **5 V** version of the motor with a 5 V supply. The 12 V version of the 28BYJ-48
> exists and must not be fed from 5 V (it will be weak) nor the 5 V version from 12 V.

### Wiring

| Signal | Raspberry Pi (BCM) | Header pin | Notes |
|--------|--------------------|------------|-------|
| ULN2003 IN1 | GPIO17 | 11 | |
| ULN2003 IN2 | GPIO27 | 13 | |
| ULN2003 IN3 | GPIO22 | 15 | |
| ULN2003 IN4 | GPIO23 | 16 | |
| HOME switch | GPIO5 | 29 | Other side of the switch to GND |
| END switch | GPIO6 | 31 | Other side of the switch to GND |
| GND | GND | any | **Common ground** with the ULN2003 board's power supply |
| ULN2003 `+` / `-` | External 5 V supply | | **Not** from the Pi's 5 V pin; at least 500 mA |

All pins are configurable (`StepperConfig.coil_pins`, `home_switch`, `end_switch`). The
defaults avoid the pins used by the other drivers in this family (I²C on GPIO2/3, SPI on
GPIO7–11, interrupt lines on GPIO24/25). Pin numbers are **BCM numbers**, not header pins.

**Power.** The motor draws a few hundred milliamps. Use an external 5 V supply of at
least 500 mA connected to the ULN2003 board, and join its ground to the Pi's ground.
Powering the motor from the Pi's 5 V rail is discouraged by several suppliers.

**Direction.** Positive steps must move the carriage toward the **END** switch. If your
motor turns the other way, set `invert_direction=True` (or swap the wiring). If you forget,
homing walks toward the wrong switch, the driver sees the END switch activate, stops and
raises a FAULT (`unexpected_limit`) instead of crashing into the mechanics.

### Limit switches

| Wiring | Config | Behaviour |
|--------|--------|-----------|
| Normally-open switch to GND, internal pull-up (default) | `SwitchConfig(pin)` (`active_low=True`) | Pressed = pin low |
| **Normally-closed** switch to GND, internal pull-up (**fail-safe**) | `SwitchConfig(pin, active_low=False)` | Pressed **or wire broken** = pin high = "triggered", so a cable fault stops the motor |

Use short or shielded wires: the motor coils are a source of noise. Debouncing takes
`debounce_samples` consecutive samples (one sample per motor step, default 3) to accept a
change. The internal pull-up of the Pi is weak; add an external 10 kΩ pull-up for long cables.

---

## Installation

```bash
sudo apt install -y python3-pip
pip install gpiod            # libgpiod Python bindings, API v2
```

Copy the `stepper28byj/` package directory (and `demo.py`) to the Pi. No root is needed;
your user must be able to open `/dev/gpiochip0` (members of the `gpio` group can on
Raspberry Pi OS). `sudo` is only needed for the optional real-time priority.

On a CM4 the GPIO chip is `/dev/gpiochip0`; `gpiodetect` lists the chips on other boards.

---

## Quick start

Try it first **without hardware**:

```bash
python3 demo.py --simulate calibrate
python3 demo.py --simulate --max-speed 2000 --accel 20000 cycle 2
```

On the real motor, start with the defaults (500 steps/s maximum):

```bash
python3 demo.py calibrate            # home, find the end switch, save the travel
python3 demo.py range 200 3800       # work start / work end, in steps
python3 demo.py start                # go to the work start (never touches the switches)
python3 demo.py end
python3 demo.py goto 1500
python3 demo.py jog -100             # relative move; works without homing
python3 demo.py cycle 3              # start <-> end, the main thread keeps printing
python3 demo.py status
```

`Ctrl-C` once decelerates and cancels; twice triggers an emergency stop.

The same from your own code:

```python
from stepper28byj import Stepper28BYJ48, StepperConfig

cfg = StepperConfig(state_file="stepper_state.json")      # defaults: IN1-4 = 17 27 22 23,
with Stepper28BYJ48(cfg) as motor:                        # home = 5, end = 6
    print(motor.calibrate().result().info)                # {'travel_steps': ...}
    motor.set_work_range(200, 3800)

    future = motor.go_end()                               # returns immediately
    while not future.done():
        do_other_work()                                   # the application is not frozen
    print(future.result())                                # MoveResult(status=COMPLETED, ...)
    motor.go_start().result().raise_for_status()
```

After a restart only `motor.home()` is needed: the travel and work positions are loaded
from the state file.

---

## Coordinate system

```
 HOME switch                                                        END switch
     |<-- clearance -->|                         |<-- clearance -->|
     |                 0 =========== work ======= travel_steps     |
     |                 |   start_steps   end_steps    |            |
 (hard stop)        position 0                      position travel   (hard stop)
```

- **Position 0** is `clearance_steps` away from the point where the HOME switch releases.
- **Positive steps** go toward the END switch.
- After `calibrate()`, the usable range is `[0, travel_steps]`, and both ends keep
  `clearance_steps` from the switches, so normal operation never presses a switch.
- `start_steps` defaults to 0 and `end_steps` to `travel_steps`; `set_work_range()` or
  `teach_start()` / `teach_end()` move them inward.
- The position is **only valid while `homed` is true**. It is lost on a fault or an
  emergency stop, and is never trusted after a restart.

---

## How it works

### Architecture

```
 application threads                    motion thread (one)
 -------------------                    -------------------
 move_to() / go_end() / home() ...  --> command queue --> execute one command at a time
        returns a Future                                   |
 stop() / emergency_stop() / status()                      v
        (any thread)                          step loop: coils, switches, ramp
                                                           |
                                                           v
                                              Backend (libgpiod or simulator)
```

- **One motion thread, one queue.** Commands are executed strictly in submission order.
  Futures resolve to a `MoveResult`; callbacks added with `add_done_callback` run in the
  motion thread.
- **Cancellation by epoch.** `stop()` (and `replace=True`) increments an epoch counter.
  The running command notices at its next step and decelerates; queued older commands are
  cancelled without moving. Commands submitted afterwards carry the new epoch and run.
- **Two locks.** A state lock protects position, flags and the queue bookkeeping; a
  hardware lock serialises every access to the GPIO. The emergency stop sets a halt flag,
  then takes the hardware lock and de-energises the coils; the motion thread checks the
  flag under the same lock right before every coil write, so no step can follow it.
- **Drift-free timing.** Each step is scheduled from the previous scheduled time, not from
  "now", so late wake-ups do not accumulate. If the thread stalls for more than one
  interval it resynchronises instead of firing a burst of steps (`stats.late_steps`).
- Queued commands can depend on each other: `calibrate()` followed by `go_end()` is accepted
  before the calibration finishes; preconditions are checked again when each command runs.

### Motion profile

Constant acceleration `a` from `start_speed` to `max_speed` (steps/s). One step changes
`v²` by `2a`, so the speed after `s` steps is `v² = v0² + 2·a·s`. The ramp is evaluated step by
step:

| Phase | Rule |
|-------|------|
| Accelerate | `v ← min(v_max, sqrt(v² + 2a))` |
| Cruise | `v = v_max` |
| Decelerate | when `remaining ≤ (v² − v_start²) / 2a`: `v ← max(v_start, sqrt(v² − 2a))` |
| Short moves | The deceleration starts before `v_max` is reached: a triangular profile |
| Controlled stop | Decelerates immediately, stops when the speed reaches `start_speed` |

The delay after each step is `1 / v`. `stepper28byj.intervals()` and `move_time()` expose
the same arithmetic for planning.

### Homing and calibration

`home()` (and the same procedure for the END switch in `seek_end()` / `calibrate()`):

1. If the switch is already active, move away from it until it releases.
2. Approach the switch at `homing_speed` until it is active (hard stop: the speed is low).
3. Back off at `latch_speed` until it releases: **the release edge is the repeatable reference.**
4. Move `clearance_steps` further away and set the position (0 at HOME, `travel_steps` at END).

`calibrate()` runs the HOME procedure, then the END procedure; the position where it ends,
after the clearance move, is `travel_steps`, and it is saved if `state_file` is set.

Every phase has a search limit (`max_travel_steps`, `max_release_steps`); exceeding it is a
`SEARCH_FAILED` fault. While looking for one switch, activation of the **other** one means the
direction is wrong: the motion stops with a `LIMIT_HIT` fault.

### Coil handling

The coils are de-energised when the queue becomes empty (`hold_when_idle=False`, the
default) to avoid heating the motor; the geared output keeps the carriage in place for
most light loads. Set `hold_when_idle=True` for loads that need holding torque.

---

## Safety model

| Situation | Reaction | Result status | State afterwards |
|-----------|----------|---------------|------------------|
| `stop()` | Decelerates, cancels the running and the queued commands | `CANCELLED` | IDLE, position still valid |
| `emergency_stop()` | Coils off immediately, everything cancelled | `EMERGENCY_STOP` | FAULT, homing lost |
| Switch hit during a normal move | Immediate stop, coils off | `LIMIT_HIT` | FAULT, homing lost |
| Move toward an already-active switch | Refused before moving | `LIMIT_HIT` | FAULT |
| Moving away from an active switch | Allowed | – | – |
| HOME and END both active (stuck switch, wiring error) | Stops, coils off | `FAULT` | FAULT |
| Switch signal not stable at the start of a move | Stops | `FAULT` | FAULT |
| Switch not found within `max_travel_steps` / does not release | Stops | `SEARCH_FAILED` | FAULT |
| GPIO error (exception from the backend) | Stops, coils off (best effort) | `FAULT` | FAULT |
| Target outside `[0, travel_steps]` | Rejected | `LimitError` (at submit) or `REFUSED` | unchanged |
| Absolute move before homing | Rejected | `NotHomedError` (at submit) or `REFUSED` | unchanged |

While in FAULT every new command raises `FaultError`. Recover with:

```python
motor.clear_fault()          # position is invalid: homing is required again
motor.home().result()
# only if you are certain that no steps were lost:
motor.clear_fault(keep_position=True)
```

`emergency_stop()` can be called from any thread, including a signal handler or a
watchdog. Software limits are **not** a replacement for mechanical hard stops and an
independent emergency-stop circuit where people or equipment are at risk.

---

## API reference

### `Stepper28BYJ48(config=None, backend=None, *, clock=time.monotonic, sleep=time.sleep, autostart=True)`

Usable as a context manager (`with` stops, waits and closes). `clock` and `sleep` can be
replaced for tests.

#### Motion (all return a `Future[MoveResult]` immediately)

| Method | Description |
|--------|-------------|
| `move_to(position, *, speed=None, accel=None, replace=False)` | Absolute move in steps. Needs a homed axis; checks `[0, travel_steps]` |
| `move_by(steps, ...)` | Relative to the position **when the command runs**. Needs a homed axis |
| `jog(steps, ...)` | Relative move without homing and without soft limits (the switches still protect) |
| `go_start(...)` / `go_end(...)` | Go to the work start / end position. They never touch the switches |
| `home(*, replace=False)` | Reference against HOME; position 0 |
| `seek_end(*, replace=False)` | Reference against END; position `travel_steps`. Needs calibration |
| `calibrate(*, replace=False)` | `home()` + measure the travel. `result.info["travel_steps"]` |

`speed` is clamped to `max_speed`. `replace=True` decelerates and cancels everything pending,
then runs this command.

#### Control

| Method | Description |
|--------|-------------|
| `stop()` | Controlled stop of the running command; cancels the queue |
| `emergency_stop(reason)` | Immediate stop, FAULT |
| `clear_fault(keep_position=False)` | Leave FAULT (waits for the motion thread to finish) |
| `wait_idle(timeout=None)` | Block until every submitted command has finished; `False` on timeout |
| `close()` | Stop the thread, de-energise the coils, release the GPIO |

#### Work positions and queries

| Item | Description |
|------|-------------|
| `set_work_range(start, end)` | Validate (`0 ≤ start < end ≤ travel`) and save |
| `teach_start()` / `teach_end()` | Store the current position (axis must be homed) |
| `position`, `homed`, `calibrated`, `travel_steps`, `start_steps`, `end_steps`, `state`, `fault` | Thread-safe properties |
| `status()` | Snapshot `Status` including the live switch states |
| `read_switches()` | Raw `(home_active, end_active)` |
| `steps_to_degrees(n)`, `degrees_to_steps(d)` | Conversions using `steps_per_rev` |
| `add_listener(cb)` / `remove_listener(cb)` | Events, called from the motion thread |
| `stats` | `steps`, `late_steps`, `max_late_s` |

### `MoveResult`

| Field | Meaning |
|-------|---------|
| `status` | `COMPLETED`, `CANCELLED`, `LIMIT_HIT`, `EMERGENCY_STOP`, `SEARCH_FAILED`, `REFUSED`, `FAULT` |
| `steps` | Signed steps moved by this command (0 for homing/calibration commands) |
| `position` | Position afterwards, or `None` if the axis is not homed |
| `message`, `info` | Details; `info["travel_steps"]` for `calibrate()` |
| `ok`, `raise_for_status()` | `status is COMPLETED`; raises `MoveFailed(result)` otherwise |

### States and events

`State`: `IDLE`, `MOVING`, `HOMING`, `CALIBRATING`, `FAULT`, `CLOSED`.
Events (`Event.kind`, `Event.data`): `move_started`, `move_done`, `homed`, `calibrated`,
`fault`, `fault_cleared`.

### Exceptions

`StepperError` is the base class: `NotHomedError`, `NotCalibratedError`, `LimitError`,
`FaultError`, `ClosedError`, `MoveFailed`. The first five are raised synchronously at submit time
when a precondition is already known to fail.

### `StepperConfig`

| Field | Default | Meaning |
|-------|---------|---------|
| `coil_pins` | `(17, 27, 22, 23)` | ULN2003 IN1..IN4 (BCM) |
| `home_switch`, `end_switch` | `SwitchConfig(5)`, `SwitchConfig(6)` | `SwitchConfig(pin, active_low=True, bias="pull_up", debounce_samples=3)` or `None` |
| `chip` | `/dev/gpiochip0` | GPIO chip |
| `step_mode` | `"half"` | `"half"` (8 phases) or `"full"` (4 phases, two coils on) |
| `invert_direction` | `False` | Positive must go toward END |
| `steps_per_rev` | `None` | 4096 (half) / 2048 (full) unless set |
| `max_speed` | `500` | Steps/s |
| `start_speed` | `100` | Speed at the start and end of every ramp |
| `acceleration` | `400` | Steps/s² |
| `homing_speed` | `300` | Approach speed to a switch |
| `latch_speed` | `100` | Release speed and clearance move |
| `clearance_steps` | `64` | Distance kept from each switch |
| `max_travel_steps` | `40000` | Search limit; set it a bit above your mechanical travel |
| `max_release_steps` | `2048` | Limit when backing off a switch |
| `hold_when_idle` | `False` | Keep the coils energised when idle |
| `state_file` | `None` | JSON file for travel and work positions |
| `realtime_priority` | `None` | `SCHED_FIFO` priority for the motion thread (needs root; failure is only logged) |

---

## Usage patterns

**Typical application** (calibrate once, then shuttle):

```python
motor = Stepper28BYJ48(StepperConfig(state_file="stepper_state.json"))
if not motor.calibrated:
    motor.calibrate().result().raise_for_status()
    motor.set_work_range(200, motor.travel_steps - 300)
else:
    motor.home().result().raise_for_status()      # after a restart: reference only

motor.go_start()                                   # queue moves, keep working
motor.go_end()
...
motor.wait_idle()
```

**Teach the work positions by moving the carriage to them:**

```python
motor.home().result()
motor.move_to(300).result()        # or move_by(+/-n) until the carriage is where you want it
motor.teach_start()                # stores the current position and saves it
motor.move_to(2500).result()
motor.teach_end()
```

**Redirect a move in flight:** `motor.move_to(1000, replace=True)` decelerates the current move
and then goes to 1000.

**Reacting to results without blocking:**

```python
motor.go_end().add_done_callback(lambda f: print("arrived:", f.result().status))
```

The callback runs in the motion thread: keep it short, and never call `.result()` on a
later command inside it (that would wait for the thread it runs on).

**asyncio:**

```python
result = await asyncio.wrap_future(motor.go_end())
```

**GUI toolkits:** listeners and callbacks run in the motion thread. Hand the event to the
GUI thread (for example with Tk `after`, or a Qt signal) instead of touching widgets directly.

**Fault handling in the application:**

```python
motor.add_listener(lambda e: log.error("motor fault: %s", e.data) if e.kind == "fault" else None)
...
if motor.state.name == "FAULT":
    motor.clear_fault()
    motor.home().result()
```

**Emergency stop on shutdown signals:**

```python
signal.signal(signal.SIGTERM, lambda *a: motor.emergency_stop("SIGTERM"))
```

---

## Tuning

| Goal | How |
|------|-----|
| Motor stalls or skips steps | Lower `max_speed` and `acceleration`; check the supply and the mechanical load |
| More torque | `step_mode="full"` (two coils on) at the cost of resolution |
| Smoother, quieter motion | `step_mode="half"` (default) |
| Less heating | Keep `hold_when_idle=False` |
| Safer homing | Lower `homing_speed` and `latch_speed` |
| Repeatable zero | The release edge at `latch_speed` is the reference; keep `latch_speed` low and the mechanics rigid |
| Early or false switch detection | Raise `debounce_samples`; use shielded wires, an external pull-up and short cables |
| Steps arrive late | Watch `stats.late_steps` and `stats.max_late_s`; lower the speed, reduce load on the Pi, or set `realtime_priority` |

The defaults (500 steps/s, 400 steps/s², 100 steps/s start) are conservative for an unloaded
motor, in line with the published no-load pull-in of 500 pulses/s. With ramps you can go
faster, but the maximum reliable speed depends on the supply, the load and the individual motor.
Treat any higher value as something to test.

**Timing.** Steps are paced from Python with `time.sleep`. This is adequate for the few hundred
to ~1000 steps/s this motor can follow, but the Linux scheduler does not guarantee it. A
late step is only a slightly irregular pulse; the ramp tolerates it, and the scheduler never
accumulates the error. For stricter timing, run the application on an otherwise idle CPU
core or use `realtime_priority`.

---

## Testing and the simulator

```bash
python3 -m unittest -v test_stepper28byj
```

The tests use `SimBackend`, a carriage on a rail: coil phases move the position, the HOME
switch is active at or below `home_trigger`, the END switch at or above `end_trigger`, and
mechanical hard stops lie beyond them (hitting one is counted in `collisions`). The
simulator can inject glitches (`glitches`), stuck switches (`stuck`) and GPIO failures
(`fail_after_writes`). `VirtualClock` replaces the clock and `sleep`, so 17000 steps run in
milliseconds with exact timing. A few tests use the real clock and real threads.

They cover: the ramp (shape, bounds, total time, controlled stop); configuration validation;
the libgpiod backend with a fake `gpiod`; homing from any start including on the switch,
repeatability, search failures, stuck and conflicting switches, calibration and `seek_end`;
exact moves and the profile seen at the coils; soft limits; named positions never touching
the switches; queuing, ordering, `replace`, cancelling pending futures; `stop()` with
deceleration; emergency stop; unexpected limits (lost steps); wrong direction wiring;
debouncing; GPIO errors; lifecycle; events; persistence; stall resynchronisation; and the
threading behaviour (calls return at once, stop and emergency stop from other threads,
polling `status()` while moving, many threads submitting).

The tests prove that the logic is self-consistent, not that your motor follows the ramp.

---

## Troubleshooting

| Problem | Likely cause / fix |
|---------|--------------------|
| `PermissionError` or `FileNotFoundError` for `/dev/gpiochip0` | Add your user to the `gpio` group; check `gpiodetect` for the chip name |
| `ModuleNotFoundError: gpiod` or `AttributeError` on `request_lines` | `pip install gpiod` (v2 API); the older v1 bindings differ |
| Motor buzzes but does not turn | Too fast or too little current: lower `max_speed`/`acceleration`, check the 5 V supply, check the coil wiring order IN1..IN4 against the motor connector |
| Motor turns the wrong way | `invert_direction=True` |
| `FAULT unexpected_limit` during `home()` | Direction wrong (see above) or the switches are swapped |
| `FAULT search_failed`: switch not found | Switch not wired, wrong pin or polarity (`active_low`), or `max_travel_steps` smaller than the travel |
| `FAULT search_failed`: does not release | Switch stuck, wrong polarity, or `max_release_steps` too small |
| `FAULT both_switches` | Both inputs read "triggered": wrong `active_low`, a short, or a disconnected NC chain |
| `FAULT switch_unstable` | Noisy switch signal: shielded wire, external pull-up, more `debounce_samples` |
| `FAULT backend_error` | GPIO error: check the log (`logging.getLogger("stepper28byj")`) |
| `NotHomedError` | Call `home()` or `calibrate()` first (or use `jog()`) |
| `LimitError` | The target is outside `[0, travel_steps]` |
| Position drifts between runs | Lost steps (open loop): reduce speed/acceleration or load, and re-home periodically |
| Motor is hot | Use `hold_when_idle=False`; a motor left energised is warm by design |
| `stats.late_steps` grows | The Pi is busy: lower the speed, free CPU, try `realtime_priority` |

---

## Known limitations

- **Not yet validated on a real motor.** Speeds, accelerations and the switch behaviour were
  only exercised in simulation.
- **Open loop.** There is no feedback: lost steps cannot be detected except by reaching a
  switch. The switches are the only position check; re-home periodically.
- **Homing is invalidated by any fault** (and by an emergency stop), by design.
- **Position is not persistent**, only the calibration is. Always call `home()` after a restart.
- **Python timing.** Steps are paced by a Python thread. It is good enough for this motor's
  speed range, but it is not hard real-time. A backend that uses hardware-timed pulses could
  be added behind the `Backend` interface.
- **Hard stop on a switch.** At a switch the motor stops immediately without a ramp; this is
  acceptable at `homing_speed`, which must stay low. A normal move that hits a switch stops
  the same way, from cruise speed.
- **`MoveResult.steps` is 0 for `home()`, `seek_end()` and `calibrate()`.** Use `info` and the
  position instead.
- **One motor per driver instance**, and each GPIO line can be requested by one process.
- **Safety.** The software limits and the fault logic do not replace mechanical stops or an
  external emergency-stop circuit.

---

## Credits and licence

- Motor and driver-board figures come from published data sheets and application notes for
  the 28BYJ-48 and the ULN2003; the real gear ratio (63.68395:1) is a community measurement.
- GPIO access uses [libgpiod](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/)
  through its Python bindings (v2).

Licence for the code in this repository: MIT Licence.
