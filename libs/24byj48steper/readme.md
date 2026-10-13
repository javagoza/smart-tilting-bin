# 28BYJ-48 Stepper Motor Driver for Raspberry Pi CM4

A Python library designed to drive **28BYJ-48 stepper motors** via **ULN2003 drivers** on the **Raspberry Pi Compute Module 4 (CM4)**. This library incorporates hardware abstraction via backends (featuring `gpiod`), safe acceleration/deceleration ramping, homing routines, and limit switch safety monitoring.

---

## Table of Contents

1. [Features](#features)
2. [Project Architecture & How It Works](#project-architecture--how-it-works)
3. [Installation & Requirements](#installation--requirements)
4. [Quickstart Guide](#quickstart-guide)
5. [Code Examples](#code-examples)
6. [Extending the Library](#extending-the-library)
7. [Assumptions & Areas to Review](#assumptions--areas-to-review)

---

## Features

- **Hardware Abstraction Layer (HAL):** Modular backend design allowing easy swapping or mocking of GPIO controllers (defaults to modern `libgpiod`).
- **Safe Movement & Ramping:** Integrated trapezoidal/s-curve velocity ramping (`Ramp`, interval calculation) to prevent motor stalling or skipping steps.
- **Homing & Calibration:** Built-in homing routines utilizing physical limit switches or home sensors.
- **Comprehensive Error Handling:** Fine-grained exception hierarchy (`ClosedError`, `LimitError`, `NotHomedError`, `FaultError`, etc.) for robust error recovery in embedded environments.
- **Full & Half-Step Sequences:** Supports both full-step and half-step commutation configurations.

---

## Project Architecture & How It Works

The library is organized into specialized modules:

1. **`backends`**: Manages low-level pin control. The default `GpiodBackend` uses Linux `libgpiod` for high reliability and compatibility with modern Raspberry Pi OS releases.
2. **`config`**: Defines dataclasses for motor settings (`StepperConfig`), limit switches (`SwitchConfig`), and standard commutation step tables (`FULL_STEP_SEQUENCE`, `HALF_STEP_SEQUENCE`).
3. **`ramp`**: Computes time intervals between step pulses to implement smooth acceleration and deceleration profiles.
4. **`driver`**: Core orchestrator (`Stepper28BYJ48`). Manages the state machine (`State`, `Status`), handles movement tracking, validates boundary conditions against limit switches, and raises domain-specific exceptions (`StepperError`).

### Control Flow
When a move command is issued:
1. The driver checks if the motor is initialized, homed (if required), and not faulted.
2. Step intervals are calculated via the `Ramp` engine.
3. The backend toggles the designated GPIO pins sequentially according to the selected step sequence.
4. Limit switch states are monitored to safeguard against mechanical over-travel.

---

## Installation & Requirements

### Hardware Requirements
- Raspberry Pi Compute Module 4 (CM4)
- 28BYJ-48 Stepper Motor (5V or 12V variant)
- ULN2003 Stepper Motor Driver Board
- Optional: Limit switches / optical endstops connected to GPIO pins.

### Software Dependencies
- Python 3.9+
- Python `gpiod` bindings (`python3-libgpiod`)

```bash
# Install system-level gpiod dependencies if necessary
sudo apt-get update && sudo apt-get install -y python3-libgpiod

# Install the package in editable mode from your local repository
pip install -e .
```

---

## Quickstart Guide

Here is a quick overview of how to initialize the driver and perform a basic relative move.

```python
from stepper28byj48 import Stepper28BYJ48, StepperConfig, SwitchConfig, GpiodBackend

# 1. Configure GPIO pins for the ULN2003 driver (IN1, IN2, IN3, IN4)
config = StepperConfig(
    pins=[17, 18, 27, 22],
    step_type="half",  # or "full"
    steps_per_revolution=4096  # Standard for 28BYJ-48 geared down ~1/64
)

# 2. (Optional) Configure a limit switch on GPIO pin 23
switch_config = SwitchConfig(pin=23, active_low=True)

# 3. Instantiate backend and driver
backend = GpiodBoardBackend() # Or your specific implementation
stepper = Stepper28BYJ48(config=config, switch=switch_config, backend=backend)

# 4. Use as a context manager for safe resource cleanup
with stepper:
    stepper.home()
    stepper.move_steps(1024, speed=500.0)
```

---

## Code Examples

### Example 1: Absolute Positioning with Exception Handling

```python
from stepper28byj48 import Stepper28BYJ48, StepperConfig, GpiodBackend
from stepper28byj48.driver import LimitError, NotHomedError

config = StepperConfig(pins=[17, 18, 27, 22])
stepper = Stepper28BYJ48(config=config)

try:
    stepper.open()
    stepper.home()
    
    # Move to absolute position 2048 steps from home
    stepper.move_to(2048)
    print(f"Current state: {stepper.status}")

except LimitError:
    print("Movement aborted: Limit switch triggered!")
except NotHomedError:
    print("Error: Attempted move before completing homing sequence.")
finally:
    stepper.close()
```

### Example 2: Custom Speed Ramping

```python
from stepper28byj48 import Stepper28BYJ48, StepperConfig, Ramp

config = StepperConfig(pins=[17, 18, 27, 22])
stepper = Stepper28BYJ48(config=config)

with stepper:
    # Custom ramp configuration for smoother acceleration
    ramp = Ramp(initial_interval=0.01, final_interval=0.002, acceleration=50.0)
    stepper.move_with_ramp(steps=4096, ramp=ramp)
```

---

## Extending the Library

### Adding a Custom Backend
To support a different GPIO library (e.g., `RPi.GPIO` or `lgpio`), inherit from the base `Backend` abstract class:

```python
from stepper28byj48.backends import Backend

class MyCustomBackend(Backend):
    def setup_pins(self, pins):
        # Initialize pins here
        pass

    def write_sequence(self, step_values):
        # Apply logic to set pin states
        pass

    def cleanup(self):
        # Release hardware resources
        pass
```

---

## Assumptions & Areas to Review

When evaluating, auditing, or extending this library, please take note of the following architectural assumptions and items requiring review:

1. **Gear Ratio & Step Count:** 
   - *Assumption:* The library assumes the standard 28BYJ-48 internal gear reduction ratio (~1/64, yielding roughly 4096 steps per revolution in half-step mode). 
   - *Review:* If using non-standard gearboxes or direct drives, verify that `steps_per_revolution` is explicitly overridden in `StepperConfig`.
2. **Threading & Concurrency:**
   - *Assumption:* Step generation loops may block the calling thread unless managed asynchronously. 
   - *Review:* Check whether step timing loops utilize non-blocking yields or dedicated background worker threads, especially when running concurrent tasks on the CM4.
3. **GPIO Chip Selection (`libgpiod`):**
   - *Assumption:* `GpiodBackend` assumes the default GPIO chip chip0 (`/dev/gpiochip0`).
   - *Review:* On newer Raspberry Pi OS versions utilizing kernel updates or custom HATs, verify if the chip index or line naming conventions need explicit parameterization.
4. **Limit Switch Debouncing:**
   - *Assumption:* Software or hardware debouncing for limit switches is either handled externally or integrated into the backend implementation.
   - *Review:* Inspect switch polling logic to ensure high-speed homing routines do not register false triggers from electrical noise.
