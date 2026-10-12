# 28BYJ-48 Stepper Motor Control for Raspberry Pi CM4

A small Python library for controlling a common 5 V 28BYJ-48 stepper motor through a ULN2003 driver board. It includes unit tests that run without Raspberry Pi hardware and a hardware demo.

## Hardware

You need:

- Raspberry Pi Compute Module 4 (CM4) running Raspberry Pi OS
- 28BYJ-48 stepper motor (commonly 5 V)
- ULN2003 driver board compatible with the motor
- A suitable regulated 5 V supply for the motor, if required by your setup
- Jumper wires

### Wiring

The library uses **BCM GPIO numbering**. Connect the ULN2003 input pins as follows:

| Raspberry Pi CM4 GPIO (BCM) | ULN2003 input |
|---:|---|
| GPIO 17 | IN1 |
| GPIO 18 | IN2 |
| GPIO 27 | IN3 |
| GPIO 22 | IN4 |
| GND | GND (common ground) |

Connect the motor to the motor socket on the ULN2003 board. Power the ULN2003/motor according to the board and motor specifications. The GPIO pins are 3.3 V logic: do not apply 5 V to a CM4 GPIO. Ensure the motor supply ground and Raspberry Pi ground are common. Do not power the motor directly from a GPIO pin.

**Important:** Check the pin labels on your particular board. If the motor vibrates but does not rotate smoothly, the input order may differ; verify the wiring and adjust the `pins` order or sequence for your hardware.

## Installation

On Raspberry Pi OS, install the GPIO Zero and `lgpio` packages:

```bash
sudo apt update
sudo apt install python3-gpiozero python3-lgpio
```

Copy this project to the CM4, then from the project directory run the demo with the system Python:

```bash
python3 examples/demo.py
```

The GPIO backend uses GPIO Zero's `DigitalOutputDevice`. The operating system's GPIO stack and permissions must be configured for your Raspberry Pi OS release. The package imports GPIO Zero only when the hardware backend is created, so unit tests can run on a desktop without GPIO packages.

## Quick start

```python
from stepper28byj48 import Stepper28BYJ48

with Stepper28BYJ48(
    pins=(17, 18, 27, 22),  # BCM pins for IN1, IN2, IN3, IN4
    step_delay=0.004,       # seconds between half-steps
) as motor:
    motor.step(1024)        # forward
    motor.step(-1024)       # reverse
```

Positive step counts rotate in one direction; negative counts rotate in the other. `step()` counts **half-steps**, not full steps or degrees. The exact rotation angle depends on the motor and gearbox. The often-quoted 4096 half-steps per output-shaft revolution is only an approximation.

### API

- `Stepper28BYJ48(pins=(17, 18, 27, 22), step_delay=0.004, backend=None)`
  - `pins`: four distinct BCM GPIO numbers in ULN2003 input order IN1 to IN4.
  - `step_delay`: delay in seconds between half-steps. A smaller delay increases speed but can cause missed steps or stalling. Start at 0.003–0.005 seconds.
  - `backend`: optional GPIO backend; intended for testing or alternative GPIO implementations.
- `motor.step(steps, step_delay=None)`: move a signed number of half-steps.
- `motor.release()`: turn all four outputs off while retaining GPIO ownership.
- `motor.close()`: release the coils and clean up GPIO resources.
- Context manager support (`with ... as motor`) calls `close()` automatically.
- `motor.position`: net half-steps commanded since the object was created. This is not encoder feedback and does not measure the motor's actual physical position.

## Run unit tests

From the project root:

```bash
python3 -m unittest discover -s tests -v
```

The tests use a fake GPIO backend, so they do not energise a real motor and can run on a development computer.

## Demo behaviour

The demo moves forward by 1024 half-steps, pauses for one second, then reverses by 1024 half-steps. Change the step counts and delay to suit your application. Keep in mind that a 28BYJ-48 is a geared, relatively low-torque motor; acceleration control is not implemented, so very high speeds can cause missed steps.

## Project layout

```text
cm4_28byj48/
├── README.md
├── stepper28byj48/
│   ├── __init__.py
│   └── motor.py
├── tests/
│   └── test_motor.py
└── examples/
    └── demo.py
```

## Limitations

- This is open-loop control: there is no encoder or confirmation that a step physically occurred.
- Step timing uses Python sleep and is not hard real-time; timing accuracy depends on system load.
- No acceleration ramp, homing switch, or position persistence is provided.
- Always stop the motor and check wiring if the driver or motor becomes unusually hot.
