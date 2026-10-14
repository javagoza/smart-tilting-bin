"""Configuration for the 28BYJ-48 + ULN2003 stepper driver.

All GPIO numbers are BCM numbers (libgpiod line offsets).
"""
from dataclasses import dataclass
from typing import Optional, Tuple

# ULN2003 board inputs IN1..IN4 (motor wires: blue, pink, yellow, orange).
# Half step: A-AB-B-BC-C-CD-D-DA, 8 phases -> ~4096 steps per output revolution.
HALF_STEP_SEQUENCE = (
    (1, 0, 0, 0),
    (1, 1, 0, 0),
    (0, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 0),
    (0, 0, 1, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
)
# Full step with two coils on (higher torque), 4 phases -> ~2048 steps per revolution.
FULL_STEP_SEQUENCE = (
    (1, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 1),
    (1, 0, 0, 1),
)
# Nominal values. The real gear ratio is 63.68395:1, i.e. ~4075.77 half steps per revolution.
NOMINAL_STEPS_PER_REV = {"half": 4096, "full": 2048}

BIAS_CHOICES = ("pull_up", "pull_down", "disabled")


@dataclass(frozen=True)
class SwitchConfig:
    """A limit switch connected to a GPIO.

    active_low=True : the switch pulls the pin to GND when triggered (normally-open switch
                      to GND with the internal pull-up).
    active_low=False: the pin reads high when triggered. With a normally-CLOSED switch to
                      GND and a pull-up this is the fail-safe wiring: a broken wire reads
                      as "triggered" and stops the motor.
    debounce_samples: consecutive samples (one per motor step) needed to accept a change.
    """

    pin: int
    active_low: bool = True
    bias: str = "pull_up"
    debounce_samples: int = 3

    def validate(self, name):
        if self.bias not in BIAS_CHOICES:
            raise ValueError("%s: bias must be one of %s" % (name, BIAS_CHOICES))
        if self.debounce_samples < 1:
            raise ValueError("%s: debounce_samples must be >= 1" % name)


@dataclass
class StepperConfig:
    # --- wiring ---
    coil_pins: Tuple[int, int, int, int] = (17, 27, 22, 23)      # ULN2003 IN1..IN4
    home_switch: Optional[SwitchConfig] = SwitchConfig(5)
    end_switch: Optional[SwitchConfig] = SwitchConfig(6)
    chip: str = "/dev/gpiochip0"
    # --- motor ---
    step_mode: str = "half"                 # "half" (8 phases) or "full" (4 phases)
    invert_direction: bool = False          # flip so that positive steps move toward the END switch
    steps_per_rev: Optional[int] = None     # for angle conversions; default 4096 (half) / 2048 (full)
    # --- motion (steps per second, steps per second squared) ---
    max_speed: float = 500.0
    start_speed: float = 100.0
    acceleration: float = 400.0
    # --- homing / calibration ---
    homing_speed: float = 300.0             # approach speed towards a switch
    latch_speed: float = 100.0              # slow speed used to release the switch and for clearance
    clearance_steps: int = 64               # distance kept from each switch release edge
    max_travel_steps: int = 40000           # search limit when looking for a switch
    max_release_steps: int = 2048           # search limit when backing off a switch
    debounce_gap_s: float = 0.001           # spacing of the samples taken before a move
    # --- behaviour ---
    hold_when_idle: bool = False            # keep the coils energised when idle (more heat)
    state_file: Optional[str] = None        # JSON file for travel / work positions
    realtime_priority: Optional[int] = None  # SCHED_FIFO priority for the motion thread (needs root)

    def sequence(self):
        return HALF_STEP_SEQUENCE if self.step_mode == "half" else FULL_STEP_SEQUENCE

    @property
    def effective_steps_per_rev(self):
        return self.steps_per_rev or NOMINAL_STEPS_PER_REV[self.step_mode]

    def validate(self):
        if self.step_mode not in NOMINAL_STEPS_PER_REV:
            raise ValueError("step_mode must be 'half' or 'full'")
        if len(self.coil_pins) != 4:
            raise ValueError("coil_pins must contain 4 GPIO numbers (IN1..IN4)")
        pins = list(self.coil_pins)
        for name, sw in (("home_switch", self.home_switch), ("end_switch", self.end_switch)):
            if sw is not None:
                sw.validate(name)
                pins.append(sw.pin)
        if len(set(pins)) != len(pins):
            raise ValueError("GPIO numbers must be unique across coils and switches")
        if any((not isinstance(p, int)) or p < 0 for p in pins):
            raise ValueError("GPIO numbers must be non-negative integers")
        for name in ("max_speed", "start_speed", "acceleration", "homing_speed", "latch_speed"):
            if getattr(self, name) <= 0:
                raise ValueError("%s must be > 0" % name)
        if self.start_speed > self.max_speed:
            raise ValueError("start_speed must not exceed max_speed")
        if self.clearance_steps < 0:
            raise ValueError("clearance_steps must be >= 0")
        if self.max_travel_steps <= 0 or self.max_release_steps <= 0:
            raise ValueError("max_travel_steps and max_release_steps must be > 0")
        if self.debounce_gap_s < 0:
            raise ValueError("debounce_gap_s must be >= 0")
        if self.steps_per_rev is not None and self.steps_per_rev <= 0:
            raise ValueError("steps_per_rev must be > 0")
