"""Acceleration ramp (constant acceleration, trapezoidal or triangular profile).

Speeds are in steps/s. For constant acceleration a, the speed after s steps is
v^2 = v0^2 + 2*a*s, so one step changes v^2 by 2*a. The ramp is evaluated step by
step, which lets the motion thread react to a "decelerate now" request at any time.
"""
import math


class Ramp:
    def __init__(self, start_speed, max_speed, accel):
        if start_speed <= 0 or max_speed <= 0 or accel <= 0:
            raise ValueError("speeds and acceleration must be > 0")
        self.max = float(max_speed)
        self.start = float(min(start_speed, max_speed))
        self.accel = float(accel)
        self.v = self.start

    @property
    def at_start_speed(self):
        return self.v <= self.start + 1e-9

    def stop_steps(self):
        """Steps needed to slow down from the current speed to the start speed."""
        return (self.v * self.v - self.start * self.start) / (2.0 * self.accel)

    def next_interval(self, remaining=None, decel_now=False):
        """Delay (s) to wait after the step that was just made, then advance the speed.

        remaining: steps still to do after the step just made (None = unknown, never
                   plans a deceleration, e.g. while searching for a switch).
        decel_now: slow down immediately (controlled stop).
        """
        v = self.v
        interval = 1.0 / v
        if decel_now or (remaining is not None and remaining <= math.ceil(self.stop_steps())):
            self.v = math.sqrt(max(self.start * self.start, v * v - 2.0 * self.accel))
        elif v < self.max:
            self.v = min(self.max, math.sqrt(v * v + 2.0 * self.accel))
        return interval


def intervals(steps, start_speed, max_speed, accel):
    """Delays after each of `steps` steps for a complete move (pure helper for docs/tests)."""
    ramp = Ramp(start_speed, max_speed, accel)
    return [ramp.next_interval(steps - i - 1) for i in range(steps)]


def move_time(steps, start_speed, max_speed, accel):
    """Total duration in seconds of a move of `steps` steps."""
    return sum(intervals(steps, start_speed, max_speed, accel))
