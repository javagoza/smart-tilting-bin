"""
SG90 Servo Motor Controller Library for Raspberry Pi CM4.
Uses RPi.GPIO (or rpi-lgpio compatibility package) to manage software PWM.
"""

import time

try:
    import RPi.GPIO as GPIO
except ImportError:
    # Fallback mock for testing environments where RPi.GPIO isn't installed
    GPIO = None


class SG90Servo:
    """Controls an SG90 9g servo motor via BCM GPIO PWM."""

    def __init__(self, pin: int, freq_hz: float = 50.0):
        self.pin = pin
        self.freq_hz = freq_hz
        self._pwm = None
        self._current_angle = None

        if GPIO is not None:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.OUT)
            self._pwm = GPIO.PWM(self.pin, self.freq_hz)
            self._pwm.start(0)  # Start with 0 duty cycle (signal off)

    def set_angle(self, angle: float) -> None:
        """Sets the servo motor to a specific angle between 0 and 180 degrees."""
        if not (0.0 <= angle <= 180.0):
            raise ValueError(f"Angle {angle} out of range. Must be between 0 and 180.")

        # Formula mapping 0-180 degrees to standard 2.5% - 12.5% duty cycle
        duty_cycle = (angle / 18.0) + 2.5

        if self._pwm is not None:
            self._pwm.ChangeDutyCycle(duty_cycle)
            # Give the servo small time to reach position
            time.sleep(0.05)
            # Optional: clear duty cycle to prevent jitter if needed, 
            # though keeping it active holds position.

        self._current_angle = angle

    def get_angle(self) -> float:
        """Returns the last commanded angle."""
        return self._current_angle

    def detach(self) -> None:
        """Stops PWM and cleans up the channel."""
        if self._pwm is not None:
            self._pwm.stop()
        self._current_angle = None

    def cleanup(self) -> None:
        """Global cleanup for GPIO."""
        self.detach()
        if GPIO is not None:
            GPIO.cleanup(self.pin)



