"""Small, testable 28BYJ-48 stepper motor driver.

Pins are Broadcom (BCM) GPIO numbers, ordered as ULN2003 IN1, IN2, IN3, IN4.
The default half-step sequence is suitable for the common 5 V 28BYJ-48 motor.
"""
from __future__ import annotations

import time
from typing import Callable, Protocol, Sequence


HALF_STEP_SEQUENCE: tuple[tuple[int, int, int, int], ...] = (
    (1, 0, 0, 0),
    (1, 1, 0, 0),
    (0, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 0),
    (0, 0, 1, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
)


class GPIOBackend(Protocol):
    """Minimal GPIO interface used by the motor class."""
    def setup(self, pins: Sequence[int]) -> None: ...
    def write(self, pin: int, value: int) -> None: ...
    def cleanup(self, pins: Sequence[int]) -> None: ...


class GpioZeroBackend:
    """GPIO Zero backend. On current Raspberry Pi OS, install gpiozero and lgpio."""
    def __init__(self) -> None:
        try:
            from gpiozero import DigitalOutputDevice
        except ImportError as exc:
            raise RuntimeError(
                "GPIO Zero is required on the Raspberry Pi. Install it with: "
                "sudo apt install python3-gpiozero python3-lgpio"
            ) from exc
        self._device_class = DigitalOutputDevice
        self._devices: dict[int, DigitalOutputDevice] = {}

    def setup(self, pins: Sequence[int]) -> None:
        for pin in pins:
            if pin not in self._devices:
                self._devices[pin] = self._device_class(pin, initial_value=False)

    def write(self, pin: int, value: int) -> None:
        device = self._devices[pin]
        device.on() if value else device.off()

    def cleanup(self, pins: Sequence[int]) -> None:
        for pin in pins:
            device = self._devices.pop(pin, None)
            if device is not None:
                device.off()
                device.close()


class Stepper28BYJ48:
    """Control a 28BYJ-48 motor via a ULN2003 driver.

    Args:
        pins: BCM GPIO pins connected to ULN2003 IN1..IN4, in that order.
        step_delay: Seconds between half-steps. Start around 0.003–0.005 s.
        backend: Optional GPIO backend, useful for tests or alternate hardware.
        sequence: Eight-state half-step sequence; normally leave unchanged.
        sleep: Injectable delay function, primarily for unit tests.
    """
    def __init__(
        self,
        pins: Sequence[int] = (17, 18, 27, 22),
        step_delay: float = 0.004,
        backend: GPIOBackend | None = None,
        sequence: Sequence[Sequence[int]] = HALF_STEP_SEQUENCE,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if len(pins) != 4 or len(set(pins)) != 4:
            raise ValueError("pins must contain four distinct BCM GPIO numbers")
        if any(not isinstance(pin, int) or pin < 0 for pin in pins):
            raise ValueError("GPIO pin numbers must be non-negative integers")
        if step_delay <= 0:
            raise ValueError("step_delay must be greater than zero")
        if not sequence or any(len(state) != 4 for state in sequence):
            raise ValueError("sequence must contain states with four values each")
        if any(value not in (0, 1) for state in sequence for value in state):
            raise ValueError("sequence values must be 0 or 1")

        self.pins = tuple(pins)
        self.step_delay = float(step_delay)
        self.sequence = tuple(tuple(state) for state in sequence)
        self._backend = backend if backend is not None else GpioZeroBackend()
        self._sleep = sleep
        self._position = 0
        self._closed = False
        self._backend.setup(self.pins)

    @property
    def position(self) -> int:
        """Net half-steps commanded since this instance was created."""
        return self._position

    def step(self, steps: int, step_delay: float | None = None) -> None:
        """Move a number of half-steps. Positive and negative values reverse direction."""
        if self._closed:
            raise RuntimeError("motor is closed")
        if not isinstance(steps, int):
            raise TypeError("steps must be an integer")
        delay = self.step_delay if step_delay is None else float(step_delay)
        if delay <= 0:
            raise ValueError("step_delay must be greater than zero")
        direction = 1 if steps >= 0 else -1
        count = abs(steps)
        state_count = len(self.sequence)
        index = self._position % state_count
        try:
            for _ in range(count):
                index = (index + direction) % state_count
                state = self.sequence[index]
                for pin, value in zip(self.pins, state):
                    self._backend.write(pin, value)
                self._sleep(delay)
                self._position += direction
        except KeyboardInterrupt:
            self.release()
            raise

    def release(self) -> None:
        """De-energise all coils without releasing the GPIO resources."""
        if not self._closed:
            for pin in self.pins:
                self._backend.write(pin, 0)

    def close(self) -> None:
        """De-energise the coils and release GPIO resources. Safe to call repeatedly."""
        if not self._closed:
            try:
                self.release()
            finally:
                self._backend.cleanup(self.pins)
                self._closed = True

    def __enter__(self) -> "Stepper28BYJ48":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()
