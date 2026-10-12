import unittest

from stepper28byj48 import Stepper28BYJ48
from stepper28byj48.motor import HALF_STEP_SEQUENCE


class FakeGPIO:
    def __init__(self):
        self.events = []
        self.configured = None
        self.cleaned = None

    def setup(self, pins):
        self.configured = tuple(pins)

    def write(self, pin, value):
        self.events.append((pin, value))

    def cleanup(self, pins):
        self.cleaned = tuple(pins)


class StepperTests(unittest.TestCase):
    def setUp(self):
        self.gpio = FakeGPIO()
        self.motor = Stepper28BYJ48(
            pins=(17, 18, 27, 22),
            backend=self.gpio,
            sleep=lambda _: None,
        )

    def tearDown(self):
        self.motor.close()

    def test_initialises_configured_pins(self):
        self.assertEqual(self.gpio.configured, (17, 18, 27, 22))

    def test_one_step_writes_four_outputs(self):
        self.motor.step(1)
        self.assertEqual(len(self.gpio.events), 4)
        self.assertEqual(self.gpio.events, [
            (17, 1), (18, 1), (27, 0), (22, 0)
        ])
        self.assertEqual(self.motor.position, 1)

    def test_negative_steps_reverse_direction(self):
        self.motor.step(-1)
        self.assertEqual(self.gpio.events, [
            (17, 1), (18, 0), (27, 0), (22, 1)
        ])
        self.assertEqual(self.motor.position, -1)

    def test_full_sequence_cycles(self):
        self.motor.step(len(HALF_STEP_SEQUENCE))
        self.assertEqual(self.motor.position, 8)
        self.assertEqual(len(self.gpio.events), 8 * 4)

    def test_zero_steps_does_not_write(self):
        self.motor.step(0)
        self.assertEqual(self.gpio.events, [])

    def test_invalid_pins_rejected(self):
        with self.assertRaises(ValueError):
            Stepper28BYJ48(pins=(1, 2, 2, 4), backend=self.gpio, sleep=lambda _: None)

    def test_invalid_delay_rejected(self):
        with self.assertRaises(ValueError):
            Stepper28BYJ48(step_delay=0, backend=self.gpio, sleep=lambda _: None)

    def test_close_releases_and_cleans_up(self):
        self.motor.close()
        self.assertEqual(self.gpio.cleaned, (17, 18, 27, 22))
        self.assertTrue(all(value == 0 for _, value in self.gpio.events))
        with self.assertRaises(RuntimeError):
            self.motor.step(1)

    def test_close_is_idempotent(self):
        self.motor.close()
        event_count = len(self.gpio.events)
        self.motor.close()
        self.assertEqual(len(self.gpio.events), event_count)


if __name__ == "__main__":
    unittest.main()
