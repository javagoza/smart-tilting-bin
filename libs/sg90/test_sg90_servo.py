import unittest
from unittest.mock import MagicMock, patch
import sys

# Mock RPi.GPIO if running on a non-Raspberry Pi host machine
sys.modules['RPi.GPIO'] = MagicMock()

from sg90_servo import SG90Servo


class TestSG90Servo(unittest.TestCase):
    
    def setUp(self):
        self.pin = 18
        self.servo = SG90Servo(pin=self.pin)

    def test_initial_state(self):
        self.assertEqual(self.servo.pin, 18)
        self.assertIsNone(self.servo.get_angle())

    def test_valid_angles(self):
        # Test boundary angles
        self.servo.set_angle(0.0)
        self.assertEqual(self.servo.get_angle(), 0.0)

        self.servo.set_angle(90.0)
        self.assertEqual(self.servo.get_angle(), 90.0)

        self.servo.set_angle(180.0)
        self.assertEqual(self.servo.get_angle(), 180.0)

    def test_invalid_angles_raise_error(self):
        with self.assertRaises(ValueError):
            self.servo.set_angle(-1.0)

        with self.assertRaises(ValueError):
            self.servo.set_angle(181.0)

    def test_detach_and_cleanup(self):
        self.servo.detach()
        self.assertIsNone(self.servo.get_angle())


if __name__ == '__main__':
    unittest.main()
