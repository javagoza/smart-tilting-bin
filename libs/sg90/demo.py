#!/usr/bin/env python3
"""
Demo script for SG90 Servo Motor on Raspberry Pi CM4.
Sweeps the servo smoothly from 0 to 180 degrees and back.
"""

import time
from sg90_servo import SG90Servo

# Choose your BCM GPIO pin (e.g., GPIO 18)
SERVO_PIN = 18

def main():
    print(f"Initializing SG90 Servo on BCM Pin {SERVO_PIN}...")
    servo = SG90Servo(pin=SERVO_PIN)

    try:
        print("Moving to 0 degrees...")
        servo.set_angle(0)
        time.sleep(1)

        print("Moving to 90 degrees (Center)...")
        servo.set_angle(90)
        time.sleep(1)

        print("Moving to 180 degrees...")
        servo.set_angle(180)
        time.sleep(1)

        print("Running sweep demo (0 -> 180 -> 0)...")
        for angle in range(0, 181, 10):
            print(f"Angle: {angle}°")
            servo.set_angle(angle)
            time.sleep(0.1)

        for angle in range(180, -1, -10):
            print(f"Angle: {angle}°")
            servo.set_angle(angle)
            time.sleep(0.1)

    KeyboardInterrupt:
        print("\nDemo interrupted by user.")

    finally:
        print("Cleaning up resources...")
        servo.cleanup()
        print("Done.")


if __name__ == "__main__":
    main()
  
