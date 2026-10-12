"""Hardware demo: rotate forward, pause, reverse, then release the coils."""
import time

from stepper28byj48 import Stepper28BYJ48


def main() -> None:
    # BCM numbering; connect these pins to ULN2003 IN1, IN2, IN3, IN4.
    # 4096 half-steps is an approximate revolution for many geared 28BYJ-48 motors.
    # Actual output angle varies by motor and gearbox.
    with Stepper28BYJ48(
        pins=(17, 18, 27, 22),
        step_delay=0.004,
    ) as motor:
        print("Moving forward...")
        motor.step(1024)
        time.sleep(1)
        print("Moving backward...")
        motor.step(-1024)
        print("Demo complete. Coils will be de-energised.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")
