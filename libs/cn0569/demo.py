#!/usr/bin/env python3
"""Demo: gestures + presence with the EVAL-CN0569-PMDZ on a Raspberry Pi CM4.

  python3 demo.py                     # bus 1, address 0x64, 512 Hz
  python3 demo.py --record data.csv   # also saves raw frames (for training)
  python3 demo.py --no-burst          # if the FIFO data looks inconsistent
  python3 demo.py --int-gpio 25       # sleep on the Pmod INT line (GPIO0) wired to GPIO25
  python3 demo.py --int-gpio 25 --int-active-low
"""
import argparse
import csv
import time

from adpd1080 import ADPD1080, SMBus2Bus
from gesture import GestureEngine


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0x64)
    ap.add_argument("--fs", type=int, default=512)
    ap.add_argument("--l-thresh", type=int, default=1000)
    ap.add_argument("--d-thresh", type=float, default=0.07)
    ap.add_argument("--no-burst", action="store_true")
    ap.add_argument("--int-gpio", type=int, metavar="BCM",
                    help="BCM GPIO wired to the Pmod INT pin: wait for the FIFO interrupt "
                         "instead of polling")
    ap.add_argument("--int-active-low", action="store_true",
                    help="configure INT as active low (falling edge)")
    ap.add_argument("--int-open-drain", action="store_true",
                    help="drive INT only while asserted (needs a pull resistor on the line)")
    ap.add_argument("--int-packets", type=int, default=2, choices=[1, 2, 3],
                    help="samples in the FIFO that raise the interrupt (default 2)")
    ap.add_argument("--gpio-chip", default="/dev/gpiochip0")
    ap.add_argument("--raw", action="store_true", help="only print raw frames")
    ap.add_argument("--record", metavar="CSV")
    args = ap.parse_args()

    dev = ADPD1080(SMBus2Bus(args.bus, args.addr), fs_hz=args.fs,
                   fifo_burst=not args.no_burst)
    dev.init()

    irq = None
    if args.int_gpio is not None:
        from irq import InterruptLine

        irq = InterruptLine(args.int_gpio, args.gpio_chip, active_low=args.int_active_low)
        dev.use_interrupt(irq.wait, packets=args.int_packets, active_low=args.int_active_low,
                          open_drain=args.int_open_drain)
    dev.start()

    if args.raw:  # recommended first step: check that the 8 channels react
        try:
            while True:
                print(dev.next_frame())
        except KeyboardInterrupt:
            dev.stop()
            if irq:
                print("interrupt waits without an edge: %d" % dev.irq_timeouts)
                irq.close()
            return

    writer = fh = None
    if args.record:
        fh = open(args.record, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(["t"] + ["ch%d" % i for i in range(8)])

    def read_frame():
        f = dev.next_frame()
        if writer:
            writer.writerow([time.time()] + f)
        return f

    def on_event(e):
        if e.kind == "gesture":
            print("GESTURE sensor %d: %s" % (e.sensor, e.gesture.name))
        else:
            print(e.kind.upper())

    eng = GestureEngine(read_frame, on_event, l_thresh=args.l_thresh,
                        d_thresh=args.d_thresh)
    print("Calibrating (keep your hand away)...")
    eng.calibrate()
    print("Ready.")
    try:
        eng.run()
    except KeyboardInterrupt:
        pass
    finally:
        dev.stop()
        if irq:
            print("interrupt waits without an edge: %d" % dev.irq_timeouts)
            irq.close()
        if fh:
            fh.close()


if __name__ == "__main__":
    main()
