#!/usr/bin/env python3
"""Demo: gestures + presence with the EVAL-CN0569-PMDZ on a Raspberry Pi CM4.

  python3 demo.py                     # bus 1, address 0x64, 512 Hz
  python3 demo.py --record data.csv   # also saves raw frames (for training)
  python3 demo.py --no-burst          # if the FIFO data looks inconsistent
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
    ap.add_argument("--raw", action="store_true", help="only print raw frames")
    ap.add_argument("--record", metavar="CSV")
    args = ap.parse_args()

    dev = ADPD1080(SMBus2Bus(args.bus, args.addr), fs_hz=args.fs,
                   fifo_burst=not args.no_burst)
    dev.init()
    dev.start()

    if args.raw:  # recommended first step: check that the 8 channels react
        try:
            while True:
                print(dev.next_frame())
        except KeyboardInterrupt:
            dev.stop()
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
        if fh:
            fh.close()


if __name__ == "__main__":
    main()
