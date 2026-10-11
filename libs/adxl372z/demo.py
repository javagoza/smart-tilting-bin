#!/usr/bin/env python3
"""Demo for the ADXL372 / EVAL-ADXL372Z on a Raspberry Pi CM4.

  python3 demo.py sample                       # single reads over SPI (CE0), 10 Hz
  python3 demo.py sample --iface i2c           # only for boards that expose I2C
  python3 demo.py stream --odr 3200 --record shock.csv
  python3 demo.py activity --act-mg 2000       # report activity / inactivity events
"""
import argparse
import csv
import time

import adxl372 as A


def build(args):
    if args.iface == "i2c":
        bus = A.I2CBus(args.bus, args.addr)
    else:
        bus = A.SPIBus(args.bus, args.cs, args.spi_hz)
    dev = A.ADXL372(bus, odr_hz=args.odr, bandwidth_hz=args.bw)
    dev.init()
    return dev


def run_sample(dev, args):
    dev.start()
    while True:
        x, y, z = dev.read_xyz_g()
        print("x=%+8.3f g  y=%+8.3f g  z=%+8.3f g" % (x, y, z))
        time.sleep(0.1)


def run_stream(dev, args):
    dev.configure_fifo(A.FifoMode.STREAMED, A.FifoFormat.XYZ, 300)
    dev.start()
    writer = fh = None
    if args.record:
        fh = open(args.record, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(["t", "x_g", "y_g", "z_g"])
    total, t0 = 0, time.monotonic()
    try:
        while True:
            sets = dev.read_fifo()
            if dev.fifo_overrun:
                print("WARNING: FIFO overrun - read faster or lower --odr")
            now = time.monotonic()
            for i, s in enumerate(sets):
                g = [c * A.G_PER_LSB for c in s]
                if writer:
                    # spread timestamps back from `now` using the ODR
                    writer.writerow([now - (len(sets) - 1 - i) / dev.odr_hz] + g)
            total += len(sets)
            if now - t0 >= 1.0:
                print("%d sets/s (expected ~%d); last: %s" % (
                    total / (now - t0), dev.odr_hz,
                    ["%+.2f" % (c * A.G_PER_LSB) for c in sets[-1]] if sets else "-"))
                total, t0 = 0, now
            time.sleep(0.02)
    finally:
        dev.stop()
        if fh:
            fh.close()


def run_activity(dev, args):
    dev.set_activity_threshold("activity", args.act_mg, referenced=True, enable=True)
    dev.set_activity_threshold("inactivity", args.inact_mg, referenced=True, enable=True)
    dev.set_activity_time_ms(args.act_ms)
    dev.set_inactivity_time_ms(args.inact_ms)
    dev.set_activity_processing(A.ActProcMode.LOOPED)  # same as ADI's Linux driver
    dev.start()
    print("Waiting for activity > %d mg ... (Ctrl-C to stop)" % args.act_mg)
    try:
        while True:
            st = dev.get_status()
            if st.act:
                print("ACTIVITY")
            if st.inact:
                print("INACTIVITY")
            time.sleep(0.01)
    finally:
        dev.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["sample", "stream", "activity"])
    ap.add_argument("--iface", choices=["i2c", "spi"], default="spi")
    ap.add_argument("--bus", type=int, default=None, help="spi: 0 (default); i2c: 1 (default)")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=A.I2C_ADDR_DEFAULT, help="I2C address (I2C boards only)")
    ap.add_argument("--cs", type=int, default=0, help="SPI chip select")
    ap.add_argument("--spi-hz", type=int, default=1000000)
    ap.add_argument("--odr", type=int, default=3200, choices=A.ODR_HZ)
    ap.add_argument("--bw", type=int, default=1600, choices=A.BANDWIDTH_HZ)
    ap.add_argument("--record", metavar="CSV")
    ap.add_argument("--act-mg", type=int, default=2000)
    ap.add_argument("--act-ms", type=int, default=1)
    ap.add_argument("--inact-mg", type=int, default=300)
    ap.add_argument("--inact-ms", type=int, default=1000)
    args = ap.parse_args()
    if args.bus is None:
        args.bus = 1 if args.iface == "i2c" else 0

    dev = build(args)
    try:
        {"sample": run_sample, "stream": run_stream, "activity": run_activity}[args.mode](dev, args)
    except KeyboardInterrupt:
        pass
    finally:
        dev.stop()
        dev.close()


if __name__ == "__main__":
    main()
