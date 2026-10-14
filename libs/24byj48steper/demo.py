#!/usr/bin/env python3
"""Command-line demo for the 28BYJ-48 + ULN2003 driver with two limit switches.

Real hardware (BCM pin numbers, defaults: IN1..IN4 = 17 27 22 23, home = 5, end = 6):

  python3 demo.py calibrate          # home, find the end switch, save travel
  python3 demo.py home
  python3 demo.py range 200 3800     # work start / end positions (steps)
  python3 demo.py start              # go to the work start (never touches the switches)
  python3 demo.py end
  python3 demo.py goto 1500
  python3 demo.py jog -100           # relative move, no homing needed
  python3 demo.py cycle 3            # start <-> end, with the main thread still running
  python3 demo.py status

Without hardware (a simulated carriage on a rail, real-time speeds):

  python3 demo.py --simulate calibrate
  python3 demo.py --simulate --max-speed 2000 --accel 20000 cycle 2

Ctrl-C once = controlled stop (decelerates); Ctrl-C twice = emergency stop.
"""
import argparse
import logging
import signal
import sys
import time

from stepper28byj import (FaultError, NotCalibratedError, NotHomedError, LimitError,
                          Stepper28BYJ48, StepperConfig, SwitchConfig)


def build(args):
    sw = dict(active_low=not args.normally_closed, bias="pull_up", debounce_samples=args.debounce)
    cfg = StepperConfig(
        coil_pins=tuple(args.coils),
        home_switch=SwitchConfig(args.home_pin, **sw),
        end_switch=SwitchConfig(args.end_pin, **sw),
        chip=args.chip,
        step_mode=args.mode,
        invert_direction=args.invert,
        max_speed=args.max_speed,
        start_speed=min(args.start_speed, args.max_speed),
        acceleration=args.accel,
        homing_speed=min(args.homing_speed, args.max_speed),
        clearance_steps=args.clearance,
        max_travel_steps=args.max_travel,
        hold_when_idle=args.hold,
        state_file=args.state_file,
    )
    backend = None
    if args.simulate:
        from stepper28byj.sim import SimBackend
        backend = SimBackend(cfg, time.monotonic, position=1500, home_trigger=0,
                             end_trigger=args.sim_length)
    return Stepper28BYJ48(cfg, backend)


def wait(m, future, label):
    """Wait for a command while the 'application' keeps running (prints a heartbeat)."""
    t0 = time.monotonic()
    while not future.done():
        s = m.status()
        pos = "%6d" % s.position if s.homed else "     ?"      # meaningless until homed
        print("\r%-12s state=%-11s position=%s   %4.1fs" % (
            label, s.state.value, pos, time.monotonic() - t0), end="", flush=True)
        time.sleep(0.2)
    print("\r" + " " * 60 + "\r", end="")
    r = future.result()
    print("%s: %s (steps %+d, position %s) %s" % (label, r.status.value, r.steps, r.position,
                                                  r.message))
    return r


def ensure_homed(m):
    if not m.homed:
        print("axis not homed: homing first")
        r = wait(m, m.home(), "home")
        if not r.ok:
            sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("command", choices=["home", "calibrate", "seek-end", "goto", "start", "end",
                                        "jog", "range", "cycle", "status"])
    ap.add_argument("values", nargs="*", type=int, help="goto POS | jog STEPS | range START END | cycle N")
    ap.add_argument("--simulate", action="store_true", help="use a simulated carriage")
    ap.add_argument("--sim-length", type=int, default=5000, help="simulated rail length in steps")
    ap.add_argument("--coils", type=int, nargs=4, default=[17, 27, 22, 23], metavar="BCM")
    ap.add_argument("--home-pin", type=int, default=5)
    ap.add_argument("--end-pin", type=int, default=6)
    ap.add_argument("--normally-closed", action="store_true",
                    help="NC switches to GND (fail-safe: a broken wire reads as triggered)")
    ap.add_argument("--debounce", type=int, default=3)
    ap.add_argument("--chip", default="/dev/gpiochip0")
    ap.add_argument("--mode", choices=["half", "full"], default="half")
    ap.add_argument("--invert", action="store_true", help="invert the motor direction")
    ap.add_argument("--max-speed", type=float, default=500.0, help="steps/s")
    ap.add_argument("--start-speed", type=float, default=100.0, help="steps/s")
    ap.add_argument("--accel", type=float, default=400.0, help="steps/s^2")
    ap.add_argument("--homing-speed", type=float, default=300.0, help="steps/s")
    ap.add_argument("--clearance", type=int, default=64, help="steps kept away from each switch")
    ap.add_argument("--max-travel", type=int, default=40000, help="search limit in steps")
    ap.add_argument("--hold", action="store_true", help="keep the coils energised when idle")
    ap.add_argument("--state-file", default="stepper_state.json")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")

    m = build(args)
    presses = {"n": 0}

    def on_sigint(signum, frame):
        presses["n"] += 1
        if presses["n"] == 1:
            print("\nstopping (Ctrl-C again for an emergency stop)...")
            m.stop()
        else:
            m.emergency_stop("Ctrl-C")

    signal.signal(signal.SIGINT, on_sigint)
    m.add_listener(lambda e: logging.getLogger("demo").info("event %s %s", e.kind, e.data))

    code = 0
    try:
        cmd, v = args.command, args.values
        if cmd == "status":
            s = m.status()
            print("state=%s position=%d homed=%s calibrated=%s travel=%s start=%s end=%s "
                  "home_switch=%s end_switch=%s" % (s.state.value, s.position, s.homed,
                                                    s.calibrated, s.travel_steps, s.start_steps,
                                                    s.end_steps, s.home_switch, s.end_switch))
        elif cmd == "home":
            code = 0 if wait(m, m.home(), "home").ok else 1
        elif cmd == "calibrate":
            r = wait(m, m.calibrate(), "calibrate")
            code = 0 if r.ok else 1
            if r.ok:
                print("travel: %d steps = %.1f degrees of the output shaft" % (
                    r.info["travel_steps"], m.steps_to_degrees(r.info["travel_steps"])))
        elif cmd == "seek-end":
            code = 0 if wait(m, m.seek_end(), "seek-end").ok else 1
        elif cmd == "range":
            m.set_work_range(v[0], v[1])
            print("work range saved: start=%d end=%d" % (v[0], v[1]))
        elif cmd == "jog":
            code = 0 if wait(m, m.jog(v[0]), "jog").ok else 1
        else:
            ensure_homed(m)
            if cmd == "goto":
                code = 0 if wait(m, m.move_to(v[0]), "goto").ok else 1
            elif cmd == "start":
                code = 0 if wait(m, m.go_start(), "start").ok else 1
            elif cmd == "end":
                code = 0 if wait(m, m.go_end(), "end").ok else 1
            elif cmd == "cycle":
                for i in range(v[0] if v else 1):
                    print("cycle %d" % (i + 1))
                    if not wait(m, m.go_end(), "-> end").ok or not wait(m, m.go_start(), "-> start").ok:
                        code = 1
                        break
    except (NotHomedError, NotCalibratedError, LimitError, FaultError, IndexError) as exc:
        print("error: %s" % exc)
        code = 2
    finally:
        m.stop()
        m.wait_idle(10)
        m.close()
    sys.exit(code)


if __name__ == "__main__":
    main()
