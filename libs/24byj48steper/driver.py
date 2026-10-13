"""Thread-based driver for a 28BYJ-48 stepper (ULN2003 board) with two limit switches.

Design
------
* One motion thread executes commands one after another from a queue. All public
  methods are thread-safe and return immediately with a concurrent.futures.Future
  that resolves to a MoveResult, so the main application never blocks.
* Coordinates are in steps. 0 is `clearance_steps` away from the HOME switch release
  edge, positive steps go toward the END switch. After calibrate() the usable range is
  [0, travel_steps]; both ends keep `clearance_steps` away from the switches.
* The switches are used to calibrate (home, find the end) and as a safety net: if a
  switch is hit during a normal move, the motor stops at once and the driver enters the
  FAULT state until clear_fault(). Normal moves between named positions never need to
  reach a switch.
* stop() decelerates and cancels everything; emergency_stop() de-energises the coils
  immediately and forces a new homing.
"""
import enum
import json
import logging
import os
import queue
import tempfile
import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass, field
from typing import Optional

from .backends import GpiodBackend
from .config import StepperConfig
from .ramp import Ramp

log = logging.getLogger("stepper28byj")

STATE_VERSION = 1


# ---------------------------------------------------------------- public types
class State(enum.Enum):
    IDLE = "idle"
    MOVING = "moving"
    HOMING = "homing"
    CALIBRATING = "calibrating"
    FAULT = "fault"
    CLOSED = "closed"


class MoveStatus(enum.Enum):
    COMPLETED = "completed"
    CANCELLED = "cancelled"            # stop() or replace=True: decelerated, no error
    LIMIT_HIT = "limit_hit"            # a switch was hit unexpectedly -> FAULT
    EMERGENCY_STOP = "emergency_stop"
    SEARCH_FAILED = "search_failed"    # switch not found / does not release -> FAULT
    REFUSED = "refused"                # preconditions not met, nothing moved
    FAULT = "fault"                    # internal or hardware error -> FAULT


@dataclass
class MoveResult:
    status: MoveStatus
    steps: int = 0                     # signed steps actually moved by this command
    position: Optional[int] = None     # position after the command (None if not homed)
    message: str = ""
    info: dict = field(default_factory=dict)

    @property
    def ok(self):
        return self.status is MoveStatus.COMPLETED

    def raise_for_status(self):
        if not self.ok:
            raise MoveFailed(self)
        return self


@dataclass(frozen=True)
class FaultInfo:
    reason: str
    detail: str
    time: float


@dataclass(frozen=True)
class Status:
    state: State
    busy: bool
    position: int
    homed: bool
    calibrated: bool
    travel_steps: Optional[int]
    start_steps: Optional[int]
    end_steps: Optional[int]
    fault: Optional[FaultInfo]
    home_switch: bool
    end_switch: bool


@dataclass
class Event:
    kind: str                          # move_started, move_done, homed, calibrated, fault, fault_cleared
    data: dict
    time: float


@dataclass
class Stats:
    steps: int = 0
    late_steps: int = 0                # times the thread fell more than one interval behind
    max_late_s: float = 0.0


class StepperError(Exception):
    pass


class NotHomedError(StepperError):
    pass


class NotCalibratedError(StepperError):
    pass


class LimitError(StepperError):
    pass


class FaultError(StepperError):
    pass


class ClosedError(StepperError):
    pass


class MoveFailed(StepperError):
    def __init__(self, result):
        super().__init__("%s: %s" % (result.status.value, result.message))
        self.result = result


# ------------------------------------------------------------------- internals
class _End(enum.Enum):
    DONE = 1        # requested number of steps made
    FOUND = 2       # `until` condition met
    CANCELLED = 3   # stop() / replace
    LIMIT = 4       # forbidden switch active
    HALT = 5        # emergency stop or fault from another thread
    FAULT = 6       # fault raised inside the step loop


class _Cmd:
    __slots__ = ("kind", "args", "future", "epoch")

    def __init__(self, kind, args, epoch):
        self.kind = kind
        self.args = args
        self.future = Future()
        self.epoch = epoch


class _Debounce:
    def __init__(self, n, state=False):
        self.n = n
        self.state = state
        self.count = 0

    def update(self, raw):
        if raw == self.state:
            self.count = 0
        else:
            self.count += 1
            if self.count >= self.n:
                self.state = raw
                self.count = 0
        return self.state


class _Unstable(Exception):
    pass


REF_KINDS = ("home", "seek_end", "calibrate")


class Stepper28BYJ48:
    def __init__(self, config=None, backend=None, *, clock=time.monotonic, sleep=time.sleep,
                 autostart=True):
        self.cfg = config or StepperConfig()
        self.cfg.validate()
        self._backend = backend if backend is not None else GpiodBackend(self.cfg)
        self._clock = clock
        self._sleep = sleep
        self._seq = self.cfg.sequence()
        self._phase = 0
        self._n_home = self.cfg.home_switch.debounce_samples if self.cfg.home_switch else 1
        self._n_end = self.cfg.end_switch.debounce_samples if self.cfg.end_switch else 1

        self._queue = queue.Queue()
        self._lock = threading.RLock()          # state, counters, queue bookkeeping
        self._hw_lock = threading.RLock()       # every backend access
        self._idle_cv = threading.Condition(self._lock)
        self._halt = threading.Event()          # emergency stop / fault: stop stepping now
        self._epoch = 0                         # bumped by stop(): older commands are cancelled
        self._pending = 0
        self._pending_ref = 0
        self._pending_cal = 0

        self._position = 0
        self._homed = False
        self._homed_before_fault = False
        self._travel = None
        self._start_steps = None
        self._end_steps = None
        self._state = State.IDLE
        self._fault = None
        self._closed = False
        self._listeners = []
        self.stats = Stats()

        self._load_state()
        self._thread = threading.Thread(target=self._worker_main, name="stepper28byj", daemon=True)
        self._started = False
        if autostart:
            self.start()

    # ------------------------------------------------------------ lifecycle
    def start(self):
        if not self._started:
            self._started = True
            self._thread.start()
        return self

    def close(self, timeout=5.0):
        """Stop the motion thread, de-energise the coils and release the GPIO."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._epoch += 1
            self._state = State.CLOSED
        self._queue.put(None)
        if self._started:
            self._thread.join(timeout)
        with self._hw_lock:
            try:
                self._backend.release()
            finally:
                self._backend.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        try:
            self.stop()
            self.wait_idle(timeout=5.0)
        finally:
            self.close()

    # --------------------------------------------------------------- queries
    @property
    def position(self):
        with self._lock:
            return self._position

    @property
    def homed(self):
        with self._lock:
            return self._homed

    @property
    def calibrated(self):
        with self._lock:
            return self._travel is not None

    @property
    def travel_steps(self):
        with self._lock:
            return self._travel

    @property
    def state(self):
        with self._lock:
            return self._state

    @property
    def fault(self):
        with self._lock:
            return self._fault

    @property
    def start_steps(self):
        with self._lock:
            return self._start_steps if self._start_steps is not None else 0

    @property
    def end_steps(self):
        with self._lock:
            return self._end_steps if self._end_steps is not None else self._travel

    def read_switches(self):
        """Raw (home_active, end_active), polarity applied, without debouncing."""
        return self._read_switches_hw()

    def status(self):
        home, end = self._read_switches_hw()
        with self._lock:
            return Status(
                state=self._state, busy=self._pending > 0, position=self._position,
                homed=self._homed, calibrated=self._travel is not None,
                travel_steps=self._travel, start_steps=self._start_steps,
                end_steps=self._end_steps, fault=self._fault, home_switch=home, end_switch=end,
            )

    def steps_to_degrees(self, steps):
        return steps * 360.0 / self.cfg.effective_steps_per_rev

    def degrees_to_steps(self, degrees):
        return int(round(degrees * self.cfg.effective_steps_per_rev / 360.0))

    def add_listener(self, callback):
        """callback(Event) is called from the motion thread (keep it short)."""
        with self._lock:
            self._listeners.append(callback)

    def remove_listener(self, callback):
        with self._lock:
            if callback in self._listeners:
                self._listeners.remove(callback)

    def wait_idle(self, timeout=None):
        """Block until every submitted command has finished. Returns False on timeout."""
        with self._idle_cv:
            return self._idle_cv.wait_for(lambda: self._pending == 0, timeout)

    # ------------------------------------------------------- motion commands
    def move_to(self, position, *, speed=None, accel=None, replace=False):
        """Absolute move in steps. Requires a homed axis. Returns a Future[MoveResult]."""
        position = int(round(position))
        with self._lock:
            self._check_accepting()
            self._require_homed_for_submit()
            if self._homed and self._pending_ref == 0:
                self._check_soft_limits(position)
        return self._submit("move_to", dict(position=position, speed=speed, accel=accel), replace)

    def move_by(self, steps, *, speed=None, accel=None, replace=False):
        """Relative move (position at execution time + steps). Requires a homed axis."""
        with self._lock:
            self._check_accepting()
            self._require_homed_for_submit()
        return self._submit("move_by", dict(steps=int(round(steps)), speed=speed, accel=accel),
                            replace)

    def jog(self, steps, *, speed=None, accel=None, replace=False):
        """Relative move that does not need homing (no soft limits; the switches still protect)."""
        with self._lock:
            self._check_accepting()
        return self._submit("jog", dict(steps=int(round(steps)), speed=speed, accel=accel), replace)

    def go_start(self, *, speed=None, accel=None, replace=False):
        """Go to the work start position (default 0). It does not touch the HOME switch."""
        with self._lock:
            self._check_accepting()
            self._require_homed_for_submit()
        return self._submit("go", dict(name="start", speed=speed, accel=accel), replace)

    def go_end(self, *, speed=None, accel=None, replace=False):
        """Go to the work end position (default travel_steps). It does not touch the END switch."""
        with self._lock:
            self._check_accepting()
            self._require_homed_for_submit()
            self._require_calibrated_for_submit()
        return self._submit("go", dict(name="end", speed=speed, accel=accel), replace)

    def home(self, *, replace=False):
        """Reference against the HOME switch and set position 0 (clearance_steps away from it)."""
        with self._lock:
            self._check_accepting()
        if self.cfg.home_switch is None:
            raise StepperError("no home switch configured")
        return self._submit("home", {}, replace)

    def seek_end(self, *, replace=False):
        """Reference against the END switch: position becomes travel_steps. Needs calibration."""
        with self._lock:
            self._check_accepting()
            self._require_calibrated_for_submit()
        if self.cfg.end_switch is None:
            raise StepperError("no end switch configured")
        return self._submit("seek_end", {}, replace)

    def calibrate(self, *, replace=False):
        """Home, then find the END switch to measure travel_steps. The result carries
        info['travel_steps']. The axis ends at position travel_steps."""
        with self._lock:
            self._check_accepting()
        if self.cfg.home_switch is None or self.cfg.end_switch is None:
            raise StepperError("calibrate() needs both switches")
        return self._submit("calibrate", {}, replace)

    # ---------------------------------------------------------------- control
    def stop(self):
        """Decelerate to a halt and cancel the running and all queued commands."""
        with self._lock:
            self._epoch += 1

    def emergency_stop(self, reason="emergency stop"):
        """Immediately de-energise the coils, cancel everything and enter FAULT.
        Safe to call from any thread (and from a signal handler). Homing is lost."""
        self._halt.set()
        with self._lock:
            self._epoch += 1
            self._set_fault_locked("emergency_stop", reason)
        with self._hw_lock:
            try:
                self._backend.release()
            except Exception:
                log.exception("failed to release the coils")
        self._emit("fault", reason="emergency_stop", detail=reason)

    def clear_fault(self, keep_position=False):
        """Leave the FAULT state. The position is invalid and homing is required again
        unless keep_position=True (only if you are sure no steps were lost)."""
        if not self.wait_idle(timeout=5.0):
            raise StepperError("the motion thread is still busy")
        with self._lock:
            if self._state is not State.FAULT:
                return
            self._halt.clear()
            self._fault = None
            self._state = State.IDLE
            self._homed = bool(keep_position and self._homed_before_fault)
        self._emit("fault_cleared")

    # --------------------------------------------------------- work positions
    def set_work_range(self, start, end):
        """Define the named positions used by go_start() / go_end() (in steps)."""
        start, end = int(start), int(end)
        with self._lock:
            hi = self._travel
            if not 0 <= start < end or (hi is not None and end > hi):
                raise LimitError("work range must satisfy 0 <= start < end <= travel_steps")
            self._start_steps, self._end_steps = start, end
        self._save_state()

    def teach_start(self):
        """Store the current position as the work start (axis must be homed)."""
        with self._lock:
            if not self._homed:
                raise NotHomedError("home the axis first")
            self._start_steps = self._position
        self._save_state()
        return self._start_steps

    def teach_end(self):
        """Store the current position as the work end (axis must be homed)."""
        with self._lock:
            if not self._homed:
                raise NotHomedError("home the axis first")
            self._end_steps = self._position
        self._save_state()
        return self._end_steps

    # ------------------------------------------------------ submit / validation
    def _check_accepting(self):
        if self._closed:
            raise ClosedError("driver is closed")
        if self._state is State.FAULT:
            raise FaultError("driver in FAULT (%s): call clear_fault() and home again"
                             % (self._fault.reason if self._fault else "?"))

    def _require_homed_for_submit(self):
        if not self._homed and self._pending_ref == 0:
            raise NotHomedError("axis not homed: call home() or calibrate() first")

    def _require_calibrated_for_submit(self):
        if self._travel is None and self._pending_cal == 0:
            raise NotCalibratedError("travel unknown: call calibrate() first")

    def _check_soft_limits(self, target):
        hi = self._travel
        if target < 0 or (hi is not None and target > hi):
            raise LimitError("target %d outside [0, %s]" % (target, hi))

    def _submit(self, kind, args, replace):
        with self._lock:
            self._check_accepting()
            if replace:
                self._epoch += 1
            cmd = _Cmd(kind, args, self._epoch)
            self._pending += 1
            if kind in REF_KINDS:
                self._pending_ref += 1
            if kind == "calibrate":
                self._pending_cal += 1
            self._queue.put(cmd)
        return cmd.future

    # ------------------------------------------------------------- hardware
    def _read_switches_hw(self):
        with self._hw_lock:
            return self._backend.read_switches()

    def _do_step(self, direction):
        d = -direction if self.cfg.invert_direction else direction
        self._phase = (self._phase + d) % len(self._seq)
        with self._hw_lock:
            if self._halt.is_set():
                return False
            self._backend.set_coils(self._seq[self._phase])
        return True

    def _stable_switches(self):
        """Read until n identical consecutive samples agree; returns (home, end)."""
        n = max(self._n_home, self._n_end, 2)
        last, agree, tries = None, 0, 0
        while True:
            s = self._read_switches_hw()
            if s == last:
                agree += 1
            else:
                last, agree = s, 1
            if agree >= n:
                return s
            tries += 1
            if tries > 8 * n:
                raise _Unstable("limit switch signal is not stable")
            self._sleep(self.cfg.debounce_gap_s)

    def _release_coils(self):
        with self._hw_lock:
            try:
                self._backend.release()
            except Exception:
                log.exception("failed to release the coils")

    # ---------------------------------------------------------- state / events
    def _set_state(self, state):
        with self._lock:
            if self._state is not State.FAULT and self._state is not State.CLOSED:
                self._state = state

    def _set_fault_locked(self, reason, detail):
        if self._state is State.FAULT:
            return False
        self._homed_before_fault = self._homed
        self._homed = False
        self._fault = FaultInfo(reason, detail, time.time())
        self._state = State.FAULT
        return True

    def _enter_fault(self, reason, detail):
        self._halt.set()
        with self._lock:
            self._epoch += 1
            first = self._set_fault_locked(reason, detail)
        self._release_coils()
        if first:
            log.error("FAULT %s: %s", reason, detail)
            self._emit("fault", reason=reason, detail=detail)

    def _emit(self, name, **data):
        with self._lock:
            listeners = list(self._listeners)
        if not listeners:
            return
        ev = Event(name, data, time.time())
        for cb in listeners:
            try:
                cb(ev)
            except Exception:
                log.exception("listener raised")

    # ----------------------------------------------------------- worker thread
    def _apply_realtime(self):
        prio = self.cfg.realtime_priority
        if prio is None:
            return
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(prio))
            log.info("motion thread running with SCHED_FIFO %d", prio)
        except (AttributeError, PermissionError, OSError) as exc:
            log.warning("could not set real-time priority: %s", exc)

    def _worker_main(self):
        self._apply_realtime()
        while True:
            cmd = self._queue.get()
            if cmd is None:
                break
            try:
                if not cmd.future.set_running_or_notify_cancel():
                    result = None
                else:
                    try:
                        result = self._execute(cmd)
                    except _Unstable as exc:
                        self._enter_fault("switch_unstable", str(exc))
                        result = MoveResult(MoveStatus.FAULT, message=str(exc))
                    except Exception as exc:  # hardware or internal error: stop safely
                        log.exception("motion thread error")
                        self._enter_fault("backend_error", "%s: %s" % (type(exc).__name__, exc))
                        result = MoveResult(MoveStatus.FAULT, message=str(exc))
                if result is not None:
                    result.position = self._position_or_none()
                    cmd.future.set_result(result)
                    self._emit("move_done", kind=cmd.kind, status=result.status.value,
                               steps=result.steps, position=result.position)
            except Exception:
                log.exception("unexpected error in the motion thread loop")
                if not cmd.future.done():
                    cmd.future.set_exception(StepperError("internal error, see the log"))
            finally:
                # release the coils BEFORE reporting idle, so wait_idle() guarantees it
                if self._queue.empty() and not self.cfg.hold_when_idle and not self._closed:
                    self._release_coils()
                with self._lock:
                    self._pending -= 1
                    if cmd.kind in REF_KINDS:
                        self._pending_ref -= 1
                    if cmd.kind == "calibrate":
                        self._pending_cal -= 1
                    if self._state not in (State.FAULT, State.CLOSED):
                        self._state = State.IDLE
                    self._idle_cv.notify_all()

    def _position_or_none(self):
        with self._lock:
            return self._position if self._homed else None

    def _execute(self, cmd):
        with self._lock:
            closed, fault, epoch = self._closed, self._fault, self._epoch
        if closed or cmd.epoch != epoch:
            return MoveResult(MoveStatus.CANCELLED, message="cancelled before it started")
        if self._halt.is_set():
            status = (MoveStatus.EMERGENCY_STOP if fault and fault.reason == "emergency_stop"
                      else MoveStatus.FAULT)
            return MoveResult(status, message="driver in FAULT")
        state = {"home": State.HOMING, "seek_end": State.HOMING,
                 "calibrate": State.CALIBRATING}.get(cmd.kind, State.MOVING)
        self._set_state(state)
        self._emit("move_started", kind=cmd.kind)
        if cmd.kind in ("move_to", "move_by", "jog", "go"):
            return self._run_move(cmd)
        if cmd.kind == "home":
            return self._run_reference(cmd, "home", measure=False)
        if cmd.kind == "seek_end":
            return self._run_reference(cmd, "end", measure=False)
        return self._run_calibrate(cmd)

    # ---------------------------------------------------------------- stepping
    def _step_loop(self, cmd, direction, max_steps, *, speed, accel=None, until=None,
                   forbid="auto", plan_decel=True, cancel_mode="decel"):
        """Run steps in `direction` (+1 toward END, -1 toward HOME).

        until(home, end): stop at once (hard stop) when it returns True -> _End.FOUND.
        forbid: switch ("home"/"end") that must not become active; "auto" = the one in the
                travel direction. A forbidden switch that is already active is ignored while
                moving away from it, and re-armed once it releases.
        Returns (_End, steps_done, detail).
        """
        cfg = self.cfg
        ramp = Ramp(cfg.start_speed, speed, accel or cfg.acceleration)
        if forbid == "auto":
            forbid = "end" if direction > 0 else "home"
        h, e = self._stable_switches()
        if h and e:
            self._enter_fault("both_switches", "home and end switches are both active")
            return _End.FAULT, 0, "both_switches"
        deb = (_Debounce(self._n_home, h), _Debounce(self._n_end, e))
        fidx, armed = None, False
        if forbid is not None:
            fidx = 0 if forbid == "home" else 1
            initially = (h, e)[fidx]
            toward = -1 if fidx == 0 else 1
            if initially and direction == toward:
                return _End.LIMIT, 0, forbid          # refuse to drive into an active switch
            armed = not initially

        steps = 0
        decel = False
        t_next = self._clock()
        while True:
            if self._halt.is_set():
                return _End.HALT, steps, None
            if not decel and cmd.epoch != self._epoch:
                if cancel_mode == "immediate" or ramp.at_start_speed:
                    return _End.CANCELLED, steps, None
                decel = True
            if not self._do_step(direction):
                return _End.HALT, steps, None
            steps += 1
            with self._lock:
                self._position += direction
            self.stats.steps += 1

            hr, er = self._read_switches_hw()
            home_on, end_on = deb[0].update(hr), deb[1].update(er)
            if home_on and end_on:
                self._enter_fault("both_switches", "home and end switches are both active")
                return _End.FAULT, steps, "both_switches"
            if until is not None and until(home_on, end_on):
                return _End.FOUND, steps, None
            if fidx is not None:
                if (home_on, end_on)[fidx]:
                    if armed:
                        return _End.LIMIT, steps, forbid
                else:
                    armed = True
            if max_steps is not None and steps >= max_steps:
                return _End.DONE, steps, None
            if decel and ramp.at_start_speed:
                return _End.CANCELLED, steps, None

            remaining = (max_steps - steps) if (plan_decel and max_steps is not None) else None
            interval = ramp.next_interval(remaining, decel)
            t_next += interval
            now = self._clock()
            late = now - t_next
            if late > 0:
                if late > self.stats.max_late_s:
                    self.stats.max_late_s = late
                if late > max(interval, 0.002):       # stalled: resynchronise instead of bursting
                    self.stats.late_steps += 1
                    t_next = now
            else:
                self._sleep(-late)

    def _fault_status(self):
        with self._lock:
            f = self._fault
        return MoveStatus.EMERGENCY_STOP if f and f.reason == "emergency_stop" else MoveStatus.FAULT

    def _failure(self, end, steps, detail, context):
        """MoveResult for any outcome of a step loop that is not a success."""
        if end is _End.CANCELLED:
            return MoveResult(MoveStatus.CANCELLED, steps, message="stopped")
        if end is _End.LIMIT:
            self._enter_fault("unexpected_limit",
                              "%s switch hit during %s at position %d" % (detail, context, self.position))
            return MoveResult(MoveStatus.LIMIT_HIT, steps,
                              message="%s switch hit during %s" % (detail, context))
        status = self._fault_status()
        with self._lock:
            msg = self._fault.detail if self._fault else "halted"
        return MoveResult(status, steps, message=msg)

    # ------------------------------------------------------------ move commands
    def _resolve_named(self, name):
        with self._lock:
            if name == "start":
                return self._start_steps if self._start_steps is not None else 0
            return self._end_steps if self._end_steps is not None else self._travel

    def _run_move(self, cmd):
        a = cmd.args
        cfg = self.cfg
        with self._lock:
            pos, homed, travel = self._position, self._homed, self._travel
        check_soft = True
        if cmd.kind == "jog":
            target, check_soft = pos + a["steps"], False
        else:
            if not homed:
                return MoveResult(MoveStatus.REFUSED, message="axis not homed")
            if cmd.kind == "move_to":
                target = a["position"]
            elif cmd.kind == "move_by":
                target = pos + a["steps"]
            else:
                target = self._resolve_named(a["name"])
                if target is None:
                    return MoveResult(MoveStatus.REFUSED, message="travel unknown: calibrate first")
        if check_soft and (target < 0 or (travel is not None and target > travel)):
            return MoveResult(MoveStatus.REFUSED,
                              message="target %d outside [0, %s]" % (target, travel))
        steps = target - pos
        if steps == 0:
            return MoveResult(MoveStatus.COMPLETED)
        direction = 1 if steps > 0 else -1
        speed = min(a["speed"] or cfg.max_speed, cfg.max_speed)
        end, done, detail = self._step_loop(cmd, direction, abs(steps), speed=speed,
                                            accel=a["accel"])
        if end is _End.DONE:
            return MoveResult(MoveStatus.COMPLETED, done * direction)
        return self._failure(end, done * direction, detail, "move")

    # ------------------------------------------------------- homing / calibration
    def _seek(self, cmd, direction, until, speed, max_steps, forbid, what):
        """Move until `until` is met. Returns (steps, MoveResult or None on success)."""
        end, n, detail = self._step_loop(cmd, direction, max_steps, speed=speed, until=until,
                                         forbid=forbid, plan_decel=False, cancel_mode="immediate")
        if end is _End.FOUND:
            return n, None
        if end is _End.DONE:
            self._enter_fault("search_failed", what)
            return n, MoveResult(MoveStatus.SEARCH_FAILED, n * direction, message=what)
        return n, self._failure(end, n * direction, detail, what)

    def _reference(self, cmd, which, measure):
        """Find the `which` switch ("home" or "end"), release it and keep clearance_steps away.
        Returns a failure MoveResult, or None on success (position is then the clearance point)."""
        cfg = self.cfg
        sw = cfg.home_switch if which == "home" else cfg.end_switch
        if sw is None:
            return MoveResult(MoveStatus.REFUSED, message="no %s switch configured" % which)
        if which == "end" and not measure and self.travel_steps is None:
            return MoveResult(MoveStatus.REFUSED, message="travel unknown: calibrate first")
        to = -1 if which == "home" else 1
        other = "end" if which == "home" else "home"
        hit = (lambda h, e: h) if which == "home" else (lambda h, e: e)
        released = lambda h, e: not hit(h, e)

        h, e = self._stable_switches()
        if h and e:
            self._enter_fault("both_switches", "home and end switches are both active")
            return MoveResult(MoveStatus.FAULT, message="both switches active")
        if hit(h, e):                                   # already on the switch: back off first
            _, res = self._seek(cmd, -to, released, cfg.homing_speed, cfg.max_release_steps,
                                other, "%s switch does not release" % which)
            if res is not None:
                return res
        _, res = self._seek(cmd, to, hit, cfg.homing_speed, cfg.max_travel_steps, other,
                            "%s switch not found within %d steps" % (which, cfg.max_travel_steps))
        if res is not None:
            return res
        _, res = self._seek(cmd, -to, released, cfg.latch_speed, cfg.max_release_steps, other,
                            "%s switch does not release" % which)
        if res is not None:
            return res
        if cfg.clearance_steps > 0:
            end, n, detail = self._step_loop(cmd, -to, cfg.clearance_steps,
                                             speed=cfg.latch_speed, forbid=other)
            if end is not _End.DONE:
                return self._failure(end, n * -to, detail, "clearance move")
        return None

    def _run_reference(self, cmd, which, measure=False):
        fail = self._reference(cmd, which, measure)
        if fail is not None:
            return fail
        with self._lock:
            if which == "home":
                self._position = 0
            else:
                self._position = self._travel
            self._homed = True
            position = self._position
        self._emit("homed", position=position, switch=which)
        return MoveResult(MoveStatus.COMPLETED, message="referenced on %s switch" % which,
                          info={"position": position})

    def _run_calibrate(self, cmd):
        res = self._run_reference(cmd, "home")
        if not res.ok:
            return res
        fail = self._reference(cmd, "end", measure=True)
        if fail is not None:
            return fail
        with self._lock:
            self._travel = self._position
            travel = self._travel
            for name in ("_start_steps", "_end_steps"):    # drop saved values beyond the new travel
                v = getattr(self, name)
                if v is not None and v > travel:
                    setattr(self, name, None)
            self._homed = True
        self._save_state()
        self._emit("calibrated", travel_steps=travel)
        return MoveResult(MoveStatus.COMPLETED, message="calibrated", info={"travel_steps": travel})

    # -------------------------------------------------------------- persistence
    def _state_dict(self):
        with self._lock:
            return {
                "version": STATE_VERSION,
                "step_mode": self.cfg.step_mode,
                "travel_steps": self._travel,
                "start_steps": self._start_steps,
                "end_steps": self._end_steps,
            }

    def _save_state(self):
        path = self.cfg.state_file
        if not path:
            return
        try:
            data = json.dumps(self._state_dict(), indent=2)
            directory = os.path.dirname(os.path.abspath(path))
            fd, tmp = tempfile.mkstemp(dir=directory, prefix=".stepper-", suffix=".tmp")
            with os.fdopen(fd, "w") as f:
                f.write(data)
            os.replace(tmp, path)
        except OSError:
            log.exception("could not save the state file %s", path)

    def _load_state(self):
        path = self.cfg.state_file
        if not path or not os.path.exists(path):
            return
        try:
            with open(path) as f:
                d = json.load(f)
            if d.get("version") != STATE_VERSION or d.get("step_mode") != self.cfg.step_mode:
                log.warning("ignoring state file %s (different version or step mode)", path)
                return
            self._travel = d.get("travel_steps")
            self._start_steps = d.get("start_steps")
            self._end_steps = d.get("end_steps")
        except (OSError, ValueError):
            log.exception("could not read the state file %s", path)
