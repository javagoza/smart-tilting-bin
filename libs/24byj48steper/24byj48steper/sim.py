"""Simulated hardware for tests and for trying the driver without a motor.

SimBackend models a carriage on a rail: the coil phases move the position, the home
switch is active at or below `home_trigger`, the end switch at or above `end_trigger`,
and mechanical hard stops lie `hard_margin` steps beyond them (hitting one is counted
in `collisions`, which a correct run keeps at 0).
"""
import threading
import time

from .backends import Backend


class VirtualClock:
    """A clock whose sleep() returns immediately and advances virtual time."""

    def __init__(self, t0=1000.0):
        self._t = t0
        self._lock = threading.Lock()

    def now(self):
        with self._lock:
            return self._t

    def sleep(self, dt):
        if dt > 0:
            with self._lock:
                self._t += dt


class SimBackend(Backend):
    def __init__(self, cfg, clock=time.monotonic, *, position=1500, home_trigger=0,
                 end_trigger=5000, hard_margin=300, wiring_reversed=False):
        self.cfg = cfg
        self.clock = clock
        self.seq = list(cfg.sequence())
        self.position = position
        self.home_trigger = home_trigger
        self.end_trigger = end_trigger
        self.hard_min = home_trigger - hard_margin
        self.hard_max = end_trigger + hard_margin
        self.wiring_reversed = wiring_reversed
        self.collisions = 0
        self.step_log = []                 # (time, position) for every position change
        self.on_step = None                # callback(step_count), called after each step
        self.glitches = {"home": 0, "end": 0}   # next N reads return the inverted value
        self.stuck = {"home": False, "end": False}
        self.fail_after_writes = None      # raise OSError after N coil writes
        self.writes = 0
        self.released_count = 0
        self.energized = False
        self.closed = False
        self._idx = 0                      # the driver starts at phase 0 (rotor aligned with it)
        self._lock = threading.Lock()

    # --- Backend interface ---
    def set_coils(self, pattern):
        callback = None
        with self._lock:
            self.writes += 1
            if self.fail_after_writes is not None and self.writes > self.fail_after_writes:
                raise OSError("simulated GPIO failure")
            pattern = tuple(pattern)
            if not any(pattern):
                self.energized = False
                return
            self.energized = True
            if pattern not in self.seq:
                return
            idx = self.seq.index(pattern)
            n = len(self.seq)
            d = (idx - self._idx + n // 2) % n - n // 2
            self._idx = idx
            if d:
                self._move(-d if self.wiring_reversed else d)
                callback = self.on_step
            count = len(self.step_log)
        if callback is not None:
            callback(count)

    def _move(self, d):
        new = self.position + d
        if new < self.hard_min or new > self.hard_max:
            self.collisions += 1
            new = min(max(new, self.hard_min), self.hard_max)
        self.position = new
        self.step_log.append((self.clock(), new))

    def read_switches(self):
        with self._lock:
            home = self.position <= self.home_trigger
            end = self.position >= self.end_trigger
            if self.stuck["home"]:
                home = True
            if self.stuck["end"]:
                end = True
            if self.glitches["home"] > 0:
                self.glitches["home"] -= 1
                home = not home
            if self.glitches["end"] > 0:
                self.glitches["end"] -= 1
                end = not end
            return home, end

    def release(self):
        with self._lock:
            self.released_count += 1
            self.energized = False

    def close(self):
        self.closed = True
        self.release()

    # --- helpers for tests ---
    def step_intervals(self):
        t = [e[0] for e in self.step_log]
        return [b - a for a, b in zip(t, t[1:])]
