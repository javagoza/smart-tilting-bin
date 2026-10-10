"""Gesture and presence detection for the EVAL-CN0569-PMDZ.

Port of ADI's algorithm (adpd2140_gesture_sensor in pyadi-iio), decoupled from
the hardware: it works on "frames" of 8 integers (channels 0-3 = sensor 0,
4-7 = sensor 1).

Intentional changes with respect to the original:
  * Both sensors are processed with the SAME frame (the original calls rx()
    twice and each call consumes different data).
  * The baseline is per instance (in the original `avg` is a class attribute).
  * A gesture that is too short is discarded (the original left has_gest=True
    with a stale start point).
  * The end position is that of the last frame WITH signal, not of the frame in
    which the signal had already fallen below the threshold (there the x/y
    quotient is just noise).
  * The (x, y, L) trajectory of each gesture is kept: it can feed a classifier
    (e.g. rotations) and be used to record data.
"""
import math
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, List, Optional, Tuple


class Gesture(IntEnum):  # same numbering as ADI's demo
    CLICK = 0
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4


def position(c):
    """x = (c1-c0)/(c1+c0), y = (c3-c2)/(c3+c2); 0.0 if there is no signal."""
    sx = c[1] + c[0]
    sy = c[3] + c[2]
    x = (c[1] - c[0]) / sx if sx else 0.0
    y = (c[3] - c[2]) / sy if sy else 0.0
    return x, y


def classify(start, end, d_thresh):
    sx, sy = start
    ex, ey = end
    d = math.hypot(sx - ex, sy - ey)
    if d < d_thresh:
        return Gesture.CLICK
    m = (sy - ey) / (sx - ex + 1e-6)
    if abs(m) > 1:
        return Gesture.UP if sy < ey else Gesture.DOWN
    if abs(m) < 1:
        return Gesture.LEFT if sx > ex else Gesture.RIGHT
    return Gesture.CLICK


class SensorTracker:
    """State machine for one ADPD2140 (4 channels, baseline already removed)."""

    def __init__(self, l_thresh=1000, d_thresh=0.07, min_frames=5):
        self.l_thresh = l_thresh
        self.d_thresh = d_thresh
        self.min_frames = min_frames
        self.level = 0
        self.active = False
        self.frames = 0
        self.trajectory: List[Tuple[float, float, int]] = []
        self.last_trajectory: List[Tuple[float, float, int]] = []

    def update(self, v) -> Optional[Gesture]:
        self.level = L = sum(v)
        if L > self.l_thresh:
            x, y = position(v)
            if not self.active:
                self.active = True
                self.frames = 0
                self.trajectory = []
            else:
                self.frames += 1
            self.trajectory.append((x, y, L))
            return None
        if self.active:
            self.active = False
            traj = self.trajectory
            self.trajectory = []
            if self.frames >= self.min_frames:
                self.last_trajectory = traj
                return classify(traj[0][:2], traj[-1][:2], self.d_thresh)
        return None


@dataclass
class Event:
    kind: str                  # "gesture" | "presence" | "absence"
    sensor: Optional[int] = None
    gesture: Optional[Gesture] = None
    timestamp: float = 0.0
    trajectory: list = field(default_factory=list)


class GestureEngine:
    def __init__(
        self,
        read_frame: Callable[[], list],
        on_event: Optional[Callable[[Event], None]] = None,
        l_thresh=1000,
        d_thresh=0.07,
        min_frames=5,
        presence_on=600,
        presence_off=300,
        presence_on_frames=2,
        presence_off_frames=10,
    ):
        self.read_frame = read_frame
        self.on_event = on_event or (lambda e: None)
        self.trackers = [
            SensorTracker(l_thresh, d_thresh, min_frames) for _ in range(2)
        ]
        self.baseline = [0] * 8
        self.presence_on = presence_on
        self.presence_off = presence_off
        self.presence_on_frames = presence_on_frames
        self.presence_off_frames = presence_off_frames
        self.present = False
        self._cnt = 0

    def calibrate(self, n=16):
        """Average n frames WITHOUT a hand in front of the sensor."""
        acc = [0] * 8
        for _ in range(n):
            f = self.read_frame()
            for i, v in enumerate(f):
                acc[i] += v
        self.baseline = [int(a / n) for a in acc]

    def process_frame(self, frame, t=None) -> List[Event]:
        t = time.time() if t is None else t
        v = [max(0, f - b) for f, b in zip(frame, self.baseline)]
        events = []
        for k, tr in enumerate(self.trackers):
            g = tr.update(v[4 * k:4 * k + 4])
            if g is not None:
                events.append(Event("gesture", k, g, t, list(tr.last_trajectory)))

        level = max(tr.level for tr in self.trackers)
        if not self.present:
            self._cnt = self._cnt + 1 if level > self.presence_on else 0
            if self._cnt >= self.presence_on_frames:
                self.present, self._cnt = True, 0
                events.insert(0, Event("presence", timestamp=t))
        else:
            self._cnt = self._cnt + 1 if level < self.presence_off else 0
            if self._cnt >= self.presence_off_frames:
                self.present, self._cnt = False, 0
                events.append(Event("absence", timestamp=t))

        for e in events:
            self.on_event(e)
        return events

    def run(self, should_stop: Callable[[], bool] = lambda: False):
        while not should_stop():
            self.process_frame(self.read_frame())
