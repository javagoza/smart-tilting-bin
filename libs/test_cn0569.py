"""Hardware-free tests: simulated I2C bus + synthetic hand movements.
Run with:  python3 -m unittest -v test_cn0569
"""
import unittest

import adpd1080 as A
from adpd1080 import ADPD1080
from gesture import Gesture, GestureEngine


class FakeBus:
    def __init__(self):
        self.regs = {A.REG_DEVID: A.DEVICE_ID_ADPD108X, A.REG_CLK_RATIO: 2000}
        self.fifo = []
        self.writes = []

    def push_sample(self, ch8):
        for v in ch8:
            self.fifo += [v & 0xFFFF, (v >> 16) & 0xFFFF]

    def read_words(self, reg, count=1):
        if reg == A.REG_STATUS:
            return [(min(len(self.fifo) * 2, 255) << 8)]
        if reg == A.REG_FIFO_ACCESS:
            out, self.fifo = self.fifo[:count], self.fifo[count:]
            return out
        return [self.regs.get(reg, 0)]

    def write_word(self, reg, val):
        self.writes.append((reg, val))
        self.regs[reg] = val


def sensor_channels(x, y, intensity):
    """4 channels of one sensor such that (c1-c0)/(c1+c0)=x and (c3-c2)/(c3+c2)=y."""
    return [intensity * (1 - x) / 2, intensity * (1 + x) / 2,
            intensity * (1 - y) / 2, intensity * (1 + y) / 2]


def swipe(p0, p1, n=14, peak=3000, base=100):
    """8-channel frames: the hand moves from p0 to p1 with a bell-shaped intensity."""
    frames = []
    for i in range(n):
        s = i / (n - 1)
        x = p0[0] + (p1[0] - p0[0]) * s
        y = p0[1] + (p1[1] - p0[1]) * s
        inten = peak * (1 - (2 * s - 1) ** 2) + 400  # >0 at the ends
        ch = sensor_channels(x, y, inten)
        frames.append([int(c) + base for c in ch] * 2)
    return frames + [[base] * 8] * 12  # hand removed


def run_engine(frames, **kw):
    it = iter(frames)
    eng = GestureEngine(lambda: next(it), **kw)
    eng.baseline = [100] * 8
    events = []
    for f in frames:
        events += eng.process_frame(f)
    return events


def gestures(events):
    return [(e.sensor, e.gesture) for e in events if e.kind == "gesture"]


class DriverTests(unittest.TestCase):
    def test_init_sequence(self):
        bus = FakeBus()
        ADPD1080(bus, fs_hz=512).init()
        r = bus.regs
        self.assertEqual(r[A.REG_PD_LED_SELECT], 0x0455)
        self.assertEqual(r[A.REG_FSAMPLE], 15)
        self.assertEqual(r[A.REG_SLOTA_NUMPULSES], 0x040E)
        self.assertEqual(r[A.REG_SLOTB_AFE_WINDOW], 0x22F0)
        self.assertEqual(r[A.REG_MATH], 0x0544)
        self.assertEqual(r[A.REG_ILED1_COARSE], 0x2036)
        self.assertEqual(r[A.REG_INT_SEQ_A], 0x9)
        slot = r[A.REG_SLOT_EN]
        self.assertEqual((slot >> 2) & 7, 6)
        self.assertEqual((slot >> 6) & 7, 6)
        self.assertTrue(slot & 0x21 == 0x21)

    def test_bad_device_id(self):
        bus = FakeBus()
        bus.regs[A.REG_DEVID] = 0x1234
        with self.assertRaises(RuntimeError):
            ADPD1080(bus).init()

    def test_fifo_parse_and_frame(self):
        bus = FakeBus()
        dev = ADPD1080(bus)
        for k in range(8):
            bus.push_sample([70000 * (c + 1) + k for c in range(8)])
        frame = dev.next_frame(8)
        self.assertEqual(frame[0], sum(70000 + k for k in range(8)))
        self.assertEqual(frame[7], sum(70000 * 8 + k for k in range(8)))

    def test_fifo_no_burst(self):
        bus = FakeBus()
        dev = ADPD1080(bus, fifo_burst=False)
        bus.push_sample(list(range(100000, 100008)))
        self.assertEqual(dev.read_samples(1)[0], tuple(range(100000, 100008)))

    def test_timeout(self):
        with self.assertRaises(TimeoutError):
            ADPD1080(FakeBus()).read_samples(1, timeout=0.01)


class GestureTests(unittest.TestCase):
    def check(self, p0, p1, expected):
        ev = run_engine(swipe(p0, p1))
        self.assertEqual(gestures(ev), [(0, expected), (1, expected)])

    def test_right(self): self.check((-0.7, 0), (0.7, 0), Gesture.RIGHT)
    def test_left(self): self.check((0.7, 0), (-0.7, 0), Gesture.LEFT)
    def test_up(self): self.check((0, -0.7), (0, 0.7), Gesture.UP)
    def test_down(self): self.check((0, 0.7), (0, -0.7), Gesture.DOWN)
    def test_click(self): self.check((0.0, 0.0), (0.02, 0.0), Gesture.CLICK)

    def test_short_blip_discarded(self):
        self.assertEqual(gestures(run_engine(swipe((-0.7, 0), (0.7, 0), n=4))), [])

    def test_no_signal_no_events(self):
        self.assertEqual(run_engine([[100] * 8] * 50), [])

    def test_presence_hysteresis(self):
        ev = run_engine(swipe((-0.7, 0), (0.7, 0)))
        kinds = [e.kind for e in ev]
        self.assertEqual(kinds[0], "presence")
        self.assertEqual(kinds[-1], "absence")
        self.assertEqual(kinds.count("presence"), 1)

    def test_trajectory_recorded(self):
        ev = [e for e in run_engine(swipe((-0.7, 0), (0.7, 0))) if e.kind == "gesture"]
        traj = ev[0].trajectory
        self.assertGreater(len(traj), 5)
        self.assertLess(traj[0][0], traj[-1][0])

    def test_two_swipes_in_a_row(self):
        fr = swipe((-0.7, 0), (0.7, 0)) + swipe((0, -0.7), (0, 0.7))
        got = [g for _, g in gestures(run_engine(fr))]
        self.assertEqual(got, [Gesture.RIGHT, Gesture.RIGHT, Gesture.UP, Gesture.UP])


if __name__ == "__main__":
    unittest.main()
