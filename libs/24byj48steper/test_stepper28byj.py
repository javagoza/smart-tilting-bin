"""Hardware-free tests: simulated carriage + virtual clock (fast, deterministic) and a few
real-time tests for the concurrency behaviour.

Run with:  python3 -m unittest -v test_stepper28byj
"""
import json
import logging
import math
import os
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace

from stepper28byj import (ClosedError, FaultError, LimitError, MoveStatus, NotCalibratedError,
                          NotHomedError, State, Stepper28BYJ48, StepperConfig, SwitchConfig,
                          intervals, move_time)
from stepper28byj.backends import GpiodBackend
from stepper28byj.config import FULL_STEP_SEQUENCE, HALF_STEP_SEQUENCE
from stepper28byj.ramp import Ramp
from stepper28byj.sim import SimBackend, VirtualClock


logging.getLogger("stepper28byj").setLevel(logging.CRITICAL)   # tests provoke faults on purpose


def fast_cfg(**kw):
    base = dict(max_speed=800, start_speed=100, acceleration=2000, homing_speed=400,
                latch_speed=150, clearance_steps=50, max_travel_steps=9000)
    base.update(kw)
    return StepperConfig(**base)


def make(cfg=None, **sim_kw):
    """Driver + simulator running on a virtual clock (sleep returns instantly)."""
    clk = VirtualClock()
    cfg = cfg or fast_cfg()
    sim = SimBackend(cfg, clk.now, **sim_kw)
    m = Stepper28BYJ48(cfg, sim, clock=clk.now, sleep=clk.sleep)
    return m, sim, clk


def make_real(cfg=None, **sim_kw):
    """Same, but on the real clock: used to test the threading behaviour."""
    cfg = cfg or fast_cfg(max_speed=2000, start_speed=1000, acceleration=20000)
    sim = SimBackend(cfg, time.monotonic, **sim_kw)
    return Stepper28BYJ48(cfg, sim), sim


def homed(**sim_kw):
    m, sim, clk = make(**sim_kw)
    r = m.home().result(30)
    assert r.ok, r
    return m, sim, clk, sim.position           # sim.position is the physical position of logical 0


def calibrated(**sim_kw):
    m, sim, clk, zero = homed(**sim_kw)
    r = m.calibrate().result(30)
    assert r.ok, r
    return m, sim, clk, zero


class RampTests(unittest.TestCase):
    def test_first_interval_is_the_start_speed(self):
        self.assertAlmostEqual(intervals(100, 100, 500, 400)[0], 0.01)

    def test_trapezoid_shape(self):
        iv = intervals(2000, 100, 500, 400)
        peak = iv.index(min(iv))
        self.assertAlmostEqual(min(iv), 1 / 500)
        self.assertTrue(all(a >= b for a, b in zip(iv[:peak], iv[1:peak + 1])))   # speeding up
        self.assertTrue(all(a <= b for a, b in zip(iv[-200:], iv[-199:])))        # slowing down
        self.assertGreater(iv[-1], iv[1000])
        self.assertEqual(sum(1 for v in iv if abs(v - 1 / 500) < 1e-12) > 1000, True)  # cruise

    def test_short_move_is_triangular(self):
        iv = intervals(40, 100, 500, 400)
        self.assertGreater(min(iv), 1 / 500)       # never reaches the maximum speed
        self.assertEqual(len(iv), 40)

    def test_total_time_matches_the_formula(self):
        vs, vm, a, n = 100.0, 500.0, 400.0, 3000
        d = (vm * vm - vs * vs) / (2 * a)
        expected = 2 * (vm - vs) / a + (n - 2 * d) / vm
        self.assertAlmostEqual(move_time(n, vs, vm, a) / expected, 1.0, delta=0.03)

    def test_speed_stays_within_bounds(self):
        r = Ramp(100, 500, 400)
        for i in range(3000):
            r.next_interval(3000 - i - 1)
            self.assertLessEqual(r.v, 500 + 1e-9)
            self.assertGreaterEqual(r.v, 100 - 1e-9)

    def test_decel_now_returns_to_start_speed(self):
        r = Ramp(100, 500, 400)
        for _ in range(400):
            r.next_interval(None)
        self.assertAlmostEqual(r.v, 500)
        n = 0
        while not r.at_start_speed:
            r.next_interval(None, decel_now=True)
            n += 1
        self.assertLessEqual(n, math.ceil((500 ** 2 - 100 ** 2) / (2 * 400)) + 1)

    def test_start_speed_is_clamped_to_max(self):
        self.assertEqual(Ramp(900, 500, 100).start, 500)

    def test_invalid(self):
        with self.assertRaises(ValueError):
            Ramp(0, 500, 400)


class ConfigTests(unittest.TestCase):
    def test_defaults_are_valid(self):
        cfg = StepperConfig()
        cfg.validate()
        self.assertEqual(cfg.effective_steps_per_rev, 4096)
        self.assertEqual(StepperConfig(step_mode="full").effective_steps_per_rev, 2048)
        self.assertEqual(StepperConfig(steps_per_rev=4076).effective_steps_per_rev, 4076)

    def test_validation_errors(self):
        bad = [
            dict(step_mode="quarter"),
            dict(coil_pins=(1, 2, 3)),
            dict(coil_pins=(17, 27, 22, 5)),                      # clashes with the home switch
            dict(max_speed=0),
            dict(start_speed=900, max_speed=500),
            dict(acceleration=-1),
            dict(clearance_steps=-1),
            dict(max_travel_steps=0),
            dict(home_switch=SwitchConfig(5, bias="sideways")),
            dict(end_switch=SwitchConfig(6, debounce_samples=0)),
        ]
        for kw in bad:
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                StepperConfig(**kw).validate()

    def test_sequences(self):
        for seq, coils in ((HALF_STEP_SEQUENCE, None), (FULL_STEP_SEQUENCE, 2)):
            n = len(seq)
            for i in range(n):
                a, b = seq[i], seq[(i + 1) % n]
                self.assertEqual(sum(x != y for x, y in zip(a, b)), 1 if coils is None else 2)
        self.assertEqual(len(set(HALF_STEP_SEQUENCE)), 8)


class FakeRequest:
    def __init__(self, config):
        self.config = config
        self.sets, self.released = [], False
        self.inputs = {}

    def set_values(self, values):
        self.sets.append(dict(values))

    def get_values(self, pins):
        return [self.inputs.get(p, Value.INACTIVE) for p in pins]

    def release(self):
        self.released = True


class Value:
    INACTIVE, ACTIVE = "INACTIVE", "ACTIVE"


class FakeGpiod:
    class LineSettings:
        def __init__(self, **kw):
            self.kw = kw

    line = SimpleNamespace(
        Value=Value,
        Direction=SimpleNamespace(INPUT="INPUT", OUTPUT="OUTPUT"),
        Bias=SimpleNamespace(PULL_UP="PULL_UP", PULL_DOWN="PULL_DOWN", DISABLED="DISABLED"),
    )

    def __init__(self):
        self.request = None
        self.args = None

    def request_lines(self, chip, consumer=None, config=None):
        self.args = (chip, consumer)
        self.request = FakeRequest(config)
        return self.request


class GpiodBackendTests(unittest.TestCase):
    def setUp(self):
        self.g = FakeGpiod()
        self.cfg = StepperConfig()
        self.b = GpiodBackend(self.cfg, _gpiod=self.g)
        self.req = self.g.request

    def test_line_request(self):
        self.assertEqual(self.g.args, ("/dev/gpiochip0", "stepper28byj"))
        cfg = self.req.config
        self.assertEqual(set(cfg), {17, 27, 22, 23, 5, 6})
        for pin in (17, 27, 22, 23):
            self.assertEqual(cfg[pin].kw["direction"], "OUTPUT")
            self.assertEqual(cfg[pin].kw["output_value"], "INACTIVE")
        for pin in (5, 6):
            self.assertEqual(cfg[pin].kw["direction"], "INPUT")
            self.assertEqual(cfg[pin].kw["bias"], "PULL_UP")
            self.assertTrue(cfg[pin].kw["active_low"])

    def test_normally_closed_switch_configuration(self):
        g = FakeGpiod()
        cfg = StepperConfig(home_switch=SwitchConfig(5, active_low=False, bias="pull_up"))
        GpiodBackend(cfg, _gpiod=g)
        self.assertFalse(g.request.config[5].kw["active_low"])

    def test_set_coils_single_call(self):
        self.b.set_coils((1, 1, 0, 0))
        self.assertEqual(len(self.req.sets), 1)
        self.assertEqual(self.req.sets[0], {17: "ACTIVE", 27: "ACTIVE", 22: "INACTIVE", 23: "INACTIVE"})

    def test_read_switches(self):
        self.assertEqual(self.b.read_switches(), (False, False))
        self.req.inputs[6] = "ACTIVE"
        self.assertEqual(self.b.read_switches(), (False, True))
        self.req.inputs[5] = "ACTIVE"
        self.assertEqual(self.b.read_switches(), (True, True))

    def test_single_switch_and_none(self):
        g = FakeGpiod()
        b = GpiodBackend(StepperConfig(end_switch=None), _gpiod=g)
        g.request.inputs[5] = "ACTIVE"
        self.assertEqual(b.read_switches(), (True, False))
        g2 = FakeGpiod()
        b2 = GpiodBackend(StepperConfig(home_switch=None, end_switch=None), _gpiod=g2)
        self.assertEqual(b2.read_switches(), (False, False))

    def test_release_and_close(self):
        self.b.release()
        self.assertEqual(set(self.req.sets[-1].values()), {"INACTIVE"})
        self.b.close()
        self.assertTrue(self.req.released)


class SimTests(unittest.TestCase):
    def test_phases_move_the_carriage(self):
        cfg = fast_cfg()
        sim = SimBackend(cfg, position=100)
        for p in HALF_STEP_SEQUENCE:
            sim.set_coils(p)
        sim.set_coils(HALF_STEP_SEQUENCE[0])
        self.assertEqual(sim.position, 108)
        for p in reversed(HALF_STEP_SEQUENCE[1:]):
            sim.set_coils(p)
        self.assertEqual(sim.position, 101)

    def test_switches_and_hard_stops(self):
        sim = SimBackend(fast_cfg(), position=-299, home_trigger=0, end_trigger=100)
        self.assertEqual(sim.read_switches(), (True, False))
        sim.set_coils(HALF_STEP_SEQUENCE[7])       # one step back: reaches the hard stop
        self.assertEqual((sim.position, sim.collisions), (-300, 0))
        sim.set_coils(HALF_STEP_SEQUENCE[6])       # two more steps back: blocked and counted
        sim.set_coils(HALF_STEP_SEQUENCE[5])
        self.assertEqual((sim.position, sim.collisions), (-300, 2))


class HomingTests(unittest.TestCase):
    def test_home_from_the_middle(self):
        m, sim, clk, zero = homed(position=1500)
        try:
            self.assertTrue(m.homed)
            self.assertEqual(m.position, 0)
            self.assertEqual(sim.read_switches(), (False, False))
            self.assertEqual(sim.collisions, 0)
            self.assertTrue(1 < zero <= 1 + 3 + 50 + 1)        # release edge + debounce + clearance
            self.assertFalse(sim.energized)                    # coils released when idle
        finally:
            m.close()

    def test_home_is_repeatable_from_any_start(self):
        zeros = []
        for start in (300, 1500, 4000, -100):                  # -100: already on the switch
            m, sim, clk, zero = homed(position=start)
            zeros.append(zero)
            self.assertEqual(sim.collisions, 0)
            m.close()
        self.assertEqual(len(set(zeros)), 1, zeros)

    def test_home_not_found_faults(self):
        cfg = fast_cfg(max_travel_steps=500)
        m, sim, clk = make(cfg, position=3000, home_trigger=-100000)
        try:
            r = m.home().result(30)
            self.assertEqual(r.status, MoveStatus.SEARCH_FAILED)
            self.assertEqual(m.state, State.FAULT)
            self.assertEqual(m.fault.reason, "search_failed")
            self.assertFalse(sim.energized)
            self.assertFalse(m.homed)
            with self.assertRaises(FaultError):
                m.home()
        finally:
            m.close()

    def test_switch_that_never_releases_faults(self):
        cfg = fast_cfg(max_release_steps=200)
        m, sim, clk = make(cfg, position=-100)
        sim.stuck["home"] = True
        try:
            r = m.home().result(30)
            self.assertEqual(r.status, MoveStatus.SEARCH_FAILED)
            self.assertIn("release", r.message)
        finally:
            m.close()

    def test_both_switches_active_faults(self):
        m, sim, clk = make()
        sim.stuck["home"] = sim.stuck["end"] = True
        try:
            r = m.home().result(30)
            self.assertEqual(r.status, MoveStatus.FAULT)
            self.assertEqual(m.fault.reason, "both_switches")
            self.assertEqual(sim.writes, 0)                    # refused before moving
        finally:
            m.close()

    def test_calibrate_measures_the_travel(self):
        m, sim, clk, zero = calibrated(position=1500)
        try:
            self.assertTrue(m.calibrated)
            self.assertEqual(m.position, m.travel_steps)
            self.assertEqual(sim.position - zero, m.travel_steps)
            self.assertEqual(sim.collisions, 0)
            self.assertEqual(sim.read_switches(), (False, False))
            # end switch released at 4999 -> detected after 3 samples; minus clearance both sides
            self.assertTrue(abs(m.travel_steps - (5000 - 2 * 50 - 6)) <= 6, m.travel_steps)
        finally:
            m.close()

    def test_calibrate_is_repeatable(self):
        travels = []
        for start in (200, 2500, 4900):
            m, sim, clk, zero = calibrated(position=start)
            travels.append(m.travel_steps)
            m.close()
        self.assertEqual(len(set(travels)), 1, travels)

    def test_seek_end_restores_the_reference(self):
        m, sim, clk, zero = calibrated()
        try:
            end_physical = sim.position
            m.move_to(1000).result(30)
            r = m.seek_end().result(30)
            self.assertTrue(r.ok)
            self.assertEqual(m.position, m.travel_steps)
            self.assertEqual(sim.position, end_physical)
        finally:
            m.close()

    def test_seek_end_needs_calibration(self):
        m, sim, clk = make()
        try:
            with self.assertRaises(NotCalibratedError):
                m.seek_end()
        finally:
            m.close()

    def test_wrong_direction_wiring_is_caught_safely(self):
        # motor wired backwards and invert_direction not set: homing walks into the END switch
        m, sim, clk = make(position=1500, wiring_reversed=True)
        try:
            r = m.home().result(30)
            self.assertEqual(r.status, MoveStatus.LIMIT_HIT)
            self.assertEqual(sim.collisions, 0)
            self.assertEqual(m.state, State.FAULT)
        finally:
            m.close()

    def test_invert_direction_fixes_the_wiring(self):
        m, sim, clk = make(fast_cfg(invert_direction=True), position=1500, wiring_reversed=True)
        try:
            self.assertTrue(m.calibrate().result(30).ok)
            self.assertEqual(sim.collisions, 0)
        finally:
            m.close()

    def test_debounce_ignores_short_glitches(self):
        zeros = []
        for glitch in (0, 1, 2):
            m, sim, clk = make(position=1500)
            if glitch:
                def hook(count, sim=sim, glitch=glitch):
                    if count == 800:
                        sim.glitches["home"] = glitch
                sim.on_step = hook
            self.assertTrue(m.home().result(30).ok)
            zeros.append(sim.position)
            m.close()
        self.assertEqual(len(set(zeros)), 1, zeros)


class MotionTests(unittest.TestCase):
    def assert_sync(self, m, sim, zero):
        self.assertEqual(sim.position - zero, m.position)

    def test_unhomed_commands_are_rejected_but_jog_works(self):
        m, sim, clk = make(position=1500)
        try:
            for call in (lambda: m.move_to(10), lambda: m.move_by(10), m.go_start):
                with self.assertRaises(NotHomedError):
                    call()
            r = m.jog(200).result(30)
            self.assertTrue(r.ok)
            self.assertEqual(r.steps, 200)
            self.assertIsNone(r.position)                     # position unknown when not homed
            self.assertEqual(sim.position, 1700)
        finally:
            m.close()

    def test_move_to_is_exact_with_a_trapezoidal_profile(self):
        m, sim, clk, zero = calibrated()
        try:
            n0 = len(sim.step_log)
            r = m.move_to(1000).result(30)
            self.assertTrue(r.ok)
            self.assertEqual(r.position, 1000)
            self.assertEqual(r.steps, 1000 - m.travel_steps)
            self.assert_sync(m, sim, zero)
            iv = sim.step_intervals()[n0:]
            steps = abs(r.steps)
            self.assertEqual(len(iv), steps - 1)
            self.assertAlmostEqual(iv[0], 1 / 100)             # starts at start_speed
            self.assertAlmostEqual(min(iv), 1 / 800)           # reaches max_speed
            # the recorded gaps are exactly the ramp delays (the last delay is never waited)
            self.assertAlmostEqual(sum(iv), sum(intervals(steps, 100, 800, 2000)[:-1]), places=6)
            self.assertAlmostEqual(sum(iv), move_time(steps, 100, 800, 2000), delta=0.02)
            self.assertGreater(iv[-1], iv[len(iv) // 2])       # decelerates at the end
        finally:
            m.close()

    def test_speed_and_accel_overrides(self):
        m, sim, clk, zero = calibrated()
        try:
            n0 = len(sim.step_log)
            m.move_to(2000, speed=300, accel=500).result(30)
            iv = sim.step_intervals()[n0:]
            self.assertAlmostEqual(min(iv), 1 / 300)
            # the speed can never exceed the configured max_speed
            n1 = len(sim.step_log)
            m.move_to(0, speed=99999).result(30)
            self.assertAlmostEqual(min(sim.step_intervals()[n1:]), 1 / 800)
        finally:
            m.close()

    def test_soft_limits(self):
        m, sim, clk, zero = calibrated()
        try:
            for target in (-1, m.travel_steps + 1):
                with self.assertRaises(LimitError):
                    m.move_to(target)
            f = m.move_to(0)
            g = m.move_by(-10)                                # relative: validated when it runs
            self.assertTrue(f.result(30).ok)
            r = g.result(30)
            self.assertEqual(r.status, MoveStatus.REFUSED)
            self.assertEqual(m.state, State.IDLE)             # a refusal is not a fault
        finally:
            m.close()

    def test_normal_moves_never_touch_the_switches(self):
        m, sim, clk, zero = calibrated()
        try:
            m.set_work_range(200, 4000)
            n0 = len(sim.step_log)
            for _ in range(3):
                self.assertTrue(m.go_start().result(30).ok)
                self.assertTrue(m.go_end().result(30).ok)
            self.assertEqual(m.position, 4000)
            positions = [p for _, p in sim.step_log[n0:]]
            self.assertGreaterEqual(min(positions) - zero, 200)           # work range, not 0
            self.assertLessEqual(max(positions) - zero, m.travel_steps)   # never past the clearance
            self.assertLess(max(positions), sim.end_trigger)
            self.assertEqual(sim.collisions, 0)
            self.assert_sync(m, sim, zero)
        finally:
            m.close()

    def test_work_range_validation_and_teaching(self):
        m, sim, clk, zero = calibrated()
        try:
            for bad in ((-1, 100), (100, 100), (200, 100), (0, m.travel_steps + 1)):
                with self.assertRaises(LimitError):
                    m.set_work_range(*bad)
            m.move_to(300).result(30)
            self.assertEqual(m.teach_start(), 300)
            m.move_to(2500).result(30)
            self.assertEqual(m.teach_end(), 2500)
            m.move_to(0).result(30)
            m.go_end().result(30)
            self.assertEqual(m.position, 2500)
            m.go_start().result(30)
            self.assertEqual(m.position, 300)
        finally:
            m.close()

    def test_defaults_for_named_positions(self):
        m, sim, clk, zero = calibrated()
        try:
            m.go_start().result(30)
            self.assertEqual(m.position, 0)
            m.go_end().result(30)
            self.assertEqual(m.position, m.travel_steps)
        finally:
            m.close()

    def test_go_end_needs_calibration(self):
        m, sim, clk, zero = homed()
        try:
            with self.assertRaises(NotCalibratedError):
                m.go_end()
        finally:
            m.close()

    def test_commands_can_be_queued_before_homing_finishes(self):
        m, sim, clk = make(position=1500)
        try:
            f1 = m.calibrate()
            f2 = m.go_end()                                   # accepted: calibrate is queued
            f3 = m.move_to(1000)
            f4 = m.go_start()
            results = [f.result(60) for f in (f1, f2, f3, f4)]
            self.assertTrue(all(r.ok for r in results), results)
            self.assertEqual(m.position, 0)
            self.assertEqual(sim.collisions, 0)
        finally:
            m.close()

    def test_commands_run_in_order(self):
        m, sim, clk, zero = calibrated()
        try:
            order = []
            futures = [m.move_to(p) for p in (500, 1500, 800)]
            for i, f in enumerate(futures):
                f.add_done_callback(lambda _f, i=i: order.append(i))
            [f.result(30) for f in futures]
            self.assertEqual(order, [0, 1, 2])
            self.assertEqual(m.position, 800)
        finally:
            m.close()

    def test_wait_idle(self):
        m, sim, clk, zero = calibrated()
        try:
            m.move_to(100)
            m.move_to(2000)
            self.assertTrue(m.wait_idle(30))
            self.assertEqual(m.position, 2000)
            self.assertFalse(m.status().busy)
        finally:
            m.close()

    def test_replace_cancels_and_redirects(self):
        m, sim, clk, zero = calibrated()
        try:
            m.move_to(0).result(30)
            first = m.move_to(4000)
            holder = {}

            def hook(count, base=len(sim.step_log)):
                if count == base + 200 and "f" not in holder:
                    holder["f"] = m.move_to(1000, replace=True)
            sim.on_step = hook
            self.assertEqual(first.result(30).status, MoveStatus.CANCELLED)
            self.assertTrue(holder["f"].result(30).ok)
            self.assertEqual(m.position, 1000)
            self.assert_sync(m, sim, zero)
        finally:
            m.close()

    def test_cancelling_a_pending_future(self):
        m, sim, clk, zero = calibrated()
        try:
            m.move_to(0).result(30)
            first = m.move_to(2000)
            pending = m.move_to(500)
            sim.on_step = lambda c: pending.cancel()
            self.assertTrue(first.result(30).ok)
            self.assertTrue(pending.cancelled())
            self.assertTrue(m.wait_idle(30))
            self.assertEqual(m.position, 2000)
        finally:
            m.close()

    def test_degree_conversions(self):
        m, sim, clk = make()
        try:
            self.assertEqual(m.degrees_to_steps(360), 4096)
            self.assertAlmostEqual(m.steps_to_degrees(1024), 90.0)
        finally:
            m.close()


class StopTests(unittest.TestCase):
    def test_stop_decelerates_without_a_fault(self):
        m, sim, clk, zero = calibrated()
        try:
            m.move_to(0).result(30)
            base = len(sim.step_log)
            sim.on_step = lambda c: m.stop() if c == base + 500 else None
            n0 = len(sim.step_log)
            r = m.move_to(4000).result(30)
            self.assertEqual(r.status, MoveStatus.CANCELLED)
            self.assertEqual(m.state, State.IDLE)
            self.assertTrue(m.homed)
            self.assertEqual(sim.position - zero, m.position)
            extra = r.steps - 500
            self.assertGreater(extra, 50)                                  # it slowed down first
            self.assertLessEqual(extra, math.ceil((800 ** 2 - 100 ** 2) / (2 * 2000)) + 2)
            iv = sim.step_intervals()[n0:]
            self.assertGreater(iv[-1], 5 * min(iv))                         # ends at low speed
            self.assertTrue(m.move_to(100).result(30).ok)                   # and keeps working
        finally:
            m.close()

    def test_stop_cancels_queued_commands(self):
        m, sim, clk, zero = calibrated()
        try:
            m.move_to(0).result(30)
            base = len(sim.step_log)
            sim.on_step = lambda c: m.stop() if c == base + 100 else None
            futures = [m.move_to(3000), m.move_to(1000), m.move_to(2000)]
            res = [f.result(30) for f in futures]
            self.assertTrue(all(r.status is MoveStatus.CANCELLED for r in res), res)
            self.assertEqual(res[1].steps, 0)
            self.assertEqual(sim.position - zero, m.position)
        finally:
            m.close()

    def test_emergency_stop_cuts_the_coils_and_requires_homing(self):
        m, sim, clk, zero = calibrated()
        try:
            m.move_to(0).result(30)
            base = len(sim.step_log)
            sim.on_step = lambda c: m.emergency_stop("test") if c == base + 50 else None
            r = m.move_to(3000).result(30)
            self.assertEqual(r.status, MoveStatus.EMERGENCY_STOP)
            self.assertEqual(r.steps, 50)                      # no deceleration
            self.assertEqual(m.state, State.FAULT)
            self.assertFalse(m.homed)
            self.assertFalse(sim.energized)
            self.assertEqual(m.fault.reason, "emergency_stop")
            with self.assertRaises(FaultError):
                m.move_to(10)
            m.clear_fault()
            self.assertEqual(m.state, State.IDLE)
            self.assertFalse(m.homed)
            with self.assertRaises(NotHomedError):
                m.move_to(10)
            self.assertTrue(m.home().result(30).ok)
        finally:
            m.close()

    def test_clear_fault_can_keep_the_position(self):
        m, sim, clk, zero = calibrated()
        try:
            m.emergency_stop("test")
            m.clear_fault(keep_position=True)
            self.assertTrue(m.homed)
            self.assertTrue(m.move_to(100).result(30).ok)
        finally:
            m.close()

    def test_unexpected_limit_stops_and_faults(self):
        m, sim, clk, zero = calibrated()
        try:
            m.move_to(1000).result(30)
            sim.position += 400                                 # the carriage "lost" 400 steps
            r = m.move_to(m.travel_steps - 20).result(30)
            self.assertEqual(r.status, MoveStatus.LIMIT_HIT)
            self.assertIn("end", r.message)
            self.assertEqual(m.state, State.FAULT)
            self.assertFalse(m.homed)
            self.assertFalse(sim.energized)
            self.assertEqual(sim.collisions, 0)                 # stopped by the switch, not the hard stop
            self.assertEqual(m.fault.reason, "unexpected_limit")
        finally:
            m.close()

    def test_driving_into_an_active_switch_is_refused(self):
        m, sim, clk, zero = calibrated()
        try:
            sim.position = sim.end_trigger + 10                  # sitting on the end switch
            writes = sim.writes
            r = m.move_by(-10).result(30)                        # moving away is fine
            self.assertTrue(r.ok)
            sim.position = sim.end_trigger + 10
            m.move_to(0).result(30)
            sim.position = sim.end_trigger + 10
            r = m.jog(+5).result(30)                             # toward the active switch
            self.assertEqual(r.status, MoveStatus.LIMIT_HIT)
            self.assertEqual(r.steps, 0)
            self.assertEqual(m.state, State.FAULT)
        finally:
            m.close()

    def test_backend_error_is_a_safe_fault(self):
        m, sim, clk, zero = calibrated()
        try:
            sim.fail_after_writes = sim.writes + 20
            r = m.move_to(0).result(30)
            self.assertEqual(r.status, MoveStatus.FAULT)
            self.assertEqual(m.fault.reason, "backend_error")
            self.assertEqual(m.state, State.FAULT)
            self.assertFalse(m.homed)
        finally:
            m.close()


class LifecycleTests(unittest.TestCase):
    def test_close(self):
        m, sim, clk = make()
        m.close()
        self.assertTrue(sim.closed)
        self.assertFalse(sim.energized)
        self.assertFalse(m._thread.is_alive())
        self.assertEqual(m.state, State.CLOSED)
        with self.assertRaises(ClosedError):
            m.jog(10)
        m.close()                                               # idempotent

    def test_context_manager_stops_and_closes(self):
        cfg = fast_cfg()
        clk = VirtualClock()
        sim = SimBackend(cfg, clk.now)
        with Stepper28BYJ48(cfg, sim, clock=clk.now, sleep=clk.sleep) as m:
            f = m.jog(100000)
            sim.on_step = lambda c: None
        self.assertTrue(sim.closed)
        self.assertTrue(f.done())
        self.assertFalse(m._thread.is_alive())

    def test_events(self):
        m, sim, clk = make()
        events = []
        m.add_listener(lambda e: events.append(e.kind))
        try:
            m.calibrate().result(30)
            m.wait_idle(30)
            self.assertEqual(events[0], "move_started")
            self.assertIn("homed", events)
            self.assertLess(events.index("homed"), events.index("calibrated"))
            self.assertEqual(events[-1], "move_done")
            m.emergency_stop("x")
            self.assertIn("fault", events)
            m.clear_fault()
            self.assertEqual(events[-1], "fault_cleared")
        finally:
            m.close()

    def test_a_failing_listener_does_not_stop_the_driver(self):
        m, sim, clk = make()

        def bad(event):
            raise RuntimeError("boom")
        m.add_listener(bad)
        try:
            self.assertTrue(m.jog(50).result(30).ok)
            self.assertTrue(m.jog(-50).result(30).ok)
        finally:
            m.close()

    def test_hold_when_idle(self):
        for hold in (False, True):
            m, sim, clk = make(fast_cfg(hold_when_idle=hold))
            m.jog(40).result(30)
            m.wait_idle(30)
            self.assertEqual(sim.energized, hold)
            m.close()

    def test_stall_resynchronises_instead_of_bursting(self):
        cfg = fast_cfg()
        clk = VirtualClock()
        sim = SimBackend(cfg, clk.now)
        calls = {"n": 0}

        def sleep(dt):
            calls["n"] += 1
            clk.sleep(dt + (0.5 if calls["n"] == 60 else 0.0))   # the OS "freezes" the thread once

        m = Stepper28BYJ48(cfg, sim, clock=clk.now, sleep=sleep)
        try:
            m.jog(300).result(30)
            self.assertEqual(m.stats.late_steps, 1)
            self.assertGreaterEqual(m.stats.max_late_s, 0.4)
            iv = sim.step_intervals()
            self.assertTrue(all(v < 0.05 for v in iv[70:]))      # no burst of back-to-back steps
            self.assertGreater(min(iv[61:70]), 1e-4)
        finally:
            m.close()

    def test_realtime_priority_failure_is_not_fatal(self):
        m, sim, clk = make(fast_cfg(realtime_priority=10))
        try:
            self.assertTrue(m.jog(20).result(30).ok)
        finally:
            m.close()


class PersistenceTests(unittest.TestCase):
    def test_travel_and_work_range_survive_a_restart(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "stepper.json")
            clk = VirtualClock()
            cfg = fast_cfg(state_file=path)
            sim = SimBackend(cfg, clk.now)
            m = Stepper28BYJ48(cfg, sim, clock=clk.now, sleep=clk.sleep)
            m.calibrate().result(30)
            m.set_work_range(250, 3900)
            travel = m.travel_steps
            m.close()
            with open(path) as f:
                saved = json.load(f)
            self.assertEqual(saved["travel_steps"], travel)
            self.assertEqual((saved["start_steps"], saved["end_steps"]), (250, 3900))

            clk2 = VirtualClock()
            sim2 = SimBackend(cfg, clk2.now, position=2222)
            m2 = Stepper28BYJ48(cfg, sim2, clock=clk2.now, sleep=clk2.sleep)
            try:
                self.assertEqual(m2.travel_steps, travel)
                self.assertFalse(m2.homed)                           # position is never trusted
                with self.assertRaises(NotHomedError):
                    m2.go_end()
                self.assertTrue(m2.home().result(30).ok)
                self.assertTrue(m2.go_end().result(30).ok)           # no recalibration needed
                self.assertEqual(m2.position, 3900)
                self.assertEqual(m2.start_steps, 250)
            finally:
                m2.close()
            self.assertEqual(os.listdir(d), ["stepper.json"])        # no temp files left behind

    def test_state_for_another_step_mode_is_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "s.json")
            with open(path, "w") as f:
                json.dump({"version": 1, "step_mode": "half", "travel_steps": 999,
                           "start_steps": 1, "end_steps": 2}, f)
            cfg = fast_cfg(state_file=path, step_mode="full")
            sim = SimBackend(cfg)
            m = Stepper28BYJ48(cfg, sim)
            try:
                self.assertIsNone(m.travel_steps)
            finally:
                m.close()

    def test_corrupt_state_file_is_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "s.json")
            with open(path, "w") as f:
                f.write("{not json")
            cfg = fast_cfg(state_file=path)
            m = Stepper28BYJ48(cfg, SimBackend(cfg))
            try:
                self.assertIsNone(m.travel_steps)
            finally:
                m.close()


class ConcurrencyTests(unittest.TestCase):
    """Real clock and real threads."""

    def test_calls_return_immediately_and_the_main_thread_keeps_running(self):
        m, sim = make_real(position=1500)
        try:
            t0 = time.monotonic()
            f = m.jog(1500)                                  # ~0.8 s of motion
            self.assertLess(time.monotonic() - t0, 0.05)
            self.assertFalse(f.done())
            ticks = 0
            while not f.done():
                ticks += 1
                time.sleep(0.005)                            # the "application" does other work
            self.assertGreater(ticks, 20)
            self.assertTrue(f.result().ok)
            self.assertEqual(sim.position, 3000)
        finally:
            m.close()

    def test_emergency_stop_from_another_thread(self):
        m, sim = make_real(position=1500)
        try:
            f = m.jog(3000)
            time.sleep(0.15)
            t0 = time.monotonic()
            threading.Thread(target=m.emergency_stop, args=("from a thread",)).start()
            r = f.result(5)
            self.assertLess(time.monotonic() - t0, 0.2)
            self.assertEqual(r.status, MoveStatus.EMERGENCY_STOP)
            self.assertLess(abs(r.steps), 3000)
            self.assertFalse(sim.energized)
            self.assertEqual(sim.position - 1500, m.position)
        finally:
            m.close()

    def test_stop_from_another_thread_decelerates(self):
        m, sim = make_real(position=1500)
        try:
            f = m.jog(3000)
            time.sleep(0.15)
            m.stop()
            r = f.result(5)
            self.assertEqual(r.status, MoveStatus.CANCELLED)
            self.assertEqual(m.state, State.IDLE)
            self.assertEqual(sim.position - 1500, m.position)
        finally:
            m.close()

    def test_status_can_be_polled_while_moving(self):
        m, sim = make_real(position=1500)
        errors, seen = [], []

        def poll(f):
            try:
                while not f.done():
                    seen.append(m.status().position)
                    time.sleep(0.001)
            except Exception as exc:                          # pragma: no cover
                errors.append(exc)

        try:
            f = m.jog(800)
            t = threading.Thread(target=poll, args=(f,))
            t.start()
            f.result(10)
            t.join()
            self.assertEqual(errors, [])
            self.assertGreater(len(seen), 50)
            self.assertEqual(seen, sorted(seen))             # monotonic while moving forward
            self.assertLessEqual(seen[-1], 800)
            self.assertEqual(m.position, 800)
        finally:
            m.close()

    def test_many_threads_can_submit_commands(self):
        m, sim = make_real(position=1500)
        results = []

        def worker(delta):
            results.append(m.jog(delta).result(10))

        try:
            threads = [threading.Thread(target=worker, args=(d,)) for d in (100, -50, 30, 70, -20)]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
            self.assertEqual(len(results), 5)
            self.assertTrue(all(r.ok for r in results))
            self.assertEqual(sim.position, 1500 + 130)       # commands are serialised, none lost
        finally:
            m.close()


if __name__ == "__main__":
    unittest.main()
