"""Tests for the failsafes: the mission stops the rover when it has no fix, is too unsure, or has left the field."""
import os
import sys
import tempfile
import unittest
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import sweep_plan  # noqa: E402
from create_waypoint_list import plan_field  # noqa: E402
from lawnmower import geometry, mission, runlog, sim  # noqa: E402
from lawnmower.field import Field  # noqa: E402
from lawnmower.mission import Mission, MissionConfig  # noqa: E402

REF = (37.0, -122.0)
SQUARE = [geometry.to_gps(x, y, *REF) for x, y in ((0, 0), (20, 0), (20, 20), (0, 20))]


def at(east_m, north_m):
    return geometry.to_gps(east_m, north_m, *REF)


class FailsafeTests(unittest.TestCase):
    def started(self, **config):
        m = Mission([at(10, 5), at(10, 15)], MissionConfig(**config), now=0.0, boundary=SQUARE)
        m.step(1.0, 0.0, 0.0, 0.0, at(10, 5))   # first fix
        return m

    def test_stops_when_there_is_no_fix_for_longer_than_the_timeout(self):
        m = self.started(no_fix_timeout_s=5.0, max_position_std_m=None)
        for t in (2, 3, 4, 5, 6):
            self.assertFalse(m.step(float(t), 0.0, 0.0, 0.0, None).done)   # 5 s silent so far: still allowed
        cmd = m.step(7.0, 0.0, 0.0, 0.0, None)
        self.assertTrue(cmd.done)
        self.assertEqual(cmd.reason, mission.NO_FIX)
        self.assertEqual((cmd.left, cmd.right), (0, 0))
        self.assertIn("no GPS fix for 6.0 s", cmd.messages[0])

    def test_a_fix_in_time_keeps_the_rover_going(self):
        m = self.started(no_fix_timeout_s=5.0, max_position_std_m=None)
        for t in (2, 3, 4, 5):
            m.step(float(t), 0.0, 0.0, 0.0, None)
        cmd = m.step(6.0, 0.0, 0.0, 0.0, at(10, 5))
        self.assertFalse(cmd.done)
        for t in (7, 8, 9):   # the clock starts again from that fix
            self.assertFalse(m.step(float(t), 0.0, 0.0, 0.0, None).done)

    def test_waiting_for_the_first_fix_is_not_a_failsafe(self):
        m = Mission([at(10, 5), at(10, 15)], MissionConfig(no_fix_timeout_s=5.0), now=0.0, boundary=SQUARE)
        cmd = m.step(60.0, 0.0, 0.0, 0.0, None)
        self.assertEqual(cmd.reason, mission.WAITING_FOR_FIX)

    def test_stops_when_the_position_is_too_uncertain(self):
        m = self.started(no_fix_timeout_s=None, max_position_std_m=8.0)
        reasons = [m.step(1.0 + 0.5 * i, 0.0, 0.0, 0.0, None).reason for i in range(1, 20)]
        self.assertIn(mission.UNCERTAIN, reasons)
        self.assertNotIn(mission.UNCERTAIN, reasons[:3])   # not straight away: it takes seconds to grow

    def test_stops_when_the_estimate_is_far_outside_the_field(self):
        m = self.started(max_outside_m=5.0)
        for i in range(10):   # fixes say the rover is 12 m east of the boundary
            cmd = m.step(2.0 + i, 0.0, 0.0, 0.0, at(32, 10))
            if cmd.done:
                break
        self.assertTrue(cmd.done)
        self.assertEqual(cmd.reason, mission.OUT_OF_BOUNDS)
        self.assertGreater(m.outside_m(), 5.0)
        self.assertIn("outside the field", cmd.messages[0])

    def test_a_little_outside_is_allowed(self):
        m = self.started(max_outside_m=5.0)
        for i in range(10):
            cmd = m.step(2.0 + i, 0.0, 0.0, 0.0, at(22, 10))   # 2 m outside
            self.assertFalse(cmd.done)

    def test_inside_the_field_is_zero_outside(self):
        m = self.started()
        self.assertEqual(m.outside_m(), 0.0)

    def test_checks_can_be_switched_off(self):
        config = MissionConfig(no_fix_timeout_s=None, max_position_std_m=None, max_outside_m=None)
        m = Mission([at(10, 5), at(10, 15)], config, now=0.0, boundary=SQUARE)
        m.step(1.0, 0.0, 0.0, 0.0, at(10, 5))
        for i in range(60):
            self.assertFalse(m.step(2.0 + i, 0.0, 0.0, 0.0, None).done)

    def test_no_boundary_means_no_outside_check(self):
        m = Mission([at(10, 5), at(10, 15)], MissionConfig(), now=0.0)
        m.step(1.0, 0.0, 0.0, 0.0, at(500, 500))
        self.assertEqual(m.outside_m(), 0.0)


FIELD = Field(boundary=tuple(SQUARE), pass_spacing_m=1.0, inset_m=2.0, turn_depth_m=1.0)


def drive(field=FIELD, seed=0, **kwargs):
    waypoints = plan_field(field)
    config = MissionConfig(runtime_limit_s=1500, turn_speed=0)
    return sim.simulate(waypoints, config, sim.RoverParams(), sweep_plan.noise_for(1.0), seed=seed,
                        boundary=field.boundary, **kwargs)


class ClosedLoopFailsafeTests(unittest.TestCase):
    def test_healthy_runs_are_never_stopped_by_a_failsafe(self):
        for seed in range(6):
            result = drive(seed=seed)
            self.assertNotIn(result.reason, mission.FAILSAFES, f"seed {seed}: {result.reason}")
            self.assertTrue(result.finished, f"seed {seed}: {result.reason}")

    def test_unplugging_the_gps_stops_the_rover_within_the_timeout(self):
        result = drive(gps_dropouts=[(100.0, 10_000.0)])
        # Whichever fires first: the filter's uncertainty reaches its 10 m limit at about the same moment as the timeout.
        self.assertIn(result.reason, (mission.NO_FIX, mission.UNCERTAIN))
        self.assertLess(result.elapsed_s - 100.0, MissionConfig().no_fix_timeout_s + 2.0)
        # ... and it stopped moving: the true track ends where the failsafe fired.
        self.assertAlmostEqual(result.track[-1][0], result.elapsed_s - 0.1, delta=0.2)

    def test_a_short_dropout_is_ridden_out(self):
        result = drive(gps_dropouts=[(100.0, 103.0)])
        self.assertNotIn(result.reason, mission.FAILSAFES)

    def test_a_drift_off_the_field_is_caught_before_it_goes_far(self):
        # A compass that reads 180 degrees out sends the rover away from every target.
        waypoints = plan_field(FIELD)
        noise = replace(sweep_plan.noise_for(1.0), compass_bias_deg=180.0)
        result = sim.simulate(waypoints, MissionConfig(runtime_limit_s=1500), sim.RoverParams(), noise, seed=0,
                              boundary=FIELD.boundary)
        self.assertEqual(result.reason, mission.OUT_OF_BOUNDS)
        east, north = result.track[-1][1], result.track[-1][2]
        # The estimate trips the limit at 5 m; the rover is a little beyond that at most, not wandering off.
        self.assertLess(max(-east, east - 20, -north, north - 20), 5.0 + 4.0)

    def test_without_the_failsafe_the_same_drift_runs_on(self):
        waypoints = plan_field(FIELD)
        noise = replace(sweep_plan.noise_for(1.0), compass_bias_deg=180.0)
        config = replace(MissionConfig(runtime_limit_s=200), max_outside_m=None)
        result = sim.simulate(waypoints, config, sim.RoverParams(), noise, seed=0, boundary=FIELD.boundary)
        self.assertNotEqual(result.reason, mission.OUT_OF_BOUNDS)

    def test_a_logged_failsafe_run_replays_to_the_same_decisions(self):
        waypoints = plan_field(FIELD)
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "run.jsonl")
            m = Mission(waypoints, MissionConfig(), now=0.0, boundary=FIELD.boundary)
            log = runlog.RunLog(path, m)
            for i in range(1, 40):
                t = float(i)
                fix = at(2, 2) if i < 5 else None
                cmd = m.step(t, 0.0, 0.0, 0.0, fix)
                log.record(m, t, 0.0, 0.0, 0.0, fix, cmd)
                if cmd.done:
                    break
            log.close()
            self.assertEqual(cmd.reason, mission.NO_FIX)
            header, steps = runlog.read_log(path)
            self.assertIsNotNone(header["boundary"])
            self.assertEqual(runlog.differences(steps, runlog.replay(header, steps)), [])


if __name__ == "__main__":
    unittest.main()
