"""Tests for the mission logic (waypoint advance, row projection, finishing) with scripted sensor input."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lawnmower import geometry, mission  # noqa: E402
from lawnmower.mission import Mission, MissionConfig  # noqa: E402

REF = (37.0, -122.0)


def at(east_m, north_m):
    return geometry.to_gps(east_m, north_m, *REF)


def run(m, fixes, heading=0.0, t0=0.0, dt=1.0):
    """Feed one fix per step with no acceleration; return the commands."""
    commands = []
    for i, fix in enumerate(fixes):
        commands.append(m.step(t0 + (i + 1) * dt, heading, 0.0, 0.0, fix))
    return commands


class MissionTests(unittest.TestCase):
    def test_needs_a_waypoint(self):
        with self.assertRaises(ValueError):
            Mission([])

    def test_waits_until_there_is_a_fix_and_leaves_the_motors_alone(self):
        m = Mission([at(0, 20)])
        cmd = m.step(1.0, 0.0, 0.0, 0.0, None)
        self.assertEqual(cmd.reason, mission.WAITING_FOR_FIX)
        self.assertIsNone(cmd.left)
        self.assertIsNone(cmd.right)
        self.assertFalse(cmd.done)

    def test_drives_straight_when_pointing_at_the_target(self):
        m = Mission([at(0, 20)])
        cmd = run(m, [at(0, 0)], heading=0.0)[0]
        self.assertEqual(cmd.reason, mission.STEERING)
        self.assertEqual((cmd.left, cmd.right), (100, 100))

    def test_turns_right_when_the_target_is_to_the_right(self):
        m = Mission([at(20, 0)])
        cmd = run(m, [at(0, 0)], heading=0.0)[0]   # target due east, rover facing north
        self.assertEqual((cmd.left, cmd.right), (100, 80))

    def test_turns_left_when_the_target_is_to_the_left(self):
        m = Mission([at(-20, 0)])
        cmd = run(m, [at(0, 0)], heading=0.0)[0]
        self.assertEqual((cmd.left, cmd.right), (80, 100))

    def test_settings_come_from_the_config(self):
        m = Mission([at(20, 0)], MissionConfig(forward_speed=60, turn_speed=30))
        cmd = run(m, [at(0, 0)], heading=0.0)[0]
        self.assertEqual((cmd.left, cmd.right), (60, 30))

    def test_reaching_a_waypoint_moves_to_the_next_without_changing_the_motors(self):
        m = Mission([at(0, 0), at(0, 20)])
        cmd = run(m, [at(0, 0)])[0]   # starts on the first waypoint
        self.assertEqual(cmd.reason, mission.WAYPOINT_REACHED)
        self.assertIsNone(cmd.left)
        self.assertEqual(m.index, 1)
        self.assertEqual(m.prev_waypoint, at(0, 0))

    def test_finishes_after_the_last_waypoint(self):
        m = Mission([at(0, 0)])
        cmd = run(m, [at(0, 0)])[0]
        self.assertTrue(cmd.done)
        self.assertEqual(cmd.reason, mission.PATH_COMPLETE)
        self.assertIn("All waypoints reached! Path complete.", cmd.messages)

    def test_stops_at_the_time_limit(self):
        m = Mission([at(0, 20)], MissionConfig(runtime_limit_s=10), now=100.0)
        cmd = m.step(110.0, 0.0, 0.0, 0.0, at(0, 0))
        self.assertTrue(cmd.done)
        self.assertEqual(cmd.reason, mission.TIME_LIMIT)

    def test_projects_onto_the_row_so_sideways_gps_noise_does_not_steer(self):
        # Second leg runs north along east = 0. The fix is 2 m east of the row, rover facing north,
        # so without projection the target would be off to the left; with it the rover goes straight.
        m = Mission([at(0, 0), at(0, 30)])
        run(m, [at(0, 0)])                       # reach waypoint 0
        cmd = m.step(10.0, 0.0, 0.0, 0.0, at(2, 5))
        for _ in range(20):                      # let the filter settle on the same fix
            cmd = m.step(11.0 + _, 0.0, 0.0, 0.0, at(2, 5))
        self.assertEqual((cmd.left, cmd.right), (100, 100))

    def test_walks_a_whole_path_to_the_end(self):
        points = [at(0, 0), at(0, 10), at(2, 10)]
        m = Mission(points)
        reached = []
        for i in range(10):
            fix = points[min(len(reached), len(points) - 1)]
            cmd = m.step(i + 1.0, 0.0, 0.0, 0.0, fix)
            if cmd.reason in (mission.WAYPOINT_REACHED, mission.PATH_COMPLETE):
                reached.append(cmd.reason)
            if cmd.done:
                break
        self.assertEqual(reached[-1], mission.PATH_COMPLETE)

    def test_start_messages(self):
        m = Mission([at(0, 0), at(0, 10)])
        self.assertEqual(m.start_messages()[0], "Path loaded: 2 waypoints.")
        self.assertTrue(m.start_messages()[1].startswith("Navigating to waypoint 0: ("))


if __name__ == "__main__":
    unittest.main()
