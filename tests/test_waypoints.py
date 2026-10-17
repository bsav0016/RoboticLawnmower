"""Unit tests for the parts of the planner that need no Raspberry Pi hardware."""
import math
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import create_waypoint_list as planner  # noqa: E402
import field_config  # noqa: E402

REF_LAT, REF_LON = 37.0, -122.0


def rectangle(east_m, north_m):
    """Four (lat, lon) corners of an axis-aligned rectangle, in scrambled order."""
    corners_m = [(0, 0), (east_m, north_m), (east_m, 0), (0, north_m)]
    return [planner.to_gps(x, y, REF_LAT, REF_LON) for x, y in corners_m]


class CoordinateTests(unittest.TestCase):
    def test_reference_point_is_the_origin(self):
        self.assertEqual(planner.to_meters(REF_LAT, REF_LON, REF_LAT, REF_LON), (0.0, 0.0))

    def test_ten_metres_north_is_about_ten_metres(self):
        lat, lon = planner.to_gps(0, 10, REF_LAT, REF_LON)
        x, y = planner.to_meters(lat, lon, REF_LAT, REF_LON)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 10.0, places=6)

    def test_round_trip_through_meters_and_back(self):
        lat, lon = planner.to_gps(123.4, -56.7, REF_LAT, REF_LON)
        x, y = planner.to_meters(lat, lon, REF_LAT, REF_LON)
        self.assertAlmostEqual(x, 123.4, places=6)
        self.assertAlmostEqual(y, -56.7, places=6)


class OrderCornersTests(unittest.TestCase):
    def test_keeps_the_same_four_corners(self):
        corners = rectangle(30, 10)
        self.assertCountEqual(planner.order_corners(corners), corners)

    def test_result_does_not_depend_on_input_order(self):
        corners = rectangle(30, 10)
        expected = planner.order_corners(corners)
        self.assertEqual(planner.order_corners(list(reversed(corners))), expected)
        self.assertEqual(planner.order_corners(corners[1:] + corners[:1]), expected)


class LawnmowerPatternTests(unittest.TestCase):
    def passes(self, waypoints):
        """Waypoints as (x, y) metres, grouped into the pairs that make up each pass."""
        points = [planner.to_meters(lat, lon, REF_LAT, REF_LON) for lat, lon in waypoints]
        return [points[i:i + 2] for i in range(0, len(points), 2)]

    def test_wide_field_is_swept_along_its_long_side(self):
        waypoints = planner.create_waypoint_list(*rectangle(30, 10.5), spacing_m=1.0)
        passes = self.passes(waypoints)
        self.assertEqual(len(passes), 12)  # ceil(10.5 / 1) + 1
        for start, end in passes:
            self.assertAlmostEqual(abs(end[0] - start[0]), 30.0, places=4)
            self.assertAlmostEqual(end[1], start[1], places=4)

    def test_tall_field_is_swept_along_its_long_side(self):
        waypoints = planner.create_waypoint_list(*rectangle(10.5, 30), spacing_m=1.0)
        passes = self.passes(waypoints)
        self.assertEqual(len(passes), 12)
        for start, end in passes:
            self.assertAlmostEqual(abs(end[1] - start[1]), 30.0, places=4)
            self.assertAlmostEqual(end[0], start[0], places=4)

    def test_passes_are_one_spacing_apart_and_cover_the_whole_field(self):
        passes = self.passes(planner.create_waypoint_list(*rectangle(30, 10.5), spacing_m=1.0))
        rows = [start[1] for start, _ in passes]
        gaps = [b - a for a, b in zip(rows, rows[1:])]
        for gap in gaps[:-1]:
            self.assertAlmostEqual(abs(gap), 1.0, places=4)
        self.assertAlmostEqual(abs(max(rows) - min(rows)), 10.5, places=4)

    def test_direction_alternates_so_each_pass_starts_where_the_last_ended(self):
        passes = self.passes(planner.create_waypoint_list(*rectangle(30, 10.5), spacing_m=1.0))
        for (_, previous_end), (next_start, _) in zip(passes, passes[1:]):
            self.assertLess(math.hypot(next_start[0] - previous_end[0], next_start[1] - previous_end[1]), 1.1)

    def test_every_waypoint_stays_inside_the_field(self):
        for x, y in (p for pair in self.passes(planner.create_waypoint_list(*rectangle(30, 10.5))) for p in pair):
            self.assertTrue(-1e-4 <= x <= 30 + 1e-4 and -1e-4 <= y <= 10.5 + 1e-4)

    def test_wider_spacing_makes_fewer_passes(self):
        fine = planner.create_waypoint_list(*rectangle(30, 10.5), spacing_m=1.0)
        coarse = planner.create_waypoint_list(*rectangle(30, 10.5), spacing_m=3.0)
        self.assertLess(len(coarse), len(fine))

    def test_corner_order_does_not_change_the_waypoint_count(self):
        corners = rectangle(30, 10.5)
        self.assertEqual(
            len(planner.create_waypoint_list(*corners)),
            len(planner.create_waypoint_list(*reversed(corners))),
        )


class FieldConfigTests(unittest.TestCase):
    NAMES = ("CORNER_A", "CORNER_B", "CORNER_C", "CORNER_D")

    def test_reads_a_corner_from_the_environment(self):
        with mock.patch.dict(os.environ, {"CORNER_A": "37.5,-122.25"}):
            self.assertEqual(field_config._corner("CORNER_A", {}), (37.5, -122.25))

    def test_environment_wins_over_the_env_file(self):
        with mock.patch.dict(os.environ, {"CORNER_A": "1,2"}):
            self.assertEqual(field_config._corner("CORNER_A", {"CORNER_A": "3,4"}), (1.0, 2.0))

    def test_falls_back_to_the_env_file(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(field_config._corner("CORNER_A", {"CORNER_A": "3,4"}), (3.0, 4.0))

    def test_missing_corner_stops_with_a_helpful_message(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(SystemExit) as raised:
                field_config._corner("CORNER_A", {})
        self.assertIn("CORNER_A", str(raised.exception))

    def test_env_file_ignores_comments_and_blank_lines(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, ".env")
            with open(path, "w") as f:
                f.write("# comment\n\nCORNER_A = 1,2\nnot a setting\nCORNER_B=3,4\n")
            self.assertEqual(field_config._load_env_file(path), {"CORNER_A": "1,2", "CORNER_B": "3,4"})

    def test_missing_env_file_gives_no_values(self):
        self.assertEqual(field_config._load_env_file(os.path.join(tempfile.gettempdir(), "no-such-dir", ".env")), {})

    def test_load_corners_returns_all_four(self):
        values = {name: f"{i},{i + 1}" for i, name in enumerate(self.NAMES)}
        with mock.patch.dict(os.environ, values):
            self.assertEqual(field_config.load_corners(), [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0)])


if __name__ == "__main__":
    unittest.main()
