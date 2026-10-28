"""Tests for the field checks that run before any waypoints are planned."""
import math
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import create_waypoint_list as planner  # noqa: E402
import field_config  # noqa: E402
from lawnmower.field import FieldError, MAX_EXTENT_M, MAX_WAYPOINTS, check_corners  # noqa: E402

REF_LAT, REF_LON = 37.0, -122.0


def corners_m(points):
    """(x, y) metre points as (lat, lon) corners."""
    return [planner.to_gps(x, y, REF_LAT, REF_LON) for x, y in points]


def rectangle(east_m, north_m):
    return corners_m([(0, 0), (east_m, 0), (east_m, north_m), (0, north_m)])


class GoodFieldTests(unittest.TestCase):
    def test_a_normal_lawn_passes_and_reports_the_waypoint_count(self):
        self.assertEqual(check_corners(rectangle(30, 10.5), 1.0), 24)  # 12 passes, 2 waypoints each

    def test_the_example_env_field_passes(self):
        corners = [(37.0, -122.0), (37.0, -122.0001), (37.0001, -122.0001), (37.0001, -122.0)]
        check_corners(corners, 1.0)

    def test_corner_order_does_not_matter(self):
        corners = rectangle(30, 10.5)
        self.assertEqual(check_corners(corners, 1.0), check_corners(list(reversed(corners)), 1.0))

    def test_a_slanted_four_sided_field_passes(self):
        check_corners(corners_m([(0, 0), (30, 0), (20, 10), (5, 10)]), 1.0)

    def test_the_biggest_allowed_field_is_still_inside_the_waypoint_limit(self):
        side = MAX_EXTENT_M - 1
        self.assertLess(check_corners(rectangle(side, side), 1.0), MAX_WAYPOINTS)


class TypoTests(unittest.TestCase):
    """The mistake the README describes: -86 for -85 in one longitude."""

    def test_a_longitude_off_by_one_degree_is_refused(self):
        corners = rectangle(30, 10.5)
        lat, _ = corners[1]
        corners[1] = (lat, REF_LON - 1.0)
        with self.assertRaises(FieldError) as raised:
            check_corners(corners, 1.0)
        self.assertIn("km", str(raised.exception))
        self.assertIn("mistyped", str(raised.exception))

    def test_planning_refuses_it_before_generating_anything(self):
        corners = rectangle(30, 10.5)
        corners[1] = (corners[1][0], REF_LON - 1.0)
        with self.assertRaises(FieldError):
            planner.create_waypoint_list(*corners)

    def test_a_field_just_over_the_size_limit_is_refused(self):
        with self.assertRaises(FieldError):
            check_corners(rectangle(MAX_EXTENT_M + 20, 10), 1.0)


class ShapeTests(unittest.TestCase):
    def test_three_corners_is_not_a_field(self):
        with self.assertRaises(FieldError) as raised:
            check_corners(rectangle(30, 10)[:3], 1.0)
        self.assertIn("4 corners", str(raised.exception))

    def test_two_corners_at_the_same_point_are_refused(self):
        corners = rectangle(30, 10)
        corners[1] = corners[0]
        with self.assertRaises(FieldError) as raised:
            check_corners(corners, 1.0)
        self.assertIn("A and B", str(raised.exception))

    def test_four_points_in_a_line_are_refused(self):
        with self.assertRaises(FieldError) as raised:
            check_corners(corners_m([(0, 0), (10, 0), (20, 0), (30, 0)]), 1.0)
        self.assertIn("line", str(raised.exception))

    def test_a_sliver_with_almost_no_width_is_refused(self):
        with self.assertRaises(FieldError):
            check_corners(corners_m([(0, 0), (30, 0), (30, 0.02), (0, 0.02)]), 1.0)


class CoordinateTests(unittest.TestCase):
    def test_latitude_out_of_range(self):
        corners = rectangle(30, 10)
        corners[2] = (91.0, REF_LON)
        with self.assertRaises(FieldError) as raised:
            check_corners(corners, 1.0)
        self.assertIn("Corner C", str(raised.exception))

    def test_longitude_out_of_range(self):
        corners = rectangle(30, 10)
        corners[0] = (REF_LAT, -181.0)
        with self.assertRaises(FieldError) as raised:
            check_corners(corners, 1.0)
        self.assertIn("Corner A", str(raised.exception))

    def test_not_a_number(self):
        corners = rectangle(30, 10)
        corners[3] = (math.nan, REF_LON)
        with self.assertRaises(FieldError):
            check_corners(corners, 1.0)


class SpacingTests(unittest.TestCase):
    def test_zero_spacing_is_refused(self):
        with self.assertRaises(FieldError):
            check_corners(rectangle(30, 10), 0.0)

    def test_negative_spacing_is_refused(self):
        with self.assertRaises(FieldError):
            check_corners(rectangle(30, 10), -1.0)

    def test_spacing_so_fine_that_the_plan_is_huge_is_refused(self):
        with self.assertRaises(FieldError) as raised:
            check_corners(rectangle(400, 400), 0.1)  # 4,000 passes, 8,000 waypoints
        self.assertIn("waypoints", str(raised.exception))

    def test_the_count_matches_what_the_planner_really_generates(self):
        corners = rectangle(30, 10.5)
        self.assertEqual(check_corners(corners, 1.0), len(planner.create_waypoint_list(*corners, spacing_m=1.0)))
        slanted = corners_m([(0, 0), (30, 0), (20, 10), (5, 10)])
        self.assertEqual(check_corners(slanted, 1.0), len(planner.create_waypoint_list(*slanted, spacing_m=1.0)))


class CornerSettingTests(unittest.TestCase):
    def test_a_corner_that_is_not_a_pair_gives_a_readable_message(self):
        with mock.patch.dict(os.environ, {"CORNER_A": "37.0"}):
            with self.assertRaises(SystemExit) as raised:
                field_config._corner("CORNER_A", {})
        self.assertIn("lat,lon", str(raised.exception))

    def test_a_corner_with_text_in_it_gives_a_readable_message(self):
        with mock.patch.dict(os.environ, {"CORNER_A": "north,west"}):
            with self.assertRaises(SystemExit) as raised:
                field_config._corner("CORNER_A", {})
        self.assertIn("CORNER_A", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
