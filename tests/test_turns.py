"""Tests for cut width / overlap and the swing-out turns between passes."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import create_waypoint_list as planner  # noqa: E402
from lawnmower.coverage import plan_coverage  # noqa: E402
from lawnmower.field import Field, FieldError, MAX_CUT_WIDTH_M, parse_field  # noqa: E402
from lawnmower.geometry import to_gps, to_meters  # noqa: E402
from lawnmower.polygon import distance_to_boundary, point_in_polygon  # noqa: E402
from lawnmower.preview import plan_geojson, plan_svg  # noqa: E402

REF = (37.0, -122.0)
SLACK = 0.03


def gps(points):
    return tuple(to_gps(x, y, *REF) for x, y in points)


def metres(waypoints):
    return [to_meters(lat, lon, *REF) for lat, lon in waypoints]


def rect(w, h, x0=0.0, y0=0.0):
    return [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]


def samples(points, per_segment=16):
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        for k in range(per_segment + 1):
            t = k / per_segment
            yield x1 + t * (x2 - x1), y1 + t * (y2 - y1)


PENTAGON = [(0, 0), (20, 0), (28, 12), (10, 22), (-6, 12)]
TRIANGLE = [(0, 0), (30, 0), (12, 14)]
L_SHAPE = [(0, 0), (30, 0), (30, 10), (10, 10), (10, 25), (0, 25)]


def plan(boundary, zones=(), **options):
    return plan_coverage(gps(boundary), options.pop("spacing", 1.0), exclusions=[gps(z) for z in zones], **options)


class CutWidthTests(unittest.TestCase):
    BOUNDARY = [[37.0, -122.0], [37.0, -121.9999], [37.0001, -121.9999], [37.0001, -122.0]]

    def test_spacing_is_the_cut_width_minus_the_overlap(self):
        field = parse_field({"boundary": self.BOUNDARY, "cut_width_m": 0.9, "overlap_m": 0.1})
        self.assertAlmostEqual(field.pass_spacing_m, 0.8)
        self.assertEqual(field.cut_width_m, 0.9)
        self.assertEqual(field.overlap_m, 0.1)

    def test_overlap_defaults_to_none(self):
        field = parse_field({"boundary": self.BOUNDARY, "cut_width_m": 0.9})
        self.assertEqual(field.pass_spacing_m, 0.9)
        self.assertEqual(field.overlap_m, 0.0)

    def test_without_a_cut_width_nothing_changes(self):
        field = parse_field({"boundary": self.BOUNDARY, "pass_spacing_m": 1.5})
        self.assertEqual(field.pass_spacing_m, 1.5)
        self.assertIsNone(field.cut_width_m)

    def test_giving_both_spacing_and_cut_width_is_refused(self):
        with self.assertRaises(FieldError) as raised:
            parse_field({"boundary": self.BOUNDARY, "cut_width_m": 0.9, "pass_spacing_m": 1.0})
        self.assertIn("either", str(raised.exception))

    def test_overlap_needs_a_cut_width(self):
        with self.assertRaises(FieldError) as raised:
            parse_field({"boundary": self.BOUNDARY, "overlap_m": 0.1})
        self.assertIn("cut_width_m", str(raised.exception))

    def test_bad_cut_widths_and_overlaps_are_refused(self):
        for key, bad in (("cut_width_m", 0), ("cut_width_m", -1), ("cut_width_m", MAX_CUT_WIDTH_M + 1),
                         ("cut_width_m", "wide"), ("cut_width_m", True)):
            with self.subTest(key=key, bad=bad):
                with self.assertRaises(FieldError):
                    parse_field({"boundary": self.BOUNDARY, key: bad})
        for bad in (-0.1, "a bit", True):
            with self.subTest(overlap=bad):
                with self.assertRaises(FieldError):
                    parse_field({"boundary": self.BOUNDARY, "cut_width_m": 0.9, "overlap_m": bad})

    def test_an_overlap_that_leaves_no_spacing_is_refused(self):
        with self.assertRaises(FieldError) as raised:
            parse_field({"boundary": self.BOUNDARY, "cut_width_m": 0.9, "overlap_m": 0.85})
        self.assertIn("spacing", str(raised.exception))

    def test_the_planner_uses_the_spacing(self):
        corners = gps(rect(30, 10))
        wide = parse_field({"boundary": [list(p) for p in corners], "cut_width_m": 2.0, "overlap_m": 0.0})
        narrow = parse_field({"boundary": [list(p) for p in corners], "cut_width_m": 2.0, "overlap_m": 1.0})
        self.assertLess(len(planner.plan_field(wide)), len(planner.plan_field(narrow)))
        self.assertEqual(len(planner.plan_field(narrow)),
                         len(planner.plan_field(Field(boundary=corners, pass_spacing_m=1.0))))


class SwingOutTests(unittest.TestCase):
    def test_a_swing_out_adds_two_waypoints_at_every_turn(self):
        base = plan(rect(30, 12), inset_m=2.0)
        turned = plan(rect(30, 12), inset_m=2.0, turn_depth_m=1.5)
        rows = len(base) // 2
        self.assertEqual(len(turned), len(base) + 2 * (rows - 1))

    def test_the_swing_goes_exactly_the_depth_beyond_the_end_of_the_pass(self):
        points = metres(plan(rect(30, 12), inset_m=2.0, turn_depth_m=1.5, heading_deg=90))
        # Pass: (2, y) -> (28, y); then (29.5, y), (29.5, y + 1), then the next pass starts at (28, y + 1).
        first_end = points[1]
        self.assertAlmostEqual(first_end[0], 28.0, places=2)
        self.assertAlmostEqual(points[2][0], 29.5, places=2)
        self.assertAlmostEqual(points[2][1], first_end[1], places=2)
        self.assertAlmostEqual(points[3][0], 29.5, places=2)
        self.assertAlmostEqual(points[3][1], first_end[1] + 1.0, places=2)
        self.assertAlmostEqual(points[4][0], 28.0, places=2)

    def test_the_swing_alternates_sides_with_the_passes(self):
        points = metres(plan(rect(30, 12), inset_m=2.0, turn_depth_m=1.5, heading_deg=90))
        xs = {round(p[0], 1) for p in points}
        self.assertEqual(xs, {0.5, 2.0, 28.0, 29.5})  # swings at 0.5 and 29.5, passes at 2 and 28

    def test_turn_depth_zero_changes_nothing(self):
        self.assertEqual(plan(rect(30, 12), inset_m=2.0), plan(rect(30, 12), inset_m=2.0, turn_depth_m=0.0))

    def test_the_whole_path_stays_inside_the_real_boundary(self):
        for name, shape in (("rectangle", rect(30, 12)), ("pentagon", PENTAGON), ("L", L_SHAPE), ("triangle", TRIANGLE)):
            with self.subTest(name):
                for point in samples(metres(plan(shape, inset_m=2.0, turn_depth_m=1.5))):
                    self.assertTrue(point_in_polygon(point, shape) or distance_to_boundary(point, shape) < SLACK,
                                    f"left the field at {point}")

    def test_slanted_sides_get_swings_too(self):
        base, turned = plan(PENTAGON, inset_m=2.0, spacing=1.5), plan(PENTAGON, inset_m=2.0, spacing=1.5, turn_depth_m=1.5)
        self.assertGreater(len(turned), len(base))

    def test_a_turn_without_room_is_left_as_it_was(self):
        # A sharp triangle with a thin margin: some turns near the tip cannot swing out without leaving the field.
        base = plan(TRIANGLE, inset_m=1.0)
        turned = plan(TRIANGLE, inset_m=1.0, turn_depth_m=1.0)
        self.assertGreaterEqual(len(turned), len(base))
        for point in samples(metres(turned)):
            self.assertTrue(point_in_polygon(point, TRIANGLE) or distance_to_boundary(point, TRIANGLE) < SLACK)

    def test_swings_never_enter_a_no_go_zone(self):
        field, zone = rect(30, 12), rect(4, 4, 13, 4)
        for point in samples(metres(plan(field, [zone], inset_m=1.5, turn_depth_m=1.5, spacing=1.0))):
            inside = point_in_polygon(point, zone) and distance_to_boundary(point, zone) > SLACK
            self.assertFalse(inside, f"entered the zone at {point}")

    def test_every_swing_with_a_zone_stays_out_of_the_zone_itself(self):
        field, zone = rect(30, 12), rect(4, 4, 13, 4)
        base = metres(plan(field, [zone], inset_m=1.5))
        turned = metres(plan(field, [zone], inset_m=1.5, turn_depth_m=1.5))
        self.assertGreater(len(turned), len(base))
        for point in turned:
            self.assertGreaterEqual(distance_to_boundary(point, zone), -SLACK)
            self.assertFalse(point_in_polygon(point, zone) and distance_to_boundary(point, zone) > SLACK)

    def test_any_heading_works(self):
        for heading in (0, 30, 90, 135):
            with self.subTest(heading=heading):
                for point in samples(metres(plan(rect(30, 20), inset_m=2.0, turn_depth_m=1.5, heading_deg=heading))):
                    self.assertTrue(point_in_polygon(point, rect(30, 20)) or distance_to_boundary(point, rect(30, 20)) < SLACK)

    def test_a_depth_beyond_the_margin_is_refused(self):
        with self.assertRaises(FieldError) as raised:
            plan(rect(30, 12), inset_m=1.0, turn_depth_m=1.5)
        self.assertIn("inset_m", str(raised.exception))
        with self.assertRaises(FieldError):
            plan(rect(30, 12), inset_m=1.0, turn_depth_m=-0.5)

    def test_without_a_margin_there_is_no_room_to_swing(self):
        with self.assertRaises(FieldError):
            plan(rect(30, 12), turn_depth_m=0.5)

    def test_a_field_with_swings_goes_through_the_polygon_planner(self):
        corners = gps(rect(30, 12))
        field = Field(boundary=corners, inset_m=2.0, turn_depth_m=1.5)
        self.assertEqual(planner.plan_field(field), plan_coverage(corners, 1.0, inset_m=2.0, turn_depth_m=1.5))

    def test_the_plan_is_the_same_every_time(self):
        self.assertEqual(plan(PENTAGON, inset_m=2.0, turn_depth_m=1.5), plan(PENTAGON, inset_m=2.0, turn_depth_m=1.5))


class TurnSettingTests(unittest.TestCase):
    BOUNDARY = [[37.0, -122.0], [37.0, -121.9999], [37.0001, -121.9999], [37.0001, -122.0]]

    def test_turn_depth_is_read(self):
        field = parse_field({"boundary": self.BOUNDARY, "inset_m": 2, "turn_depth_m": 1.5})
        self.assertEqual(field.turn_depth_m, 1.5)
        self.assertEqual(parse_field({"boundary": self.BOUNDARY}).turn_depth_m, 0.0)

    def test_a_depth_more_than_the_inset_is_refused_in_the_file(self):
        with self.assertRaises(FieldError) as raised:
            parse_field({"boundary": self.BOUNDARY, "inset_m": 1, "turn_depth_m": 1.5})
        self.assertIn("margin", str(raised.exception))

    def test_bad_depths_are_refused(self):
        for bad in (-1, "deep", True, 99):
            with self.subTest(bad=bad):
                with self.assertRaises(FieldError):
                    parse_field({"boundary": self.BOUNDARY, "inset_m": 5, "turn_depth_m": bad})


class PreviewTests(unittest.TestCase):
    def test_the_summary_line_shows_cut_width_overlap_and_turns(self):
        field = parse_field({"boundary": [list(p) for p in gps(rect(30, 12))], "cut_width_m": 0.9, "overlap_m": 0.1,
                             "inset_m": 2, "turn_depth_m": 1.5})
        waypoints = planner.plan_field(field)
        svg = plan_svg(field, waypoints)
        self.assertIn("cut 0.9 m, overlap 0.1 m", svg)
        self.assertIn("turns 1.5 m out", svg)
        properties = plan_geojson(field, waypoints)["features"][0]["properties"]
        self.assertEqual(properties["turn_depth_m"], 1.5)
        self.assertEqual(properties["cut_width_m"], 0.9)

    def test_a_plain_field_shows_neither(self):
        field = Field(boundary=gps(rect(30, 12)))
        svg = plan_svg(field, planner.plan_field(field))
        self.assertNotIn("cut ", svg)
        self.assertNotIn("turns", svg)


if __name__ == "__main__":
    unittest.main()
