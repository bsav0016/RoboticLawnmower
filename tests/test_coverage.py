"""Tests for polygon coverage planning: shapes, inset, heading, and that the rover's path never leaves the field."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import create_waypoint_list as planner  # noqa: E402
from lawnmower.coverage import plan_coverage  # noqa: E402
from lawnmower.field import Field, FieldError, MAX_INSET_M, parse_field  # noqa: E402
from lawnmower.geometry import to_gps, to_meters  # noqa: E402
from lawnmower.polygon import (  # noqa: E402
    PolygonError, distance_to_boundary, inset, is_simple, point_in_polygon, signed_area,
)

REF = (37.0, -122.0)

L_SHAPE = [(0, 0), (30, 0), (30, 10), (10, 10), (10, 25), (0, 25)]
TRIANGLE = [(0, 0), (30, 0), (10, 20)]
PENTAGON = [(0, 0), (20, 0), (28, 12), (10, 22), (-6, 12)]
TRAPEZOID = [(0, 0), (30, 0), (20, 10), (5, 10)]
U_SHAPE = [(0, 0), (10, 0), (10, 20), (20, 20), (20, 0), (30, 0), (30, 30), (0, 30)]
STAR = [(20 * math.cos(math.radians(90 + 36 * k)) if k % 2 == 0 else 8 * math.cos(math.radians(90 + 36 * k)),
         20 * math.sin(math.radians(90 + 36 * k)) if k % 2 == 0 else 8 * math.sin(math.radians(90 + 36 * k)))
        for k in range(10)]


def gps(points):
    return [to_gps(x, y, *REF) for x, y in points]


def metres(waypoints):
    return [to_meters(lat, lon, *REF) for lat, lon in waypoints]


def rectangle(w, h):
    return [(0, 0), (w, 0), (w, h), (0, h)]


def inside_or_on(point, ring, tolerance=1e-3):
    return point_in_polygon(point, ring) or distance_to_boundary(point, ring) < tolerance


def path_samples(points, per_segment=12):
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        for k in range(per_segment + 1):
            t = k / per_segment
            yield x1 + t * (x2 - x1), y1 + t * (y2 - y1)


class PolygonHelperTests(unittest.TestCase):
    def test_area_sign_follows_the_winding(self):
        square = rectangle(4, 3)
        self.assertEqual(signed_area(square), 12)
        self.assertEqual(signed_area(list(reversed(square))), -12)

    def test_point_in_polygon(self):
        self.assertTrue(point_in_polygon((5, 5), L_SHAPE))
        self.assertFalse(point_in_polygon((20, 20), L_SHAPE))  # the notch of the L

    def test_a_bow_tie_is_not_simple(self):
        self.assertFalse(is_simple([(0, 0), (10, 10), (10, 0), (0, 10)]))

    def test_a_vertex_touching_another_edge_is_not_simple(self):
        self.assertFalse(is_simple([(0, 0), (10, 0), (10, 10), (5, 0.001), (0, 10)]))

    def test_ordinary_shapes_are_simple(self):
        for shape in (L_SHAPE, TRIANGLE, PENTAGON, U_SHAPE, STAR):
            self.assertTrue(is_simple(shape))

    def test_inset_of_a_rectangle(self):
        moved = inset(rectangle(30, 10), 1.0)
        xs, ys = [p[0] for p in moved], [p[1] for p in moved]
        self.assertAlmostEqual(min(xs), 1.0)
        self.assertAlmostEqual(max(xs), 29.0)
        self.assertAlmostEqual(min(ys), 1.0)
        self.assertAlmostEqual(max(ys), 9.0)

    def test_inset_does_not_depend_on_the_winding(self):
        self.assertAlmostEqual(signed_area(inset(L_SHAPE, 1.0)), signed_area(inset(list(reversed(L_SHAPE)), 1.0)))

    def test_inset_larger_than_the_field_is_refused(self):
        with self.assertRaises(PolygonError):
            inset(rectangle(30, 10), 6.0)

    def test_a_very_sharp_corner_cannot_be_moved_inwards(self):
        with self.assertRaises(PolygonError):
            inset([(0, 0), (30, 0), (30, 1)], 0.1)


class RectangleTests(unittest.TestCase):
    def test_covers_the_same_places_as_the_four_corner_planner(self):
        corners = gps(rectangle(30, 10.5))
        legacy = {(round(x, 3), round(y, 3)) for x, y in metres(planner.create_waypoint_list(*corners))}
        # Same rows and the same pass ends; the planners may differ only in which end they start from.
        new = {(round(x, 3), round(y, 3)) for x, y in metres(plan_coverage(corners, 1.0))}
        self.assertEqual(new, legacy)

    def test_every_pass_runs_along_the_long_side(self):
        points = metres(plan_coverage(gps(rectangle(30, 10.5)), 1.0))
        for (x1, y1), (x2, y2) in zip(points[0::2], points[1::2]):
            self.assertAlmostEqual(abs(x2 - x1), 30.0, places=3)
            self.assertAlmostEqual(y1, y2, places=3)

    def test_no_row_is_driven_twice_when_the_height_is_a_whole_number_of_spacings(self):
        points = metres(plan_coverage(gps(rectangle(30, 10)), 1.0))
        rows = [round(y, 3) for (_, y) in points[0::2]]
        self.assertEqual(len(rows), len(set(rows)))
        self.assertEqual(len(rows), 11)

    def test_four_points_may_be_given_in_any_order(self):
        scrambled = [(30, 10), (0, 0), (30, 0), (0, 10)]
        self.assertCountEqual(
            [(round(x, 3), round(y, 3)) for x, y in metres(plan_coverage(gps(scrambled), 1.0))],
            [(round(x, 3), round(y, 3)) for x, y in metres(plan_coverage(gps(rectangle(30, 10)), 1.0))],
        )


class ShapeTests(unittest.TestCase):
    def check_stays_inside(self, shape, **options):
        waypoints = metres(plan_coverage(gps(shape), 1.0, **options))
        self.assertGreater(len(waypoints), 4)
        ring = shape
        for point in path_samples(waypoints):
            self.assertTrue(inside_or_on(point, ring), f"left the field at {point}")
        return waypoints

    def test_the_whole_path_of_each_shape_stays_inside_the_field(self):
        for name, shape in (("L", L_SHAPE), ("triangle", TRIANGLE), ("pentagon", PENTAGON),
                            ("trapezoid", TRAPEZOID), ("U", U_SHAPE)):
            with self.subTest(name):
                self.check_stays_inside(shape)

    def test_clockwise_and_counter_clockwise_give_the_same_plan(self):
        a = metres(plan_coverage(gps(L_SHAPE), 1.0))
        b = metres(plan_coverage(gps(list(reversed(L_SHAPE))), 1.0))
        self.assertEqual({(round(x, 3), round(y, 3)) for x, y in a}, {(round(x, 3), round(y, 3)) for x, y in b})

    def test_every_point_of_the_field_is_close_to_the_path(self):
        spacing = 1.0
        for name, shape in (("L", L_SHAPE), ("pentagon", PENTAGON), ("trapezoid", TRAPEZOID)):
            with self.subTest(name):
                waypoints = metres(plan_coverage(gps(shape), spacing))
                segments = list(zip(waypoints, waypoints[1:]))
                xs, ys = [p[0] for p in shape], [p[1] for p in shape]
                x = min(xs)
                while x <= max(xs):
                    y = min(ys)
                    while y <= max(ys):
                        if point_in_polygon((x, y), shape) and distance_to_boundary((x, y), shape) > 0.2:
                            nearest = min(_segment_distance((x, y), a, b) for a, b in segments)
                            # Half a spacing between rows, a little more beside a slanted edge where the pass ends differ.
                            self.assertLessEqual(nearest, spacing * 0.75, f"{(x, y)} is not covered")
                        y += 0.5
                    x += 0.5

    def test_the_l_shape_steps_around_the_corner_instead_of_cutting_it(self):
        waypoints = metres(plan_coverage(gps(L_SHAPE), 1.0))
        # A direct drive from the wide part to the narrow arm would pass through the notch at (20, 15).
        for point in path_samples(waypoints, per_segment=40):
            self.assertFalse(point[0] > 10.02 and point[1] > 10.02, f"cut across the notch at {point}")

    def test_the_longest_side_sets_the_default_pass_direction(self):
        waypoints = metres(plan_coverage(gps(TRAPEZOID), 1.0))
        first_pass = waypoints[1][0] - waypoints[0][0], waypoints[1][1] - waypoints[0][1]
        self.assertAlmostEqual(first_pass[1], 0.0, places=2)  # 30 m bottom edge is along x

    def test_a_field_that_is_cut_into_pieces_in_every_direction_is_refused(self):
        with self.assertRaises(FieldError) as raised:
            plan_coverage(gps(STAR), 1.0)
        self.assertIn("more than one piece", str(raised.exception))

    def test_a_u_shape_is_planned_by_sweeping_across_its_arms(self):
        waypoints = self.check_stays_inside(U_SHAPE)
        self.assertGreater(len(waypoints), 10)

    def test_a_heading_that_cuts_the_field_in_two_is_refused_with_advice(self):
        with self.assertRaises(FieldError) as raised:
            plan_coverage(gps(U_SHAPE), 1.0, heading_deg=90)  # rows along x cross both arms of the U
        self.assertIn("heading_deg", str(raised.exception))

    def test_a_boundary_that_crosses_itself_is_refused(self):
        with self.assertRaises(FieldError) as raised:
            plan_coverage(gps([(0, 0), (30, 0), (0, 20), (30, 20), (15, 30)]), 1.0)
        self.assertIn("crosses itself", str(raised.exception))

    def test_fewer_than_three_points_is_refused(self):
        with self.assertRaises(FieldError):
            plan_coverage(gps([(0, 0), (10, 0)]), 1.0)

    def test_a_mistyped_coordinate_is_still_caught(self):
        corners = gps(PENTAGON)
        corners[2] = (corners[2][0], REF[1] - 1.0)
        with self.assertRaises(FieldError) as raised:
            plan_coverage(corners, 1.0)
        self.assertIn("km", str(raised.exception))


def _segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


class InsetTests(unittest.TestCase):
    def test_every_waypoint_keeps_the_distance_from_the_boundary(self):
        for name, shape in (("rectangle", rectangle(30, 12)), ("L", L_SHAPE), ("pentagon", PENTAGON)):
            with self.subTest(name):
                for point in metres(plan_coverage(gps(shape), 1.0, inset_m=1.5)):
                    self.assertTrue(point_in_polygon(point, shape) or distance_to_boundary(point, shape) < 1e-3)
                    self.assertGreaterEqual(distance_to_boundary(point, shape), 1.5 - 0.01)

    def test_inset_shortens_the_rows_and_the_field_height(self):
        points = metres(plan_coverage(gps(rectangle(30, 12)), 1.0, inset_m=2.0))
        for (x1, y1), (x2, y2) in zip(points[0::2], points[1::2]):
            self.assertAlmostEqual(abs(x2 - x1), 26.0, places=3)
        ys = [p[1] for p in points]
        self.assertAlmostEqual(min(ys), 2.0, places=3)
        self.assertAlmostEqual(max(ys), 10.0, places=3)

    def test_an_inset_too_big_for_the_field_is_refused(self):
        with self.assertRaises(FieldError) as raised:
            plan_coverage(gps(rectangle(30, 10)), 1.0, inset_m=6.0)
        self.assertIn("inwards", str(raised.exception))


class HeadingTests(unittest.TestCase):
    def test_a_north_heading_makes_north_south_passes(self):
        points = metres(plan_coverage(gps(rectangle(30, 10)), 1.0, heading_deg=0))
        for (x1, y1), (x2, y2) in zip(points[0::2], points[1::2]):
            self.assertAlmostEqual(x1, x2, places=2)
            self.assertAlmostEqual(abs(y2 - y1), 10.0, places=2)

    def test_a_diagonal_heading_makes_diagonal_passes(self):
        points = metres(plan_coverage(gps(rectangle(30, 30)), 2.0, heading_deg=45))
        for (x1, y1), (x2, y2) in zip(points[0::2], points[1::2]):
            self.assertAlmostEqual(abs(x2 - x1), abs(y2 - y1), places=2)

    def test_rotated_passes_still_stay_inside(self):
        waypoints = metres(plan_coverage(gps(PENTAGON), 1.0, heading_deg=30))
        for point in path_samples(waypoints):
            self.assertTrue(inside_or_on(point, PENTAGON))

    def test_opposite_headings_are_the_same_line(self):
        a = plan_coverage(gps(L_SHAPE), 1.0, heading_deg=90)
        b = plan_coverage(gps(L_SHAPE), 1.0, heading_deg=270)
        self.assertEqual(a, b)


class PlanFieldTests(unittest.TestCase):
    def test_a_plain_four_sided_field_still_uses_the_four_corner_planner(self):
        corners = tuple(gps(TRAPEZOID))
        self.assertEqual(planner.plan_field(Field(boundary=corners)), planner.create_waypoint_list(*corners))

    def test_an_inset_or_heading_uses_the_polygon_planner(self):
        corners = tuple(gps(rectangle(30, 12)))
        self.assertEqual(planner.plan_field(Field(boundary=corners, inset_m=1.0)),
                         plan_coverage(corners, 1.0, inset_m=1.0))
        self.assertEqual(planner.plan_field(Field(boundary=corners, heading_deg=0.0)),
                         plan_coverage(corners, 1.0, heading_deg=0.0))

    def test_more_than_four_points_uses_the_polygon_planner(self):
        corners = tuple(gps(L_SHAPE))
        self.assertEqual(planner.plan_field(Field(boundary=corners)), plan_coverage(corners, 1.0))

    def test_spacing_comes_from_the_field(self):
        corners = tuple(gps(L_SHAPE))
        fine = planner.plan_field(Field(boundary=corners, pass_spacing_m=1.0))
        coarse = planner.plan_field(Field(boundary=corners, pass_spacing_m=3.0))
        self.assertLess(len(coarse), len(fine))


class FieldSettingsTests(unittest.TestCase):
    BOUNDARY = [[37.0, -122.0], [37.0, -121.9999], [37.0001, -121.9999], [37.0001, -122.0]]

    def test_inset_and_heading_are_read(self):
        field = parse_field({"boundary": self.BOUNDARY, "inset_m": 1.5, "heading_deg": 270})
        self.assertEqual(field.inset_m, 1.5)
        self.assertEqual(field.heading_deg, 90.0)

    def test_they_default_to_no_inset_and_automatic_heading(self):
        field = parse_field({"boundary": self.BOUNDARY})
        self.assertEqual(field.inset_m, 0.0)
        self.assertIsNone(field.heading_deg)

    def test_bad_values_are_refused(self):
        for key, bad in (("inset_m", -1), ("inset_m", MAX_INSET_M + 1), ("inset_m", "2"), ("inset_m", True),
                         ("heading_deg", "east"), ("heading_deg", True), ("heading_deg", float("nan"))):
            with self.subTest(key=key, bad=bad):
                with self.assertRaises(FieldError):
                    parse_field({"boundary": self.BOUNDARY, key: bad})


if __name__ == "__main__":
    unittest.main()
