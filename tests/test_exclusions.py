"""Tests for no-go zones: the path never enters one, keeps its margin, covers the rest, and bad zones are refused."""
import json
import os
import sys
import time
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import create_waypoint_list as planner  # noqa: E402
from lawnmower.coverage import plan_coverage  # noqa: E402
from lawnmower.field import Field, FieldError  # noqa: E402
from lawnmower.geometry import to_gps, to_meters  # noqa: E402
from lawnmower.polygon import (  # noqa: E402
    PolygonError, distance_to_boundary, outset, point_in_polygon, rings_clear,
)
from lawnmower.preview import plan_geojson, plan_svg  # noqa: E402

REF = (37.0, -122.0)
SLACK = 0.03  # metres of rounding and tolerance in the geometry


def gps(points):
    return tuple(to_gps(x, y, *REF) for x, y in points)


def metres(waypoints):
    return [to_meters(lat, lon, *REF) for lat, lon in waypoints]


def rect(w, h, x0=0.0, y0=0.0):
    return [(x0, y0), (x0 + w, y0), (x0 + w, y0 + h), (x0, y0 + h)]


L_SHAPE = [(0, 0), (30, 0), (30, 10), (10, 10), (10, 25), (0, 25)]


def plan(boundary, zones=(), **options):
    return plan_coverage(gps(boundary), options.pop("spacing", 1.0), exclusions=[gps(z) for z in zones], **options)


def samples(points, per_segment=16):
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        for k in range(per_segment + 1):
            t = k / per_segment
            yield x1 + t * (x2 - x1), y1 + t * (y2 - y1)


def segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0:
        return ((p[0] - a[0]) ** 2 + (p[1] - a[1]) ** 2) ** 0.5
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
    return ((p[0] - (a[0] + t * dx)) ** 2 + (p[1] - (a[1] + t * dy)) ** 2) ** 0.5


class PathStaysInTheFreeSpace(unittest.TestCase):
    def assert_free(self, waypoints, boundary, zones, margin=0.0):
        """Every point along the path is inside the boundary and at least `margin` outside every zone."""
        for point in samples(metres(waypoints)):
            self.assertTrue(point_in_polygon(point, boundary) or distance_to_boundary(point, boundary) < SLACK,
                            f"left the field at {point}")
            for number, zone in enumerate(zones, start=1):
                distance = distance_to_boundary(point, zone)
                self.assertFalse(point_in_polygon(point, zone) and distance > SLACK,
                                 f"entered exclusion {number} at {point}")
                self.assertGreaterEqual(distance, margin - SLACK, f"closer than {margin} m to exclusion {number} at {point}")

    def assert_covered(self, waypoints, boundary, zones, margin, spacing):
        """Every point well inside the free space is within about a spacing of the path."""
        path = metres(waypoints)
        segments = list(zip(path, path[1:]))
        xs, ys = [p[0] for p in boundary], [p[1] for p in boundary]
        x = min(xs)
        while x <= max(xs):
            y = min(ys)
            while y <= max(ys):
                free = point_in_polygon((x, y), boundary) and distance_to_boundary((x, y), boundary) > margin + 0.4
                for zone in zones:
                    free = free and not point_in_polygon((x, y), zone) and distance_to_boundary((x, y), zone) > margin + 0.4
                if free:
                    nearest = min(segment_distance((x, y), a, b) for a, b in segments)
                    self.assertLessEqual(nearest, spacing * 0.75, f"{(x, y)} is not covered")
                y += 0.5
            x += 0.5

    def test_a_hole_in_the_middle_is_driven_around(self):
        field, zone = rect(40, 30), rect(8, 8, 16, 11)
        waypoints = plan(field, [zone])
        self.assert_free(waypoints, field, [zone])
        self.assert_covered(waypoints, field, [zone], 0.0, 1.0)

    def test_two_holes(self):
        field, zones = rect(50, 30), [rect(6, 10, 12, 10), rect(8, 6, 32, 12)]
        waypoints = plan(field, zones, inset_m=1.0)
        self.assert_free(waypoints, field, zones, margin=1.0)
        self.assert_covered(waypoints, field, zones, 1.0, 1.0)

    def test_a_hole_in_an_l_shape(self):
        zone = rect(4, 4, 3, 12)
        waypoints = plan(L_SHAPE, [zone], inset_m=0.5)
        self.assert_free(waypoints, L_SHAPE, [zone], margin=0.5)
        self.assert_covered(waypoints, L_SHAPE, [zone], 0.5, 1.0)

    def test_a_zone_thinner_than_the_row_spacing_is_still_avoided(self):
        field, zone = rect(30, 20), [(10, 9.2), (20, 9.2), (20, 9.8), (10, 9.8)]  # between the rows at 9 and 10
        self.assert_free(plan(field, [zone]), field, [zone])
        self.assert_free(plan(field, [zone], heading_deg=0), field, [zone])

    def test_a_zone_that_blocks_most_of_the_width_leaves_two_narrow_gaps(self):
        field, zone = rect(40, 20), rect(34, 6, 3, 7)
        waypoints = plan(field, [zone], inset_m=1.0)
        self.assert_free(waypoints, field, [zone], margin=1.0)

    def test_any_heading_works_with_a_zone(self):
        field, zone = rect(40, 30), rect(8, 8, 16, 11)
        for heading in (0, 30, 45, 90, 135):
            with self.subTest(heading=heading):
                waypoints = plan(field, [zone], heading_deg=heading, inset_m=0.5)
                self.assert_free(waypoints, field, [zone], margin=0.5)

    def test_the_margin_keeps_every_waypoint_away_from_the_zone(self):
        field, zone = rect(40, 30), rect(8, 8, 16, 11)
        for point in metres(plan(field, [zone], inset_m=1.5)):
            self.assertFalse(point_in_polygon(point, zone), f"{point} is inside the zone")
            self.assertGreaterEqual(distance_to_boundary(point, zone), 1.5 - SLACK)

    def test_a_u_shape_swept_across_its_arms_stays_inside(self):
        u_shape = [(0, 0), (10, 0), (10, 20), (20, 20), (20, 0), (30, 0), (30, 30), (0, 30)]
        self.assert_free(plan(u_shape, heading_deg=90), u_shape, [])

    def test_the_same_field_always_gives_the_same_plan(self):
        field, zone = rect(40, 30), rect(8, 8, 16, 11)
        self.assertEqual(plan(field, [zone], inset_m=1.0), plan(field, [zone], inset_m=1.0))

    def test_a_field_with_many_zones_plans_quickly(self):
        zones = [rect(6, 6, 10 + 20 * i, 10 + 20 * j) for i in range(6) for j in range(6)]
        started = time.time()
        waypoints = plan(rect(140, 140), zones, inset_m=1.0)
        self.assertLess(time.time() - started, 20)
        self.assert_free(waypoints[:200], rect(140, 140), zones, margin=1.0)


class BadZoneTests(unittest.TestCase):
    FIELD = rect(40, 30)

    def refused(self, zones, **options):
        with self.assertRaises(FieldError) as raised:
            plan(self.FIELD, zones, **options)
        return str(raised.exception)

    def test_a_zone_outside_the_field_is_refused(self):
        self.assertIn("Exclusion 1", self.refused([rect(5, 5, 60, 5)]))

    def test_a_zone_crossing_the_boundary_is_refused(self):
        self.assertIn("edge of the field", self.refused([rect(10, 10, 35, 10)]))

    def test_a_zone_against_the_boundary_is_refused(self):
        self.assertIn("Exclusion 1", self.refused([rect(5, 5, 0, 10)]))

    def test_a_zone_too_close_to_the_edge_once_the_margin_is_added_says_so(self):
        message = self.refused([rect(5, 5, 1, 10)], inset_m=1.5)
        self.assertIn("margin", message)
        self.assertIn("inset_m", message)

    def test_overlapping_zones_are_refused_naming_both(self):
        message = self.refused([rect(8, 8, 10, 10), rect(8, 8, 15, 12)])
        self.assertIn("Exclusions 1 and 2", message)

    def test_a_zone_inside_another_is_refused(self):
        self.assertIn("Exclusions 1 and 2", self.refused([rect(12, 12, 10, 8), rect(4, 4, 14, 12)]))

    def test_zones_whose_margins_meet_are_refused(self):
        message = self.refused([rect(5, 5, 10, 10), rect(5, 5, 17, 10)], inset_m=1.5)  # 2 m apart, 3 m of margin
        self.assertIn("margin", message)

    def test_a_zone_that_crosses_itself_is_refused(self):
        self.assertIn("crosses itself", self.refused([[(10, 10), (20, 20), (20, 10), (10, 20)]]))

    def test_a_zone_that_is_a_line_is_refused(self):
        self.assertIn("almost in a line", self.refused([[(10, 10), (20, 10), (30, 10)]]))

    def test_a_mistyped_zone_coordinate_is_caught(self):
        zone = list(gps(rect(5, 5, 10, 10)))
        zone[1] = (zone[1][0], REF[1] - 1.0)
        with self.assertRaises(FieldError) as raised:
            plan_coverage(gps(self.FIELD), 1.0, exclusions=[tuple(zone)])
        self.assertIn("Exclusion 1", str(raised.exception))

    def test_a_zone_with_two_points_is_refused(self):
        with self.assertRaises(FieldError):
            plan_coverage(gps(self.FIELD), 1.0, exclusions=[gps([(10, 10), (20, 20)])])


class RoutingTests(unittest.TestCase):
    def test_a_field_with_zones_goes_through_the_polygon_planner(self):
        corners = gps(rect(40, 30))
        zone = gps(rect(8, 8, 16, 11))
        field = Field(boundary=corners, exclusions=(zone,))
        self.assertEqual(planner.plan_field(field), plan_coverage(corners, 1.0, exclusions=[zone]))

    def test_a_plain_rectangle_still_uses_the_four_corner_planner(self):
        corners = gps(rect(40, 30))
        self.assertEqual(planner.plan_field(Field(boundary=corners)), planner.create_waypoint_list(*corners))

    def test_a_zone_makes_the_path_longer_than_the_field_alone(self):
        field, zone = rect(40, 30), rect(8, 8, 16, 11)
        with_zone = len(plan(field, [zone]))
        without = len(plan(field, heading_deg=90))
        self.assertGreater(with_zone, without)


class PolygonHelperTests(unittest.TestCase):
    def test_outset_grows_a_square_evenly(self):
        grown = outset(rect(10, 10), 2.0)
        xs, ys = [p[0] for p in grown], [p[1] for p in grown]
        self.assertAlmostEqual(min(xs), -2.0)
        self.assertAlmostEqual(max(xs), 12.0)
        self.assertAlmostEqual(min(ys), -2.0)
        self.assertAlmostEqual(max(ys), 12.0)

    def test_outset_of_a_very_sharp_corner_is_refused(self):
        with self.assertRaises(PolygonError):
            outset([(0, 0), (30, 0), (30, 1)], 1.0)

    def test_rings_clear(self):
        self.assertTrue(rings_clear(rect(10, 10), rect(4, 4, 3, 3)))  # one inside the other still has clear edges
        self.assertTrue(rings_clear(rect(10, 10), rect(4, 4, 20, 20)))
        self.assertFalse(rings_clear(rect(10, 10), rect(10, 10, 5, 5)))  # crossing
        self.assertFalse(rings_clear(rect(10, 10), rect(5, 5, 10, 2)))  # sharing an edge


class PreviewWithZonesTests(unittest.TestCase):
    def setUp(self):
        self.field = Field(boundary=gps(rect(40, 30)), exclusions=(gps(rect(8, 8, 16, 11)),), inset_m=1.0)
        self.waypoints = planner.plan_field(self.field)

    def test_geojson_has_the_zone_and_its_margin(self):
        collection = plan_geojson(self.field, self.waypoints)
        roles = [f["properties"]["role"] for f in collection["features"]]
        self.assertEqual(roles.count("exclusion"), 1)
        self.assertEqual(roles.count("keepout"), 1)
        json.dumps(collection)

    def test_geojson_has_no_margin_feature_without_an_inset(self):
        field = Field(boundary=self.field.boundary, exclusions=self.field.exclusions)
        collection = plan_geojson(field, planner.plan_field(field))
        roles = [f["properties"]["role"] for f in collection["features"]]
        self.assertEqual(roles.count("exclusion"), 1)
        self.assertEqual(roles.count("keepout"), 0)

    def test_svg_draws_the_zone_and_its_margin(self):
        root = ET.fromstring(plan_svg(self.field, self.waypoints))
        ids = {e.get("id") for e in root.iter("{http://www.w3.org/2000/svg}polygon")}
        self.assertIn("exclusion-1", ids)
        self.assertIn("keepout-1", ids)


if __name__ == "__main__":
    unittest.main()
