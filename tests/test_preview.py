"""Tests for the plan preview: GeoJSON and SVG structure, and the preview_plan command."""
import json
import math
import os
import re
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import create_waypoint_list as planner  # noqa: E402
import preview_plan  # noqa: E402
from lawnmower.field import Field  # noqa: E402
from lawnmower.geometry import to_gps  # noqa: E402
from lawnmower.preview import MAX_DOTS, path_length_m, plan_geojson, plan_svg  # noqa: E402

REF = (37.0, -122.0)
SVG_NS = {"svg": "http://www.w3.org/2000/svg"}

L_SHAPE = [(0, 0), (30, 0), (30, 10), (10, 10), (10, 25), (0, 25)]


def gps(points):
    return tuple(to_gps(x, y, *REF) for x, y in points)


def field_of(points, **options):
    return Field(boundary=gps(points), **options)


def features(collection, role):
    return [f for f in collection["features"] if f["properties"]["role"] == role]


class PathLengthTests(unittest.TestCase):
    def test_a_straight_path_has_its_real_length(self):
        a = REF
        b = to_gps(30, 0, *REF)
        self.assertAlmostEqual(path_length_m([a, b]), 30.0, delta=0.05)

    def test_no_path_has_no_length(self):
        self.assertEqual(path_length_m([REF]), 0)


class GeoJsonTests(unittest.TestCase):
    def setUp(self):
        self.field = field_of(L_SHAPE)
        self.waypoints = planner.plan_field(self.field)
        self.collection = plan_geojson(self.field, self.waypoints)

    def test_is_a_feature_collection_that_survives_json(self):
        self.assertEqual(self.collection["type"], "FeatureCollection")
        self.assertEqual(json.loads(json.dumps(self.collection)), self.collection)

    def test_boundary_is_a_closed_ring_in_longitude_latitude_order(self):
        (boundary,) = features(self.collection, "boundary")
        ring = boundary["geometry"]["coordinates"][0]
        self.assertEqual(boundary["geometry"]["type"], "Polygon")
        self.assertEqual(ring[0], ring[-1])
        self.assertEqual(len(ring), len(L_SHAPE) + 1)
        for lon, lat in ring:
            self.assertAlmostEqual(lon, REF[1], delta=0.001)  # about -122, not 37
            self.assertAlmostEqual(lat, REF[0], delta=0.001)

    def test_path_has_one_coordinate_per_waypoint_in_order(self):
        (path,) = features(self.collection, "path")
        coordinates = path["geometry"]["coordinates"]
        self.assertEqual(path["geometry"]["type"], "LineString")
        self.assertEqual(coordinates, [[lon, lat] for lat, lon in self.waypoints])

    def test_start_and_end_are_the_first_and_last_waypoints(self):
        (start,) = features(self.collection, "start")
        (end,) = features(self.collection, "end")
        self.assertEqual(start["geometry"]["coordinates"], [self.waypoints[0][1], self.waypoints[0][0]])
        self.assertEqual(end["geometry"]["coordinates"], [self.waypoints[-1][1], self.waypoints[-1][0]])

    def test_the_boundary_carries_the_summary(self):
        (boundary,) = features(self.collection, "boundary")
        properties = boundary["properties"]
        self.assertEqual(properties["waypoints"], len(self.waypoints))
        self.assertAlmostEqual(properties["area_m2"], 450.0, delta=0.5)  # 30 x 10 plus 10 x 15
        self.assertAlmostEqual(properties["path_length_m"], path_length_m(self.waypoints), delta=0.1)

    def test_there_is_an_inset_boundary_only_when_the_inset_is_set(self):
        self.assertEqual(features(self.collection, "inset-boundary"), [])
        field = field_of(L_SHAPE, inset_m=1.5)
        collection = plan_geojson(field, planner.plan_field(field))
        (inset,) = features(collection, "inset-boundary")
        self.assertEqual(inset["geometry"]["type"], "Polygon")

    def test_a_plain_four_sided_field_previews_too(self):
        field = field_of([(0, 0), (30, 0), (30, 10), (0, 10)])
        collection = plan_geojson(field, planner.plan_field(field))
        self.assertEqual(len(features(collection, "path")), 1)


class SvgTests(unittest.TestCase):
    def render(self, **options):
        field = field_of(L_SHAPE, **options)
        waypoints = planner.plan_field(field)
        text = plan_svg(field, waypoints)
        return text, ET.fromstring(text), waypoints

    def find(self, root, tag, **attributes):
        found = []
        for element in root.iter(f"{{{SVG_NS['svg']}}}{tag}"):
            if all(element.get(key) == value for key, value in attributes.items()):
                found.append(element)
        return found

    def test_is_well_formed_svg_with_a_viewbox(self):
        text, root, _ = self.render()
        self.assertEqual(root.tag, f"{{{SVG_NS['svg']}}}svg")
        self.assertTrue(text.startswith("<svg"))
        self.assertEqual(len(root.get("viewBox").split()), 4)

    def test_has_the_boundary_the_path_start_and_end(self):
        _, root, waypoints = self.render()
        self.assertEqual(len(self.find(root, "polygon", id="boundary")), 1)
        (path,) = self.find(root, "polyline", id="path")
        self.assertEqual(len(path.get("points").split()), len(waypoints))
        self.assertEqual(len(self.find(root, "circle", id="start")), 1)
        self.assertEqual(len(self.find(root, "circle", id="end")), 1)

    def test_everything_drawn_is_inside_the_canvas(self):
        _, root, _ = self.render()
        width, height = (float(v) for v in root.get("viewBox").split()[2:])
        for element in list(self.find(root, "polygon")) + list(self.find(root, "polyline")):
            for pair in element.get("points").split():
                x, y = (float(v) for v in pair.split(","))
                self.assertTrue(0 <= x <= width and 0 <= y <= height, f"{pair} is off the canvas")

    def test_north_is_up(self):
        _, root, waypoints = self.render()
        (start,) = self.find(root, "circle", id="start")
        (end,) = self.find(root, "circle", id="end")
        # The plan starts at the south end of the L and finishes at the north end, so end is drawn above start.
        self.assertLess(float(end.get("cy")), float(start.get("cy")))
        self.assertGreater(waypoints[-1][0], waypoints[0][0])

    def test_the_inset_boundary_is_drawn_dashed_only_when_set(self):
        _, plain, _ = self.render()
        self.assertEqual(self.find(plain, "polygon", id="inset-boundary"), [])
        _, inset, _ = self.render(inset_m=1.5)
        (dashed,) = self.find(inset, "polygon", id="inset-boundary")
        self.assertIsNotNone(dashed.get("stroke-dasharray"))

    def test_the_summary_line_and_scale_bar_are_there(self):
        text, _, waypoints = self.render()
        self.assertIn(f"{len(waypoints)} waypoints", text)
        self.assertIn("field 450 m2", text)
        self.assertRegex(text, r">\d+ m<")  # the scale bar label

    def test_a_big_plan_leaves_out_the_waypoint_dots(self):
        field = field_of([(0, 0), (300, 0), (300, 300), (0, 300)], pass_spacing_m=0.5)
        waypoints = planner.plan_field(field)
        self.assertGreater(len(waypoints), MAX_DOTS)
        root = ET.fromstring(plan_svg(field, waypoints))
        self.assertEqual(self.find(root, "circle", **{"class": "waypoint"}), [])

    def test_the_output_is_the_same_every_time(self):
        field = field_of(L_SHAPE)
        waypoints = planner.plan_field(field)
        self.assertEqual(plan_svg(field, waypoints), plan_svg(field, waypoints))

    def test_no_absolute_coordinates_appear_in_the_svg(self):
        text, _, _ = self.render()
        self.assertNotRegex(text, r"-?122\.0|37\.0")


class PreviewCommandTests(unittest.TestCase):
    def write_field(self, folder, points, **options):
        path = os.path.join(folder, "field.json")
        data = {"boundary": [list(p) for p in gps(points)], **options}
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f)
        return path

    def test_writes_both_files(self):
        with tempfile.TemporaryDirectory() as folder:
            field_path = self.write_field(folder, L_SHAPE, inset_m=1.0)
            out = os.path.join(folder, "previews")
            self.assertEqual(preview_plan.main(["--field", field_path, "--out", out]), 0)
            ET.parse(os.path.join(out, "plan.svg"))
            with open(os.path.join(out, "plan.geojson"), encoding="utf-8") as f:
                self.assertEqual(json.load(f)["type"], "FeatureCollection")

    def test_an_unplannable_field_stops_with_a_message_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as folder:
            bow_tie = [(0, 0), (30, 0), (0, 20), (30, 20), (15, 30)]  # the edges cross
            field_path = self.write_field(folder, bow_tie)
            out = os.path.join(folder, "previews")
            with self.assertRaises(SystemExit) as raised:
                preview_plan.main(["--field", field_path, "--out", out])
            self.assertIn("Cannot plan this field", str(raised.exception))
            self.assertIn("crosses itself", str(raised.exception))
            self.assertFalse(os.path.exists(out))

    def test_a_missing_field_file_stops_with_a_message(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(SystemExit) as raised:
                preview_plan.main(["--field", os.path.join(folder, "nope.json"), "--out", folder])
            self.assertIn("does not exist", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
