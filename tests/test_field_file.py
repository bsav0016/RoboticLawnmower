"""Tests for field.json: parsing, loading, and that the example field plans exactly what the .env corners did."""
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import create_waypoint_list as planner  # noqa: E402
import field_config  # noqa: E402
from lawnmower.field import DEFAULT_SPACING_M, Field, FieldError, parse_field  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE_FILE = os.path.join(ROOT, "field.example.json")
EXAMPLE_CORNERS = [(37.0, -122.0), (37.0, -122.0001), (37.0001, -122.0001), (37.0001, -122.0)]

# What the planner produced for the example corners before field.json existed (spacing 1 m).
GOLDEN_WAYPOINTS = [
    (37.0, -122.0), (37.0001, -122.0), (37.0001, -122.00001127), (37.0, -122.00001127),
    (37.0, -122.00002254), (37.0001, -122.00002254), (37.0001, -122.00003381), (37.0, -122.00003381),
    (37.0, -122.00004508), (37.0001, -122.00004508), (37.0001, -122.00005635), (37.0, -122.00005635),
    (37.0, -122.00006762), (37.0001, -122.00006762), (37.0001, -122.00007888), (37.0, -122.00007888),
    (37.0, -122.00009015), (37.0001, -122.00009015), (37.0001, -122.0001), (37.0, -122.0001),
]

SQUARE = [[37.0, -122.0], [37.0, -121.9999], [37.0001, -121.9999], [37.0001, -122.0]]


class ParseFieldTests(unittest.TestCase):
    def test_a_boundary_alone_is_enough_and_spacing_defaults(self):
        field = parse_field({"boundary": SQUARE})
        self.assertEqual(field.boundary, tuple(tuple(p) for p in SQUARE))
        self.assertEqual(field.pass_spacing_m, DEFAULT_SPACING_M)

    def test_spacing_can_be_set(self):
        self.assertEqual(parse_field({"boundary": SQUARE, "pass_spacing_m": 2}).pass_spacing_m, 2.0)

    def test_the_file_must_be_an_object(self):
        for bad in ([], "x", 5, None):
            with self.assertRaises(FieldError):
                parse_field(bad)

    def test_boundary_is_required(self):
        with self.assertRaises(FieldError) as raised:
            parse_field({"pass_spacing_m": 1})
        self.assertIn("boundary", str(raised.exception))

    def test_a_misspelt_setting_is_reported_not_ignored(self):
        with self.assertRaises(FieldError) as raised:
            parse_field({"boundary": SQUARE, "pass_spacin_m": 1})
        self.assertIn("pass_spacin_m", str(raised.exception))

    def test_exclusions_are_refused_until_they_are_supported(self):
        with self.assertRaises(FieldError) as raised:
            parse_field({"boundary": SQUARE, "exclusions": []})
        self.assertIn("not supported yet", str(raised.exception))

    def test_boundary_needs_at_least_three_points(self):
        for bad in ([], [[1, 2], [3, 4]], "abc", {"a": 1}):
            with self.assertRaises(FieldError):
                parse_field({"boundary": bad})

    def test_each_point_must_be_two_numbers(self):
        for bad in ([1], [1, 2, 3], ["37", "-122"], [True, False], None):
            with self.assertRaises(FieldError) as raised:
                parse_field({"boundary": SQUARE[:3] + [bad]})
            self.assertIn("point 4", str(raised.exception))

    def test_spacing_must_be_a_number(self):
        for bad in ("1", None, True, [1]):
            with self.assertRaises(FieldError):
                parse_field({"boundary": SQUARE, "pass_spacing_m": bad})


class CornersTests(unittest.TestCase):
    def test_four_points_are_the_corners(self):
        self.assertEqual(parse_field({"boundary": SQUARE}).corners, [tuple(p) for p in SQUARE])

    def test_other_counts_cannot_be_planned_yet(self):
        with self.assertRaises(FieldError) as raised:
            parse_field({"boundary": SQUARE[:3]}).corners
        self.assertIn("four-sided", str(raised.exception))
        with self.assertRaises(FieldError):
            parse_field({"boundary": SQUARE + [[37.0002, -122.0]]}).corners


class LoadFieldTests(unittest.TestCase):
    ENV = {"CORNER_A": "1,2", "CORNER_B": "3,4", "CORNER_C": "5,6", "CORNER_D": "7,8"}

    def write(self, folder, text, name="field.json"):
        path = os.path.join(folder, name)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return path

    def test_reads_a_field_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.write(folder, json.dumps({"boundary": SQUARE, "pass_spacing_m": 1.5}))
            field = field_config.load_field(field_path=path)
        self.assertEqual(field.pass_spacing_m, 1.5)
        self.assertEqual(len(field.boundary), 4)

    def test_falls_back_to_the_env_corners_when_there_is_no_field_file(self):
        with tempfile.TemporaryDirectory() as folder:
            with mock.patch.object(field_config, "_FIELD_PATH", os.path.join(folder, "none.json")):
                with mock.patch.dict(os.environ, self.ENV, clear=True):
                    field = field_config.load_field(env_path=os.path.join(folder, "no.env"))
        self.assertEqual(field.boundary, ((1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (7.0, 8.0)))
        self.assertEqual(field.pass_spacing_m, DEFAULT_SPACING_M)

    def test_a_field_file_wins_over_env_corners(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.write(folder, json.dumps({"boundary": SQUARE}))
            with mock.patch.object(field_config, "_FIELD_PATH", path):
                with mock.patch.dict(os.environ, self.ENV, clear=True):
                    field = field_config.load_field()
        self.assertEqual(field.boundary[0], (37.0, -122.0))

    def test_field_file_environment_variable_points_somewhere_else(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.write(folder, json.dumps({"boundary": SQUARE}), name="back-lawn.json")
            with mock.patch.dict(os.environ, {"FIELD_FILE": path}, clear=True):
                field = field_config.load_field()
        self.assertEqual(field.boundary[0], (37.0, -122.0))

    def test_a_named_file_that_is_missing_stops_with_a_message(self):
        missing = os.path.join(tempfile.gettempdir(), "no-such-field.json")
        with mock.patch.dict(os.environ, {"FIELD_FILE": missing}, clear=True):
            with self.assertRaises(SystemExit) as raised:
                field_config.load_field()
        self.assertIn("does not exist", str(raised.exception))

    def test_broken_json_stops_with_a_message(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.write(folder, '{"boundary": [')
            with self.assertRaises(SystemExit) as raised:
                field_config.load_field(field_path=path)
        self.assertIn("not valid JSON", str(raised.exception))

    def test_a_bad_setting_names_the_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.write(folder, json.dumps({"boundary": SQUARE, "colour": "green"}))
            with self.assertRaises(SystemExit) as raised:
                field_config.load_field(field_path=path)
        self.assertIn("field.json", str(raised.exception))
        self.assertIn("colour", str(raised.exception))


class ExampleFieldTests(unittest.TestCase):
    def test_the_example_file_matches_the_example_env_corners(self):
        with open(os.path.join(ROOT, ".env.example")) as f:
            lines = [line.strip() for line in f if "=" in line and not line.startswith("#")]
        env = dict(line.split("=", 1) for line in lines)
        names = ("CORNER_A", "CORNER_B", "CORNER_C", "CORNER_D")
        env_corners = [tuple(float(v) for v in env[name].split(",")) for name in names]
        self.assertEqual(env_corners, EXAMPLE_CORNERS)
        self.assertEqual(field_config.load_field(field_path=EXAMPLE_FILE).corners, EXAMPLE_CORNERS)

    def test_the_example_field_plans_exactly_what_the_corners_always_did(self):
        field = field_config.load_field(field_path=EXAMPLE_FILE)
        waypoints = planner.create_waypoint_list(*field.corners, spacing_m=field.pass_spacing_m)
        self.assertEqual([(round(a, 8), round(b, 8)) for a, b in waypoints], GOLDEN_WAYPOINTS)

    def test_the_planner_still_checks_a_field_that_came_from_a_file(self):
        field = Field(boundary=((37.0, -122.0), (37.0, -121.0), (37.0001, -121.0), (37.0001, -122.0)))
        with self.assertRaises(FieldError):
            planner.create_waypoint_list(*field.corners)


if __name__ == "__main__":
    unittest.main()
