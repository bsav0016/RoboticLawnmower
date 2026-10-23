"""Unit tests for the pure navigation logic in lawnmower/ (no Raspberry Pi hardware needed)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lawnmower import geometry, steering  # noqa: E402

REF = (37.0, -122.0)


def offset(east_m, north_m, ref=REF):
    return geometry.to_gps(east_m, north_m, *ref)


class BearingTests(unittest.TestCase):
    def test_cardinal_directions(self):
        for east, north, expected in [(0, 50, 0), (50, 0, 90), (0, -50, 180), (-50, 0, 270)]:
            lat, lon = offset(east, north)
            self.assertAlmostEqual(geometry.calculate_bearing(*REF, lat, lon), expected, delta=0.1)

    def test_result_is_always_in_0_to_360(self):
        for east, north in [(-1, 50), (-50, 1), (30, -40), (-30, -40)]:
            lat, lon = offset(east, north)
            self.assertTrue(0 <= geometry.calculate_bearing(*REF, lat, lon) < 360)

    def test_north_west_is_just_under_360(self):
        lat, lon = offset(-1, 50)
        self.assertGreater(geometry.calculate_bearing(*REF, lat, lon), 350)


class HeadingErrorTests(unittest.TestCase):
    def test_simple_cases(self):
        self.assertEqual(geometry.heading_error(10, 40), 30)
        self.assertEqual(geometry.heading_error(40, 10), -30)
        self.assertEqual(geometry.heading_error(90, 90), 0)

    def test_wraps_the_short_way_around_north(self):
        self.assertEqual(geometry.heading_error(350, 10), 20)
        self.assertEqual(geometry.heading_error(10, 350), -20)

    def test_exactly_opposite_is_not_normalized(self):
        self.assertEqual(geometry.heading_error(0, 180), 180)
        self.assertEqual(geometry.heading_error(180, 0), -180)

    def test_always_within_180(self):
        for heading in range(0, 360, 15):
            for bearing in range(0, 360, 15):
                self.assertLessEqual(abs(geometry.heading_error(heading, bearing)), 180)


class DistanceTests(unittest.TestCase):
    def test_same_point_is_zero(self):
        self.assertEqual(geometry.distance_meters(*REF, *REF), 0)

    def test_matches_the_local_frame_for_short_distances(self):
        lat, lon = offset(30, 40)
        self.assertAlmostEqual(geometry.distance_meters(*REF, lat, lon), 50, delta=0.2)

    def test_is_symmetric(self):
        lat, lon = offset(12, -7)
        self.assertAlmostEqual(
            geometry.distance_meters(*REF, lat, lon), geometry.distance_meters(lat, lon, *REF))


class ProjectionTests(unittest.TestCase):
    def setUp(self):
        self.start = offset(0, 0)
        self.end = offset(100, 0)  # a row running due east

    def project(self, east, north):
        lat, lon = offset(east, north)
        out = geometry.project_onto_row(lat, lon, self.start, self.end, REF)
        return geometry.to_meters(*out, *REF)

    def test_sideways_drift_is_removed(self):
        x, y = self.project(40, 2.5)
        self.assertAlmostEqual(x, 40, places=3)
        self.assertAlmostEqual(y, 0, places=3)

    def test_clamps_to_the_start_of_the_row(self):
        x, y = self.project(-10, 1)
        self.assertAlmostEqual(x, 0, places=3)
        self.assertAlmostEqual(y, 0, places=3)

    def test_clamps_to_the_end_of_the_row(self):
        x, y = self.project(130, -1)
        self.assertAlmostEqual(x, 100, places=3)
        self.assertAlmostEqual(y, 0, places=3)

    def test_zero_length_row_returns_the_input(self):
        lat, lon = offset(5, 5)
        self.assertEqual(geometry.project_onto_row(lat, lon, self.start, self.start, REF), (lat, lon))


class CoordinateTests(unittest.TestCase):
    def test_round_trip(self):
        lat, lon = geometry.to_gps(12.5, -33.25, *REF)
        x, y = geometry.to_meters(lat, lon, *REF)
        self.assertAlmostEqual(x, 12.5, places=6)
        self.assertAlmostEqual(y, -33.25, places=6)


class MagnetometerHeadingTests(unittest.TestCase):
    def test_known_values_with_the_prototype_offsets(self):
        # After removing offsets (23, 8): atan2(mx, my) - 90.
        self.assertAlmostEqual(steering.heading_from_magnetometer(23, 108), 270.0)   # field along +y: -90 -> 270
        self.assertAlmostEqual(steering.heading_from_magnetometer(123, 8), 0.0)      # field along +x: 90 - 90
        self.assertAlmostEqual(steering.heading_from_magnetometer(23, -92), 90.0)    # field along -y: 180 - 90
        self.assertAlmostEqual(steering.heading_from_magnetometer(-77, 8), 180.0)    # field along -x: -90 - 90 -> 180

    def test_custom_offsets(self):
        self.assertAlmostEqual(steering.heading_from_magnetometer(100, 5, offset_x=0, offset_y=5), 0.0)

    def test_always_in_0_to_360(self):
        for mx in range(-200, 201, 40):
            for my in range(-200, 201, 40):
                self.assertTrue(0 <= steering.heading_from_magnetometer(mx, my) < 360)


class SteerTests(unittest.TestCase):
    def steer(self, error):
        return steering.steer(error, forward_speed=100, turn_speed=80, threshold_deg=10)

    def test_straight_inside_the_threshold(self):
        for error in (-10, -3, 0, 3, 10):
            self.assertEqual(self.steer(error), (100, 100))

    def test_turns_right_by_slowing_the_right_wheel(self):
        self.assertEqual(self.steer(10.5), (100, 80))
        self.assertEqual(self.steer(170), (100, 80))

    def test_turns_left_by_slowing_the_left_wheel(self):
        self.assertEqual(self.steer(-10.5), (80, 100))
        self.assertEqual(self.steer(-170), (80, 100))


if __name__ == "__main__":
    unittest.main()
