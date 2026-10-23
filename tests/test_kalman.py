"""Unit tests for the position filter (needs numpy, no Raspberry Pi)."""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lawnmower import geometry  # noqa: E402
from lawnmower.kalman import PositionFilter  # noqa: E402

REF = (37.0, -122.0)


def fix(east_m, north_m):
    return geometry.to_gps(east_m, north_m, *REF)


def started():
    kf = PositionFilter()
    kf.init(*REF)
    return kf


def error_m(kf, east_m, north_m):
    lat, lon = kf.position()
    x, y = geometry.to_meters(lat, lon, *REF)
    return math.hypot(x - east_m, y - north_m)


class InitTests(unittest.TestCase):
    def test_not_initialized_until_the_first_fix(self):
        kf = PositionFilter()
        self.assertFalse(kf.initialized)
        kf.init(*REF)
        self.assertTrue(kf.initialized)

    def test_starts_at_the_first_fix(self):
        kf = started()
        lat, lon = kf.position()
        self.assertAlmostEqual(lat, REF[0], places=9)
        self.assertAlmostEqual(lon, REF[1], places=9)

    def test_starting_uncertainty_is_the_gps_noise(self):
        self.assertAlmostEqual(started().position_std_m(), 3.0)

    def test_instances_do_not_share_state(self):
        a, b = started(), PositionFilter()
        self.assertFalse(b.initialized)
        a.predict(1.0, 0.0, 1.0)
        self.assertIsNone(b.state)


class ConvergenceTests(unittest.TestCase):
    def test_converges_to_a_stationary_target(self):
        kf = started()
        target = fix(8, -5)
        for _ in range(60):
            kf.predict(0.0, 0.0, 0.1)
            kf.update_gps(*target)
        self.assertLess(error_m(kf, 8, -5), 0.3)

    def test_uncertainty_shrinks_with_gps_updates(self):
        kf = started()
        before = kf.position_std_m()
        for _ in range(20):
            kf.predict(0.0, 0.0, 0.1)
            kf.update_gps(*REF)
        self.assertLess(kf.position_std_m(), before)

    def test_smooths_noisy_fixes_better_than_a_raw_fix(self):
        rng = random.Random(7)
        kf = started()
        raw_errors = []
        for _ in range(200):
            nx, ny = rng.gauss(0, 3), rng.gauss(0, 3)
            raw_errors.append(math.hypot(nx, ny))
            kf.predict(0.0, 0.0, 0.1)
            kf.update_gps(*fix(nx, ny))
        self.assertLess(error_m(kf, 0, 0), sum(raw_errors) / len(raw_errors))


class DeadReckoningTests(unittest.TestCase):
    def test_constant_acceleration_matches_the_kinematics(self):
        kf = started()
        for _ in range(10):          # 1 s of 2 m/s^2 eastward, no GPS
            kf.predict(2.0, 0.0, 0.1)
        # x = 0.5 * a * t^2 = 1 m, vx = a * t = 2 m/s
        self.assertAlmostEqual(kf.state[0], 1.0, places=6)
        self.assertAlmostEqual(kf.state[2], 2.0, places=6)
        self.assertAlmostEqual(kf.state[1], 0.0, places=9)

    def test_uncertainty_grows_without_gps(self):
        kf = started()
        before = kf.position_std_m()
        for _ in range(50):
            kf.predict(0.0, 0.0, 0.1)
        self.assertGreater(kf.position_std_m(), before)

    def test_a_gps_fix_pulls_back_a_drifted_estimate(self):
        kf = started()
        for _ in range(30):          # accelerometer bias makes it drift east
            kf.predict(0.5, 0.0, 0.1)
        drifted = error_m(kf, 0, 0)
        kf.update_gps(*REF)
        self.assertLess(error_m(kf, 0, 0), drifted)


if __name__ == "__main__":
    unittest.main()
