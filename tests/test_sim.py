"""Tests for the simulated rover and sensors: the physics, the noise statistics, and the wiring to Mission."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lawnmower import geometry, sim  # noqa: E402
from lawnmower.mission import MissionConfig  # noqa: E402
from lawnmower.sim import NO_NOISE, NoiseParams, Rover, RoverParams, Sensors  # noqa: E402

REF = (37.0, -122.0)
INSTANT = RoverParams(max_speed_mps=0.5, track_m=0.3, motor_lag_s=0)


def drive(rover, left, right, seconds, dt=0.01):
    for _ in range(round(seconds / dt)):
        rover.step(left, right, dt)


class RoverTests(unittest.TestCase):
    def test_straight_line_north(self):
        rover = Rover(INSTANT, heading_deg=0)
        drive(rover, 100, 100, 10)
        self.assertAlmostEqual(rover.north, 5.0, places=6)   # 0.5 m/s for 10 s
        self.assertAlmostEqual(rover.east, 0.0, places=6)
        self.assertAlmostEqual(rover.heading_deg, 0.0, places=6)

    def test_straight_line_east_at_half_speed(self):
        rover = Rover(INSTANT, heading_deg=90)
        drive(rover, 50, 50, 10)
        self.assertAlmostEqual(rover.east, 2.5, places=6)
        self.assertAlmostEqual(rover.north, 0.0, places=6)

    def test_reverse_goes_backwards(self):
        rover = Rover(INSTANT, heading_deg=0)
        drive(rover, -100, -100, 2)
        self.assertAlmostEqual(rover.north, -1.0, places=6)

    def test_slower_right_wheel_turns_right_with_the_predicted_radius(self):
        # Wheel speeds 0.5 and 0.4 m/s on a 0.3 m track: radius = (track / 2) * (vl + vr) / (vl - vr) = 1.35 m.
        rover = Rover(INSTANT, heading_deg=0)
        drive(rover, 100, 80, 1)
        self.assertGreater(rover.heading_deg, 0)   # clockwise = right
        # Turn rate = (vl - vr) / track = 0.1 / 0.3 rad/s, so a full circle takes 2 pi * 3 s; after
        # one circle the rover is back where it started, and every point stays 1.35 m from the centre.
        period = 2 * math.pi / (0.1 / 0.3)
        rover = Rover(INSTANT, heading_deg=0)
        radii = []
        centre = (1.35, 0.0)   # facing north and turning right: the centre is to the east
        for _ in range(round(period / 0.01)):
            rover.step(100, 80, 0.01)
            radii.append(math.hypot(rover.east - centre[0], rover.north - centre[1]))
        self.assertAlmostEqual(min(radii), 1.35, delta=0.02)
        self.assertAlmostEqual(max(radii), 1.35, delta=0.02)
        self.assertAlmostEqual(rover.east, 0.0, delta=0.02)
        self.assertAlmostEqual(rover.north, 0.0, delta=0.02)

    def test_slower_left_wheel_turns_left(self):
        rover = Rover(INSTANT, heading_deg=0)
        drive(rover, 80, 100, 1)
        self.assertLess(rover.heading_deg, 360)
        self.assertGreater(rover.heading_deg, 180)   # wrapped round from 0 to just under 360

    def test_opposite_wheels_spin_on_the_spot(self):
        rover = Rover(INSTANT, heading_deg=0)
        drive(rover, 100, -100, 1)   # (0.5 + 0.5) / 0.3 rad/s
        self.assertAlmostEqual(rover.east, 0.0, places=6)
        self.assertAlmostEqual(rover.north, 0.0, places=6)
        self.assertAlmostEqual(rover.heading_deg, math.degrees(1.0 / 0.3) % 360, places=4)

    def test_motor_lag_eases_the_wheels_up_to_speed(self):
        rover = Rover(RoverParams(motor_lag_s=0.5), heading_deg=0)
        drive(rover, 100, 100, 0.5)
        self.assertAlmostEqual(rover.wheels[0] / 0.5, 1 - math.exp(-1), delta=0.01)
        drive(rover, 100, 100, 5)
        self.assertAlmostEqual(rover.wheels[0], 0.5, delta=0.001)

    def test_acceleration_reports_the_change_in_velocity(self):
        rover = Rover(INSTANT, heading_deg=0)
        rover.step(100, 100, 0.1)
        self.assertAlmostEqual(rover.acceleration[1], 5.0, places=6)   # 0 to 0.5 m/s in 0.1 s
        rover.step(100, 100, 0.1)
        self.assertAlmostEqual(rover.acceleration[1], 0.0, places=6)


class SensorTests(unittest.TestCase):
    def sensors(self, noise, seed=1):
        return Sensors(Rover(INSTANT, east=3.0, north=4.0, heading_deg=90), REF, noise, np.random.default_rng(seed))

    def test_no_noise_reads_the_truth(self):
        s = self.sensors(NO_NOISE)
        east, north = geometry.to_meters(*s.gps(1.0), *REF)
        self.assertAlmostEqual(east, 3.0, places=4)
        self.assertAlmostEqual(north, 4.0, places=4)
        self.assertAlmostEqual(s.heading(), 90.0, places=9)
        self.assertEqual(s.acceleration(), (0.0, 0.0))

    def test_white_gps_error_has_the_requested_spread(self):
        s = self.sensors(NoiseParams(gps_std_m=2.0, gps_bias_std_m=0.0, accel_bias_std=0.0))
        errors = np.array([geometry.to_meters(*s.gps(1.0), *REF) for _ in range(4000)]) - [3.0, 4.0]
        self.assertAlmostEqual(errors.mean(), 0.0, delta=0.15)
        self.assertAlmostEqual(errors[:, 0].std(), 2.0, delta=0.1)
        self.assertAlmostEqual(errors[:, 1].std(), 2.0, delta=0.1)

    def test_wandering_gps_error_is_correlated_from_fix_to_fix(self):
        s = self.sensors(NoiseParams(gps_std_m=0.0, gps_bias_std_m=2.0, gps_bias_tau_s=60.0, accel_bias_std=0.0))
        east = np.array([geometry.to_meters(*s.gps(1.0), *REF)[0] - 3.0 for _ in range(6000)])
        self.assertAlmostEqual(east.std(), 2.0, delta=0.3)
        lag1 = np.corrcoef(east[:-1], east[1:])[0, 1]
        self.assertAlmostEqual(lag1, math.exp(-1 / 60.0), delta=0.03)   # about 0.98: successive fixes agree

    def test_compass_noise_and_bias(self):
        s = self.sensors(NoiseParams(compass_std_deg=3.0, compass_bias_deg=5.0, accel_bias_std=0.0))
        readings = np.array([s.heading() for _ in range(4000)])
        self.assertAlmostEqual(readings.mean(), 95.0, delta=0.3)
        self.assertAlmostEqual(readings.std(), 3.0, delta=0.2)

    def test_compass_wraps_around_north(self):
        s = Sensors(Rover(INSTANT, heading_deg=359), REF, NoiseParams(compass_std_deg=0, compass_bias_deg=5), np.random.default_rng(0))
        self.assertAlmostEqual(s.heading(), 4.0, places=9)

    def test_accelerometer_bias_is_fixed_for_a_run_and_noise_is_not(self):
        s = self.sensors(NoiseParams(accel_std=0.1, accel_bias_std=0.5))
        readings = np.array([s.acceleration() for _ in range(4000)])
        self.assertAlmostEqual(readings[:, 0].mean(), s.accel_bias[0], delta=0.01)
        self.assertAlmostEqual(readings[:, 0].std(), 0.1, delta=0.01)

    def test_same_seed_same_readings(self):
        a, b = self.sensors(NoiseParams(), seed=5), self.sensors(NoiseParams(), seed=5)
        self.assertEqual([a.gps(1.0), a.heading(), a.acceleration()], [b.gps(1.0), b.heading(), b.acceleration()])


class SimulateTests(unittest.TestCase):
    def leg(self, length_m=10.0):
        return [REF, geometry.to_gps(0, length_m, *REF)]

    def test_a_straight_leg_is_driven_to_the_end_with_perfect_sensors(self):
        result = sim.simulate(self.leg(), noise=NO_NOISE, rover_params=INSTANT)
        self.assertTrue(result.finished)
        self.assertEqual(result.reason, "path complete")
        self.assertEqual(result.waypoints_reached, 2)
        end = result.track[-1]
        self.assertAlmostEqual(end[2], 10.0, delta=1.0)   # within about the waypoint radius of the end
        self.assertAlmostEqual(end[1], 0.0, delta=1.0)

    def test_it_takes_about_as_long_as_the_distance_at_full_speed(self):
        result = sim.simulate(self.leg(10.0), noise=NO_NOISE, rover_params=INSTANT)
        self.assertAlmostEqual(result.elapsed_s, 19.0, delta=3.0)   # 9.5 m at 0.5 m/s, plus the first fix

    def test_the_mission_time_limit_ends_the_run(self):
        result = sim.simulate(self.leg(200.0), MissionConfig(runtime_limit_s=20), noise=NO_NOISE)
        self.assertFalse(result.finished)
        self.assertEqual(result.reason, "time limit")

    def test_the_sim_time_limit_ends_the_run(self):
        result = sim.simulate(self.leg(200.0), noise=NO_NOISE, max_time_s=5)
        self.assertEqual(result.reason, "sim time limit")
        self.assertAlmostEqual(result.elapsed_s, 5.0, delta=0.2)

    def test_same_seed_gives_the_same_track_and_a_different_seed_does_not(self):
        a = sim.simulate(self.leg(), seed=3)
        b = sim.simulate(self.leg(), seed=3)
        c = sim.simulate(self.leg(), seed=4)
        self.assertEqual(a.track, b.track)
        self.assertNotEqual(a.track, c.track)

    def test_a_fix_arrives_once_a_second_not_every_loop(self):
        from lawnmower.mission import Mission
        fixes = []
        original = Mission.step

        def counting_step(mission, now, heading, ax, ay, gps_fix):
            fixes.append((now, gps_fix is not None))
            return original(mission, now, heading, ax, ay, gps_fix)

        Mission.step = counting_step
        try:
            sim.simulate(self.leg(), noise=NO_NOISE, rover_params=INSTANT, max_time_s=10)
        finally:
            Mission.step = original
        with_fix = [t for t, got in fixes if got]
        self.assertEqual(len(fixes), 100)            # a loop every 0.1 s
        self.assertEqual(len(with_fix), 10)          # a fix every 1.0 s
        self.assertAlmostEqual(with_fix[1] - with_fix[0], 1.0, places=6)

    def test_starting_away_from_the_first_waypoint_faces_it(self):
        result = sim.simulate(self.leg(), noise=NO_NOISE, rover_params=INSTANT, start_offset_m=(-6.0, 0.0), max_time_s=0.2)
        self.assertAlmostEqual(result.track[0][3], 90.0, delta=0.5)   # first waypoint is due east of the start


if __name__ == "__main__":
    unittest.main()
