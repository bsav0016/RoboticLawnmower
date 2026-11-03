"""A simulated rover and its sensors, so the mission can be run against a field without the Pi.

The rover is a differential-drive vehicle on flat ground: the two motor commands (0-100, as sent to
set_motor_speed) turn into wheel speeds, which move the rover. The sensors are what Mission.step()
is given: a GPS fix about once a second, a compass heading and a world-frame accelerometer, each
with noise.

Every number in RoverParams and NoiseParams is a guess until it is measured on the real rover
(docs/FIELD_TRIALS.md step 5 measures the turning radius; a recorded run gives the GPS and compass
noise). Until then results compare settings against each other; they do not promise how the rover
will behave. All randomness comes from one seed, so a run can be repeated exactly."""
import math
from dataclasses import dataclass, field

import numpy as np

from .geometry import calculate_bearing, to_gps
from .mission import PATH_COMPLETE, Mission, MissionConfig


@dataclass
class RoverParams:
    max_speed_mps: float = 0.5   # ground speed with the motors at 100
    track_m: float = 0.30        # distance between the left and right wheels
    motor_lag_s: float = 0.3     # time for a wheel to come up to a new speed (0 = instantly)


@dataclass
class NoiseParams:
    gps_std_m: float = 1.5            # fast, uncorrelated GPS error, per axis
    gps_bias_std_m: float = 2.0       # slowly wandering GPS error, per axis
    gps_bias_tau_s: float = 60.0      # how long that wander takes to forget itself
    compass_std_deg: float = 3.0      # compass noise on every reading
    compass_bias_deg: float = 0.0     # constant compass error (a bad calibration)
    accel_std: float = 0.15           # accelerometer noise, m/s^2 per axis
    accel_bias_std: float = 0.05      # constant accelerometer offset, drawn once per run, per axis


NO_NOISE = NoiseParams(0, 0, 60.0, 0, 0, 0, 0)


class Rover:
    """True position (meters east and north of the origin) and compass heading (degrees clockwise from north)."""

    def __init__(self, params=None, east=0.0, north=0.0, heading_deg=0.0):
        self.params = params or RoverParams()
        self.east = east
        self.north = north
        self.heading_deg = heading_deg % 360
        self.wheels = (0.0, 0.0)     # left and right wheel speeds, m/s
        self.velocity = (0.0, 0.0)   # east, north, m/s; the last step's
        self.acceleration = (0.0, 0.0)

    def step(self, left, right, dt):
        """Move for dt seconds with the given motor commands (-100 to 100)."""
        p = self.params
        blend = 1.0 if p.motor_lag_s <= 0 else 1.0 - math.exp(-dt / p.motor_lag_s)
        v_left = self.wheels[0] + (left / 100.0 * p.max_speed_mps - self.wheels[0]) * blend
        v_right = self.wheels[1] + (right / 100.0 * p.max_speed_mps - self.wheels[1]) * blend
        self.wheels = (v_left, v_right)
        speed = (v_left + v_right) / 2.0
        # Compass heading runs clockwise, so a faster right wheel turns the rover left (heading falls).
        turn_rate_deg = math.degrees((v_left - v_right) / p.track_m)
        mid = math.radians(self.heading_deg + turn_rate_deg * dt / 2.0)   # drive along the middle of the turn
        old = self.velocity
        self.velocity = (speed * math.sin(mid), speed * math.cos(mid))
        self.acceleration = ((self.velocity[0] - old[0]) / dt, (self.velocity[1] - old[1]) / dt)
        self.east += self.velocity[0] * dt
        self.north += self.velocity[1] * dt
        self.heading_deg = (self.heading_deg + turn_rate_deg * dt) % 360


class Sensors:
    """Noisy readings of a Rover, in the form Mission.step() takes."""

    def __init__(self, rover, ref, noise=None, rng=None):
        self.rover = rover
        self.ref = ref   # (lat, lon) of the origin of the local frame
        self.noise = noise or NoiseParams()
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.gps_bias = np.zeros(2)
        self.accel_bias = self.rng.normal(0.0, self.noise.accel_bias_std, 2)

    def gps(self, since_last_fix_s):
        """A fix as (lat, lon). The wandering error moves on by the time since the last fix."""
        n = self.noise
        phi = math.exp(-since_last_fix_s / n.gps_bias_tau_s)
        self.gps_bias = phi * self.gps_bias + math.sqrt(1 - phi ** 2) * self.rng.normal(0.0, n.gps_bias_std_m, 2)
        error = self.gps_bias + self.rng.normal(0.0, n.gps_std_m, 2)
        return to_gps(self.rover.east + error[0], self.rover.north + error[1], *self.ref)

    def heading(self):
        n = self.noise
        return (self.rover.heading_deg + n.compass_bias_deg + self.rng.normal(0.0, n.compass_std_deg)) % 360

    def acceleration(self):
        """(east, north) in m/s^2, as read_accelerometer_world() gives."""
        a = np.array(self.rover.acceleration) + self.accel_bias + self.rng.normal(0.0, self.noise.accel_std, 2)
        return float(a[0]), float(a[1])


@dataclass
class SimResult:
    reason: str                  # why it stopped: a Mission reason, or "sim time limit"
    finished: bool               # reached the last waypoint
    waypoints_reached: int
    elapsed_s: float
    track: list = field(default_factory=list)   # (t, east, north, heading_deg) of the true rover each loop
    mission: Mission = None


def simulate(waypoints, mission_config=None, rover_params=None, noise=None, seed=0,
             start_offset_m=(0.0, 0.0), start_heading_deg=None,
             loop_dt=0.1, gps_period_s=1.0, max_time_s=None, gps_dropouts=(), boundary=None):
    """Run the mission against the simulated rover until it finishes, times out or hits max_time_s.

    The rover starts at the first waypoint plus start_offset_m (east, north), facing start_heading_deg
    (default: toward the first waypoint, or north if it starts on it). A GPS fix arrives every
    gps_period_s and the loop runs every loop_dt. gps_dropouts is a list of (start, end) times in seconds when no
    fix arrives (the receiver is unplugged or has lost the sky). boundary is the field's (lat, lon) boundary, which
    the mission needs for its outside-the-field failsafe."""
    config = mission_config or MissionConfig()
    ref = waypoints[0]
    east0, north0 = start_offset_m
    if start_heading_deg is None:
        start_heading_deg = 0.0
        if math.hypot(east0, north0) > 1e-9:
            lat, lon = to_gps(east0, north0, *ref)
            start_heading_deg = calculate_bearing(lat, lon, *ref)
    rover = Rover(rover_params, east0, north0, start_heading_deg)
    rng = np.random.default_rng(seed)
    sensors = Sensors(rover, ref, noise, rng)

    mission = Mission(waypoints, config, now=0.0, boundary=boundary)
    limit = max_time_s if max_time_s is not None else config.runtime_limit_s + 1
    track = [(0.0, rover.east, rover.north, rover.heading_deg)]
    left = right = 0.0
    t = 0.0
    last_fix_t = None
    next_fix_t = 0.0
    reason = "sim time limit"
    finished = False
    while t < limit:
        t = round(t + loop_dt, 9)
        fix = None
        if t >= next_fix_t - 1e-9 and not any(a <= t < b for a, b in gps_dropouts):
            fix = sensors.gps(gps_period_s if last_fix_t is None else t - last_fix_t)
            last_fix_t = t
            next_fix_t = t + gps_period_s
        ax, ay = sensors.acceleration()
        command = mission.step(t, sensors.heading(), ax, ay, fix)
        if command.left is not None:
            left, right = command.left, command.right
        if command.done:
            reason = command.reason
            finished = command.reason == PATH_COMPLETE
            break
        rover.step(left, right, loop_dt)
        track.append((t, rover.east, rover.north, rover.heading_deg))
    return SimResult(reason, finished, mission.index, t, track, mission)
