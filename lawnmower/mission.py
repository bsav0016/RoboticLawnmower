"""The mission: which waypoint the rover is heading for and what the motors should do next.

This is the decision logic that used to sit inside the loop of navigate_path.py, with no sensors or
motors in it. The loop reads the sensors, calls Mission.step() once, and applies the Command it gets
back, so the same logic can be driven by scripted input in tests (and later by a simulator)."""
from dataclasses import dataclass, field

from .geometry import calculate_bearing, distance_meters, heading_error, project_onto_row
from .kalman import PositionFilter
from .steering import steer

WAITING_FOR_FIX = "waiting for fix"
TIME_LIMIT = "time limit"
PATH_COMPLETE = "path complete"
WAYPOINT_REACHED = "waypoint reached"
STEERING = "steering"


@dataclass
class MissionConfig:
    stop_distance_m: float = 0.5     # a waypoint is reached when the rover is within this distance
    runtime_limit_s: float = 180     # stop after this long
    forward_speed: int = 100         # motor speed going straight (0-100)
    turn_speed: int = 80             # motor speed of the inner wheel in a turn
    heading_threshold_deg: float = 10  # heading error allowed before correcting course


@dataclass
class Command:
    """What the loop should do after one step.

    left/right are motor speeds, or None to leave the motors as they are (waiting for a fix, or the
    step that only moved on to the next waypoint). done means stop the run. messages are the lines
    the loop prints, in order."""
    reason: str
    left: float = None
    right: float = None
    done: bool = False
    messages: list = field(default_factory=list)


class Mission:
    def __init__(self, waypoints, config=None, now=0.0, position_filter=None):
        if not waypoints:
            raise ValueError("A mission needs at least one waypoint.")
        self.waypoints = list(waypoints)
        self.config = config or MissionConfig()
        self.kf = position_filter or PositionFilter()
        self.index = 0
        self.prev_waypoint = None   # row start, for path projection; set when a waypoint is reached
        self.start_time = now
        self.last_time = now

    @property
    def target(self):
        return self.waypoints[self.index]

    def start_messages(self):
        lat, lon = self.target
        return [f"Path loaded: {len(self.waypoints)} waypoints.",
                f"Navigating to waypoint 0: ({lat:.8f}, {lon:.8f})"]

    def step(self, now, heading_deg, accel_east, accel_north, gps_fix):
        """Advance one loop. gps_fix is (lat, lon) or None when this loop got no fix."""
        cfg = self.config
        dt = max(now - self.last_time, 1e-3)
        self.last_time = now
        elapsed = now - self.start_time

        if elapsed >= cfg.runtime_limit_s:
            return Command(TIME_LIMIT, done=True, messages=["Time limit reached. Stopping."])

        # Dead-reckon forward on the accelerometer, then correct with GPS whenever there is a fix.
        if self.kf.initialized:
            self.kf.predict(accel_east, accel_north, dt)
        if gps_fix is not None:
            if not self.kf.initialized:
                self.kf.init(*gps_fix)
            else:
                self.kf.update_gps(*gps_fix)

        if not self.kf.initialized:
            return Command(WAITING_FOR_FIX, messages=["Waiting for GPS fix..."])

        cur_lat, cur_lon = self.kf.position()
        target_lat, target_lon = self.target

        # Snap the lateral position to the current row line so GPS noise does not cause steering
        # corrections while going straight. Not applied on the way to the first waypoint (no row yet).
        if self.prev_waypoint is not None:
            nav_lat, nav_lon = project_onto_row(
                cur_lat, cur_lon, self.prev_waypoint, (target_lat, target_lon), self.kf.ref)
        else:
            nav_lat, nav_lon = cur_lat, cur_lon

        distance = distance_meters(nav_lat, nav_lon, target_lat, target_lon)
        raw = f"({gps_fix[0]:.6f}, {gps_fix[1]:.6f})" if gps_fix else "no fix"
        messages = [f"WP {self.index}/{len(self.waypoints) - 1} | "
                    f"GPS: {raw} | KF: ({cur_lat:.6f}, {cur_lon:.6f}) | "
                    f"Target: ({target_lat:.8f}, {target_lon:.8f}) | "
                    f"Dist: {distance:.1f}m | Elapsed: {elapsed:.1f}s"]

        if distance <= cfg.stop_distance_m:
            self.index += 1
            if self.index >= len(self.waypoints):
                messages.append("All waypoints reached! Path complete.")
                return Command(PATH_COMPLETE, done=True, messages=messages)
            self.prev_waypoint = (target_lat, target_lon)
            next_lat, next_lon = self.target
            messages.append(f"Waypoint reached. Moving to waypoint {self.index}/{len(self.waypoints) - 1}: "
                            f"({next_lat:.8f}, {next_lon:.8f})")
            return Command(WAYPOINT_REACHED, messages=messages)

        bearing = calculate_bearing(nav_lat, nav_lon, target_lat, target_lon)
        error = heading_error(heading_deg, bearing)
        messages.append(f"Heading: {heading_deg:.1f}° | Target bearing: {bearing:.1f}° | Error: {error:.1f}°")
        left, right = steer(error, cfg.forward_speed, cfg.turn_speed, cfg.heading_threshold_deg)
        return Command(STEERING, left=left, right=right, messages=messages)
