"""The mission: which waypoint the rover is heading for and what the motors should do next.

This is the decision logic that used to sit inside the loop of navigate_path.py, with no sensors or
motors in it. The loop reads the sensors, calls Mission.step() once, and applies the Command it gets
back, so the same logic can be driven by scripted input in tests (and later by a simulator)."""
from dataclasses import dataclass, field

from .geometry import calculate_bearing, distance_meters, heading_error, project_onto_row, to_meters
from .kalman import PositionFilter
from .polygon import distance_to_boundary, point_in_polygon
from .steering import steer

WAITING_FOR_FIX = "waiting for fix"
TIME_LIMIT = "time limit"
PATH_COMPLETE = "path complete"
WAYPOINT_REACHED = "waypoint reached"
STEERING = "steering"
# Failsafe stops: the rover stops and the run ends.
NO_FIX = "no gps fix"
UNCERTAIN = "position uncertain"
OUT_OF_BOUNDS = "outside the field"
FAILSAFES = (NO_FIX, UNCERTAIN, OUT_OF_BOUNDS)


@dataclass
class MissionConfig:
    stop_distance_m: float = 0.5     # a waypoint is reached when the rover is within this distance
    runtime_limit_s: float = 180     # stop after this long
    forward_speed: int = 100         # motor speed going straight (0-100)
    turn_speed: int = 80             # motor speed of the inner wheel in a turn
    heading_threshold_deg: float = 10  # heading error allowed before correcting course
    # Failsafes. Thresholds come from the simulator (docs/SIMULATION.md): with a fix every second the gap
    # between fixes is 1 s and the filter's uncertainty stays near 3 m; after 5 s without a fix it has grown to
    # about 10 m, and the estimate is no longer worth steering on. None switches a check off.
    no_fix_timeout_s: float = 5.0       # stop when no GPS fix for this long (once the first fix has come)
    max_position_std_m: float = 10.0    # stop when the filter's uncertainty (1 sigma) is above this
    max_outside_m: float = 5.0          # stop when the estimate is this far outside the boundary (needs one)


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
    def __init__(self, waypoints, config=None, now=0.0, position_filter=None, boundary=None):
        if not waypoints:
            raise ValueError("A mission needs at least one waypoint.")
        self.waypoints = list(waypoints)
        self.config = config or MissionConfig()
        self.kf = position_filter or PositionFilter()
        self.index = 0
        self.prev_waypoint = None   # row start, for path projection; set when a waypoint is reached
        self.start_time = now
        self.last_time = now
        self.last_fix_time = None
        self.boundary = list(boundary) if boundary else None   # (lat, lon) points, for the outside check
        self._boundary_m = None

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
            self.last_fix_time = now

        if not self.kf.initialized:
            return Command(WAITING_FOR_FIX, messages=["Waiting for GPS fix..."])

        stop = self._failsafe(now)
        if stop:
            return stop

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

    def _failsafe(self, now):
        """A stop Command if the rover should not carry on, else None. Called once the filter has a position."""
        cfg = self.config
        if cfg.no_fix_timeout_s is not None and self.last_fix_time is not None:
            silent = now - self.last_fix_time
            if silent > cfg.no_fix_timeout_s:
                return self._stop(NO_FIX, f"Failsafe: no GPS fix for {silent:.1f} s. Stopping.")
        if cfg.max_position_std_m is not None:
            std = self.kf.position_std_m()
            if std > cfg.max_position_std_m:
                return self._stop(UNCERTAIN, f"Failsafe: position uncertainty {std:.1f} m is above "
                                             f"{cfg.max_position_std_m:g} m. Stopping.")
        if cfg.max_outside_m is not None and self.boundary:
            outside = self.outside_m()
            if outside > cfg.max_outside_m:
                return self._stop(OUT_OF_BOUNDS, f"Failsafe: the position estimate is {outside:.1f} m outside the "
                                                 f"field (limit {cfg.max_outside_m:g} m). Stopping.")
        return None

    def outside_m(self):
        """How far the position estimate is outside the boundary, in meters (0 when inside or without a boundary)."""
        if not self.boundary or not self.kf.initialized:
            return 0.0
        if self._boundary_m is None:
            self._boundary_m = [to_meters(lat, lon, *self.kf.ref) for lat, lon in self.boundary]
        point = (float(self.kf.state[0]), float(self.kf.state[1]))
        if point_in_polygon(point, self._boundary_m):
            return 0.0
        return distance_to_boundary(point, self._boundary_m)

    @staticmethod
    def _stop(reason, message):
        return Command(reason, left=0, right=0, done=True, messages=[message])
