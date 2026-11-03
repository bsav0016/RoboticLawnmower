"""Try settings on a simulated rover before trying them on the real one.

Plans your field once for each combination of the settings you give, drives each plan in the
simulator (lawnmower/sim.py) several times with different noise, and prints one row per combination:
how much of the grass was cut, how far the rover strayed outside the boundary or into a no-go zone,
how long it took, and whether it finished.

    python sweep_plan.py --field field.json --inset 0,1,2 --turn-depth 0,1 --gps-error 2.5,4
    python sweep_plan.py --field field.json --inset 1,2 --seeds 10 --jobs 4 --csv sweep.csv --svg best.svg

The simulator's rover and noise are guesses until measured (see the README), so read the table as
"which setting is better", not "what will happen". A row is marked ok when the rover finished every run,
cut at least --coverage-target percent on average and never strayed more than --max-excursion meters."""
import argparse
import csv
import itertools
import math
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace

from create_waypoint_list import plan_field
from field_config import load_field
from lawnmower import sim
from lawnmower.field import FieldError
from lawnmower.geometry import to_gps, to_meters
from lawnmower.mission import MissionConfig
from lawnmower.polygon import PolygonError
from lawnmower.preview import path_length_m, plan_svg
from lawnmower.scoring import coverage_pct, excursions

BASE_GPS_ERROR_M = math.hypot(sim.NoiseParams().gps_std_m, sim.NoiseParams().gps_bias_std_m)
# A run may take this many times as long as driving the plan at full speed, plus a minute, before it is cut off.
TIME_ALLOWANCE = 3.0


@dataclass(frozen=True)
class Scenario:
    inset_m: float
    turn_depth_m: float
    stop_distance_m: float
    gps_error_m: float
    turn_speed: float = 80   # inner wheel speed in a turn; the outer wheel runs at 100


@dataclass
class RunScore:
    finished: bool
    coverage_pct: float
    max_outside_m: float
    max_into_zone_m: float
    elapsed_s: float
    not_reached: int
    track: list = None


def noise_for(gps_error_m):
    """Default sensor noise with the GPS error scaled so its total spread (fast plus wandering) is gps_error_m."""
    base = sim.NoiseParams()
    scale = gps_error_m / BASE_GPS_ERROR_M
    return replace(base, gps_std_m=base.gps_std_m * scale, gps_bias_std_m=base.gps_bias_std_m * scale)


def field_for(field, scenario):
    return replace(field, inset_m=scenario.inset_m, turn_depth_m=scenario.turn_depth_m)


def run_once(field, scenario, seed, rover_params=None, loop_dt=0.1, resolution_m=0.25, keep_track=False):
    """Plan the field with the scenario's settings, drive it in the simulator, and score the driven track."""
    planned = field_for(field, scenario)
    waypoints = plan_field(planned)
    rover_params = rover_params or sim.RoverParams()
    allowance = TIME_ALLOWANCE * path_length_m(waypoints) / rover_params.max_speed_mps + 60
    config = MissionConfig(stop_distance_m=scenario.stop_distance_m, runtime_limit_s=allowance,
                           turn_speed=scenario.turn_speed)
    result = sim.simulate(waypoints, config, rover_params, noise_for(scenario.gps_error_m), seed=seed,
                          loop_dt=loop_dt)

    ref = waypoints[0]   # the simulator's origin
    boundary = [to_meters(lat, lon, *ref) for lat, lon in field.boundary]
    zones = [[to_meters(lat, lon, *ref) for lat, lon in zone] for zone in field.exclusions]
    track = [(east, north) for _, east, north, _ in result.track]
    cut_width = field.cut_width_m if field.cut_width_m is not None else field.pass_spacing_m
    outside, into_zone = excursions(track, boundary, zones)
    return RunScore(result.finished, coverage_pct(track, cut_width, boundary, zones, resolution_m),
                    outside, into_zone, result.elapsed_s, len(waypoints) - result.waypoints_reached,
                    track=[(t[1], t[2]) for t in result.track] if keep_track else None), waypoints


def _worker(args):
    field, scenario, seed, rover_params, loop_dt, resolution_m = args
    try:
        return run_once(field, scenario, seed, rover_params, loop_dt, resolution_m)[0]
    except (FieldError, PolygonError, ValueError) as error:
        return str(error)


@dataclass
class Row:
    scenario: Scenario
    runs: list = None
    error: str = None

    def mean(self, name):
        return sum(getattr(r, name) for r in self.runs) / len(self.runs)

    def worst(self, name):
        return max(getattr(r, name) for r in self.runs)

    @property
    def finished_count(self):
        return sum(r.finished for r in self.runs)

    def problems(self, coverage_target, max_excursion):
        """Why this row is not ok: a list of short reasons, empty when it is."""
        if self.error:
            return ["cannot plan"]
        reasons = []
        if self.finished_count < len(self.runs):
            reasons.append("unfinished")
        if self.mean("coverage_pct") < coverage_target:
            reasons.append("coverage")
        if max(self.worst("max_outside_m"), self.worst("max_into_zone_m")) > max_excursion:
            reasons.append("excursion")
        return reasons


def sweep(field, scenarios, seeds, rover_params=None, loop_dt=0.1, resolution_m=0.25, jobs=1):
    """One Row per scenario, each with a RunScore per seed (or an error if the field cannot be planned that way)."""
    tasks = [(field, s, seed, rover_params, loop_dt, resolution_m) for s in scenarios for seed in range(seeds)]
    if jobs > 1:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            results = list(pool.map(_worker, tasks, chunksize=1))
    else:
        results = [_worker(t) for t in tasks]
    rows = []
    for i, scenario in enumerate(scenarios):
        chunk = results[i * seeds:(i + 1) * seeds]
        failed = [c for c in chunk if isinstance(c, str)]
        rows.append(Row(scenario, error=failed[0]) if failed else Row(scenario, runs=chunk))
    return rows


def scenarios_from(insets, turn_depths, stop_distances, gps_errors, turn_speeds=(80,)):
    """Every combination, leaving out turn depths larger than the inset (the planner does not allow them)."""
    return [Scenario(i, t, s, g, w) for i, t, s, g, w in itertools.product(insets, turn_depths, stop_distances,
                                                                           gps_errors, turn_speeds)
            if t <= i]


def best_row(rows, coverage_target, max_excursion):
    """The row to recommend: among rows that are ok, the one with the most coverage (then the least time);
    if none are ok, the one with the most coverage that finished every run."""
    planned = [r for r in rows if not r.error]
    ok = [r for r in planned if not r.problems(coverage_target, max_excursion)]
    pool = ok or [r for r in planned if r.finished_count == len(r.runs)] or planned
    return max(pool, key=lambda r: (r.mean("coverage_pct"), -r.mean("elapsed_s")), default=None)


COLUMNS = ("inset_m", "turn_m", "stop_m", "gps_m", "inner_wheel", "cut_%", "outside_m", "in_zone_m", "time_s", "finished", "verdict")


def table_rows(rows, coverage_target, max_excursion):
    out = []
    for row in rows:
        s = row.scenario
        head = [f"{s.inset_m:g}", f"{s.turn_depth_m:g}", f"{s.stop_distance_m:g}", f"{s.gps_error_m:g}",
                f"{s.turn_speed:g}"]
        if row.error:
            out.append(head + ["-", "-", "-", "-", "-", f"cannot plan: {row.error}"])
            continue
        problems = row.problems(coverage_target, max_excursion)
        out.append(head + [f"{row.mean('coverage_pct'):.1f}", f"{row.worst('max_outside_m'):.2f}",
                           f"{row.worst('max_into_zone_m'):.2f}", f"{row.mean('elapsed_s'):.0f}",
                           f"{row.finished_count}/{len(row.runs)}", "ok" if not problems else ", ".join(problems)])
    return out


def format_table(rows, coverage_target, max_excursion):
    body = table_rows(rows, coverage_target, max_excursion)
    widths = [max(len(c), *(len(r[i]) for r in body)) if body else len(c) for i, c in enumerate(COLUMNS)]
    widths[-1] = 0   # the last column may be long and is not padded
    lines = ["  ".join(c.ljust(w) for c, w in zip(COLUMNS, widths)).rstrip()]
    lines += ["  ".join(v.ljust(w) for v, w in zip(r, widths)).rstrip() for r in body]
    return "\n".join(lines)


def track_svg(field, track):
    """The plan for the field with the track the rover really drove laid over it."""
    waypoints = plan_field(field)
    ref = waypoints[0]
    return plan_svg(field, waypoints, track=[to_gps(x, y, *ref) for x, y in track])


def parse_numbers(text, name):
    try:
        values = [float(part) for part in text.split(",") if part.strip()]
    except ValueError:
        raise SystemExit(f'{name} must be numbers separated by commas, such as "0,1,2", not "{text}".')
    if not values:
        raise SystemExit(f"{name} needs at least one number.")
    return values


def main(argv=None):
    parser = argparse.ArgumentParser(description="Compare planner settings by driving each in the simulator.")
    parser.add_argument("--field", help="field file to use (default: field.json, or the corners in .env)")
    parser.add_argument("--inset", default="0", help="inset_m values to try, comma separated (default 0)")
    parser.add_argument("--turn-depth", default="0", help="turn_depth_m values to try (default 0)")
    parser.add_argument("--stop-distance", default="0.5", help="waypoint radius values in meters (default 0.5)")
    parser.add_argument("--gps-error", default="2.5", help="total GPS error values in meters (default 2.5)")
    parser.add_argument("--turn-speed", default="80",
                        help="inner wheel speed in turns, 0-100, outer wheel stays at 100 (default 80; lower turns tighter)")
    parser.add_argument("--seeds", type=int, default=5, help="runs per combination, each with different noise (default 5)")
    parser.add_argument("--max-speed", type=float, default=sim.RoverParams().max_speed_mps,
                        help="rover ground speed at full power, m/s (default %(default)s, a guess)")
    parser.add_argument("--track-width", type=float, default=sim.RoverParams().track_m,
                        help="distance between the wheels, m (default %(default)s, a guess)")
    parser.add_argument("--coverage-target", type=float, default=95.0, help="percent cut that counts as good (default 95)")
    parser.add_argument("--max-excursion", type=float, default=1.0,
                        help="farthest the rover may stray outside the boundary or into a zone, m (default 1)")
    parser.add_argument("--jobs", type=int, default=1, help="runs to do at once (default 1)")
    parser.add_argument("--csv", help="also write the table to this CSV file")
    parser.add_argument("--svg", help="write the recommended setting's first run, drawn over its plan, to this SVG file")
    args = parser.parse_args(argv)

    if args.seeds < 1:
        raise SystemExit("--seeds must be at least 1.")
    field = load_field(field_path=args.field)
    scenarios = scenarios_from(parse_numbers(args.inset, "--inset"), parse_numbers(args.turn_depth, "--turn-depth"),
                               parse_numbers(args.stop_distance, "--stop-distance"),
                               parse_numbers(args.gps_error, "--gps-error"), parse_numbers(args.turn_speed, "--turn-speed"))
    if not scenarios:
        raise SystemExit("No combinations to run: every --turn-depth is larger than every --inset.")
    rover = sim.RoverParams(max_speed_mps=args.max_speed, track_m=args.track_width)

    print(f"{len(scenarios)} combinations x {args.seeds} runs. The simulated rover and its noise are guesses; "
          f"compare rows, do not trust the numbers as predictions.")
    rows = sweep(field, scenarios, args.seeds, rover, jobs=args.jobs)
    print(format_table(rows, args.coverage_target, args.max_excursion))

    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(COLUMNS)
            writer.writerows(table_rows(rows, args.coverage_target, args.max_excursion))
        print(f"Wrote {args.csv}")

    best = best_row(rows, args.coverage_target, args.max_excursion)
    if best is None:
        print("None of these combinations could be planned for this field.")
        return 1
    ok = not best.problems(args.coverage_target, args.max_excursion)
    s = best.scenario
    print(f"{'Best' if ok else 'Closest (none met the targets)'}: inset_m {s.inset_m:g}, turn_depth_m {s.turn_depth_m:g}, "
          f"stop distance {s.stop_distance_m:g} m, inner wheel {s.turn_speed:g}, at {s.gps_error_m:g} m of GPS error.")
    if args.svg:
        score, _ = run_once(field, s, 0, rover, keep_track=True)
        with open(args.svg, "w", encoding="utf-8") as f:
            f.write(track_svg(field_for(field, s), score.track))
        print(f"Wrote {args.svg}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
