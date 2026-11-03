"""Run recorder and replay: a JSONL log of one run, and the same run fed back through Mission.

The first line is a header (waypoints, settings, start time). Every later line is one loop: the
sensor readings that went in and what the mission decided. Replaying a log feeds the readings back
through a fresh Mission, so a run from the field can be examined, and a change to the decision
logic can be checked against a real run, on a computer."""
import json
import os
from dataclasses import asdict

from .kalman import PositionFilter
from .mission import Mission, MissionConfig

FORMAT = 1


def _header(mission):
    kf = mission.kf
    return {
        "type": "header",
        "format": FORMAT,
        "start": mission.start_time,
        "waypoints": [list(w) for w in mission.waypoints],
        "config": asdict(mission.config),
        "filter": {"gps_std_m": kf.gps_std_m, "process_pos": kf.process_pos, "process_vel": kf.process_vel},
    }


class RunLog:
    """Appends one JSON line per loop and flushes each, so a run that is killed keeps its log."""

    def __init__(self, path, mission):
        folder = os.path.dirname(path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        self.path = path
        self._file = open(path, "w", encoding="utf-8")
        self._write(_header(mission))

    def _write(self, record):
        self._file.write(json.dumps(record) + "\n")
        self._file.flush()

    def record(self, mission, now, heading_deg, accel_east, accel_north, gps_fix, command):
        """Call after mission.step(), with the same readings that went into it."""
        estimate = None
        if mission.kf.initialized:
            lat, lon = mission.kf.position()
            estimate = {"lat": lat, "lon": lon, "std_m": mission.kf.position_std_m()}
        self._write({
            "type": "step",
            "t": now,
            "heading": heading_deg,
            "accel": [accel_east, accel_north],
            "gps": list(gps_fix) if gps_fix is not None else None,
            "index": mission.index,
            "estimate": estimate,
            "command": {"reason": command.reason, "left": command.left, "right": command.right,
                        "done": command.done},
        })

    def close(self):
        self._file.close()


def read_log(path):
    """Return (header, steps). Raises ValueError if the file is not a run log."""
    with open(path, encoding="utf-8") as f:
        records = [json.loads(line) for line in f if line.strip()]
    if not records or records[0].get("type") != "header":
        raise ValueError(f"{path} is not a run log (no header line).")
    if records[0].get("format") != FORMAT:
        raise ValueError(f"{path} has log format {records[0].get('format')}; this code reads format {FORMAT}.")
    return records[0], [r for r in records[1:] if r.get("type") == "step"]


def replay(header, steps):
    """Feed the recorded readings through a fresh Mission. Returns a list of per-step results:
    (recorded command dict, replayed Command, replayed waypoint index)."""
    waypoints = [tuple(w) for w in header["waypoints"]]
    config = MissionConfig(**header["config"])
    mission = Mission(waypoints, config, now=header["start"], position_filter=PositionFilter(**header["filter"]))
    results = []
    for step in steps:
        gps = tuple(step["gps"]) if step["gps"] is not None else None
        accel_east, accel_north = step["accel"]
        command = mission.step(step["t"], step["heading"], accel_east, accel_north, gps)
        results.append((step["command"], command, mission.index))
    return results


def differences(steps, results):
    """Steps where the replayed decision is not the one recorded: a list of (step number, what differed)."""
    found = []
    for number, (step, (recorded, command, index)) in enumerate(zip(steps, results), start=1):
        replayed = {"reason": command.reason, "left": command.left, "right": command.right, "done": command.done}
        if recorded != replayed:
            found.append((number, f"recorded {recorded}, replayed {replayed}"))
        elif step["index"] != index:
            found.append((number, f"recorded waypoint {step['index']}, replayed {index}"))
    return found
