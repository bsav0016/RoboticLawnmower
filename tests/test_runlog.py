"""Tests for the run recorder: a recorded run replays to the same decisions."""
import contextlib
import io
import json
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import replay_run  # noqa: E402
from lawnmower import geometry, runlog  # noqa: E402
from lawnmower.mission import Mission, MissionConfig  # noqa: E402

REF = (37.0, -122.0)


def at(east_m, north_m):
    return geometry.to_gps(east_m, north_m, *REF)


def record_run(path, steps=40):
    """A scripted run with noisy fixes, a few dropouts and a wandering compass, written to path."""
    rng = random.Random(7)
    waypoints = [at(0, 0), at(0, 15), at(2, 15), at(2, 0)]
    mission = Mission(waypoints, MissionConfig(runtime_limit_s=100), now=1000.0)
    log = runlog.RunLog(path, mission)
    for i in range(steps):
        now = 1000.0 + (i + 1) * 0.5
        fix = None if i in (3, 4, 20) else at(rng.gauss(0, 1), min(i * 0.5, 15) + rng.gauss(0, 1))
        heading = 10 + rng.gauss(0, 5)
        accel = (rng.gauss(0, 0.2), rng.gauss(0, 0.2))
        command = mission.step(now, heading, accel[0], accel[1], fix)
        log.record(mission, now, heading, accel[0], accel[1], fix, command)
        if command.done:
            break
    log.close()
    return mission


class RunLogTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.path = os.path.join(self.dir.name, "runs", "run.jsonl")

    def test_log_is_one_json_object_per_line_with_a_header_first(self):
        record_run(self.path)
        with open(self.path, encoding="utf-8") as f:
            lines = [json.loads(line) for line in f]
        self.assertEqual(lines[0]["type"], "header")
        self.assertEqual(lines[0]["format"], runlog.FORMAT)
        self.assertTrue(all(r["type"] == "step" for r in lines[1:]))
        self.assertEqual(len(lines[0]["waypoints"]), 4)

    def test_each_step_is_written_as_it_happens(self):
        mission = Mission([at(0, 20)], now=0.0)
        log = runlog.RunLog(self.path, mission)
        command = mission.step(1.0, 0.0, 0.0, 0.0, at(0, 0))
        log.record(mission, 1.0, 0.0, 0.0, 0.0, at(0, 0), command)
        with open(self.path, encoding="utf-8") as f:   # read before close(): the run may be killed
            self.assertEqual(len(f.readlines()), 2)
        log.close()

    def test_a_run_replays_to_the_same_decisions(self):
        record_run(self.path)
        header, steps = runlog.read_log(self.path)
        self.assertGreater(len(steps), 10)
        self.assertEqual(runlog.differences(steps, runlog.replay(header, steps)), [])

    def test_replay_shows_where_a_changed_setting_decides_differently(self):
        record_run(self.path)
        header, steps = runlog.read_log(self.path)
        header["config"]["heading_threshold_deg"] = 170   # tolerate far more heading error than the rover did
        self.assertNotEqual(runlog.differences(steps, runlog.replay(header, steps)), [])

    def test_waiting_for_a_fix_is_recorded_without_an_estimate(self):
        mission = Mission([at(0, 20)], now=0.0)
        log = runlog.RunLog(self.path, mission)
        command = mission.step(1.0, 0.0, 0.0, 0.0, None)
        log.record(mission, 1.0, 0.0, 0.0, 0.0, None, command)
        log.close()
        _, steps = runlog.read_log(self.path)
        self.assertIsNone(steps[0]["estimate"])
        self.assertIsNone(steps[0]["gps"])
        self.assertEqual(steps[0]["command"]["reason"], "waiting for fix")

    def test_reading_something_that_is_not_a_log_fails_clearly(self):
        bad = os.path.join(self.dir.name, "bad.jsonl")
        with open(bad, "w", encoding="utf-8") as f:
            f.write('{"type": "step"}\n')
        with self.assertRaises(ValueError):
            runlog.read_log(bad)

    def test_an_unknown_format_version_is_refused(self):
        record_run(self.path)
        with open(self.path, encoding="utf-8") as f:
            lines = f.read().splitlines()
        header = json.loads(lines[0])
        header["format"] = 99
        lines[0] = json.dumps(header)
        with open(self.path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        with self.assertRaises(ValueError):
            runlog.read_log(self.path)


class ReplayToolTests(unittest.TestCase):
    def test_prints_a_summary_and_reports_a_match(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "run.jsonl")
            record_run(path)
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                code = replay_run.main([path])
        text = out.getvalue()
        self.assertEqual(code, 0)
        self.assertIn("Loops with no GPS fix: 3 of", text)
        self.assertIn("Position uncertainty:", text)
        self.assertIn("same decisions as the rover did", text)

    def test_a_missing_file_exits_with_a_message(self):
        with self.assertRaises(SystemExit):
            replay_run.main(["no-such-file.jsonl"])


if __name__ == "__main__":
    unittest.main()
