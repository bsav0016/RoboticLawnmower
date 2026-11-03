"""Look at a recorded run on this computer, and check it against the current decision logic.

navigate_path.py writes a log to runs/ every time it runs. Copy the file off the Pi and:

    python replay_run.py runs/run-20261104-101500.jsonl

It prints a summary (how long, how many loops had no GPS fix, waypoints reached, how uncertain the
position estimate got) and then feeds the same sensor readings back through the mission code. If the
code now decides differently from what the rover did, it lists where."""
import argparse

from lawnmower.runlog import differences, read_log, replay


def summarize(header, steps):
    if not steps:
        return ["No steps recorded."]
    duration = steps[-1]["t"] - header["start"]
    no_fix = sum(1 for s in steps if s["gps"] is None)
    stds = [s["estimate"]["std_m"] for s in steps if s["estimate"]]
    last = steps[-1]
    lines = [
        f"{len(steps)} loops over {duration:.1f} s, {len(header['waypoints'])} waypoints in the plan",
        f"Loops with no GPS fix: {no_fix} of {len(steps)}",
        f"Ended at waypoint {last['index']} ({last['command']['reason']})",
    ]
    if stds:
        lines.append(f"Position uncertainty: {min(stds):.2f} to {max(stds):.2f} m (1 sigma)")
    if len(steps) > 1:
        gaps = [b["t"] - a["t"] for a, b in zip(steps, steps[1:])]
        lines.append(f"Loop interval: {min(gaps):.2f} to {max(gaps):.2f} s")
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description="Summarize a recorded run and replay it through the mission code.")
    parser.add_argument("log", help="a .jsonl file written by navigate_path.py")
    args = parser.parse_args(argv)

    try:
        header, steps = read_log(args.log)
    except (OSError, ValueError) as error:
        raise SystemExit(str(error))

    for line in summarize(header, steps):
        print(line)

    found = differences(steps, replay(header, steps))
    if not found:
        print("Replay: the mission code makes the same decisions as the rover did.")
        return 0
    print(f"Replay: {len(found)} loops decided differently from the rover. First few:")
    for number, text in found[:10]:
        print(f"  loop {number}: {text}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
