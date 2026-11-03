# What the simulator says about the settings

Written 2026-11-04 from `sweep_plan.py` (see the README). **Read this as "which direction", not "what will happen".** The simulated rover is a guess: 0.5 m/s top speed, wheels 0.30 m apart, 0.3 s for a wheel to come up to speed, a GPS fix once a second with 2.5 m of total error by default (fast noise plus a slowly wandering part), compass noise of 3 degrees, and a 10 Hz control loop. None of that has been measured on the rover yet; the last stage of the plan replaces the guesses with numbers from a real run. The sample field is `docs/examples/lawn-30x20.json`: a 30 m by 20 m lawn, 1 m cut width, 0.5 m overlap.

"Good" was set at: at least 95 % of the grass cut on average, the rover never more than 1 m outside the boundary or into a no-go zone, and every run finished.

```bash
python sweep_plan.py --field docs/examples/lawn-30x20.json --inset 0,1,2,3 --turn-depth 0,1,2 \
    --stop-distance 0.5,1 --turn-speed 80,60,0 --gps-error 0.3,1,2.5 --seeds 8 --jobs 8
```

162 combinations, 8 runs each. **None met the targets.** What the table shows:

## 1. The inner-wheel speed in turns matters most, and today's 80 is the worst

`navigate_path.py` turns by slowing the inner wheel to 80 while the outer wheel stays at 100. With the guessed track width that is a turning radius of about 1.35 m, which cannot turn round between passes 0.5 m apart: the rover overshoots the next waypoint and loops back for it. Across all the combinations at 1 m of GPS error:

| inner wheel in turns | mean grass cut | mean of the worst distance outside the boundary |
|---|---|---|
| 80 (today) | 49 % | 33 m |
| 60 | 56 % | 6.6 m |
| 0 (pivot, one wheel stops) | 54 % | 3.3 m |

Pivoting (0) keeps the rover on the field best; 60 cuts a little more of it. The turning radius was the thing `docs/FIELD_TRIALS.md` step 5 asks you to measure, and it is worth doing before trusting this.

## 2. There is a trade between staying inside and cutting everything

The inset margin is, by design, grass the rover never mows, so the settings that keep it safely inside cost coverage:

| GPS error | most grass cut (inset, turn depth, stop radius, inner wheel) | cut | worst outside | keeps inside best | cut | worst outside |
|---|---|---|---|---|---|---|
| 0.3 m | 0, 0, 0.5, 60 | 93 % | 2.3 m | 2, 1, 0.5, 0 | 74 % | 0.0 m |
| 1 m | 0, 0, 0.5, 60 | 89 % | 3.5 m | 3, 0, 0.5, 60 | 64 % | 0.0 m |
| 2.5 m | 0, 0, 0.5, 60 | 80 % | 7.7 m | 3, 0, 1, 0 | 27 % | 2.0 m |

So for this lawn: `inset_m` 2 with `turn_depth_m` 1 holds the rover inside when the position is good to about 0.3 m, and `inset_m` 3 when it is good to about 1 m. Turning the swing-out on (`turn_depth_m` above 0, same inset) cut more grass in 70 of the 90 matching pairs (28 of 30 with the inner wheel at 0) but kept the rover closer in only about half of them (it did so in 6 of 30 with the inner wheel at 0), so it is a coverage gain more than a safety one. A waypoint radius of 0.5 m cut more grass than 1 m in all 81 matching pairs and strayed less in 60 of them.

## 3. At 2.5 m of GPS error nothing keeps the rover on the lawn

With the NEO-6M's typical error, even the most cautious row strayed 2.0 m beyond the boundary (cutting 27 % of the grass), and the next best, at 2.5 m outside, cut 62 %. If that simulation is anywhere near right, the position estimate, not the planner settings, is what limits mowing a lawn this size; better position (an RTK GPS, or wheel encoders to hold a straight line between fixes) would matter more than any of the settings above. The simulator can compare those too once it has a model for them.

## 4. Overlap

Passes spaced exactly one cut width apart leave a strip wherever the rover wobbles. In an earlier run on the same lawn with no overlap, the most any row cut at 2.5 m of GPS error was 66 %, against 80 % with 0.5 m of overlap. More overlap costs time (about twice the path here), so it is a trade.

## What to do with this

1. Measure the turning radius (FIELD_TRIALS step 5) and the GPS error (a recorded run, `replay_run.py`), put them into `sweep_plan.py` (`--track-width`, `--gps-error`) and run it again.
2. For the first supervised field trial, try `turn_speed` lower than 80 and `inset_m` of at least 2, and watch for the rover overshooting turns.
3. Do not read the times: they come from the guessed 0.5 m/s.
