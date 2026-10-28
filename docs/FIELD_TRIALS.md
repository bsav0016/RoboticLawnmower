# Trying the planner on the rover

Everything below `preview_plan.py` has only run in tests so far. This is an ordered list for the first supervised
sessions, from "nothing moves" to "it mows around a no-go zone". Do the steps in order and stop at the first one that
surprises you. Keep the robot where you can reach it, and have the battery lead or a switch within reach.

## 1. Nothing moves (laptop)

1. `cp field.example.json field.json`, then put your real field in `boundary` (and `exclusions`, if any).
2. `python preview_plan.py` and open `plan.svg`. Check the boundary is where you expect, the path stays inside it, and
   the start is somewhere sensible. The footer shows the waypoint count and path length.
3. If the field is refused, the message says what to fix.

## 2. Wheels off the ground (Pi)

1. Put the rover on blocks so the wheels spin freely.
2. `python navigate_path.py` with the example field. It should start, print `Path loaded: N waypoints`, and steer the
   wheels left and right as the GPS position wanders. This checks the `field.json` wiring, which has not run on the Pi.
3. Stop it and confirm the motors stop.

## 3. Measure how the rover turns

The rover never turns on the spot: `steer()` slows the inner wheel (`TURN_SPEED` 80 against `FORWARD_SPEED` 100), so
it turns in an arc. The swing-out turns need to be sized to that arc.

1. With a free area, hold the steer command on one side (a short script calling `set_motor_speed(100, 80)`) and mark
   where the rover goes for one full circle.
2. Measure the circle's diameter. Half of it is the turning radius.
3. Note it down. The waypoint radius is `STOP_DISTANCE_M` = 0.5 m in `navigate_path.py`.

## 4. A plain run, as before

Mow a small plain rectangle (10 x 10 m is plenty) with no `inset_m`, `heading_deg`, `exclusions` or `turn_depth_m`.
That uses the four-corner planner that has driven the rover before, so any difference you see later comes from the
new settings.

## 5. Add the settings one at a time

Repeat the small run, changing one thing each time and looking at the preview first:

| Setting | What to try | What to watch |
| --- | --- | --- |
| `inset_m` | 1 to 2 m first. GPS error is about 3 m, so the rover may still stray outside this | How far from the edge it really gets |
| `turn_depth_m` | About the turning radius you measured, never more than `inset_m`, and more than 0.5 m (otherwise the swing point counts as reached straight away) | Does it come round into the next pass cleanly, or overshoot? |
| `cut_width_m`, `overlap_m` | Measure the real cutting width of the deck, and start with an overlap of 5 to 10% | Strips left uncut between passes |
| `heading_deg` | Leave it out unless you want a particular stripe direction | |
| `exclusions` | A zone marked with cones first, with `inset_m` at least as large as the GPS error you see | Does the rover keep its distance? Be ready to stop it |

The turn depth takes space out of the margin, so a bigger `turn_depth_m` means the rover comes closer to the boundary
(or a no-go zone) than `inset_m` suggests. Keep `turn_depth_m` well under `inset_m` near a zone.

## 6. Record what you find

Whatever you find (the turning radius, how far the rover strays, whether the swings help) is the input for tuning the defaults and for the next change. Keep the settings and the preview file next to each observation.
