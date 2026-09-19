# 01.12 — Ticks to distances and angles, and back

Lesson: [01.12 Use encoders to drive exact distances and angles — drive a square](../../../01-first-robot/01.12-encoder-distance-and-square.md)

Every "go 1 metre" and "turn 90°" on a differential-drive robot ends as a number of encoder
ticks. Write that conversion once, correctly, with signs, and the rest of the course (odometry
in 09.04, `cmd_vel` in 09.02, Nav2 later) builds on it.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `meters_per_tick(wheel_radius_m, ticks_per_rev)` | wheel circumference ÷ ticks per revolution |
| `ticks_to_distance(ticks, …)` / `distance_to_ticks(distance_m, …)` | one wheel, both directions; `distance_to_ticks` returns a whole `int` |
| `drive_straight_ticks(distance_m, …)` | `(left, right)` tick targets for a straight line |
| `turn_in_place_ticks(angle_rad, …)` | `(left, right)` for spinning on the spot, positive = counter-clockwise |
| `arc_ticks(radius_m, angle_rad, …)` | `(left, right)` for a curve of a given radius, including radii inside the track |
| `distance_from_ticks(…)` / `heading_change_from_ticks(…)` | read motion back out of a pair of tick *deltas* |
| `square_plan(side_m, …, clockwise=False)` | the eight segments of a square |

Each stub raises `NotImplementedError` — replace the body, keep the signature.

## Check

```bash
python course.py check 01.12              # your code
python course.py check 01.12 --solution   # the reference, to see what passing looks like
```

The first 24 tests are hand-computed on a deliberately round robot (wheel radius 0.05 m,
1000 ticks/rev, track 0.2 m), so you can redo every number on paper, plus a few on **your**
robot's values from `labs/config/karmel.yaml`. The last two drive the course simulator around
the square your plan describes and check that a perfect robot comes back within 5 cm, clockwise
and counter-clockwise.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Signs are the whole exercise. Forward is positive for **both** wheels; a left (counter-clockwise)
  turn is left wheel backwards, right wheel forwards.
* Turning in place: each wheel rides a circle of radius `wheel_separation_m / 2`, so the arc
  length is `angle * wheel_separation_m / 2` — nothing to do with the wheel radius until you
  convert metres to ticks.
* An arc is the same formula with the circle's radius added to / subtracted from the turn radius.
  Writing `turn_in_place_ticks` in terms of `arc_ticks(0, …)` (or the other way round) is fine.
* Don't wrap the heading in `heading_change_from_ticks`: it is a *change*, and three turns is
  three turns.
* `distance_to_ticks` must return an `int` — `round()`, not `int()`, which truncates towards zero
  and would make backwards moves systematically short.
