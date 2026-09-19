# 09.04 — Build an odometry system from scratch

Lesson: [09.04 Build an odometry system from scratch](../../../09-odometry/09.04-odometry-from-scratch.md)

Turn cumulative wheel-encoder counts into a robot pose `(x, y, theta)`. The same class will
later run on your robot, fed by `DifferentialBase.read()`.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `tick_delta(previous, current, encoder_bits=None)` | signed change in counts, correct across counter rollover |
| `ticks_to_distance(ticks, wheel_radius_m, ticks_per_rev)` | ticks → meters travelled by the wheel |
| `integrate_pose(pose, d_left, d_right, wheel_separation_m)` | exact-arc pose update, straight-line special case, heading wrapped to (−π, π] |
| `Odometry.update(left_ticks, right_ticks)` | first call only stores the counts; later calls integrate the deltas and return the pose |

Each stub raises `NotImplementedError` — replace the body, keep the signature.

## Check

```bash
python course.py check 09.04              # your code
python course.py check 09.04 --solution   # the reference, to see what passing looks like
```

The tests start with hand-computed cases (a quarter circle, a spin in place, a counter wrap),
then drive the course simulator for ~4 m of straights, arcs, spins and reversing and compare
your pose with ground truth:

* a perfect simulated robot — your odometry must match within **5 mm and 0.5°**;
* a realistic robot (mismatched wheels, wheelbase, slip) — within **25 cm and 15°**. It will not
  be exact: that gap is what calibration (09.05) and drift (09.06) are about.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Store the previous counts, not the previous distances.
* `abs(dtheta) < 1e-9` → drive straight; dividing by a tiny `dtheta` loses all precision.
* With an N-bit counter, a raw difference larger than half the counter range really went the
  short way around the wrap.
