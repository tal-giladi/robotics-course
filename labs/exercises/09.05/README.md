# 09.05 — Calibrating odometry from logged runs

Lesson: [09.05 Calibrating odometry — wheel radius, wheelbase and UMBmark](../../../09-odometry/09.05-calibrating-odometry.md)

`logged_runs.json` is what you would collect with your own robot and a tape measure: 16 runs
of a robot whose true wheel radii and wheelbase are *not* the nominal 45 mm / 200 mm. It holds
encoder counts at 25 Hz and the final pose measured by hand. From those numbers, find the real
geometry.

| Runs | What happened | Extra measurement |
|---|---|---|
| `cw-1..5`, `ccw-1..5` | UMBmark: a 2 m square, clockwise and counter-clockwise, driven on the robot's own odometry | final pose |
| `straight-1..3` | 2.5 m straight line | `measured_distance_m` |
| `spin-1..3` | about two full turns in place | `measured_heading_change_rad` (unwrapped) |

Every run starts at `(0, 0, 0)` with the encoders reset. The robot numbers used to record the
file (`ticks_per_rev`, `nominal`) are inside it. The robot was simulated by `make_logs.py`. Don't
read that file before you're done, because the hidden truth is in it.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `fit_wheel_radius(angles, distances)` | 1-parameter linear least squares from straight runs |
| `fit_wheel_separation(left, right, heading_changes)` | 1-parameter linear least squares from spins |
| `umbmark_summary(cw_errors, ccw_errors)` | centers of gravity and `E_max,syst` |
| `borenstein_correction(x_cg_cw, x_cg_ccw, side, b)` | Borenstein & Feng's closed-form factors |
| `replay_odometry(ticks, r_left, r_right, b, ticks_per_rev)` | exact-arc odometry over a log, one radius per wheel |
| `pose_residuals(params, runs, ticks_per_rev)` | residual vector: odometry end pose minus measured end pose |
| `calibrate_least_squares(runs, ticks_per_rev, initial)` | Gauss-Newton on (r_left, r_right, b) |

## Check

```bash
python course.py check 09.05              # your code
python course.py check 09.05 --solution   # the reference
```

The checker verifies hand-computed cases, then uses the log:

* uncalibrated, UMBmark `E_max,syst` is over **1 m**;
* one Borenstein correction must shrink it to less than 60 % of that;
* least squares on all 16 runs must find each parameter within **1 %** of the hidden truth,
  the right/left radius ratio within **0.2 %**, and bring `E_max,syst` under **5 cm**.

## Hints

* Replaying a log is 09.04's `integrate_pose` in a loop, with `radius_left` and `radius_right`.
* A numerical Jacobian needs no calculus: nudge one parameter by a tiny relative amount and
  divide the change in the residuals by the nudge.
* Try `calibrate_least_squares` on the squares alone. The ratio comes out fine but the overall
  scale drifts, because a closed square looks the same at any size. The lesson explains why the
  straight runs are needed.
