# 12.05 — A pure pursuit path follower

Lesson: [12.05 Local planning — pure pursuit, DWB, MPPI and obstacle avoidance](../../../12-navigation/12.05-local-planning-obstacle-avoidance.md)

Turn a global path (a list of points 5 cm apart) into `(v, ω)` commands that drive karmel along it
and stop at the end: the core of Nav2's Regulated Pure Pursuit controller, the one karmel uses.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `to_robot_frame(pose, point)` | map point → base_link coordinates |
| `curvature(x_r, y_r)` | pure pursuit arc: `κ = 2·y_r / (x_r² + y_r²)` |
| `twist_to_wheel_speeds(v, w, r, b)` | `(v ∓ ω·b/2) / r` (lesson 09.02) |
| `lookahead_point(path, position, lookahead, start_index)` | the carrot: where the lookahead circle leaves the path |
| `PurePursuitController.compute(pose)` | goal check → progress index → carrot → rotate in place if needed → approach slow-down → `ω = v·κ` with an angular limit |

The class docstring lists the six rules in order. Each stub raises `NotImplementedError`: replace the
body, keep the signature.

## Check

```bash
python course.py check 12.05              # your code
python course.py check 12.05 --solution   # the reference, to see what passing looks like
```

Hand-computed cases first (frames, curvature 2.0 and 4.0, carrots on a straight line and around a
corner, the steering sign, rotate-in-place, 0.1 m/s at 0.2 m from the goal, the angular clamp keeping
the arc). Then your controller drives `SimBase` at 50 Hz through the apartment, localized by ground
truth, with the wheel sizes from `labs/config/karmel.yaml`:

| Path | Robot | Must |
|---|---|---|
| living room → kitchen, through the doorway | perfect and 2 realistic seeds | max cross-track error **< 8 cm** |
| living room → study, two 90° corners | perfect and 1 realistic seed | max cross-track error **< 15 cm** |

and in every run: set `done` within 45 s, never touch an obstacle, end **within 8 cm** of the goal
with the wheels stopped, and not creep more than 2 cm after `done`.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* Search the closest vertex only forward from the last index. On a path that passes the same spot
  twice, a global search makes the robot turn around.
* `atan2(y_r, x_r)` of the carrot is the heading error; its sign is the turn direction.
* Run `python 12-navigation/code/local_planning.py` to see the reference pure pursuit, RPP, DWA and
  MPPI in the same apartment.
