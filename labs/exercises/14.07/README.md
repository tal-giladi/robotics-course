# 14.07 — Time-parameterised trajectories

Lesson: [14.07 Trajectories — joint vs Cartesian, trapezoidal and cubic profiles](../../../14-robotic-arm/14.07-trajectories.md)

The generator that turns "go there" into position, velocity and acceleration at every instant —
and the pre-flight check that refuses a trajectory the arm cannot execute. When this passes you
can produce exactly what `joint_trajectory_controller` ([14.08](../../../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md))
and MoveIt consume.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `trapezoid_profile(q0, q1, v_max, a_max)` | the minimum-time profile, degenerating to a triangle when the move is short |
| `profile_with_duration(q0, q1, duration, a_max)` | the same move stretched to an exact duration — the key to synchronisation |
| `sample_profile(profile, t)` | position, velocity and acceleration at arbitrary times, clamped to the move |
| `cubic_coefficients(q0, q1, T, v0, v1)` | the four coefficients from four boundary conditions |
| `polynomial_sample(coefficients, t)` | evaluate a polynomial and its first two derivatives |
| `min_duration_cubic` / `min_duration_quintic` | the shortest legal duration for each polynomial |
| `synchronize(q0, q1, v_max, a_max)` | one profile per joint, all sharing the slowest joint's duration |
| `violations(t, qd, qdd, v_max, a_max)` | every limit a sampled trajectory breaks — run this before anything moves |

The `Profile` dataclass is given.

## Check

```bash
python course.py check 14.07              # your code
python course.py check 14.07 --solution   # the reference, to see what passing looks like
```

38 tests, under a second, no hardware. They pin the hand-computed 90° move (0.5 / 1.0 / 0.5 s),
the triangle case, the exact break-even distance, the integral of your velocity against the
distance travelled, the closed-form peaks of the cubic, the quintic-beats-cubic-only-on-short-moves
crossover, and the truncated-deceleration bug from exercise 14.07-E5.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* The break-even distance is $v_{\max}^2/a_{\max}$ — compare against it *before* you compute
  anything else, and the triangle case stops being a special case you forgot.
* `profile_with_duration` solves $D = v(T - v/a)$, a quadratic in $v$. Take the **smaller** root.
  The larger one describes an arm that races past the goal and comes back.
* Sample with a boolean mask per phase (`t <= t_accel`, cruise, the rest) rather than a Python
  loop — and compute the braking phase backwards from the end (`T - t`), which makes the final
  sample land exactly on `q1` instead of a few ULPs past it.
* `sample_profile` must handle a zero-distance profile (every array all zeros) — `synchronize`
  produces those for joints that do not move.
* `violations` takes `(N, n)` arrays. `np.atleast_2d` and `np.broadcast_to` let the same code
  accept a scalar limit or a per-joint one.
