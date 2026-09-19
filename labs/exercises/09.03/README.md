# 09.03 — Dead reckoning: Euler, midpoint and exact arcs

Lesson: [09.03 Dead reckoning — integrating motion](../../../09-odometry/09.03-dead-reckoning-integration.md)

Given the robot's forward speed `v` and yaw rate `omega` over a short time step, move the pose
forward. Three ways to do it, and an experiment that shows how their error depends on the step.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `euler_step(pose, v, omega, dt)` | drive along the heading at the start of the step |
| `midpoint_step(pose, v, omega, dt)` | drive along the heading at the middle of the step (RK2) |
| `exact_step(pose, v, omega, dt)` | drive the exact circular arc (straight-line special case) |
| `integrate(step, pose, twists, dt)` | apply a step function to a sequence of `(v, omega)` samples |
| `constant_twist_error(step, v, omega, duration, dt)` | position error of a method vs the exact arc |

`wrap(theta)` is given. Each stub raises `NotImplementedError`. Replace the body and keep the signature.

## Check

```bash
python course.py check 09.03              # your code
python course.py check 09.03 --solution   # the reference
```

What the tests check:

* one big step (1 m/s, π/2 rad/s, 1 s): Euler lands at (1, 0), midpoint at (0.707, 0.707),
  exact at (0.637, 0.637);
* the time-step experiment: 5 s at 0.5 m/s and 2 rad/s with dt = 0.1 s gives Euler 48.0 mm,
  midpoint 0.80 mm, exact 0. Halving dt halves Euler's error and quarters the midpoint error;
* encoder-derived twists from the course simulator: exact arcs at 50 Hz match the perfect robot
  within 5 mm, and at 10 Hz Euler is several times worse than midpoint.

## Hints

* All three methods agree when `omega = 0`. Only the turning case differs.
* Wrap the heading with `wrap()` after every step, not only at the end.
* In `constant_twist_error`, the truth is a single `exact_step` over `round(duration/dt) * dt`.
