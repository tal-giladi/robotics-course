# 08.11 — A trapezoidal profile and a wrap-safe heading controller

Lesson: [08.11 Rotate exactly 90° — motion profiles and IMU feedback](../../../08-control/08.11-rotate-exactly-90.md)

A controller told "turn 90°" as a step has to invent its own trajectory, and does it badly. Give it
a trajectory instead: where the robot should be at every instant, and how fast it should be turning
there. Then the feedback only has to fix the difference.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `heading_error(target, measured)` | the shortest signed rotation, wrapped to (−π, π] |
| `TrapezoidalProfile.peak_v / t_accel / t_cruise / duration` | the shape: accelerate at `a_max` to `v_max`, cruise, decelerate. Triangular (`t_cruise = 0`) when the move is too short, with peak speed `sqrt(a_max * |distance|)` |
| `TrapezoidalProfile.velocity(t)` | commanded speed, 0 outside the move |
| `TrapezoidalProfile.position(t)` | commanded position — the **exact** integral of `velocity` |
| `TurnController.target(t)` | `wrap(start_heading + profile.position(t))` |
| `TurnController.update(t, measured, dt)` | `profile.velocity(t) + kp*e + integral`, clamped to ±`max_yaw_rate_rad_s`, with 08.05 anti-windup |
| `TurnController.finished(t, measured, tolerance)` | the profile has run out **and** the heading is inside the tolerance |

Negative distances (clockwise turns) must work everywhere.

## Check

```bash
python course.py check 08.11
python course.py check 08.11 --solution
```

Tests:

* wrapping by hand (+179° → −179° is **+2°**), and agreement with `robotlab.geometry.angle_diff`;
* the 90° profile by hand: ramp 0.50 s, cruise 0.547 s, total 1.547 s, peak 1.5 rad/s;
* a 10° turn is triangular with a peak of 0.7235 rad/s;
* `position` is the numerical integral of `velocity` at every instant, for five different turns;
* on profile the output is *pure feedforward* — the feedback contributes nothing;
* starting at +170°, a +90° turn targets −100° and does not unwind the long way;
* a stalled robot does not wind the integral up;
* end to end on the realistic simulator, heading from the bias-corrected gyro: ten 90° turns, all
  within **±2°**, mean bias under 1°, and no overshoot past 95°.

## Hints

* Compute `peak_v` first; every other property follows from it.
* `position` must use the same `a_max` ramps as `velocity`, or the two disagree and the feedback
  spends the whole turn fighting the feedforward. The test that integrates `velocity` catches this.
* Wrap once, at the end: `((a + pi) % (2 pi)) - pi`.
* `start_heading` is what makes the controller wrap-safe — the profile itself only ever talks about
  a *relative* rotation.
