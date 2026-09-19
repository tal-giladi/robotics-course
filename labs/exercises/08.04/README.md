# 08.04 — Proportional control: hold a wheel speed

Lesson: [08.04 Proportional control — hold a wheel speed](../../../08-control/08.04-proportional-control.md)

Build the simplest speed controller, then predict exactly how wrong it will be.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `steady_state_speed(setpoint, kp, slope, deadband)` | where a P-only loop settles for a motor with gain `slope` (rad/s per duty) and a deadband: the lesson's Level 3 formula, with the "never starts", saturation and negative-setpoint cases |
| `PController.update(setpoint, measured, dt)` | `kp * (setpoint - measured)`, clamped to ±`output_limit`; stores `error` and `output` |
| `PController.reset()` | clears `error` and `output` |

Each stub raises `NotImplementedError` — replace the body, keep the signature.

## Check

```bash
python course.py check 08.04              # your code
python course.py check 08.04 --solution   # the reference, to see what passing looks like
```

The tests start with hand-computed cases (arithmetic in the comments), then run your `PController`
on both wheels of the **realistic** simulator at 50 Hz with setpoint 8 rad/s and kp = 0.05, 0.1 and 0.2.
Your controller must leave a steady-state error (P always does), and the left wheel must settle
within **±0.15 rad/s** of what your own `steady_state_speed` predicts from karmel.yaml and the
battery voltage. The whole check takes about a second.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* Substitute `u = kp * (r - w)` into `w = slope * (u - deadband)` and solve for `w`.
* The saturation cap is the speed at duty 1.0: `slope * (1 - deadband)`.
* `math.copysign(value, setpoint)` handles negative setpoints in one line.
