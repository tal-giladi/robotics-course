# 09.02 — cmd_vel to wheel speeds (with saturation)

Lesson: [09.02 The other way — cmd_vel to wheel speeds](../../../09-odometry/09.02-inverse-kinematics-cmd-vel.md)

A planner, a joystick or Nav2 says "drive at `v` m/s while turning at `omega` rad/s". The Pico
only understands wheel speeds, and the motors have a top speed. Write the layer in between.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `twist_to_wheel_speeds(v, omega, r, L)` | inverse kinematics: body twist to left/right wheel rad/s |
| `wheel_speeds_to_twist(left, right, r, L)` | forward kinematics (09.01), for checking |
| `limit_twist(v, omega, max_linear, max_angular)` | software speed limits, one common scale factor |
| `scale_to_limit(left, right, max_wheel)` | wheel saturation that keeps the arc (curvature) |
| `keep_rotation_to_limit(left, right, max_wheel)` | wheel saturation that keeps the yaw rate instead |
| `cmd_vel_to_wheels(...)` | the pipeline: limit twist, inverse kinematics, saturate |

Each stub raises `NotImplementedError`. Replace the body and keep the signature.

## Check

```bash
python course.py check 09.02              # your code
python course.py check 09.02 --solution   # the reference, to see what passing looks like
```

The tests start with hand-computed cases (wheel radius 0.05 m, separation 0.2 m, so the numbers
are round). The last test drives the course simulator on an arc that asks the right wheel for
22 rad/s when the motors top out at 17 rad/s. The radius the simulated robot actually drives
must be within **5 %** of the 0.233 m you asked for. Clipping only the fast wheel drives a
0.32 m arc instead, and the test fails.

A skipped test means that part is not implemented yet. The check passes only when nothing is
skipped.

## Hints

* Write the wheel rim speeds first (`v ∓ omega·L/2`), then divide by `r`.
* Saturation strategies differ in *what they give up*: scaling gives up speed, keep-rotation
  gives up forward motion. Neither changes the direction of the turn.
* `math.copysign(value, sign_source)` is handy for the pure-spin case.
