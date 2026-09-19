# 14.08 — The joint map and the JointTrajectory builder

Lesson: [14.08 The arm in ROS 2 — URDF, joint states and ros2_control](../../../14-robotic-arm/14.08-arm-urdf-and-ros2-control.md)

The two pieces of glue between "a number in your Python code" and "a servo register", both of
which are where arms get broken:

* **Units.** The URDF speaks radians from the URDF's zero pose; LeRobot speaks degrees from the
  middle of each joint's calibrated range, with a per-joint sign, and the gripper in percent.
* **The message.** `trajectory_msgs/JointTrajectory` is a list of points with a
  `time_from_start`. Here it is plain dicts with the real field names, so the tests need no ROS.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `JointCalibration` | `sign` and `offset_deg` for one arm joint, and the conversion both ways |
| `GripperCalibration` | the gripper's 0–100 % ↔ radians span |
| `clamp_to_limits(urdf, limits)` | clamp into the URDF limits and report what was clamped |
| `measure_calibration(zero, probe, urdf)` | recover sign and offset from two measured poses (exercise 14.08-E1) |
| `build_joint_trajectory(names, t, q, qd)` | the message, with `time_from_start` taken from `t` |
| `check_trajectory(trajectory, limits)` | everything wrong with it, as strings — never raises |

`ARM_JOINTS` and `URDF_LIMITS` are given.

## Check

```bash
python course.py check 14.08              # your code
python course.py check 14.08 --solution   # the reference, to see what passing looks like
```

29 tests, about a second, no ROS and no hardware.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* `sign * radians(deg - offset)`, in that order. `sign * radians(deg) - offset` passes its own
  round trip (it is self-consistent) and puts the arm somewhere else entirely — there is a test
  for exactly this.
* The gripper needs a *span*, not an offset and a sign: two points define the line, and values
  outside the calibrated range extrapolate rather than clip. Clipping is `clamp_to_limits`' job.
* `measure_calibration` must refuse a joint that barely moved. The sign comes from the product of
  two deltas, and the product of two small noisy numbers is a coin flip.
* `build_joint_trajectory` with `qd=None` must leave the `"velocities"` key **out**, not set it to
  an empty list. To `joint_trajectory_controller` an empty list means "no velocity given, infer
  it" and a list of zeros means "be stationary here" — sending zeros at every point makes the arm
  stop dead at every sample.
* `check_trajectory` is the last gate before a goal leaves your process, so it reports problems
  instead of raising — even for an unknown joint name, which `clamp_to_limits` *does* raise on.
  Different jobs: one is a check, the other is a conversion.
