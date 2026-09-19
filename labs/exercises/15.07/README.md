# 15.07 — From a grasp to an executable reach

Lesson: [15.07 Exercise lesson: detect an object and move the gripper toward it](../../../15-manipulation/15.07-detect-and-approach.md)

[15.05](../../../15-manipulation/15.05-grasp-planning.md) hands you a position, a yaw and a width.
Between that and a moving arm sit a rotation matrix, four waypoints and a handful of checks that
cost microseconds and save fingers. This exercise is all three.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `approach_axis(pitch, azimuth)` | the unit vector the gripper travels along |
| `grasp_tool_pose(position, jaw_yaw, pitch)` | the 4×4 tool pose: z = approach, x = closing |
| `jaw_yaw_of(T)` | read the closing direction back out, as an *axis* |
| `approach_waypoints(T_grasp, standoff, lift)` | pre_grasp → approach → grasp → lift |
| `check_plan(waypoints, …)` | every problem you can find without an IK solver |
| `lateral_tolerance_m(stroke, width)` | the millimetres your error budget has to fit inside |

`Waypoint`, `se3` and `wrap_axis` are given.

## Check

```bash
python course.py check 15.07              # your code
python course.py check 15.07 --solution   # the reference, to see what passing looks like
```

23 tests, about a second. They pin the geometry (the rotation is orthonormal with determinant +1
at every approach angle; the jaw yaw round-trips; 30° and 210° are the same grasp), the waypoints
(a 100 mm standoff at a 60° approach really is 100 mm back **along the approach axis**, and the
lift is vertical in the *world* even then), and the checks (out of reach, too close to the base,
out of order, a non-descending approach).

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* `pitch` is measured **below the horizontal** — π/2 is straight down. That matches
  `arm_kinematics.approach_pitch`, which is what the IK solver of
  [14.05](../../../14-robotic-arm/14.05-inverse-kinematics.md) takes.
* The closing direction must be **projected perpendicular to the approach axis** and renormalised.
  Skip it and you get a matrix that is not a rotation; nothing will complain until the arm moves.
* `jaw_yaw_of` must wrap to (−π/2, π/2]. A parallel jaw closing at θ and at θ + 180° is the
  identical grasp, and comparing unwrapped angles is how a correct arm looks 180° wrong.
* The lift is along **world +z**, not along the tool axis. If the grasp slips, the object should
  fall back onto the table rather than sideways into its neighbour.
* `check_plan` returns problems, not a bool, and each string should name the waypoint. "Plan
  rejected" is not a diagnosis.
* These checks do not replace IK. The lesson's standoff grid shows a plan that passes every one of
  them and that the SO-101 still cannot execute.
