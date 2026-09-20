# 14.04 — Forward kinematics of a real chain

Lesson: [14.04 Forward kinematics — from joint angles to a pose in space](../../../14-robotic-arm/14.04-forward-kinematics.md)

Exercise **14.04-E2**. Forward kinematics is one matrix product per joint, and every arm you will
ever meet in ROS is described this way. Write it once, against the real SO-101, and MoveIt, RViz
and the Jacobian of [14.06](../../../14-robotic-arm/14.06-jacobian.md) stop being magic.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `rpy_to_matrix(roll, pitch, yaw)` | the URDF/ROS fixed-axis convention $R = R_z(yaw)R_y(pitch)R_x(roll)$ |
| `axis_angle_to_matrix(axis, angle)` | Rodrigues' formula, for a joint turning about its own axis |
| `joint_transform(joint, q)` | $T_{parent,child}(q) = \text{origin} \cdot \text{Rot}(axis, q)$ |
| `chain_frames(joints, q)` | `[(child link, T_base_child)]` for the whole chain, one `q` per revolute joint |
| `fk(joints, q)` | $T_{base,tool}(q)$ — the last frame |
| `approach_pitch(T)` | how far the tool's z axis points below the horizontal |

`Joint`, `se3`, `rot_x/y/z` and the SO-101's joint list (`SO101_JOINTS`, copied from the published
URDF) are given.

## Check

```bash
python course.py check 14.04              # your code
python course.py check 14.04 --solution   # the reference, to see what passing looks like
```

22 tests, under a second, no hardware. They compare your `fk` against the SO-101's reference poses
at $q = 0$ and at $q = (20, -30, 40, 60, 0)°$, check that every joint's frame origin lies on that
joint's own axis, and pin the tool pose of 14.04-E5.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Do `rpy_to_matrix` and `axis_angle_to_matrix` first and check them against `rot_x/y/z`; every
  later failure is otherwise impossible to localise.
* **The one that catches everybody:** `origin @ Rot(axis, q)`, not `Rot(axis, q) @ origin`. The
  `<axis>` of a URDF joint lives in the *child* frame, so it multiplies on the right. Both orders
  give the identical answer at $q = 0$, which is the pose you would check by eye.
* A consequence worth testing yourself: turning a joint must never move that joint's own frame
  origin. If it does, the order is wrong.
* `chain_frames` consumes one value from `q` per **revolute** joint and none for a `fixed` one.
  The SO-101 has five revolute joints and one fixed joint to the tool frame, so `q` has five
  values and you get six frames.
* Accumulate with `T = T @ joint_transform(...)` and append `T.copy()`. Appending `T` itself hands
  every caller the same array, which you then keep multiplying.
* The URDF writes $\pi$ as `3.14159`. That is why the "zero" entries of the reference matrices are
  around $10^{-5}$ rather than exactly 0, and why the tests compare with a tolerance.
* `approach_pitch` uses `atan2(-a_z, hypot(a_x, a_y))`, not `asin(-a_z)`: `asin` loses precision
  exactly where you need it, with the gripper pointing at the table.
