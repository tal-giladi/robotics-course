# 15.06 — The three Jacobians of a visual servo loop

Lesson: [15.06 Visual servoing — closing the loop through the camera](../../../15-manipulation/15.06-visual-servoing.md)

A visual servo is three Jacobians in a row. This exercise is all three, plus the two control laws
that sit on top of them. When it passes you can write the loop for any camera and any arm.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `interaction_matrix_point(x, y, Z)` | the 2×6 image Jacobian of one normalised point feature |
| `interaction_matrix(features, depths)` | stack it for N features → (2N, 6) |
| `damped_pinv(L, damping)` | the damped pseudo-inverse, 14.06's idea applied to the image |
| `ibvs_velocity(s, s*, Z, gain)` | `v_cam = -gain · L⁺ (s − s*)` |
| `pbvs_velocity(T_cam_obj, T*, gain)` | the pose-error law, sign and all |
| `camera_twist_to_tool_twist(v, T_base_cam, T_base_tool)` | rotate **and** move the reference point |

`project`, `rot_of`, `rotvec_of`, `skew` and `inv_T` are given.

## Check

```bash
python course.py check 15.06              # your code
python course.py check 15.06 --solution   # the reference, to see what passing looks like
```

The test that matters is `test_interaction_matrix_matches_numerical_derivative`: it differentiates
the actual projection and compares column by column, so a single wrong sign fails it. The others
pin the pseudo-inverse against `np.linalg.pinv`, check that 100 IBVS steps shrink the image error
by 20×, that PBVS answers a pure roll with a pure rotation (that is the camera-retreat result in
one assertion), and that a camera 40 mm above the tool rotating at 1 rad/s produces 40 mm/s of
tool translation.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Everything is in **normalised** image coordinates, never pixels: `x = (u − cx)/fx`. That is what
  makes the interaction matrix independent of the lens.
* The twist order is linear first: `(vx, vy, vz, wx, wy, wz)`, in the **optical** frame
  (x right, y down, z forward).
* IBVS has a minus sign (the error is `s − s*` and you want to reduce it); PBVS has a plus sign
  (the error already *is* the motion the camera should make). Getting this backwards makes the
  camera run away, which is easy to spot and easy to misdiagnose as a bad gain.
* `camera_twist_to_tool_twist` is where people lose an afternoon. A twist is a velocity **at a
  point**; changing frames changes both the coordinates *and* the point. The two tests
  `test_rotation_adds_the_lever_arm_term` and `test_no_lever_arm_when_the_frames_coincide` isolate
  exactly that.
* The flattening order of the error must match the stacking order of the interaction matrix:
  feature 0's x, feature 0's y, feature 1's x, … `np.reshape(-1)` on an (N, 2) array does this.
