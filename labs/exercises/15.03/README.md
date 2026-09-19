# 15.03 — Solving AX = XB, and knowing when not to trust the answer

Lesson: [15.03 Hand–eye calibration — eye-in-hand and eye-to-hand](../../../15-manipulation/15.03-hand-eye-calibration.md)

The one transform you cannot measure with a ruler. When this passes, you can take a stack of
`(arm pose, board pose)` records and produce the camera-to-gripper transform that the whole of
[15.04](../../../15-manipulation/15.04-object-pose-estimation.md) onward depends on — and, more
importantly, tell whether the dataset was good enough to believe it.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `rotvec_of(R)` | $\log$ of a rotation: the axis-angle 3-vector, including the 180° case |
| `rot_of(r)` | $\exp$: Rodrigues' formula |
| `motion_pairs(gs, cs, eye_in_hand)` | all $\binom{n}{2}$ relative motions $A_{ij}, B_{ij}$, for both rigs |
| `solve_ax_xb_park(A, B)` | Park & Martin's closed form: rotation by Procrustes, translation by least squares |
| `calibrate_eye_in_hand` / `calibrate_eye_to_hand` | two lines each, given the above |
| `consistency_residual(gs, cs, X)` | the field check: the target did not move, so $g_i X c_i$ must agree |
| `rotation_spread_deg(gs)` | mean relative rotation — "is my dataset varied enough?" |

`rot_xyz`, `make_T`, `inv_T`, `rotation_angle_deg` and `pose_error` are given.

## Check

```bash
python course.py check 15.03              # your code
python course.py check 15.03 --solution   # the reference, to see what passing looks like
```

The tests recover a known $X$ from clean synthetic data to $10^{-9}$, recover it to a couple of
millimetres from realistic noise (0.5° / 1 mm PnP, 0.3° / 0.5 mm FK), check that more poses beat
fewer, check both rigs, and — the point of the lesson — check that a **single-axis dataset fails**
while `rotation_spread_deg` is the thing that catches it.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* `rotvec_of` near $\theta = \pi$: the usual formula divides by $\sin\theta \approx 0$. Use
  $A = (R + I)/2$, whose diagonal holds the squared axis components; take the largest, read the
  signs off that column, normalise. A test rotates by exactly 180° about five different axes.
* `motion_pairs` for **eye-to-hand** is $A_{ij} = g_j g_i^{-1}$, not $g_j^{-1} g_i$. Getting it
  backwards still produces a converged, self-consistent, wrong answer — which is why there is a
  test that checks $AX = XB$ holds exactly on clean data for *both* rigs.
* After computing $R_X = (M^{T}M)^{-1/2} M^{T}$, **re-orthonormalise it with an SVD**. Without that
  you get a near-rotation with a determinant like 0.9997 that quietly poisons everything
  downstream, and a test will catch it.
* `consistency_residual` returns **millimetres and degrees**, not metres and radians. The whole
  point is that a human reads it and decides.
* `lstsq` on a rank-deficient system returns the minimum-norm solution without raising. That is
  exactly what happens on a single-axis dataset, and it is why `rotation_spread_deg` exists.
