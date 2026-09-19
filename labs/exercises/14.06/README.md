# 14.06 — The Jacobian toolkit

Lesson: [14.06 The Jacobian — velocities, singularities and resolved-rate control](../../../14-robotic-arm/14.06-jacobian.md)

The six functions every Cartesian controller in this course is built on. When they pass, you can
convert between joint speeds and tool speed, see a singularity coming, keep commanding through it,
and answer "can this servo hold that payload?" — all from the same matrix.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `planar_jacobian(lengths, q)` | the (2, n) linear Jacobian, built from geometry: column j is $\hat z \times (\text{tip} - \text{joint}_j)$ |
| `singular_values(J)` | the semi-axes of the velocity ellipse, largest first |
| `manipulability(J)` | Yoshikawa's $w = \sqrt{\det JJ^T}$ — must equal $l_1 l_2 \lvert\sin q_2\rvert$ for the 2-link arm |
| `dls_velocity(J, v, damping)` | damped least squares: $\dot q = J^T(JJ^T + \lambda^2 I)^{-1}v$ |
| `scale_to_limits(qdot, limit)` | respect a servo speed limit **without** changing the direction of the motion |
| `payload_torques(J, force)` | $\tau = J^T f$ — the statics side of the same matrix |
| `worst_case_joint_speed(J, speed)` | $\text{speed}/\sigma_{\min}$: the number to compare against your servo limit |

`planar_joint_points` and `planar_tip` are given (they are 14.04's forward kinematics).

## Check

```bash
python course.py check 14.06              # your code
python course.py check 14.06 --solution   # the reference, to see what passing looks like
```

23 tests, about 3 seconds, no hardware. They compare your Jacobian against a central-difference
derivative at 250 random poses (2-link and 3-link), pin the closed form of $w$, check that
`dls_velocity` reproduces the exact inverse at $\lambda = 0$, stays bounded at an exact
singularity, and actually minimises $\|J\dot q - v\|^2 + \lambda^2\|\dot q\|^2$, and verify the
power duality $f \cdot v = \tau \cdot \dot q$.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Build the Jacobian from geometry, not from finite differences. `planar_joint_points(lengths, q)`
  already gives you every point you need — no calculus, no symbolic algebra.
* In 2D, $\hat z \times r = (-r_y, r_x)$. Getting the sign backwards produces a Jacobian that looks
  plausible and moves the arm the wrong way.
* Column *j* is measured from joint *j* **to the tip**. A one-index slip here is the single most
  common bug: `pts[j]`, not `pts[j + 1]`.
* `manipulability` must survive a wide $J$ (3 joints, 2 task rows), where $\det(JJ^T)$ can come out
  as a tiny *negative* float. Clamp at zero before `sqrt`.
* At $\lambda = 0$, `np.linalg.solve` only works for a square $J$. `np.linalg.pinv` handles the
  redundant case and returns the least-norm solution, which is what the tests expect.
* The SVD of a genuinely singular matrix returns something like `1e-17`, not `0.0`. Compare against
  a tolerance.
