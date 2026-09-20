# 14.05 — Analytic IK and the branch you actually send

Lesson: [14.05 Inverse kinematics — from a point in space to joint angles](../../../14-robotic-arm/14.05-inverse-kinematics.md)

Exercise **14.05-E2**. A 2-link arm has a closed-form inverse: one law of cosines, one `atan2`,
and *two* answers. Getting the maths right is the easy half. The other half is knowing how many
answers there are before you look, and choosing between them so the arm does not flip its elbow
through a 180° arc in the middle of a pick.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `reachability(l1, l2, r)` | `"inside"`, `"boundary_inner"`, `"interior"`, `"boundary_outer"` or `"outside"` — how many solutions exist, from the target distance alone |
| `two_link_ik(l1, l2, x, y)` | every solution: `[]`, one on a boundary, two in the interior (elbow `"up"` first) |
| `pick_solution(solutions, q_current)` | the branch that moves the arm least, so it never flips mid-task |

`wrap_angle`, `planar_fk` and the `TwoLinkSolution` dataclass are given (`planar_fk` is 14.04's
forward kinematics — the tests round-trip every answer back through it).

## Check

```bash
python course.py check 14.05              # your code
python course.py check 14.05 --solution   # the reference, to see what passing looks like
```

26 tests, well under two seconds, no hardware. They pin the by-hand answers of 14.05-E1, push a
few hundred random reachable targets through IK and back through FK, check both branches and both
boundaries, and reproduce the elbow flip of 14.05-E5 and its one-line fix.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Start with `reachability`. Once `two_link_ik` can ask "how many solutions are there?", the rest
  of it is two lines of trigonometry with no special cases.
* Test the two **boundaries before** `"inside"`. When `l1 == l2` the dead hole has radius zero, so
  `r = 0` is the folded boundary, not an unreachable point.
* `cos(q2)` computed from floats lands at `1.0000000000000002` on the stretched boundary, and
  `sqrt(1 - c*c)` is then `sqrt(-4e-16)` — a NaN that travels all the way to the servo without
  raising anything. Clamp `c2` into `[-1, 1]`.
* The elbow-up branch is `sin(q2) < 0`, i.e. `q2 < 0`. If your two solutions come out in the other
  order, you have the sign of `s2` backwards, not a different convention.
* `q1 = atan2(y, x) - atan2(l2*sin q2, l1 + l2*cos q2)`. The subtraction can leave `q1` outside
  (-pi, pi]; wrap it.
* In `pick_solution`, wrap every difference before taking its absolute value. This is the whole
  bug in 14.05-E5: without wrapping, a 2° move across the ±180° seam looks like 358°.
