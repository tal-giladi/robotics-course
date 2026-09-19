# 14.10 — The gate, the statistics and the correction

Lesson: [14.10 Exercise lesson: move the gripper to a specified position](../../../14-robotic-arm/14.10-move-gripper-to-position.md)

Three things that turn "the arm moved" into "the arm did what I asked, and here is the number":
the workspace box that refuses a target before anything is solved, the two statistics that must
never be reported as one, and the correction that removes the systematic half of the error —
validated on points it was not fitted to.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `WorkspaceBox` | `violations` (naming the face), `contains`, `clearance` |
| `accuracy_and_repeatability(measured, commanded)` | the two numbers, separately |
| `fit_offset(commanded, measured)` | the best constant correction |
| `fit_affine(commanded, measured)` | the best $Ap + b$ correction |
| `apply_affine(A, b, points)` | apply it to one point or many |
| `residuals(commanded, corrected)` | per-point error magnitudes |
| `holdout_validate(commanded, measured, train, test)` | fit on one set, score on the other |

## Check

```bash
python course.py check 14.10              # your code
python course.py check 14.10 --solution   # the reference, to see what passing looks like
```

23 tests, about a second, no hardware. The "arm" is a known affine distortion of the commanded
points — which is what a joint-angle calibration bias looks like in task space. Two tests are
the point of the whole exercise: a real correction generalises to held-out points, and a fit to
pure noise does not.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* A refusal that does not name the face it hit sends the reader back into the code. Include the
  axis, the value and the bound in the message.
* Accuracy is `|mean(measured) - commanded|`; repeatability is `max |measured_i - mean(measured)|`
  — about the **mean**, not about the target. An arm can be perfectly repeatable and completely
  wrong, and that is the *good* case.
* For a constant offset the least-squares answer is just the mean of `commanded - measured`; no
  solver needed.
* `fit_affine` is one `lstsq` once you append a column of ones to `measured`. Watch the transpose:
  `lstsq` gives you a (4, 3) solution whose first three rows are $A^T$.
* `apply_affine` must handle a single (3,) point and an (N, 3) array. `points @ A.T + b`
  broadcasts correctly for the second; check `ndim` for the first.
* `holdout_validate` refusing overlapping indices is not pedantry — scoring a fit on a point it
  was fitted to is the mistake the function exists to prevent.
