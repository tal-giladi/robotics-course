# 20.02 — Wire, fuse, and where the mass ended up

Lesson: [20.02 Hardware integration](../../../20-final-robot/20.02-hardware-integration.md)

Four pieces of arithmetic stand between "it is all bolted on" and "it still works when the motors
stall". Each one has a standard mistake, and the tests pin all four.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `wire_drop_v(awg, length_m, amps)` | voltage lost in a run — **both** conductors |
| `fuse_rating_a(continuous_a, ampacity_a)` | the standard blade fuse that protects the wire |
| `centre_of_gravity(parts)` | `(mass, x, y, z)` of a list of `(mass, x, y, z)` parts |
| `tipping_decel(x_cg_m, z_cg_m)` | the braking that puts karmel on its nose |

## Check

```bash
python course.py check 20.02
python course.py check 20.02 --solution
```

The last tests compare your numbers with
[`20-final-robot/code/power_budget_v2.py`](../../../20-final-robot/code/power_budget_v2.py) for
the robot as built, and for the same robot with the arm mounted on the front deck.

## Hints

* The current goes out along one conductor and back along another, so the loop resistance is
  `2 * length * ohm_per_m`. Halving this is the standard way to convince yourself that a
  brown-out is impossible.
* `fuse_rating_a` has two bounds: above `1.25 x continuous` so it does not nuisance-blow, at or
  below the wire's ampacity so the fuse — not the wire — is the weak point. When no standard
  value satisfies both, the answer is thicker wire, so raise `ValueError`.
* The centre of gravity is a weighted mean; there is no geometry in it.
* For tipping, draw the free-body diagram first: gravity down at the CG, inertia forward at the
  CG, the pivot at the wheel axle. `x_cg >= 0` means nothing in front holds the nose down at all.
