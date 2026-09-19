# 15.02 — Gripper models and the selection logic

Lesson: [15.02 Grippers — parallel jaw, compliant, suction](../../../15-manipulation/15.02-grippers.md)

Three small physical models plus the decision procedure that turns "which gripper?" into
arithmetic. When this passes, you can look at an object and say — with numbers — whether your jaw
can pick it, whether a cup can, and which reason rules each one out.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `kgcm_to_nm(kg_cm)` | the datasheet unit conversion: 1 kg·cm = 0.0981 N·m |
| `servo_jaw_force(stall, lever, …)` | pad force from servo torque, at the course's 30% torque limit |
| `suction_force_n(dp, d, eta)` | $F = \Delta p \cdot A \cdot \eta$ with $A = \pi (d/2)^2$ |
| `JawGripper.fits(width)` | does the object fit **with approach clearance**? |
| `JawGripper.max_payload_kg(…)` | payload from pad force and pad friction |
| `SuctionGripper.normal_force_n / shear_force_n / max_payload_kg` | pull-off, sideways, and payload |
| `evaluate(obj, jaw, suction)` | one verdict sentence per gripper type, with the reason |

`TargetObject`, `MU_TABLE`, `required_normal_force` and `payload_from_force` are given.

## Check

```bash
python course.py check 15.02              # your code
python course.py check 15.02 --solution   # the reference, to see what passing looks like
```

The tests pin the lesson's numbers (6.867 N for a 15 kg·cm servo at the 30% limit, 13.19 N and
610 g for a 20 mm cup at 60 kPa, 259 g of payload at $\mu = 0.6$) and the scaling laws ($d^2$ for
suction, linear in $\mu$ for the jaw). For `evaluate` they check the **decision**, not the
wording: each verdict must start with `yes`, `no`, `risky` or `works`, and the right one must be
chosen for each of nine object/gripper situations.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* `fits` is `width + clearance <= stroke`, not `width <= stroke`. Half the exercise is remembering
  that the jaws must come down *around* the object.
* In `evaluate`, the **order** of the checks is what is tested, because the order is what makes
  the reason correct. Porosity is checked before flatness: a cardboard box has a perfectly flat
  face and still cannot hold a vacuum. Stroke is checked before force: an object that does not fit
  is not "too heavy".
* A test asserts that the parallel-jaw verdict for an object that fits but is too heavy does
  **not** mention fitting. Say why you refused, precisely.
* `max_payload_kg(mu=...)` overrides `pad_mu`; with no argument it must use `pad_mu`.
* `servo_jaw_force` must raise `ValueError` on a zero or negative lever arm — it is a division, and
  an `inf` force silently propagates into "this gripper can pick anything".
