# 15.01 — The contact model: friction cones, grip force and force closure

Lesson: [15.01 The physics of grasping — force, friction and friction cones](../../../15-manipulation/15.01-physics-of-grasping.md)

Everything a grasp planner needs to know about physics, in seven functions. When this passes, you
can answer "will this grasp hold?" with arithmetic instead of opinion — and
[15.05](../../../15-manipulation/15.05-grasp-planning.md) calls `epsilon_quality` once per grasp
candidate.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `friction_cone_half_angle(mu)` | $\alpha = \arctan \mu$ — the cone that decides whether a finger sticks or slips |
| `required_normal_force(m, mu, …)` | the force per pad that holds `m` against slip, with an acceleration and a safety factor |
| `max_payload_kg(F, mu, …)` | the inverse: what a given squeeze can hold |
| `torsional_friction_moment(F, mu, r)` | the torque a *soft* circular patch resists about its own normal |
| `Contact.force_is_feasible(f)` | is this finger force inside the cone? (push first, then the tangential bound) |
| `is_antipodal(c1, c2)` | does the grasp line lie inside **both** cones? — the planar two-finger closure test |
| `primitive_wrenches(contacts, com)` | the unit wrenches $(f_x, f_y, \tau/L)$ at every cone edge |
| `epsilon_quality(contacts, com)` | Ferrari–Canny $\varepsilon$: the radius of the biggest wrench ball inside the hull |
| `has_force_closure(contacts)` | $\varepsilon > 0$ |

`Contact`, `cross2` and `evenly_spaced_contacts` are given.

## Check

```bash
python course.py check 15.01              # your code
python course.py check 15.01 --solution   # the reference, to see what passing looks like
```

The tests pin the lesson's own numbers (6.79 N per pad for the full can, $\varepsilon = 0.2529$
for the reference pinch, 0.0422 after 15 mm of misalignment) **and** the properties any correct
implementation must have: $\varepsilon$ monotone in $\mu$, adding a finger never lowering it, force
closure implying antipodal for two planar contacts, and `max_payload_kg` inverting
`required_normal_force` exactly.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* `required_normal_force` must **raise** on `mu <= 0` rather than returning `inf`. A frictionless
  pinch holds nothing, and a silent `inf` propagates into a planner as "impossible but plausible".
* `force_is_feasible` checks two things **in order**: the normal component must be non-negative
  (a finger pushes, it never pulls), and only then $|f_t| \le \mu f_n$. Testing the magnitude
  first accepts pulling forces.
* In `is_antipodal`, $d$ runs from `c1` to `c2`, so the angle at the *second* contact is measured
  against $-d$. Coincident contacts must return `False`, not divide by zero.
* `epsilon_quality` must return **exactly `0.0`** in all three degenerate cases (fewer than four
  wrenches, a `ConvexHull` that raises on a non-full-dimensional set, and the origin outside the
  hull). `hull.equations` rows are `[nx, ny, nz, offset]` with unit normals and "inside" meaning
  `n · x + offset <= 0`, so the distance from the origin to the nearest facet is `-max(offsets)`.
* Normalise the force direction after rotating the contact normal. It should already be unit, but
  floating point drifts and the test checks it.
