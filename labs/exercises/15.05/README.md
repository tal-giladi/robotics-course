# 15.05 — Generate, filter, rank: a top-down grasp planner

Lesson: [15.05 Grasp planning — heuristics first, learned grasps second](../../../15-manipulation/15.05-grasp-planning.md)

An `ObjectPose` and a cluster go in; a ranked list of grasps comes out, each carrying the four
numbers that explain its score. When this passes you have the baseline every learned grasp model
has to beat — and, unlike one, you can debug it.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `footprint_hull(points)` | the convex hull of the footprint — the right model for a jaw |
| `width_along(hull, d)` | how wide the object is along a closing direction |
| `contact_points(hull, centre, d)` | where the pads touch, both slid onto the closing line |
| `hull_normal_at(hull, p)` | the inward normal of the nearest hull edge — the direction a pad pushes |
| `grasp_candidates(points, pose, gripper, …)` | generate every yaw, filter the impossible, score and sort |

`Contact`, `epsilon_quality` (from 15.01), `JawGripper`, `ObjectPose`, `ScoreWeights` and `Grasp`
are given.

## Check

```bash
python course.py check 15.05              # your code
python course.py check 15.05 --solution   # the reference, to see what passing looks like
```

The footprints are hand-checkable: a 60 × 30 mm block at 0° and 25°, a 100 mm square, a disc.
The tests pin the filter (no returned grasp exceeds the stroke; a 150 × 72 mm object returns an
empty list from a 45 mm jaw), the geometry (both contacts on the closing line; the narrowest
direction of a block at 25° is 115°), the scoring (the stability term equals
`epsilon_quality / 0.3` on the same contacts; corner grasps score exactly 0; higher friction never
lowers it), and the interactions (an obstacle changes **only** the clearance term).

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* `contact_points` must **slide both contacts onto the closing line through the centre**. Skip
  that and the two pads act along parallel but different lines, so $\varepsilon$ comes out wrong
  and a test that checks `(c - centre) · perp == 0` will tell you so.
* `hull_normal_at` must flip the edge normal to point **into** the hull. The wrong sign gives
  $\varepsilon = 0$ everywhere, which looks like a physics result and is a bug.
* Filtering is `continue`, not a low score. There is a test asserting that every returned grasp
  satisfies `width + open_margin <= stroke`.
* The stability term is relative to the contacts' **midpoint**: pass `c1 - mid` and `c2 - mid`
  with `com=(0, 0)`, not the raw base-frame positions.
* The clearance term looks *outside* each contact (`c1 - d * clearance_m` and
  `c2 + d * clearance_m`) — that is where the finger body will be, not where the pad face is.
* Sample yaw over `[0, 180)`, not `[0, 360)`. A parallel jaw closing at $\theta$ and $\theta + 180°$
  makes the identical grasp; a test counts exactly 36 candidates at a 5° step.
