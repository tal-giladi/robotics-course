# 15.09 — Keeping the planner's world model true

Lesson: [15.09 Collision avoidance, the planning scene and failure recovery](../../../15-manipulation/15.09-collision-avoidance-and-recovery.md)

The planner is only as good as the world you told it about. This exercise is the four pieces that
keep that world honest — and each one is a bug you will otherwise meet at 23:00 with the arm
refusing to move.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `box_distance(box, point)` | the arithmetic under every collision check, negative inside |
| `collides(T_tool, obstacles, …)` | the id of the first obstacle the gripper is inside |
| `reconcile(scene, detections, …)` | keep the *same* id for the same physical object across frames |
| `grasp_allowances(target_id)` | the ACM entries a grasp needs, and only those |
| `recovery_plan(failure, attempt, holding)` | the escalation ladder, one more rung per attempt |

`CollisionBox`, `PlanningScene`, `SceneDiff`, `GRIPPER_SPHERES`, `gripper_points`, `Failure`,
`LADDER`, `HOLDING_PREFIX` and `NEEDS_EMPTY_GRIPPER` are given.

## Check

```bash
python course.py check 15.09              # your code
python course.py check 15.09 --solution   # the reference, to see what passing looks like
```

28 tests, under a second. The ones worth reading before you start:
`test_a_still_table_produces_no_diff` (at 10 Hz, re-publishing every object every frame is 10
scene updates a second for a table that is not moving), `test_the_table_is_never_removed` and
`test_the_attached_object_is_not_removed_for_being_unseen` (a reconciler that deletes whatever it
did not see deletes the table on frame one and the object in your gripper on frame two), and
`test_a_short_object_needs_the_table_allowance` (a 24 mm eraser is grasped 9 mm above the table,
which is inside the padded table).

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* `box_distance` has exactly one non-zero term: `norm(max(d, 0))` when the point is outside,
  `min(max(d), 0)` when it is inside. Rotate into the box's frame by `-yaw` first.
* `collides` returns the **id**, not a bool. A checker that says "collision" without saying with
  what turns every failure into an afternoon.
* `reconcile`'s greedy match must **claim** ids — two detections near the same old object must not
  both match it, or you will publish two boxes with the same id and one of them will vanish.
* Send nothing for an object that did not move. `move_threshold_m` exists because perception noise
  is a few millimetres and `/planning_scene` is not free.
* The `HOLDING_PREFIX` only applies to failures whose recovery assumes an empty gripper
  (`NEEDS_EMPTY_GRIPPER`). Prepending "put the object down" to `DROPPED` — which happens *because*
  you no longer have it — is nonsense, and there is a test for that.
