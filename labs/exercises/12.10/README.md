# 12.10 — The collision monitor and the keepout filter, exactly like Nav2

Lesson: [12.10 Recovery behaviors, failure handling and navigation safety](../../../12-navigation/12.10-recovery-and-navigation-safety.md)

Two safety mechanisms that sit *outside* the planner, so they still work when the planner is
confused, the costmap is stale or the behavior tree is in the middle of a recovery.

* **`nav2_collision_monitor`** is the reflex: it sits between `cmd_vel_smoothed` and `/cmd_vel`
  and overrides the command from **raw sensor points**, with four action types — `stop`,
  `slowdown`, `limit` and `approach`.
* **`nav2_costmap_2d::KeepoutFilter`** is the prevention: a second map ("filter mask") merged into
  the costmap so the planner never routes through a zone you marked.

When this passes, your numbers are Nav2's numbers: you can set `time_before_collision` and draw a
keepout mask and know exactly what the robot will do.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `points_in_polygon(polygon, points)` | how many points are inside a closed polygon (ray casting) |
| `SafetyPolygon.collision_time(points, vel)` | seconds until the polygon would touch `min_points` of the obstacle points, projecting the robot along the commanded twist |
| `CollisionMonitor.process(cmd_vel, points)` | apply every polygon, return the slowest requirement with the action type and polygon name |
| `mask_cost(value, base, multiplier)` | one filter-mask cell (−1, or 0..100) as a costmap cost |
| `apply_keepout(master, mask, base, multiplier)` | merge the mask into a costmap Nav2's way |

`Velocity`, `ActionType`, `MonitorAction`, `SafetyPolygon`'s fields, `project_state` and
`transform_points` are given.

## Check

```bash
python course.py check 12.10              # your code
python course.py check 12.10 --solution   # the reference, to see what passing looks like
```

The tests use karmel's shipped `FootprintApproach` polygon (0.27 × 0.23 m, `min_points: 6`,
`time_before_collision: 1.2`, `simulation_time_step: 0.1`) driving at 0.25 m/s at a wall, an
arcing command that sweeps into a wall the straight command would miss, each of the four actions
in isolation, two polygons competing, and the keepout merge including its unknown-space rule.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* In `collision_time`, project the pose **before** checking, and return the loop's `time`, not the
  time you just simulated. Nav2's returned time is one `simulation_time_step` behind the pose, and
  the tests pin the resulting values (0.2 / 0.6 / 1.0 s for a wall at 0.2 / 0.3 / 0.4 m).
* `transform_points` moves the *points* into the robot's new frame — the polygon never moves.
* `approach` produces a candidate whenever `collision_time >= 0`, including `0.0`, which scales the
  command to a full stop.
* `limit` must keep the sign: a robot backing away from an obstacle is still allowed to back away.
* `apply_keepout` is not `np.maximum`: a mask value must also overwrite a master cell that is
  `NO_INFORMATION` (255), even when the new cost is lower.
