# 14.09 — Collision checking and the resolution that decides whether it works

Lesson: [14.09 MoveIt 2 — motion planning and collision checking](../../../14-robotic-arm/14.09-moveit2-motion-planning.md)

What `move_group` calls tens of thousands of times per plan, in miniature: links as capsules,
obstacles as spheres, a state check, a path check, and the arithmetic that turns MoveIt's
`longest_valid_segment_fraction` into millimetres of gripper motion.

No ROS and no MoveIt: a 3-DOF toy arm's forward kinematics are given, everything else is numpy.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `closest_point_on_segment(a, b, p)` | the nearest point of the **segment** (clamp the parameter!) |
| `capsule_sphere_clearance(a, b, r, c, R)` | signed clearance; negative means overlap |
| `state_clearance(q, obstacles)` | the worst link-vs-obstacle clearance at one pose |
| `interpolate_states(q0, q1, max_step)` | straight-line joint interpolation, endpoints included |
| `path_clearance(q0, q1, obstacles, max_step)` | the worst clearance along the path, and where |
| `required_segment_fraction(limits, ‖J‖, tool_step)` | MoveIt's fraction for a wanted tool resolution |

`arm_points`, `link_segments` and `max_jacobian_column` are given.

## Check

```bash
python course.py check 14.09              # your code
python course.py check 14.09 --solution   # the reference, to see what passing looks like
```

28 tests, about a second. One of them is the whole lesson in a single assertion: the same obstacle
is **missed** by a 5-step check and **found** by a 200-step one.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Clamp the projection parameter to $[0, 1]$. Without it you get the nearest point on the infinite
  line, and the arm reports collisions with things behind its own base.
* Clearance subtracts **both** radii from the centre-to-axis distance. Forgetting the capsule's
  radius makes every check optimistic by 2 cm, which is exactly the margin you were trying to keep.
* `interpolate_states` returns `ceil(d / max_step) + 1` states. The `+ 1` is the goal state; drop
  it and you have a checker that never checks where the arm ends up.
* `state_clearance` must check **every** link, not just the last one. Most real collisions are the
  elbow, not the gripper.
* `required_segment_fraction` uses the **sum** of the joint ranges as the state-space extent,
  because that is what MoveIt's joint-space state space reports as its maximum extent.
* Refining the step is not monotone: a finer sample set does not contain the coarser one, so a
  finer check can report slightly *more* clearance. Only a dense reference tells you the truth —
  which is itself the reason a guarantee has to come from the formula, not from "it worked".
