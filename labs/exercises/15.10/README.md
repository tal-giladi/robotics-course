# 15.10 — Support polygon, centre of mass, and the frame chain

Lesson: [15.10 Mobile manipulation — the arm on the moving base](../../../15-manipulation/15.10-mobile-manipulation.md)

Bolting the arm to a moving base adds three questions that did not exist on a table: what is
holding the robot up, where is the mass once the arm is out, and where is the object from the
*arm's* point of view. This exercise is all three, plus the decision they feed.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `convex_hull_2d(points)` | the hull of the ground contacts, counter-clockwise |
| `support_polygon(track, caster_x, extra_caster_x)` | karmel's contacts, hulled |
| `combined_com(parts)` | mass-weighted centre of base + arm + payload |
| `stability_margin(support, com_xy)` | metres to the nearest edge; **negative means it tips** |
| `object_in_arm_frame(base_xy_yaw, mount_xyz, object_map_xyz)` | map → base_link → arm base |
| `should_use_arm(manipulability, margin)` | move the arm, or re-dock the base |

## Check

```bash
python course.py check 15.10              # your code
python course.py check 15.10 --solution   # the reference, to see what passing looks like
```

26 tests, under a second. The two you should read before writing anything:

- `test_karmel_support_is_a_triangle_ending_at_the_axle` — the single ball caster is 100 mm in
  *front* of the wheels, so **nothing holds this base up behind them**: the support polygon is a
  triangle whose rear edge is the line `x = 0` and whose front narrows to a point at `x = +0.100`.
  Every stability result in the lesson follows from that one shape.
- `test_the_mount_offset_is_subtracted_in_the_base_frame_not_the_map` — rotate first, then subtract
  the mount. The arm is bolted to the robot, not to the world, and getting it backwards is correct
  for every hand-written test where yaw = 0.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* `convex_hull_2d` must drop points that are strictly inside. A wheel inside the hull carries load
  and contributes nothing to stability, so leaving it in makes the robot look more stable than it
  is — the direction of error you least want.
* The shoelace area of your hull must be **positive**; `stability_margin` relies on the ring being
  counter-clockwise so the left normal of each edge points inward.
* `stability_margin` returns the smallest magnitude either way, signed by whether *every* edge's
  signed distance is non-negative. Do not return the smallest signed value: for a point outside a
  corner, two edges are negative and the nearest one is the answer.
* `combined_com` raises on an empty list or zero total mass. A NaN here becomes a robot that
  believes it is stable.
* `should_use_arm` needs **both** conditions. Manipulability alone lets it tip; margin alone lets
  it work at a singularity.
