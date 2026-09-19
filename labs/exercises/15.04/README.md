# 15.04 — Tabletop perception: plane, clusters, and a graspable pose

Lesson: [15.04 Object pose for grasping — from detections and depth to a 3D pose](../../../15-manipulation/15.04-object-pose-estimation.md)

Point cloud in, one `ObjectPose` per object out. When this passes, you can hand
[15.05](../../../15-manipulation/15.05-grasp-planning.md) the four numbers a top-down jaw needs:
centre, yaw, width and top height.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `fit_plane_ransac(points, …)` | RANSAC + an "up" prior + a least-squares refit on the inliers |
| `cluster_points(points, voxel_m, …)` | voxel-grid connected components, largest cluster first |
| `wrap_axis_angle(a)` | wrap an *axis* angle to $(-\pi/2, \pi/2]$ — PCA cannot tell $\theta$ from $\theta + \pi$ |
| `pose_from_cluster(points, table)` | 2D PCA on the footprint → centre, yaw, extents, top height |
| `segment_tabletop(points, …)` | the whole pipeline |

`Plane` and `ObjectPose` are given.

## Check

```bash
python course.py check 15.04              # your code
python course.py check 15.04 --solution   # the reference, to see what passing looks like
```

The clouds are small and synthetic so every expectation is hand-checkable: the eight-point
60 × 30 mm rectangle at 30° from the lesson (yaw 30.000°, extents 60.0 × 30.0 mm, centre
(0.240, 0.060, 0.020)), a table with 30% outliers, a wall with three times as many points as the
table, two blocks 12 mm apart at two voxel sizes, and a 10 mm-thick "phone" disappearing at a
25 mm plane threshold.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* The **refit** is not optional and there is a test for it: the RANSAC winner comes from three
  noisy points, the refit uses all of them. The normal is the *last* right-singular vector of the
  centred inliers — the direction of least variance.
* The `up_hint` is what stops RANSAC from returning a wall when the wall has more points than the
  table. Reject a candidate before counting its inliers, not after.
* `cluster_points`: use `np.unique(keys, axis=0, return_inverse=True)` on the voxel keys, flood-fill
  over the 26 neighbours with a dict from key tuple to index, then map labels back to points
  through `inverse`. Sorting by size and dropping small clusters both have tests.
* `wrap_axis_angle` has a boundary case worth thinking about: $+\pi/2$ and $-\pi/2$ are the same
  axis, and the interval is half-open, so both must return $+\pi/2$.
* Extents come from the **min/max of the rotated points**, not from the eigenvalues. A test pins
  this: the eigenvalue ratio for the 60 × 30 rectangle is 4, the width ratio is 2.
* `top_z` is absolute (table height + object height) while `extents[2]` is the height above the
  table. On a level table at z = 0 they are equal, which is why a test uses a table at z = 0.10.
