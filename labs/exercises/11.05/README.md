# 11.05 — Pose graphs and loop closure

Lesson: [11.05 Pose graphs and loop closure](../../../11-slam/11.05-pose-graphs-loop-closure.md)

Build the back end of a graph-SLAM system: the error of one relative-pose edge, its Jacobians,
and a Gauss-Newton optimizer that spreads the drift over the whole trajectory once a loop
closure says "you are back where you started".

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `relative_pose(xi, xj)` | pose of node j in node i's frame, angle wrapped |
| `edge_error(xi, xj, z)` | 3-vector residual of an edge, in the measurement frame, angle wrapped |
| `edge_jacobians(xi, xj, z)` | `A = ∂e/∂xi`, `B = ∂e/∂xj` (3×3 each), closed form |
| `optimize_pose_graph(poses, edges, iterations, tolerance)` | Gauss-Newton with node 0 anchored; returns a new `(N, 3)` array |

`Edge`, `wrap`, `rot` and `chi2` are given. Each stub raises `NotImplementedError`: replace the
body, keep the signature.

## Check

```bash
python course.py check 11.05              # your code
python course.py check 11.05 --solution   # the reference, to see what passing looks like
```

What the tests check:

* hand-computed relative poses and edge errors (including one expressed in a rotated
  measurement frame, and angles that wrap across ±180°);
* your Jacobians against central finite differences on random poses (1e-5);
* a consistent graph does not move; node 0 stays fixed; the input array is not modified;
* a 2 m square with biased odometry and **one** loop closure: χ² drops by more than 10×, the
  RMSE at least halves (≈0.16–0.20 m → ≈0.03 m with the reference), the last node ends within
  5 cm of the start. Without the loop closure nothing changes;
* the realistic simulated karmel drives a 1.6 m × 1.0 m rectangle; wheel-odometry edges plus one
  loop closure (true relative pose + 1 cm / 0.5° noise, as ICP would report) must at least halve
  the odometry RMSE.

A skipped test means that part is not implemented yet. The check passes only when nothing is
skipped.

## Hints

* Build `H` and `b` from scratch in every iteration: they depend on the current poses.
* Wrap angles in the error, *and* after adding `dx`.
* If the finite-difference test fails only for column 2 of `A`, the `dRiᵀ/dθi` term is wrong.
