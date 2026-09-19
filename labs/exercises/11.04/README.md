# 11.04 — Scan matching with ICP

Lesson: [11.04 Scan matching with ICP](../../../11-slam/11.04-scan-matching-icp.md)

Estimate how the robot moved between two LiDAR scans by aligning the scans: point-to-point
Iterative Closest Point with an SVD (Kabsch) alignment step, nearest neighbours from a KD-tree,
outlier rejection by distance, and a reliability test that decides whether to trust the result.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `best_fit_transform(src, dst)` | rotation `R` (det +1) and translation `t` that best map corresponding points `src` onto `dst` |
| `icp(source, target, initial, max_iterations, tolerance, max_correspondence_distance)` | iterate nearest neighbours → reject far pairs → Kabsch → compose; returns an `IcpResult` |
| `match_is_reliable(result, min_inlier_fraction, max_rmse)` | accept only converged matches with enough close partners |

`rotation`, `apply`, `compose` and the `IcpResult` dataclass are given. Each stub raises
`NotImplementedError`: replace the body, keep the signature.

## Check

```bash
python course.py check 11.04              # your code
python course.py check 11.04 --solution   # the reference, to see what passing looks like
```

What the tests check:

* **Kabsch:** the lesson's 3-point example (30°, (1.0, 0.5)) exactly, identity, a mirror image
  that must still give a rotation, and 500 noisy pairs (within 0.1° and 3 mm).
* **ICP:** exact recovery on scattered points (0.1 mm); convergence from a good initial guess
  for a 120° rotation; two simulated apartment scans 34 cm and 10° apart, starting from identity,
  with and without 1 cm range noise, within **3 cm and 1°**.
* **Rejection:** 30% clutter points that only one scan sees must not bend the result (still 3 cm
  and 1°), and `inlier_fraction` must count them out. When nothing matches, ICP must stop
  without claiming convergence.
* **Reliability:** scans of the living room and the bedroom forced together must be rejected; a
  small real motion in the same room must be accepted.

A skipped test means that part is not implemented yet. The check passes only when nothing is
skipped.

## Hints

* `H = (src - μs).T @ (dst - μd)` is 2×2. Get the order wrong (`dst` first) and you get the
  inverse rotation.
* Build the KD-tree on the **target** once; `tree.query(points)` returns `(distances, indices)`.
* Compose the correction on the **left**: the step was computed on already-moved points.
