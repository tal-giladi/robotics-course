# 10.06 — The Extended Kalman Filter for a differential-drive robot

Lesson: [10.06 The Extended Kalman Filter for a differential-drive robot](../../../10-localization/10.06-extended-kalman-filter.md)

Localize karmel in the simulated apartment: predict with wheel odometry, correct with range–bearing
observations of wall-mounted landmarks, and prove your Jacobians right before trusting them.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `motion_model(x, u, b)` | midpoint odometry model → `(x_pred, F, G)` with `F = ∂f/∂x`, `G = ∂f/∂u` |
| `wheel_noise(u, k)` | `M = diag(k²·|d_L|, k²·|d_R|)` |
| `range_bearing(x, landmark)` | expected `[r, φ]` (φ wrapped) and `H = ∂h/∂x` |
| `numerical_jacobian(func, x, eps, angle_rows)` | central differences, wrapping angle outputs |
| `EKF(x0, P0)` | `.predict(u, b, M)`, `.innovation(z, landmark, R)`, `.update(z, landmark, R) → NIS` (Joseph form, wrapped bearing innovation) |
| `associate(ekf, z, landmarks, R, gate)` | nearest landmark by Mahalanobis distance, `None` outside the gate |

Provided: `tour_sim.py` — `drive_tour(seed, observe_every, scan_every, …)` drives karmel through the
living room, kitchen and study of `World.apartment()` and logs ticks, gyro, landmark observations,
scans and the ground truth. 10.07–10.10 reuse it.

## Check

```bash
python course.py check 10.06
python course.py check 10.06 --solution
```

The tests compare `F`, `G` and `H` with central differences at random poses, check predict and
update against hand-computed numbers, wrap a bearing innovation across ±π, keep `P` symmetric
positive definite over 500 hard updates, and pick the right landmark by Mahalanobis (not Euclidean)
distance. Then the filter runs on two realistic 50-second tours: position RMSE below 5 cm, maximum
error below 12 cm, heading RMSE below 4°, dead reckoning at least 5× worse, mean NIS between 1 and
3.5, mean NEES below 6; and once more without landmark ids, with zero wrong associations.

## Hints

* Write `F`, `G`, `H` on paper first; then let `numerical_jacobian` tell you which entry is wrong.
* `th_m = th + (dR − dL)/(2b)` depends on the wheel travels: that is where the `ds·sin(th_m)/(2b)` terms in `G` come from.
* `K = np.linalg.solve(S, H @ P).T` equals `P Hᵀ S⁻¹` because `P` and `S` are symmetric.
