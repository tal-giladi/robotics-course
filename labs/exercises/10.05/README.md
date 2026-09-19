# 10.05 — The multivariate Kalman filter

Lesson: [10.05 The multivariate Kalman filter](../../../10-localization/10.05-kalman-filter-multivariate.md)

Generalize 10.04 to a state vector with matrices: track karmel's position **and velocity** from a
position sensor (ToF + map) and a velocity sensor (encoders), and prove the filter is consistent.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `constant_velocity_model(dt, accel_std)` | `F` and `Q` for the state `[position, velocity]` |
| `KalmanFilter(x0, P0)` | `.predict(F, Q, B=None, u=None)`, `.update(z, H, R)` (Joseph form); keeps `x`, `P`, and the last `nu`, `S`, `K` |
| `nees(x_true, x_est, P)` | normalized estimation error squared (needs ground truth) |
| `nis(nu, S)` | normalized innovation squared (no ground truth needed) |

This exercise reuses the provided `../10.04/wall_sim.py` for the simulated drive.

## Check

```bash
python course.py check 10.05
python course.py check 10.05 --solution
```

The tests check `F P Fᵀ` from FM.07, a position update that also corrects velocity, a two-sensor
update, symmetry of `P` over 3,000 steps, and NEES/NIS by hand. Then a 50-run Monte Carlo: the
correctly tuned filter must average NEES ≈ 2 with at least 85% of time steps inside the 95% χ²
bounds [1.48, 2.59], and filters with `Q` 100× too small / too big must be flagged as over- /
under-confident. Finally the filter fuses ToF and encoder speed on the simulated robot: position
RMSE below half the ToF's, velocity RMSE below 2 cm/s, ToF NIS averaging 0.5–2, NEES below 5.99.

## Hints

* `K = np.linalg.solve(S, H @ P).T` equals `P Hᵀ S⁻¹` because `P` and `S` are symmetric.
* `np.atleast_1d(z)`, `np.atleast_2d(H)` let callers pass scalars and lists.
