# 10.04 — The Kalman filter in 1D

Lesson: [10.04 The Kalman filter in 1D — numbers first](../../../10-localization/10.04-kalman-filter-1d.md)

karmel drives toward a wall. Wheel encoders predict how the distance changes; a noisy front ToF
sensor measures it. Fuse them with a scalar Kalman filter.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `kalman_gain(P_pred, R)` | `P_pred / (P_pred + R)`, with input checks |
| `steady_state_gain(Q, R)` | the gain a constant-Q/R filter converges to (solve a quadratic) |
| `KalmanFilter1D(x0, P0)` | `.predict(u, Q)`, `.update(z, R)`; keeps `x`, `P` and the last `K`, `nu`, `S` |
| `run_kf_1d(x0, P0, controls, measurements, Q, R)` | runs the filter over a log, skipping `None` measurements |

`wall_sim.py` is **provided**: it drives the simulated robot toward a wall and logs the odometry
controls, ToF readings and ground truth.

## Check

```bash
python course.py check 10.04
python course.py check 10.04 --solution
```

Hand-computed cases first (the FM.16 wall example, a predict/update pair, "information adds",
the steady-state gain), then two simulated drives with a ToF of σ = 3 cm: your filtered RMSE must
be **below half the sensor's RMSE**, the final distance within 3 cm, and most errors within 3σ.

## Hints

* `Q`, `R` and `P` are variances (m²). σ = 3 cm means `R = 0.03**2 = 9e-4`.
* The innovation is computed with the *predicted* estimate, before you change `x`.
