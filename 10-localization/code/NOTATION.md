# Module 10 notation

One set of symbols for every localization lesson (10.01–10.10) and its code. Lessons 10.01–10.05
introduce them; 10.06 onward (EKF, robot_localization, particle filters, AMCL, AprilTags) keeps them.
SI units everywhere, angles in radians wrapped to (−π, π], REP-103 frames (x forward, y left, z up).

## Time and belief

| Symbol | Code | Meaning |
|---|---|---|
| $k$ | `k` | time step index; $\Delta t$ = `dt` in seconds |
| $\mathbf{u}_k$ | `u` | control / odometry input that moves the state from $k-1$ to $k$ |
| $\mathbf{z}_k$ | `z` | measurement at step $k$ (`None` when the sensor gave no reading) |
| $bel(\mathbf{x}_k)$ | `belief` | $p(\mathbf{x}_k \mid \mathbf{z}_{1:k}, \mathbf{u}_{1:k})$, after the update |
| $\overline{bel}(\mathbf{x}_k)$ | `belief` after `predict` | $p(\mathbf{x}_k \mid \mathbf{z}_{1:k-1}, \mathbf{u}_{1:k})$, before the update |

Histogram filter (10.03): `belief` is a 1D numpy array over cells summing to 1; the motion model is a
`kernel` (probability of each real move offset); the measurement model is a `likelihood` array
$p(z \mid \text{cell})$ that does **not** sum to 1.

## Gaussian filters (KF 10.04–10.05, EKF 10.06)

| Symbol | Code | Shape | Meaning |
|---|---|---|---|
| $\mathbf{x}$ | `x` | (n,) | state estimate (mean). Posterior after update |
| $\bar{\mathbf{x}}$ | `x_pred` | (n,) | predicted state (prior), before the update |
| $P$ | `P` | (n, n) | state covariance; $\bar P$ = `P_pred` |
| $F$ | `F` | (n, n) | state-transition matrix; in the EKF the Jacobian $\partial f/\partial \mathbf{x}$ |
| $B$ | `B` | (n, l) | control matrix (linear KF only) |
| $f(\mathbf{x}, \mathbf{u})$ | `f` | | nonlinear motion model (EKF) |
| $Q$ | `Q` | (n, n) | process-noise covariance, per step |
| $G$ | `G` | (n, p) | how the noise sources enter the state: $Q = G\,\Sigma_{noise}\,G^\top$ |
| $H$ | `H` | (m, n) | measurement matrix; in the EKF the Jacobian $\partial h/\partial \mathbf{x}$ |
| $h(\mathbf{x})$ | `h` | | nonlinear measurement model (EKF), e.g. range–bearing to a landmark |
| $R$ | `R` | (m, m) | measurement-noise covariance |
| $\boldsymbol\nu$ | `nu` | (m,) | innovation $\mathbf{z} - H\bar{\mathbf{x}}$ (EKF: $\mathbf{z} - h(\bar{\mathbf{x}})$, bearings wrapped) |
| $S$ | `S` | (m, m) | innovation covariance $H\bar P H^\top + R$ |
| $K$ | `K` | (n, m) | Kalman gain $\bar P H^\top S^{-1}$ |
| $\epsilon$ | `nees(x_true, x, P)` | scalar | NEES $(\mathbf{x}_{true}-\mathbf{x})^\top P^{-1}(\mathbf{x}_{true}-\mathbf{x})$, averages $n$ when consistent (simulation only) |
| $\epsilon_\nu$ | `nis(nu, S)` | scalar | NIS $\boldsymbol\nu^\top S^{-1}\boldsymbol\nu$, averages $m$ when consistent (works on the real robot) |

$Q$, $R$, $P$ are **variances/covariances** (m², rad², m²/s²), never standard deviations. In 1D
(10.04) the same letters are scalars: `x`, `P`, `Q`, `R`, `K`, `nu`, `S`.

### The equations, in this notation

Predict:
$$\bar{\mathbf{x}} = F\mathbf{x} + B\mathbf{u} \quad(\text{EKF: } f(\mathbf{x},\mathbf{u})), \qquad \bar P = F P F^\top + Q$$

Update:
$$\boldsymbol\nu = \mathbf{z} - H\bar{\mathbf{x}}, \quad S = H\bar P H^\top + R, \quad K = \bar P H^\top S^{-1}, \quad \mathbf{x} = \bar{\mathbf{x}} + K\boldsymbol\nu, \quad P = (I - KH)\bar P (I-KH)^\top + K R K^\top$$

(The last form is the Joseph form; it equals $(I-KH)\bar P$ for the optimal $K$ and keeps $P$ symmetric.)

## Conventions for 10.06+

* Robot pose state: $\mathbf{x} = [x, y, \theta]^\top$ in the `map` (or `odom`) frame; control from
  odometry $\mathbf{u} = [\Delta s, \Delta\theta]^\top$ or wheel travels $[d_L, d_R]^\top$ with noise
  covariance $M$ mapped through $G = \partial f/\partial \mathbf{u}$: $Q = G M G^\top$.
* Landmark measurement from `robotlab` `observe_landmarks()`: $\mathbf{z} = [r, \varphi]^\top$
  (`range_m`, `bearing_rad`, from `base_link`), $R = \mathrm{diag}(\sigma_r^2, \sigma_\varphi^2)$
  (realistic sim: $\sigma_r = 0.05$ m, $\sigma_\varphi = 0.03$ rad). Always wrap the bearing innovation.
* Gating: accept a measurement if NIS $< \chi^2_{m}(0.95)$ (3.841 for m = 1, 5.991 for m = 2, 7.815 for m = 3).
* Ellipses: say which one. $k$-sigma ellipse = Mahalanobis distance $k$ (2D: 1σ holds 39.3%, 2σ 86.5%);
  the 95% ellipse in 2D is $d^2 = 5.991$ (2.448σ). `robotlab.sim.viz.draw_covariance_ellipse(..., n_sigma=)`
  takes the k-sigma scale.
* Particle filters (10.08): particles `(N, 3)` array of poses, weights `w` `(N,)` summing to 1.
* Covariance in ROS messages: 6×6 row-major `(x, y, z, roll, pitch, yaw)`, see FM.15.

## Shared code in this folder

* `loc_common.py` — `load_exercise(id, solution)`, `load_provided(id, module)`, χ² and ellipse
  helpers, the living-room drive with an odometry Monte Carlo cloud and landmark residuals.
* `labs/exercises/10.03/corridor_sim.py` — corridor with doors (histogram filter inputs).
* `labs/exercises/10.04/wall_sim.py` — drive toward a wall: odometry controls, ToF, truth, speeds.
