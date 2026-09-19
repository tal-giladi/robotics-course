"""10.07 — Fusing a gyro with wheel odometry: what fusion can and cannot fix.

    python 10-localization/code/imu_odom_fusion.py
    python 10-localization/code/imu_odom_fusion.py --seeds 5 --out localization_out

Everything here is about ONE number: the heading. Position follows from it, which is why
`robot_localization`'s configuration for a differential-drive robot is mostly an argument about who
is allowed to say what the yaw rate is.

1. The two raw headings: wheel odometry (wheelbase 4 % off -> a SCALE error on every turn) and the
   integrated gyro (a constant bias -> a RAMP). Both end ~30 degrees wrong after 50 s.
2. The complementary filter on the raw signals: the RMSE-vs-alpha curve has a deep minimum — and it
   is luck. Flip the sign of the gyro bias and the minimum jumps somewhere else (the script shows
   both curves). Two biases cannot be blended away.
3. Calibrate first, fuse second: measure the gyro bias while the robot is stationary (what every IMU
   driver does at boot), then use the de-biased gyro to fit the odometry's two UMBmark errors.
4. With both sources unbiased, ANY sensible alpha works, and a 2-state Kalman filter
   [theta, gyro_bias] also recovers the bias and keeps tracking it as it drifts.
"""

from __future__ import annotations

import argparse
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import loc_common as lc  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World  # noqa: E402

GYRO_STD = 0.005        # rad/s, SensorParams.realistic()
GYRO_BIAS_TRUE = 0.01   # rad/s, the value the robot has to discover for itself
BIAS_WALK_STD = 0.0005  # rad/s per sqrt(s): how fast we allow the bias estimate to move
ODOM_RATE_STD = 0.05    # rad/s: slip + tick quantisation on the odometry yaw rate
STILL_SECONDS = 10.0    # how long the robot sits still while the IMU driver measures its bias


# --- step 1: the two raw sources -----------------------------------------------------------------
def yaw_rates(log) -> dict[str, np.ndarray]:
    """Per-step yaw rates from both sensors plus the truth, all rad/s (length T-1)."""
    cfg = lc.load_config()
    travels = lc.wheel_travels(log.ticks)  # (T-1, 2) meters, nominal wheel radius
    omega_odom = (travels[:, 1] - travels[:, 0]) / cfg.drive.wheel_separation_m / log.dt
    v_forward = 0.5 * (travels[:, 0] + travels[:, 1]) / log.dt
    truth = np.unwrap(log.truth[:, 2])
    return {"odom": omega_odom, "gyro": log.gyro[1:], "v": v_forward,
            "true": np.diff(truth) / log.dt, "theta_true": truth}


def integrate(omega: np.ndarray, theta0: float, dt: float) -> np.ndarray:
    """Heading from a yaw-rate sequence (rectangular integration, left unwrapped for plotting)."""
    return np.concatenate([[theta0], theta0 + np.cumsum(omega) * dt])


def complementary(omega_gyro, omega_odom, theta0, dt, alpha) -> np.ndarray:
    """theta += [alpha * gyro + (1 - alpha) * odom] * dt.

    A one-line high-pass on the gyro and low-pass on the wheels. No state, no covariance, one
    constant — and no way to tell you when it is wrong.
    """
    return integrate(alpha * omega_gyro + (1.0 - alpha) * omega_odom, theta0, dt)


# --- step 3: calibration -------------------------------------------------------------------------
def measure_gyro_bias(seconds: float = STILL_SECONDS, dt: float = 0.02, seed: int = 11) -> tuple[float, float]:
    """Average the gyro of a stationary robot: (estimated bias, standard error). Exactly what an
    IMU driver's boot-time calibration does — and why you must not move the robot while it runs."""
    sim = DiffDriveSim(World.apartment(), DiffDriveParams.realistic(), SensorParams.realistic(),
                       pose=(1.0, 1.3, 0.0), seed=seed)
    samples = []
    for _ in range(int(seconds / dt)):
        sim.set_velocity(0.0, 0.0)
        sim.step(dt)
        samples.append(sim.gyro_z())
    s = np.array(samples)
    return float(s.mean()), float(s.std(ddof=1) / math.sqrt(s.size))


def odometry_yaw_calibration(omega_odom: np.ndarray, omega_gyro_debiased: np.ndarray,
                             v_forward: np.ndarray) -> tuple[float, float]:
    """Least squares for the two UMBmark error types (09.05), using the de-biased gyro as truth:

        omega_odom  ~  a * omega_true  +  c * v

    ``a`` is the wheel-separation (type B) error: the odometry over- or under-reports every turn.
    ``c`` (rad/m) is the wheel-diameter (type A) error: unequal wheels curve the robot while it
    believes it is driving straight, so a spurious yaw rate grows with FORWARD speed, not with
    turning. Correct a rate with ``(omega_odom - c * v) / a``.
    """
    A = np.column_stack([omega_gyro_debiased, v_forward])
    a, c = np.linalg.lstsq(A, omega_odom, rcond=None)[0]
    return float(a), float(c)


# --- step 4: the 2-state Kalman filter -----------------------------------------------------------
def gyro_bias_kf(omega_gyro, omega_odom, theta0, dt, gyro_std=GYRO_STD, odom_std=ODOM_RATE_STD,
                 bias_walk=BIAS_WALK_STD, bias0=0.0, P0=(1e-6, 1e-4)):
    """State [theta, gyro_bias]; predict with the gyro, correct with the odometry yaw rate.

    predict: theta += (gyro - b) dt,  b unchanged   -> F = [[1, -dt], [0, 1]]
    update : z = omega_odom,  h(x) = gyro - b       -> H = [0, -1],  nu = z - (gyro - b)

    The odometry yaw rate is the only absolute reference for the bias, so it must be UNBIASED
    itself: run this on a calibrated odometry or the filter will happily learn the wrong bias.
    Returns (theta, bias, sigma_theta) per step.
    """
    x = np.array([theta0, bias0])
    P = np.diag(P0)
    F = np.array([[1.0, -dt], [0.0, 1.0]])
    Q = np.diag([(gyro_std * dt) ** 2, bias_walk**2 * dt])
    H = np.array([[0.0, -1.0]])
    R = np.array([[odom_std**2]])
    th, bias, sig = [x[0]], [x[1]], [math.sqrt(P[0, 0])]
    for g, z in zip(omega_gyro, omega_odom):
        x = np.array([x[0] + (g - x[1]) * dt, x[1]])
        P = F @ P @ F.T + Q
        nu = np.array([z - (g - x[1])])
        S = H @ P @ H.T + R
        K = np.linalg.solve(S, H @ P).T
        x = x + K @ nu
        I_KH = np.eye(2) - K @ H
        P = I_KH @ P @ I_KH.T + K @ R @ K.T
        th.append(x[0])
        bias.append(x[1])
        sig.append(math.sqrt(P[0, 0]))
    return np.array(th), np.array(bias), np.array(sig)


def rmse_deg(theta: np.ndarray, truth: np.ndarray) -> float:
    return math.degrees(float(np.sqrt(np.mean((theta - truth) ** 2))))


def final_deg(theta: np.ndarray, truth: np.ndarray) -> float:
    return math.degrees(abs(float(theta[-1] - truth[-1])))


def plot(run, alphas, raw_sweep, cal_sweep, path_run, path_sweep) -> None:
    t, truth = run["t"], run["truth"]
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for key, label, color in (("odom", "wheel odometry (raw)", "tab:orange"),
                              ("gyro", "integrated gyro (raw)", "tab:green"),
                              ("comp_cal", "complementary (calibrated)", "tab:purple"),
                              ("kf", "KF [theta, gyro_bias]", "tab:blue")):
        axes[0].plot(t, np.degrees(run[key] - truth), color=color, label=label)
    axes[0].plot(t, 2 * np.degrees(run["sigma"]), "k:", lw=0.8, label="KF 2 sigma")
    axes[0].plot(t, -2 * np.degrees(run["sigma"]), "k:", lw=0.8)
    axes[0].set_ylabel("heading error (deg)")
    axes[0].legend(fontsize=8)
    axes[1].plot(t, run["bias"] * 1000, color="tab:blue", label="estimated gyro bias")
    axes[1].axhline(GYRO_BIAS_TRUE * 1000, color="k", ls="--", lw=0.8, label="true bias 10 mrad/s")
    axes[1].set_ylabel("bias (mrad/s)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path_run, dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(alphas, raw_sweep, color="tab:red", label="raw sensors (two biases)")
    ax.plot(alphas, cal_sweep, color="tab:purple", label="calibrated sensors")
    ax.set_yscale("log")
    ax.set_xlabel("alpha (0 = wheels only, 1 = gyro only)")
    ax.set_ylabel("heading RMSE (deg, log scale)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path_sweep, dpi=120)
    plt.close(fig)


def main(argv: list[str] | None = None) -> dict:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--out", default="localization_out")
    args = parser.parse_args(argv)
    out = lc.out_dir(args.out)
    alphas = np.linspace(0.0, 1.0, 21)

    # --- 3. calibration first, because it changes everything below -------------------------------
    bias_hat, bias_se = measure_gyro_bias()
    logs = [lc.drive_tour(seed=args.seed + i, observe_every=1000) for i in range(args.seeds)]
    rates = [yaw_rates(lg) for lg in logs]
    fits = [odometry_yaw_calibration(r["odom"], r["gyro"] - bias_hat, r["v"]) for r in rates]
    scale = float(np.mean([f[0] for f in fits]))
    curve = float(np.mean([f[1] for f in fits]))

    raw, flip, cal, runs = [], [], [], []
    for lg, r in zip(logs, rates):
        truth, th0, dt = r["theta_true"], r["theta_true"][0], lg.dt
        g_raw, o_raw = r["gyro"], r["odom"]
        g_cal, o_cal = r["gyro"] - bias_hat, (r["odom"] - curve * r["v"]) / scale
        raw.append([rmse_deg(complementary(g_raw, o_raw, th0, dt, a), truth) for a in alphas])
        flip.append([rmse_deg(complementary(g_raw - 2 * GYRO_BIAS_TRUE, o_raw, th0, dt, a), truth) for a in alphas])
        cal.append([rmse_deg(complementary(g_cal, o_cal, th0, dt, a), truth) for a in alphas])
        th_kf, bias, sigma = gyro_bias_kf(g_raw, o_cal, th0, dt)
        runs.append({
            "t": lg.t, "truth": truth,
            "odom": integrate(o_raw, th0, dt), "gyro": integrate(g_raw, th0, dt),
            "comp_cal": complementary(g_cal, o_cal, th0, dt, 0.9),
            "kf": th_kf, "bias": bias, "sigma": sigma,
            "rmse_odom": rmse_deg(integrate(o_raw, th0, dt), truth),
            "rmse_gyro": rmse_deg(integrate(g_raw, th0, dt), truth),
            "rmse_kf": rmse_deg(th_kf, truth),
            "final_odom": final_deg(integrate(o_raw, th0, dt), truth),
            "final_gyro": final_deg(integrate(g_raw, th0, dt), truth),
            "final_kf": final_deg(th_kf, truth),
            "bias_20s": float(bias[int(20.0 / dt)]), "bias_final": float(bias[-1]),
            "sigma_final": math.degrees(float(sigma[-1])),
        })
    raw_sweep, flip_sweep, cal_sweep = np.mean(raw, axis=0), np.mean(flip, axis=0), np.mean(cal, axis=0)
    run = runs[0]
    mean = lambda k: float(np.mean([r[k] for r in runs]))  # noqa: E731

    print(f"1) Raw heading over the {run['t'][-1]:.0f} s apartment tour, mean of {args.seeds} seeds")
    print(f"   wheel odometry alone   : RMSE {mean('rmse_odom'):5.2f} deg, final {mean('final_odom'):5.2f} deg  (wheelbase scale error)")
    print(f"   integrated gyro alone  : RMSE {mean('rmse_gyro'):5.2f} deg, final {mean('final_gyro'):5.2f} deg  (constant bias)")

    print("\n2) Complementary filter on the RAW signals: heading RMSE (deg) vs alpha")
    b_raw, b_flip = int(np.argmin(raw_sweep)), int(np.argmin(flip_sweep))
    print("   alpha    gyro bias +0.010   gyro bias -0.010 rad/s")
    for i in range(0, len(alphas), 2):
        mark = "  <- min" if i == b_raw else ("  <- min (flipped)" if i == b_flip else "")
        print(f"   {alphas[i]:4.2f}   {raw_sweep[i]:15.2f}   {flip_sweep[i]:17.2f}{mark}")
    print(f"   Best alpha {alphas[b_raw]:.2f} ({raw_sweep[b_raw]:.2f} deg) with the bias as built, "
          f"{alphas[b_flip]:.2f} ({flip_sweep[b_flip]:.2f} deg) with its sign flipped.")
    print("   That is two biases cancelling, not fusion — a number you cannot carry to another robot.")

    print("\n3) Calibrate first")
    print(f"   gyro bias from {STILL_SECONDS:.0f} s standing still: {bias_hat:.5f} +/- {bias_se:.5f} rad/s (true {GYRO_BIAS_TRUE})")
    print(f"   odometry yaw-rate scale a  : {scale:.4f}      (true wheel_separation_scale 1.04)")
    print(f"   curvature term c           : {curve:+.4f} rad/m (unequal wheel radii, -0.5 % / +0.8 %)")

    print("\n4) Complementary filter on the CALIBRATED signals: heading RMSE (deg) vs alpha")
    b_cal = int(np.argmin(cal_sweep))
    for i in range(0, len(alphas), 2):
        print(f"   alpha = {alphas[i]:4.2f}: {cal_sweep[i]:6.2f}" + ("   <- minimum" if i == b_cal else ""))
    print(f"   Under {cal_sweep.max():.2f} deg for EVERY alpha: now any sensible blend works. Best "
          f"{cal_sweep[b_cal]:.2f} deg at alpha = {alphas[b_cal]:.2f} — a shallow minimum, which is what a"
          " robust tuning looks like.")

    print("\n5) The 2-state Kalman filter [theta, gyro_bias] (raw gyro, calibrated odometry)")
    print(f"   heading RMSE {mean('rmse_kf'):.2f} deg, final error {mean('final_kf'):.2f} deg")
    print(f"   estimated gyro bias: {mean('bias_20s'):.4f} rad/s after 20 s, {mean('bias_final'):.4f} at the end"
          f" (true {GYRO_BIAS_TRUE})")
    print(f"   filter's own sigma_theta at the end: {mean('sigma_final'):.2f} deg")

    plot(run, alphas, raw_sweep, cal_sweep, out / "imu_odom_run.png", out / "imu_odom_alpha.png")
    print(f"\nwrote {out / 'imu_odom_run.png'} and {out / 'imu_odom_alpha.png'}")
    return {
        "bias_hat": bias_hat, "bias_se": bias_se, "scale": scale, "curve": curve,
        "alphas": alphas.tolist(), "raw_sweep": raw_sweep.tolist(), "flip_sweep": flip_sweep.tolist(), "cal_sweep": cal_sweep.tolist(),
        "best_flip": (float(alphas[b_flip]), float(flip_sweep[b_flip])),
        "best_raw": (float(alphas[b_raw]), float(raw_sweep[b_raw])),
        "best_cal": (float(alphas[b_cal]), float(cal_sweep[b_cal])),
        "rmse": {k: mean(k) for k in ("rmse_odom", "rmse_gyro", "rmse_kf")},
        "final": {k: mean(k) for k in ("final_odom", "final_gyro", "final_kf")},
        "bias_20s": mean("bias_20s"), "bias_final": mean("bias_final"), "sigma_final": mean("sigma_final"),
    }


if __name__ == "__main__":
    main()
