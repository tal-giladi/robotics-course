"""10.04 — The 1D Kalman filter: karmel drives toward a wall with encoders and a noisy ToF.

    python 10-localization/code/kf1d_wall.py              # YOUR labs/exercises/10.04/student.py
    python 10-localization/code/kf1d_wall.py --solution   # the reference

1. The first three filter steps printed with every number (x_pred, P_pred, nu, S, K, x, P).
2. One run plotted: truth, ToF readings, filtered estimate with its 2-sigma band, and the gain K.
3. Tuning experiments on the same log:
   a. sweep Q with correct odometry and with odometry that under-reads travel by 5%
      (a wrong wheel radius) -> RMSE, and the mean innovation that exposes the bias;
   b. sweep the R the filter assumes while the real sensor sigma stays 3 cm.
"""

from __future__ import annotations

import argparse
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import loc_common as lc  # noqa: E402

SENSOR_STD = 0.03   # m: a noisy ToF / ultrasonic
Q_DEFAULT = 1e-6    # m^2 per 50 ms step (sigma 1 mm per step)
SETTLE = 20         # ignore the first second in RMSE


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((np.asarray(a) - np.asarray(b)) ** 2)))


def innovations(ex, x0: float, P0: float, controls, measurements, Q: float, R: float) -> np.ndarray:
    kf = ex.KalmanFilter1D(x0, P0)
    out = []
    for u, z in zip(controls, measurements):
        kf.predict(u, Q)
        if z is not None:
            kf.update(z, R)
            out.append(kf.nu)
    return np.array(out)


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--solution", action="store_true")
    ap.add_argument("--out", default="localization_out")
    ap.add_argument("--seed", type=int, default=5)
    args = ap.parse_args(argv)
    out = lc.out_dir(args.out)
    ex = lc.load_exercise("10.04", solution=args.solution)
    ws = lc.load_provided("10.04", "wall_sim")
    run = ws.drive_to_wall(args.seed, realistic=True, range_noise_std_m=SENSOR_STD)
    R = SENSOR_STD**2
    z = np.array([np.nan if m is None else m for m in run.measurements])
    results: dict = {}

    # 1. the first steps, every number ----------------------------------------------------------
    print(f"1) First steps (x0 = 3.0 m guessed, P0 = 1.0 m^2; Q = {Q_DEFAULT:g}, R = {R:g})")
    kf = ex.KalmanFilter1D(3.0, 1.0)
    print(f"{'k':>2} {'u (mm)':>7} {'x_pred':>8} {'P_pred':>10} {'z':>7} {'nu':>8} {'S':>10} {'K':>7} {'x':>8} {'P':>10} {'truth':>7}")
    first_steps = []
    for k in range(3):
        u = run.controls[k]
        x_pred, P_pred = kf.predict(u, Q_DEFAULT)
        x, P = kf.update(run.measurements[k], R)
        first_steps.append((x_pred, P_pred, kf.nu, kf.S, kf.K, x, P))
        print(f"{k:2d} {u * 1000:7.2f} {x_pred:8.4f} {P_pred:10.6f} {run.measurements[k]:7.4f} {kf.nu:8.4f} "
              f"{kf.S:10.6f} {kf.K:7.4f} {x:8.4f} {P:10.6f} {run.truth[k]:7.4f}")
    results["first_steps"] = first_steps

    # 2. one run --------------------------------------------------------------------------------
    xs, Ps = ex.run_kf_1d(3.0, 1.0, run.controls, run.measurements, Q_DEFAULT, R)
    valid = ~np.isnan(z)
    meas_rmse = rmse(z[valid][SETTLE:], run.truth[valid][SETTLE:])
    kf_rmse = rmse(xs[SETTLE:], run.truth[SETTLE:])
    start = float(run.truth[0])  # odometry alone needs the start distance; we give it the true one
    odom_rmse = rmse(start + np.cumsum(run.controls), run.truth)
    odom_short_rmse = rmse(start + 0.95 * np.cumsum(run.controls), run.truth)
    print(f"\n2) Seed {args.seed}: ToF alone RMSE {meas_rmse * 1000:.1f} mm | Kalman RMSE {kf_rmse * 1000:.1f} mm "
          f"(final sigma {math.sqrt(Ps[-1]) * 1000:.1f} mm, steady-state K = {ex.steady_state_gain(Q_DEFAULT, R):.4f})")
    print(f"   odometry alone, told the TRUE start: RMSE {odom_rmse * 1000:.1f} mm; "
          f"with a 5% wheel-radius error: {odom_short_rmse * 1000:.1f} mm")
    results["run"] = {"meas_rmse": meas_rmse, "kf_rmse": kf_rmse, "final_sigma": math.sqrt(Ps[-1]),
                      "odom_rmse": odom_rmse, "odom_short_rmse": odom_short_rmse}

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    axes[0].plot(run.t, z, ".", ms=3, color="tab:gray", label="ToF readings (sigma 3 cm)")
    axes[0].plot(run.t, run.truth, color="black", label="truth")
    axes[0].plot(run.t, xs, color="tab:blue", label="Kalman estimate")
    band = 2 * np.sqrt(Ps)
    axes[0].fill_between(run.t, xs - band, xs + band, color="tab:blue", alpha=0.25, label="+/-2 sigma")
    axes[0].set_ylabel("distance to wall (m)")
    axes[0].legend()
    gains = [ex.kalman_gain(P + Q_DEFAULT, R) for P in np.concatenate([[1.0], Ps[:-1]])]
    axes[1].semilogy(run.t, gains, color="tab:orange")
    axes[1].set_ylabel("gain K")
    axes[1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(out / "kf1d_run.png", dpi=120)
    plt.close(fig)

    # 3a. Q sweep ---------------------------------------------------------------------------------
    q_values = [0.0, 1e-8, 1e-7, 1e-6, 3e-6, 1e-5, 1e-4, 1e-3, 1e-2]
    print("\n3a) Q sweep (R = 9e-4).        correct odometry          |   odometry 5% short")
    print(f"{'Q':>8} {'K_ss':>7} | {'RMSE mm':>8} {'mean nu mm':>10} | {'RMSE mm':>8} {'mean nu mm':>10}")
    sweep = {}
    fig, ax = plt.subplots(figsize=(6, 4))
    for scale, label in ((1.0, "correct odometry"), (0.95, "odometry 5% short")):
        controls = [c * scale for c in run.controls]
        rm, nus = [], []
        for Q in q_values:
            xq, _ = ex.run_kf_1d(3.0, 1.0, controls, run.measurements, Q, R)
            rm.append(rmse(xq[SETTLE:], run.truth[SETTLE:]))
            nus.append(float(np.mean(innovations(ex, 3.0, 1.0, controls, run.measurements, Q, R)[SETTLE:])))
        sweep[scale] = (rm, nus)
        ax.loglog([max(q, 1e-9) for q in q_values], np.array(rm) * 1000, "o-", label=label)
    for i, Q in enumerate(q_values):
        k_ss = ex.steady_state_gain(Q, R) if Q > 0 else 0.0
        print(f"{Q:8.0e} {k_ss:7.4f} | {sweep[1.0][0][i] * 1000:8.1f} {sweep[1.0][1][i] * 1000:10.2f} | "
              f"{sweep[0.95][0][i] * 1000:8.1f} {sweep[0.95][1][i] * 1000:10.2f}")
    ax.axhline(meas_rmse * 1000, color="tab:gray", linestyle="--", label="ToF alone")
    ax.set_xlabel("Q assumed by the filter (m^2; Q=0 plotted at 1e-9)")
    ax.set_ylabel("RMSE (mm)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "kf1d_q_sweep.png", dpi=120)
    plt.close(fig)
    results["q_sweep"] = {"Q": q_values, "correct": sweep[1.0], "short": sweep[0.95]}

    # 3b. R sweep ---------------------------------------------------------------------------------
    print("\n3b) R sweep (true sensor sigma 3 cm, Q = 1e-6, correct odometry)")
    r_rows = []
    for r_std in (0.003, 0.01, 0.03, 0.1, 0.3):
        xr, Pr = ex.run_kf_1d(3.0, 1.0, run.controls, run.measurements, Q_DEFAULT, r_std**2)
        err = xr[SETTLE:] - run.truth[SETTLE:]
        inside = float(np.mean(np.abs(err) < 2 * np.sqrt(Pr[SETTLE:])))
        r_rows.append((r_std, rmse(xr[SETTLE:], run.truth[SETTLE:]), math.sqrt(Pr[-1]), inside))
        print(f"   assumed sigma {r_std * 100:4.1f} cm: RMSE {r_rows[-1][1] * 1000:5.1f} mm, filter's own sigma "
              f"{r_rows[-1][2] * 1000:5.1f} mm, errors inside +/-2 sigma {inside * 100:5.1f}%")
    results["r_sweep"] = r_rows
    print(f"wrote {out / 'kf1d_run.png'} and {out / 'kf1d_q_sweep.png'}")
    return results


if __name__ == "__main__":
    main()
