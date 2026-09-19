"""10.06 — EKF localization of karmel with range-bearing landmarks in the simulated apartment.

    python 10-localization/code/ekf_landmarks.py              # YOUR labs/exercises/10.06/student.py
    python 10-localization/code/ekf_landmarks.py --solution   # the reference
    python 10-localization/code/ekf_landmarks.py --solution --seeds 5

1. One predict + update by the numbers, and the Jacobians checked against central differences.
2. The 50 s apartment tour: dead reckoning vs the EKF (landmarks at 5 Hz and at 1 Hz). Plots.
3. Tuning the wheel-noise constant k: RMSE, NIS and NEES averaged over seeds.
4. The classic bug: forgetting to wrap the bearing innovation.
5. Data association without ids: fine from a good start, wrong landmarks from a bad one.
6. Where linearization breaks: a 1 m drive with 30 degrees of heading uncertainty, EKF vs Monte Carlo.
"""

from __future__ import annotations

import argparse
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import loc_common as lc  # noqa: E402
from robotlab.sim import viz  # noqa: E402

R_LANDMARK = np.diag([0.05**2, 0.03**2])   # SensorParams.realistic(): sigma_r 5 cm, sigma_phi 0.03 rad
K_WHEEL = 0.01                              # m^0.5: sigma 1 cm per wheel after 1 m
P0 = np.diag([0.02**2, 0.02**2, 0.02**2])


def localize(ex, log, k=K_WHEEL, use_ids=True, x0=None, P0_=P0, ekf_class=None, gate=5.991) -> dict:
    """Run the EKF over a tour log. Returns estimates, covariances and the summary numbers."""
    b = lc.load_config().drive.wheel_separation_m
    travels = lc.wheel_travels(log.ticks)
    ekf = (ekf_class or ex.EKF)(log.truth[0] if x0 is None else x0, P0_)
    observations = dict(log.landmarks)
    est, cov, nis, nees, wrong, rejected = [ekf.x.copy()], [ekf.P.copy()], [], [], 0, 0
    for i in range(1, len(log.t)):
        u = travels[i - 1]
        ekf.predict(u, b, ex.wheel_noise(u, k))
        for obs in observations.get(i, []):
            z = [obs.range_m, obs.bearing_rad]
            if use_ids:
                j = int(np.flatnonzero(log.world.landmark_ids == obs.id)[0])
            else:
                j = ex.associate(ekf, z, log.world.landmarks, R_LANDMARK, gate)
                if j is None:
                    rejected += 1
                    continue
                wrong += int(log.world.landmark_ids[j] != obs.id)
            nis.append(ekf.update(z, log.world.landmarks[j], R_LANDMARK))
        est.append(ekf.x.copy())
        cov.append(ekf.P.copy())
        e = log.truth[i] - ekf.x
        e[2] = lc.wrap_angle(e[2])
        nees.append(float(e @ np.linalg.solve(ekf.P, e)))
    est, cov = np.array(est), np.array(cov)
    pos_err = np.hypot(est[:, 0] - log.truth[:, 0], est[:, 1] - log.truth[:, 1])
    head_err = np.abs(lc.wrap_angle(est[:, 2] - log.truth[:, 2]))
    return {
        "est": est, "cov": cov, "pos_err": pos_err,
        "rmse": float(np.sqrt(np.mean(pos_err**2))), "max": float(pos_err.max()), "final": float(pos_err[-1]),
        "heading_rmse_deg": math.degrees(float(np.sqrt(np.mean(head_err**2)))),
        "nis": float(np.mean(nis)) if nis else float("nan"), "nees": float(np.mean(nees)),
        "wrong": wrong, "rejected": rejected, "updates": len(nis),
    }


def by_the_numbers(ex) -> dict:
    print("1) One cycle by the numbers (b = 0.2 m)")
    ekf = ex.EKF([1.0, 2.0, 0.0], np.diag([0.01, 0.01, 0.01]))
    x, P = ekf.predict([0.1, 0.1], 0.2, np.diag([1e-4, 1e-4]))
    print(f"   predict u = [0.10, 0.10] m: x = {np.round(x, 4)}, diag P = {np.round(np.diag(P), 6)}, P[y,th] = {P[1, 2]:.5f}")
    ekf = ex.EKF([0.0, 0.0, 0.0], np.diag([0.04, 0.04, 0.01]))
    nis = ekf.update([1.9, 0.05], [2.0, 0.0], np.diag([0.01, 0.001]))
    print(f"   update z = [1.90 m, 0.05 rad] of landmark (2, 0): nu = {np.round(ekf.nu, 4)}, x = {np.round(ekf.x, 4)}, NIS = {nis:.3f}")
    x_check = np.array([2.0, 1.5, 2.8])
    u = np.array([0.012, 0.018])
    _, F, G = ex.motion_model(x_check, u, 0.2)
    _, H = ex.range_bearing(x_check, [0.0, 1.8])
    errs = {
        "F": np.abs(F - ex.numerical_jacobian(lambda s: ex.motion_model(s, u, 0.2)[0], x_check, angle_rows=[2])).max(),
        "G": np.abs(G - ex.numerical_jacobian(lambda v: ex.motion_model(x_check, v, 0.2)[0], u, angle_rows=[2])).max(),
        "H": np.abs(H - ex.numerical_jacobian(lambda s: ex.range_bearing(s, [0.0, 1.8])[0], x_check, angle_rows=[1])).max(),
    }
    print("   Jacobian check, max |analytic - numerical|: " + ", ".join(f"{k} {v:.1e}" for k, v in errs.items()))
    print(f"   H at x = {x_check.tolist()} for landmark (0, 1.8):\n{np.array2string(H, precision=4, prefix='   ')}")
    return {"x_update": ekf.x.tolist(), "nis": nis, "jacobian_errors": errs}


def plot_tour(log, dr, run, path) -> None:
    fig, ax = viz.new_axes(log.world, figsize=(7.5, 6.5), title="EKF with landmarks (5 Hz) vs dead reckoning")
    viz.draw_trajectory(ax, log.truth, color="k", lw=1.5, label="truth")
    viz.draw_trajectory(ax, dr, color="tab:orange", linestyle="--", label="dead reckoning")
    viz.draw_trajectory(ax, run["est"], color="tab:blue", lw=1.0, label="EKF")
    for i in range(0, len(log.t), 150):  # every 3 s: the 95% position ellipse, magnified 5x
        viz.draw_covariance_ellipse(ax, run["est"][i, :2], run["cov"][i][:2, :2] * 25.0, n_sigma=math.sqrt(5.991), color="tab:purple")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def plot_sigma(runs, path) -> None:
    """``runs``: (label, tour log, localize() result, color) — each run has its own time base."""
    fig, axes = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
    for label, log, run, color in runs:
        axes[0].plot(log.t, run["pos_err"] * 100, color=color, label=f"{label}: error")
        sigma = np.sqrt(run["cov"][:, 0, 0] + run["cov"][:, 1, 1]) * 100
        axes[0].plot(log.t, 2 * sigma, color=color, linestyle=":", label=f"{label}: 2 sigma (x, y)")
        axes[1].plot(log.t, np.degrees(np.sqrt(run["cov"][:, 2, 2])), color=color, label=label)
    axes[0].set_ylabel("position (cm)")
    axes[1].set_ylabel("sigma theta (deg)")
    axes[1].set_xlabel("time (s)")
    axes[0].legend(fontsize=8)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main(argv: list[str] | None = None) -> dict:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--solution", action="store_true", help="use labs/exercises/10.06/solution.py")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--seeds", type=int, default=5, help="seeds for the tuning sweep")
    parser.add_argument("--out", default="localization_out")
    args = parser.parse_args(argv)
    ex = lc.load_exercise("10.06", solution=args.solution)
    out = lc.out_dir(args.out)
    results: dict = {"numbers": by_the_numbers(ex)}

    # 2. the tour ----------------------------------------------------------------------------------
    log = lc.drive_tour(seed=args.seed, observe_every=10)
    log_1hz = lc.drive_tour(seed=args.seed, observe_every=50)
    dr = lc.dead_reckon(log.ticks, tuple(log.truth[0]))
    dr_err = np.hypot(dr[:, 0] - log.truth[:, 0], dr[:, 1] - log.truth[:, 1])
    run5, run1 = localize(ex, log), localize(ex, log_1hz)
    print(f"\n2) Apartment tour, seed {args.seed}, {log.t[-1]:.0f} s, {run5['updates']} landmark updates at 5 Hz")
    print(f"   dead reckoning  : RMSE {np.sqrt(np.mean(dr_err**2)) * 100:5.1f} cm, final {dr_err[-1] * 100:5.1f} cm")
    for name, r in (("EKF, tags 5 Hz", run5), ("EKF, tags 1 Hz", run1)):
        print(f"   {name}  : RMSE {r['rmse'] * 100:5.1f} cm, max {r['max'] * 100:4.1f} cm, heading RMSE {r['heading_rmse_deg']:.1f} deg, "
              f"NIS {r['nis']:.2f} (m = 2), NEES {r['nees']:.2f} (n = 3)")
    results["tour"] = {"dr_rmse": float(np.sqrt(np.mean(dr_err**2))), "ekf5": {k: v for k, v in run5.items() if np.isscalar(v)},
                       "ekf1": {k: v for k, v in run1.items() if np.isscalar(v)}}
    plot_tour(log, dr, run5, out / "ekf_tour.png")
    plot_sigma([("5 Hz", log, run5, "tab:blue"), ("1 Hz", log_1hz, run1, "tab:red")], out / "ekf_sigma.png")

    # 3. tuning k ----------------------------------------------------------------------------------
    print(f"\n3) Wheel-noise constant k, mean over {args.seeds} seeds (tags at 5 Hz)")
    logs = [log] + [lc.drive_tour(seed=args.seed + s, observe_every=10) for s in range(1, args.seeds)]
    results["k_sweep"] = {}
    for k in (0.002, 0.005, 0.01, 0.02, 0.05):
        rows = [localize(ex, lg, k=k) for lg in logs]
        mean = {key: float(np.mean([r[key] for r in rows])) for key in ("rmse", "heading_rmse_deg", "nis", "nees")}
        results["k_sweep"][k] = mean
        print(f"   k = {k:<5}: RMSE {mean['rmse'] * 100:4.2f} cm, heading {mean['heading_rmse_deg']:.2f} deg, "
              f"NIS {mean['nis']:.2f}, NEES {mean['nees']:.2f}")

    # 4. the wrap bug ------------------------------------------------------------------------------
    class NoWrapEKF(ex.EKF):
        def innovation(self, z, landmark, R):
            _, H = ex.range_bearing(self.x, landmark)
            dx, dy = landmark[0] - self.x[0], landmark[1] - self.x[1]
            z_hat = np.array([math.hypot(dx, dy), math.atan2(dy, dx) - self.x[2]])  # BUG: nothing wrapped
            return np.asarray(z, dtype=float) - z_hat, H @ self.P @ H.T + R, H

    bug = localize(ex, log, ekf_class=NoWrapEKF)
    print("\n4) Bearing innovation not wrapped (same tour)")
    print(f"   RMSE {bug['rmse'] * 100:.1f} cm, max {bug['max'] * 100:.0f} cm, heading RMSE {bug['heading_rmse_deg']:.1f} deg, "
          f"NIS {bug['nis']:.0f}, NEES {bug['nees']:.0f}")
    results["nowrap"] = {k: v for k, v in bug.items() if np.isscalar(v)}

    # 5. data association --------------------------------------------------------------------------
    print("\n5) Data association by Mahalanobis distance (ids ignored)")
    results["association"] = {}
    bad_start = log.truth[0] + np.array([0.3, -0.2, 0.2])
    cases = [
        ("good start, ids ignored", dict(use_ids=False)),
        ("start 36 cm / 11 deg off, P0 honest, ids", dict(x0=bad_start, P0_=np.diag([0.3**2, 0.3**2, 0.3**2]))),
        ("start 36 cm / 11 deg off, P0 honest, no ids", dict(x0=bad_start, P0_=np.diag([0.3**2, 0.3**2, 0.3**2]), use_ids=False)),
    ]
    for name, kwargs in cases:
        r = localize(ex, log, **kwargs)
        after_10s = float(r["pos_err"][np.searchsorted(log.t, 10.0)])
        print(f"   {name:<44}: wrong {r['wrong']:3d}, rejected {r['rejected']:3d}, error at 10 s {after_10s * 100:5.1f} cm, final {r['final'] * 100:5.1f} cm")
        results["association"][name] = {"wrong": r["wrong"], "rejected": r["rejected"], "err10": after_10s, "final": r["final"]}

    # 6. linearization limits ----------------------------------------------------------------------
    print("\n6) Drive 1 m straight with heading sigma 30 deg: EKF vs 100,000 sampled robots")
    sigma_th = math.radians(30.0)
    ekf = ex.EKF([0.0, 0.0, 0.0], np.diag([1e-6, 1e-6, sigma_th**2]))
    for _ in range(100):
        ekf.predict([0.01, 0.01], 0.2, np.zeros((2, 2)))
    rng = np.random.default_rng(0)
    th = rng.normal(0.0, sigma_th, 100_000)
    cloud = np.column_stack([np.cos(th), np.sin(th)])
    mc_mean, mc_cov = cloud.mean(axis=0), np.cov(cloud.T)
    inside = lc.fraction_inside(cloud, ekf.x[:2], ekf.P[:2, :2], 5.991)
    print(f"   EKF mean x = {ekf.x[0]:.3f} m, sigma x = {math.sqrt(ekf.P[0, 0]) * 100:.1f} cm, sigma y = {math.sqrt(ekf.P[1, 1]) * 100:.1f} cm")
    print(f"   true mean x = {mc_mean[0]:.3f} m (= exp(-sigma^2/2)), sigma x = {math.sqrt(mc_cov[0, 0]) * 100:.1f} cm, sigma y = {math.sqrt(mc_cov[1, 1]) * 100:.1f} cm")
    print(f"   samples inside the EKF's 95% ellipse: {inside * 100:.0f}%")
    results["linearization"] = {"ekf_x": float(ekf.x[0]), "mc_x": float(mc_mean[0]), "inside95": inside}
    print(f"\nwrote {out / 'ekf_tour.png'} and {out / 'ekf_sigma.png'}")
    return results


if __name__ == "__main__":
    main()
