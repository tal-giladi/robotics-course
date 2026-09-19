"""10.01 — Belief instead of certainty: the odometry cloud and the kidnapped robot.

    python 10-localization/code/belief_cloud.py                 # writes localization_out/belief_cloud.png, kidnapped.png
    python 10-localization/code/belief_cloud.py --out my_plots

Part 1: karmel (realistic simulator) drives a U through the living room. Its wheel odometry gives
ONE pose. 500 hypothetical robots integrate the same encoder ticks with plausible calibration
errors and slip: the cloud they form is the robot's belief, and it grows with distance.

Part 2: the same drive, but at t = 8.1 s the robot is picked up and put down 0.9 m away. The
encoders notice nothing. The landmark observations (range/bearing to wall tags) stop matching
what the odometry pose predicts — that mismatch is how a localizer can notice a kidnapping.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import loc_common as lc  # noqa: E402
from robotlab.sim import viz  # noqa: E402

SNAPSHOTS_S = (2.0, 7.0, 12.6, 19.7)
KIDNAP_S = 8.1


def circular_std(angles: np.ndarray) -> float:
    """Spread of headings that may straddle +-pi (np.std would see -179 deg and 179 deg as far apart)."""
    mean = np.arctan2(np.sin(angles).mean(), np.cos(angles).mean())
    return float(np.std(np.angle(np.exp(1j * (angles - mean)))))


def cloud_table(seed: int = 7, n: int = 500) -> tuple[lc.Drive, np.ndarray, np.ndarray, list[dict]]:
    drive = lc.drive_living_room(seed=seed)
    odom = lc.dead_reckon(drive.ticks, lc.START)
    cloud = lc.odometry_cloud(drive.ticks, lc.START, n=n)
    rows = []
    for t in SNAPSHOTS_S:
        k = int(np.argmin(np.abs(drive.t - t)))
        xy = cloud[k, :, :2]
        cov = np.cov(xy.T)
        rows.append({
            "t": float(drive.t[k]),
            "k": k,
            "odom_error_m": float(np.hypot(*(odom[k, :2] - drive.truth[k, :2]))),
            "sigma_x_m": float(np.sqrt(cov[0, 0])),
            "sigma_y_m": float(np.sqrt(cov[1, 1])),
            "sigma_theta_deg": float(np.degrees(circular_std(cloud[k, :, 2]))),
            "truth_inside_95": lc.fraction_inside(drive.truth[k:k + 1, :2], xy.mean(axis=0), cov, lc.CHI2_2DOF_95) == 1.0,
        })
    return drive, odom, cloud, rows


def kidnap_residuals(seed: int = 7) -> tuple[lc.Drive, np.ndarray, dict]:
    drive = lc.drive_living_room(seed=seed, kidnap_at_s=KIDNAP_S)
    odom = lc.dead_reckon(drive.ticks, lc.START)
    res = np.array(lc.landmark_residuals(drive, odom))
    before, after = res[res[:, 0] < KIDNAP_S], res[res[:, 0] >= KIDNAP_S]
    summary = {
        "before_range_m": float(np.median(np.abs(before[:, 1]))),
        "after_range_m": float(np.median(np.abs(after[:, 1]))),
        "before_bearing_deg": float(np.degrees(np.median(np.abs(before[:, 2])))),
        "after_bearing_deg": float(np.degrees(np.median(np.abs(after[:, 2])))),
        "residuals": res,
    }
    return drive, odom, summary


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="localization_out")
    args = ap.parse_args(argv)
    out = lc.out_dir(args.out)

    drive, odom, cloud, rows = cloud_table()
    print("Part 1 - the belief cloud (500 hypothetical robots, same encoder ticks)")
    print(f"{'t (s)':>6} {'odom err (cm)':>13} {'sigma_x (cm)':>12} {'sigma_y (cm)':>12} {'sigma_th (deg)':>14} {'truth in 95%':>12}")
    for r in rows:
        print(f"{r['t']:6.1f} {r['odom_error_m'] * 100:13.1f} {r['sigma_x_m'] * 100:12.1f} {r['sigma_y_m'] * 100:12.1f} "
              f"{r['sigma_theta_deg']:14.1f} {str(r['truth_inside_95']):>12}")

    fig, ax = viz.new_axes(drive.world, title="Odometry gives one pose; the belief is a cloud")
    viz.draw_trajectory(ax, drive.truth, label="truth", color="black")
    viz.draw_trajectory(ax, odom, label="wheel odometry", linestyle="--", color="tab:red")
    colors = ("tab:blue", "tab:orange", "tab:green", "tab:purple")
    for r, color in zip(rows, colors):
        pts = cloud[r["k"], :, :2]
        ax.scatter(pts[:, 0], pts[:, 1], s=2, alpha=0.35, color=color, label=f"belief at t={r['t']:.1f} s")
        viz.draw_covariance_ellipse(ax, pts.mean(axis=0), np.cov(pts.T), n_sigma=2.4477, color=color)  # 95%
    ax.set_xlim(0.0, 3.6)
    ax.set_ylim(0.0, 3.3)
    ax.legend(loc="lower right", fontsize=7)
    fig.savefig(out / "belief_cloud.png", dpi=120)
    plt.close(fig)

    _, _, kid = kidnap_residuals()
    print(f"\nPart 2 - kidnapped at t={KIDNAP_S} s (moved 0.9 m, encoders see nothing)")
    print(f"  median |range residual|   before {kid['before_range_m'] * 100:5.1f} cm   after {kid['after_range_m'] * 100:5.1f} cm")
    print(f"  median |bearing residual| before {kid['before_bearing_deg']:5.1f} deg  after {kid['after_bearing_deg']:5.1f} deg")
    res = kid["residuals"]
    fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    axes[0].plot(res[:, 0], res[:, 1] * 100, "o", ms=3)
    axes[0].set_ylabel("range residual (cm)")
    axes[1].plot(res[:, 0], np.degrees(res[:, 2]), "o", ms=3)
    axes[1].set_ylabel("bearing residual (deg)")
    axes[1].set_xlabel("time (s)")
    for a in axes:
        a.axvline(KIDNAP_S, color="tab:red", linestyle="--")
        a.grid(alpha=0.3)
    axes[0].set_title("Measured minus expected-from-odometry landmark observations")
    fig.tight_layout()
    fig.savefig(out / "kidnapped.png", dpi=120)
    plt.close(fig)
    print(f"wrote {Path(out) / 'belief_cloud.png'} and {Path(out) / 'kidnapped.png'}")
    return {"rows": rows, "kidnap": kid}


if __name__ == "__main__":
    main()
