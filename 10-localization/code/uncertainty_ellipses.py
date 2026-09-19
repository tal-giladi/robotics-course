"""10.02 — Representing uncertainty: Gaussians and covariance ellipses.

    python 10-localization/code/uncertainty_ellipses.py            # writes localization_out/ellipses_*.png

1. The worked example: eigen-decompose a 2x2 position covariance into ellipse axes.
2. How much probability 1-sigma, 2-sigma and "95%" ellipses really hold (1D vs 2D).
3. Watch the 95% ellipse grow while the simulated robot drives, and check the coverage against the
   Monte Carlo cloud (the Gaussian is an approximation: it gets worse as the cloud curves).
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

# The cloud of 10.01 at t = 12.6 s (after the first left turn), rounded: m^2
EXAMPLE_COV = np.array([[0.049, -0.026], [-0.026, 0.018]])
EXAMPLE_MEAN = np.array([2.93, 2.44])


def worked_example() -> dict:
    eigvals, eigvecs = np.linalg.eigh(EXAMPLE_COV)  # ascending
    sigma_major, sigma_minor = math.sqrt(eigvals[1]), math.sqrt(eigvals[0])
    angle = math.degrees(math.atan2(eigvecs[1, 1], eigvecs[0, 1]))
    angle = (angle + 90.0) % 180.0 - 90.0
    rho = EXAMPLE_COV[0, 1] / math.sqrt(EXAMPLE_COV[0, 0] * EXAMPLE_COV[1, 1])
    return {
        "sigma_x": math.sqrt(EXAMPLE_COV[0, 0]), "sigma_y": math.sqrt(EXAMPLE_COV[1, 1]), "rho": rho,
        "eigvals": eigvals, "sigma_major": sigma_major, "sigma_minor": sigma_minor, "angle_deg": angle,
        "trace": float(np.trace(EXAMPLE_COV)), "det": float(np.linalg.det(EXAMPLE_COV)),
    }


def coverage_table() -> list[tuple[str, float, float, float]]:
    """(name, k = d^2 threshold, P inside in 1D, P inside in 2D) for common ellipse choices."""
    rows = []
    for name, k in (("1 sigma", 1.0), ("2 sigma", 4.0), ("3 sigma", 9.0),
                    ("95%", lc.chi2_2dof_threshold(0.95)), ("99%", lc.chi2_2dof_threshold(0.99))):
        p1 = math.erf(math.sqrt(k) / math.sqrt(2.0))  # 1D: P(|z| < sqrt(k))
        p2 = 1.0 - math.exp(-k / 2.0)                  # 2D: chi-square with 2 dof, closed form
        rows.append((name, k, p1, p2))
    return rows


def ellipse_growth(seed: int = 7, n: int = 1000, every_s: float = 1.0) -> tuple[lc.Drive, np.ndarray, list[dict]]:
    drive = lc.drive_living_room(seed=seed)
    cloud = lc.odometry_cloud(drive.ticks, lc.START, n=n)
    rows = []
    for t in np.arange(every_s, drive.t[-1] + 1e-9, every_s):
        k = int(np.argmin(np.abs(drive.t - t)))
        pts = cloud[k, :, :2]
        mean, cov = pts.mean(axis=0), np.cov(pts.T)
        major, minor, angle = lc.ellipse_axes(cov, lc.CHI2_2DOF_95)
        rows.append({
            "t": float(drive.t[k]), "k": k, "mean": mean, "cov": cov, "major95": major, "minor95": minor, "angle": angle,
            "in_1sigma": lc.fraction_inside(pts, mean, cov, 1.0),
            "in_2sigma": lc.fraction_inside(pts, mean, cov, 4.0),
            "in_95": lc.fraction_inside(pts, mean, cov, lc.CHI2_2DOF_95),
        })
    return drive, cloud, rows


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="localization_out")
    args = ap.parse_args(argv)
    out = lc.out_dir(args.out)

    ex = worked_example()
    print("1) Worked example: Sigma = [[0.049, -0.026], [-0.026, 0.018]] m^2")
    print(f"   sigma_x={ex['sigma_x'] * 100:.1f} cm  sigma_y={ex['sigma_y'] * 100:.1f} cm  rho={ex['rho']:+.3f}")
    print(f"   eigenvalues={ex['eigvals'][1]:.5f}, {ex['eigvals'][0]:.5f} m^2 (sum = trace {ex['trace']:.3f}, product = det {ex['det']:.6f})")
    print(f"   sigma along axes: {ex['sigma_major'] * 100:.1f} cm (major, at {ex['angle_deg']:.1f} deg) and {ex['sigma_minor'] * 100:.1f} cm")
    for name, k, _, _ in coverage_table()[:2] + coverage_table()[3:4]:
        print(f"   {name:>7} ellipse (k={k:.3f}): semi-axes {math.sqrt(k) * ex['sigma_major'] * 100:.1f} x {math.sqrt(k) * ex['sigma_minor'] * 100:.1f} cm")

    print("2) Probability inside:      k      1D       2D")
    for name, k, p1, p2 in coverage_table():
        print(f"   {name:>7}              {k:6.3f}  {p1 * 100:5.1f}%   {p2 * 100:5.1f}%")

    drive, cloud, rows = ellipse_growth()
    print("3) The 95% ellipse while karmel drives (1000-robot Monte Carlo cloud)")
    print(f"{'t (s)':>7} {'major (cm)':>10} {'minor (cm)':>10} {'angle':>7} {'in 1sig':>8} {'in 2sig':>8} {'in 95%':>7}")
    for r in rows:
        print(f"{r['t']:7.1f} {r['major95'] * 100:10.1f} {r['minor95'] * 100:10.1f} {r['angle']:7.1f} "
              f"{r['in_1sigma']:8.3f} {r['in_2sigma']:8.3f} {r['in_95']:7.3f}")

    # Plot A: the worked example with 1-sigma, 2-sigma, 95% ellipses over samples
    rng = np.random.default_rng(2)
    samples = rng.multivariate_normal(EXAMPLE_MEAN, EXAMPLE_COV, 2000)
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.scatter(samples[:, 0], samples[:, 1], s=2, alpha=0.3, color="gray")
    for n_sigma, color, label in ((1.0, "tab:blue", "1 sigma"), (2.0, "tab:orange", "2 sigma"),
                                  (math.sqrt(lc.CHI2_2DOF_95), "tab:red", "95%")):
        viz.draw_covariance_ellipse(ax, EXAMPLE_MEAN, EXAMPLE_COV, n_sigma=n_sigma, color=color, label=label, lw=2)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend()
    ax.set_title("Sigma = [[0.049, -0.026], [-0.026, 0.018]]")
    fig.tight_layout()
    fig.savefig(out / "ellipses_example.png", dpi=120)
    plt.close(fig)

    # Plot B: ellipses along the drive
    fig, ax = viz.new_axes(drive.world, title="95% ellipse every second (odometry only)")
    viz.draw_trajectory(ax, drive.truth, label="truth", color="black")
    for r in rows:
        viz.draw_covariance_ellipse(ax, r["mean"], r["cov"], n_sigma=math.sqrt(lc.CHI2_2DOF_95), color="tab:purple")
    ax.set_xlim(0.0, 3.6)
    ax.set_ylim(0.0, 3.3)
    fig.savefig(out / "ellipses_growth.png", dpi=120)
    plt.close(fig)
    print(f"wrote {out / 'ellipses_example.png'} and {out / 'ellipses_growth.png'}")
    return {"example": ex, "coverage": coverage_table(), "growth": rows}


if __name__ == "__main__":
    main()
