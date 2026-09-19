"""09.06 — Why odometry drifts: a Monte Carlo experiment and first-order error propagation.

    python 09-odometry/code/drift_monte_carlo.py                        # 100 runs x 5 m, ~30 s
    python 09-odometry/code/drift_monte_carlo.py --runs 30 --plot drift.png

The simulated robot (DiffDriveParams.realistic) drives a 5 m straight line many times with
different random slip. Two odometries watch the SAME encoder ticks:

* ``nominal``    — karmel.yaml geometry: systematic error (wrong radii/wheelbase) + random slip
* ``calibrated`` — the true geometry of the simulated robot: only the random slip is left

For each we report the error of odometry vs truth, expressed in the true robot's frame: "along"
(direction of travel) and "lateral" (to its left), and the heading error, every meter. Then we compare the calibrated spread
with the first-order propagation of wheel noise (sigma^2 = k * |wheel travel|).
"""

from __future__ import annotations

import argparse
import dataclasses
import math

import numpy as np

from course_code import WheelOdometry
from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World

DT = 0.02
WHEEL_RAD_S = 5.5  # 0.25 m/s


def run_once(params: DiffDriveParams, distance: float, seed: int, checkpoints: list[float]):
    """Drive straight; return {name: array (len(checkpoints), 3) of [along, lateral, heading] errors}."""
    cfg = load_config()
    sim = DiffDriveSim(World(), params, SensorParams.ideal(cfg), seed=seed)
    d = cfg.drive
    odoms = {
        "nominal": WheelOdometry(d.wheel_radius_m, d.wheel_radius_m, d.wheel_separation_m, d.ticks_per_wheel_rev),
        "calibrated": WheelOdometry(
            params.true_wheel_radius_left_m, params.true_wheel_radius_right_m,
            params.true_wheel_separation_m, d.ticks_per_wheel_rev,
        ),
    }
    for odom in odoms.values():
        odom.update(*sim.ticks)
    errors = {name: [] for name in odoms}
    travelled, last = 0.0, sim.pose
    sim.set_velocity(WHEEL_RAD_S, WHEEL_RAD_S)
    for target in checkpoints:
        while travelled < target:
            sim.step(DT)
            p = sim.pose
            travelled += math.hypot(p.x - last.x, p.y - last.y)
            last = p
            for odom in odoms.values():
                odom.update(*sim.ticks)
        c, s = math.cos(p.theta), math.sin(p.theta)
        for name, odom in odoms.items():
            x, y, theta = odom.pose
            dx, dy = x - p.x, y - p.y  # error in the world frame, rotated into the TRUE robot frame:
            along, lateral = c * dx + s * dy, -s * dx + c * dy  # along = direction of travel
            errors[name].append([along, lateral, math.remainder(theta - p.theta, 2 * math.pi)])
    return {name: np.asarray(e) for name, e in errors.items()}


def monte_carlo(runs: int, distance: float, step_m: float = 1.0, params: DiffDriveParams | None = None):
    """Errors of every run: {name: array (runs, checkpoints, 3)} and the checkpoint distances."""
    params = params or DiffDriveParams.realistic(load_config())
    checkpoints = [step_m * (i + 1) for i in range(round(distance / step_m))]
    results = [run_once(params, distance, seed, checkpoints) for seed in range(runs)]
    return {name: np.stack([r[name] for r in results]) for name in results[0]}, np.asarray(checkpoints)


def predicted_sigmas(distance: np.ndarray, k: float, separation: float) -> dict[str, np.ndarray]:
    """First-order spread for a straight line when each wheel's travel error has variance k*|travel|.

    heading:  var = (k d + k d) / b^2 = 2 k d / b^2           (random walk: sigma ~ sqrt(d))
    along:    var = (k d + k d) / 4   = k d / 2               (sigma ~ sqrt(d))
    lateral:  y = integral of d' * dtheta(d') -> var = (2k / b^2) d^3 / 3   (sigma ~ d^1.5)
    """
    q = 2.0 * k / separation**2
    return {
        "heading": np.sqrt(q * distance),
        "along": np.sqrt(k * distance / 2.0),
        "lateral": np.sqrt(q * distance**3 / 3.0),
    }


def propagate_covariance(
    pose: tuple[float, float, float], cov: np.ndarray, d_left: float, d_right: float, separation: float, k: float
) -> np.ndarray:
    """One step of linearized covariance propagation: cov' = F cov F^T + G Q G^T (Chong & Kleeman style).

    State (x, y, theta); inputs (d_left, d_right) with Q = diag(k|d_left|, k|d_right|).
    Uses the midpoint model, which is exact to first order for small steps.
    """
    _, _, theta = pose
    ds, dtheta = (d_left + d_right) / 2.0, (d_right - d_left) / separation
    mid = theta + dtheta / 2.0
    c, s = math.cos(mid), math.sin(mid)
    F = np.array([[1.0, 0.0, -ds * s], [0.0, 1.0, ds * c], [0.0, 0.0, 1.0]])
    G = np.array([
        [c / 2 + ds * s / (2 * separation), c / 2 - ds * s / (2 * separation)],
        [s / 2 - ds * c / (2 * separation), s / 2 + ds * c / (2 * separation)],
        [-1.0 / separation, 1.0 / separation],
    ])
    Q = np.diag([k * abs(d_left), k * abs(d_right)])
    return F @ cov @ F.T + G @ Q @ G.T


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=100)
    ap.add_argument("--distance", type=float, default=5.0)
    ap.add_argument("--plot", help="save end-point scatter and sigma growth as a PNG")
    args = ap.parse_args()

    params = DiffDriveParams.realistic(load_config())
    errors, dist = monte_carlo(args.runs, args.distance, params=params)
    print(f"{args.runs} runs, realistic preset (true radii {params.true_wheel_radius_left_m * 1000:.2f}/"
          f"{params.true_wheel_radius_right_m * 1000:.2f} mm, wheelbase {params.true_wheel_separation_m * 1000:.0f} mm, "
          f"slip_std {params.slip_std})")
    for name, e in errors.items():
        print(f"\n{name} odometry: error = odometry - truth")
        print(f"{'d (m)':>6} {'mean along':>11} {'mean lat':>9} {'std along':>10} {'std lat':>8} {'mean hdg':>9} {'std hdg':>8}")
        for i, d in enumerate(dist):
            m, sd = e[:, i].mean(axis=0), e[:, i].std(axis=0, ddof=1)
            print(f"{d:6.1f} {m[0] * 100:9.2f}cm {m[1] * 100:7.1f}cm {sd[0] * 1000:8.1f}mm {sd[1] * 100:6.2f}cm "
                  f"{math.degrees(m[2]):7.2f}deg {math.degrees(sd[2]):6.2f}deg")

    # Wheel-noise constant of this simulator: slip is multiplicative per step on each wheel's travel,
    # so over a distance d made of steps of length delta: var = slip_std^2 * delta * d  ->  k = slip_std^2 * delta.
    delta = WHEEL_RAD_S * params.wheel_radius_m * DT
    k = params.slip_std**2 * delta
    pred = predicted_sigmas(dist, k, params.true_wheel_separation_m)
    cal = errors["calibrated"]
    print(f"\nfirst-order prediction with k = {k:.2e} m (calibrated odometry, random error only):")
    for i, d in enumerate(dist):
        sd = cal[:, i].std(axis=0, ddof=1)
        print(f"{d:6.1f} m  lateral {sd[1] * 100:5.2f} cm (pred {pred['lateral'][i] * 100:5.2f})  "
              f"along {sd[0] * 1000:5.1f} mm (pred {pred['along'][i] * 1000:5.1f})  "
              f"heading {math.degrees(sd[2]):4.2f} deg (pred {math.degrees(pred['heading'][i]):4.2f})")

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
        for name, e in errors.items():
            ax1.scatter(-e[:, -1, 0] * 100, -e[:, -1, 1] * 100, s=10, label=f"{name}: truth - odometry")
        ax1.set_xlabel("along error (cm)")
        ax1.set_ylabel("lateral error (cm)")
        ax1.set_title(f"where the robot really is after {dist[-1]:.0f} m")
        ax1.axis("equal")
        ax1.grid(True)
        ax1.legend()
        sd = cal.std(axis=0, ddof=1)
        ax2.plot(dist, sd[:, 1] * 100, "o-", label="lateral std (cm)")
        ax2.plot(dist, sd[:, 0] * 100, "s-", label="along std (cm)")
        ax2.plot(dist, pred["lateral"] * 100, "k--", label="prediction ~ d^1.5")
        ax2.set_xlabel("distance driven (m)")
        ax2.set_title("random error only (calibrated)")
        ax2.grid(True)
        ax2.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=100)
        print(f"saved {args.plot}")


if __name__ == "__main__":
    main()
