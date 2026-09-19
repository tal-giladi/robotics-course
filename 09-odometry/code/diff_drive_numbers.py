"""09.01 — Differential-drive forward kinematics with karmel's numbers.

    python 09-odometry/code/diff_drive_numbers.py                 # the worked examples as a table
    python 09-odometry/code/diff_drive_numbers.py 4.0 8.0         # your own wheel speeds (rad/s)
    python 09-odometry/code/diff_drive_numbers.py 4.0 8.0 --plot icc.png

Robot numbers come from labs/config/karmel.yaml (r = 0.045 m, L = 0.200 m, 2464 ticks/rev).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import course_code  # noqa: F401  (puts robotlab on sys.path)
from robotlab.config import load_config


@dataclass(frozen=True)
class BodyMotion:
    v: float  # forward speed of base_link (m/s)
    omega: float  # yaw rate (rad/s), + = counter-clockwise (left)
    radius: float  # signed distance from base_link to the ICC along +y (m); inf = straight
    left_rim: float  # rim speeds (m/s)
    right_rim: float


def forward_kinematics(left_rad_s: float, right_rad_s: float, r: float, separation: float) -> BodyMotion:
    """Wheel angular speeds -> body motion and instantaneous center of curvature."""
    v_l, v_r = r * left_rad_s, r * right_rad_s
    v = (v_r + v_l) / 2.0
    omega = (v_r - v_l) / separation
    radius = math.inf if abs(omega) < 1e-12 else v / omega
    return BodyMotion(v, omega, radius, v_l, v_r)


def ticks_per_period(speed_m_s: float, r: float, ticks_per_rev: int, period_s: float) -> float:
    """How many encoder ticks one wheel produces in ``period_s`` at rim speed ``speed_m_s``."""
    return speed_m_s / (2.0 * math.pi * r) * ticks_per_rev * period_s


def describe(left: float, right: float, r: float, separation: float) -> str:
    m = forward_kinematics(left, right, r, separation)
    radius = "straight" if math.isinf(m.radius) else f"{m.radius:+.3f} m"
    return (
        f"wheels L={left:+6.2f} R={right:+6.2f} rad/s | rims {m.left_rim:+.3f} {m.right_rim:+.3f} m/s | "
        f"v={m.v:+.3f} m/s  omega={m.omega:+.3f} rad/s ({math.degrees(m.omega):+6.1f} deg/s) | ICC {radius}"
    )


def plot_icc(path: str, left: float, right: float, r: float, separation: float, seconds: float = 2.0) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    m = forward_kinematics(left, right, r, separation)
    fig, ax = plt.subplots(figsize=(6, 6))
    for offset, name, style in ((0.0, "base_link", "k-"), (separation / 2, "left wheel", "g--"), (-separation / 2, "right wheel", "r--")):
        xs, ys = [], []
        for i in range(101):
            t = seconds * i / 100
            theta = m.omega * t
            if math.isinf(m.radius):
                x, y = m.v * t, 0.0
            else:
                x, y = m.radius * math.sin(theta), m.radius * (1 - math.cos(theta))
            xs.append(x - offset * math.sin(theta))
            ys.append(y + offset * math.cos(theta))
        ax.plot(xs, ys, style, label=name)
    if not math.isinf(m.radius):
        ax.plot([0], [m.radius], "bx", markersize=10, label="ICC")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{seconds:.0f} s at L={left} R={right} rad/s")
    ax.legend()
    ax.grid(True)
    fig.savefig(path, dpi=100)
    print(f"saved {path}")


def main() -> None:
    cfg = load_config()
    r, sep, tpr = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m, cfg.drive.ticks_per_wheel_rev
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("left", nargs="?", type=float)
    ap.add_argument("right", nargs="?", type=float)
    ap.add_argument("--plot", help="save a PNG of the wheel paths around the ICC")
    args = ap.parse_args()

    print(f"karmel: r = {r} m, L = {sep} m, {tpr} ticks/rev -> {2 * math.pi * r / tpr * 1000:.4f} mm/tick")
    cases = [(args.left, args.right)] if args.left is not None else [
        (6.0, 6.0), (4.0, 8.0), (-5.0, 5.0), (0.0, 8.0), (8.0, 4.0), (-6.0, -6.0),
    ]
    for left, right in cases:
        print(describe(left, right, r, sep))
    print(f"at 0.3 m/s one wheel makes {ticks_per_period(0.3, r, tpr, 0.02):.1f} ticks per 20 ms telemetry period")
    if args.plot:
        left, right = cases[0] if args.left is not None else (4.0, 8.0)
        plot_icc(args.plot, left, right, r, sep)


if __name__ == "__main__":
    main()
