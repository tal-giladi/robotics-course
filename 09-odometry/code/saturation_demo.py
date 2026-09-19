"""09.02 — What wheel saturation does to the path: clip vs scale vs keep-rotation.

    python 09-odometry/code/saturation_demo.py                    # uses YOUR labs/exercises/09.02/student.py
    python 09-odometry/code/saturation_demo.py --solution         # uses the reference solution
    python 09-odometry/code/saturation_demo.py --v 0.7 --omega 3.0 --plot arcs.png

Commands the same cmd_vel to the ideal simulator three ways and reports the radius each one
actually drives. "clip" is the tempting bug: clamp each wheel to the limit on its own.
"""

from __future__ import annotations

import argparse
import math

from course_code import load_exercise
from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World


def drive(to_wheels, seconds: float = 4.0) -> tuple[list[tuple[float, float]], float]:
    """Send wheel speeds from ``to_wheels()`` every step; return the path and the steady radius."""
    cfg = load_config()
    base = SimBase(DiffDriveSim(World(), DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg), seed=0))
    poses = []
    for _ in range(round(seconds / base.dt)):
        base.set_wheel_velocity(*to_wheels())
        base.read()
        poses.append(base.sim.pose)
    steady = poses[len(poses) // 4:]  # skip the spin-up
    arc = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(steady, steady[1:]))
    turned = sum(abs(math.remainder(b.theta - a.theta, 2 * math.pi)) for a, b in zip(steady, steady[1:]))
    return [(p.x, p.y) for p in poses], (arc / turned if turned > 1e-9 else math.inf)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--v", type=float, default=0.7, help="cmd_vel linear.x (m/s)")
    ap.add_argument("--omega", type=float, default=3.0, help="cmd_vel angular.z (rad/s)")
    ap.add_argument("--solution", action="store_true", help="use the reference solution")
    ap.add_argument("--plot", help="save the three paths as a PNG")
    args = ap.parse_args()

    ex = load_exercise("09.02", solution=args.solution)
    d = load_config().drive
    r, sep, limit = d.wheel_radius_m, d.wheel_separation_m, d.max_wheel_speed_rad_s
    raw = ex.twist_to_wheel_speeds(args.v, args.omega, r, sep)
    print(f"cmd_vel v={args.v} m/s omega={args.omega} rad/s -> asked radius {args.v / args.omega:.3f} m")
    print(f"raw wheel speeds L={raw[0]:.2f} R={raw[1]:.2f} rad/s, motor limit {limit} rad/s")

    strategies = {
        "clip each wheel": lambda: tuple(max(-limit, min(limit, w)) for w in raw),
        "scale both (keep curvature)": lambda: ex.scale_to_limit(*raw, limit),
        "keep rotation": lambda: ex.keep_rotation_to_limit(*raw, limit),
    }
    paths = {}
    for name, to_wheels in strategies.items():
        wheels = to_wheels()
        v, omega = ex.wheel_speeds_to_twist(*wheels, r, sep)
        paths[name], radius = drive(to_wheels)
        print(f"{name:28s} wheels ({wheels[0]:6.2f}, {wheels[1]:6.2f}) -> v={v:.3f} omega={omega:.3f} "
              f"| driven radius {radius:.3f} m")

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(6, 6))
        for name, path in paths.items():
            ax.plot(*zip(*path), label=name)
        ax.set_aspect("equal")
        ax.grid(True)
        ax.legend()
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        fig.savefig(args.plot, dpi=100)
        print(f"saved {args.plot}")


if __name__ == "__main__":
    main()
