"""09.04 — Run YOUR odometry (labs/exercises/09.04/student.py) on the simulator or on karmel.

    python 09-odometry/code/odometry_live.py                         # realistic simulator
    python 09-odometry/code/odometry_live.py --ideal                 # perfect simulated robot
    python 09-odometry/code/odometry_live.py --solution --plot run.png
    python 09-odometry/code/odometry_live.py --port /dev/ttyACM0     # the real robot (on the Pi)

Drives a short plan of wheel speeds (straight, arc, spin, arc, reverse: about 2.5 m in 12 s),
feeds every telemetry sample to ``Odometry.update`` and prints the pose twice a second. In the
simulator it also prints ground truth and the error.

SAFETY (real robot): clear about 1.5 m x 1.5 m of floor, start in the middle facing an open
direction, keep the power switch within reach. Ctrl+C stops the motors.
"""

from __future__ import annotations

import argparse
import math

from course_code import load_exercise, next_state, open_base
from robotlab.config import load_config

PLAN = [  # (seconds, (left_rad_s, right_rad_s))
    (2.5, (5.0, 5.0)),
    (3.0, (3.0, 6.0)),
    (1.5, (-4.0, 4.0)),
    (3.0, (6.0, 3.0)),
    (2.0, (-4.0, -4.0)),
]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port of the real robot (omit for the simulator)")
    ap.add_argument("--ideal", action="store_true", help="simulator without imperfections")
    ap.add_argument("--solution", action="store_true", help="use the reference Odometry")
    ap.add_argument("--plot", help="save estimate (and truth) as a PNG")
    args = ap.parse_args()

    cfg = load_config()
    ex = load_exercise("09.04", solution=args.solution)
    base = open_base(args.port, realistic=not args.ideal, seed=1)
    sim = getattr(base, "sim", None)
    odom = ex.Odometry(cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m, cfg.drive.ticks_per_wheel_rev)
    estimates, truths = [], []
    try:
        state = next_state(base, None)
        odom.update(state.left_ticks, state.right_ticks)
        t0, next_print = state.t, 0.0
        for seconds, wheels in PLAN:
            end = state.t + seconds
            while state.t < end:
                base.set_wheel_velocity(*wheels)  # every cycle: the firmware watchdog is 300 ms
                state = next_state(base, state)
                x, y, theta = odom.update(state.left_ticks, state.right_ticks)
                estimates.append((x, y))
                if sim is not None:
                    truths.append((sim.pose.x, sim.pose.y))
                if state.t - t0 >= next_print:
                    next_print += 0.5
                    line = f"t={state.t - t0:5.2f}s odom x={x:+.3f} y={y:+.3f} th={math.degrees(theta):+7.1f} deg"
                    if sim is not None:
                        p = sim.pose
                        err = math.hypot(x - p.x, y - p.y)
                        line += f" | truth x={p.x:+.3f} y={p.y:+.3f} th={math.degrees(p.theta):+7.1f} | err {err * 100:.1f} cm"
                    print(line)
    finally:
        base.stop()
        base.close()
    if sim is None:
        print("Now measure where the robot really is (tape from the start mark) and compare with the last line.")

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(*zip(*estimates), "--", label="odometry")
        if truths:
            ax.plot(*zip(*truths), label="truth")
        ax.set_aspect("equal")
        ax.grid(True)
        ax.legend()
        fig.savefig(args.plot, dpi=100)
        print(f"saved {args.plot}")


if __name__ == "__main__":
    main()
