"""09.03 — The time-step experiment: Euler vs midpoint vs exact arcs.

    python 09-odometry/code/integration_experiment.py              # YOUR labs/exercises/09.03/student.py
    python 09-odometry/code/integration_experiment.py --solution
    python 09-odometry/code/integration_experiment.py --solution --plot steps.png

Drives 5 s at v = 0.5 m/s, omega = 2 rad/s (a 0.25 m circle, 1.6 turns) with each method and
several time steps, and prints the final position error against the exact arc.
"""

from __future__ import annotations

import argparse

from course_code import load_exercise

STEPS = (0.5, 0.2, 0.1, 0.05, 0.02, 0.01)


def error_table(ex, v: float = 0.5, omega: float = 2.0, duration: float = 5.0) -> dict[str, list[float]]:
    methods = {"euler": ex.euler_step, "midpoint": ex.midpoint_step, "exact": ex.exact_step}
    return {name: [ex.constant_twist_error(step, v, omega, duration, dt) for dt in STEPS] for name, step in methods.items()}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--solution", action="store_true")
    ap.add_argument("--plot", help="save a log-log plot of error vs time step")
    args = ap.parse_args()
    ex = load_exercise("09.03", solution=args.solution)

    table = error_table(ex)
    print(f"{'dt (s)':>8} {'rate (Hz)':>9} {'Euler (mm)':>12} {'midpoint (mm)':>14} {'exact (mm)':>11}")
    for i, dt in enumerate(STEPS):
        print(f"{dt:8.2f} {1 / dt:9.0f} {table['euler'][i] * 1000:12.3f} {table['midpoint'][i] * 1000:14.4f} "
              f"{table['exact'][i] * 1000:11.1e}")
    for name in ("euler", "midpoint"):
        ratio = table[name][2] / table[name][3]
        print(f"{name}: halving dt from 0.1 to 0.05 s divides the error by {ratio:.2f}")

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(6, 4))
        for name in ("euler", "midpoint"):
            ax.loglog(STEPS, [e * 1000 for e in table[name]], "o-", label=name)
        ax.set_xlabel("time step dt (s)")
        ax.set_ylabel("final position error (mm)")
        ax.grid(True, which="both")
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=100)
        print(f"saved {args.plot}")


if __name__ == "__main__":
    main()
