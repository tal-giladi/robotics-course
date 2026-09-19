"""06.02 — Time step vs accuracy: the course simulator against a naive Euler simulator.

    python 06-simulation/code/timestep_experiment.py
    python 06-simulation/code/timestep_experiment.py --plot timestep.png

Part 1 — one motor. A first-order motor (tau = motor_time_constant_s) is kicked to a target speed.
Forward Euler vs the exact discretization that DiffDriveSim uses, for several time steps.

Part 2 — the whole robot. The same 3 s duty script (straight, arc, spin-ish arc) is simulated with
(a) DiffDriveSim.step and (b) a naive simulator that uses Euler for the motor AND for the pose.
Both are compared with DiffDriveSim at dt = 0.1 ms (the reference).
"""

from __future__ import annotations

import argparse
import math
import sys
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_LABS_PYTHON = Path(__file__).resolve().parent.parent.parent / "labs" / "python"
if str(_LABS_PYTHON) not in sys.path:  # robotlab without `pip install -e labs/python`
    sys.path.insert(0, str(_LABS_PYTHON))

from robotlab.config import load_config  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World  # noqa: E402
from robotlab.sim.components import deadband_speed, first_order_alpha  # noqa: E402

MOTOR_STEPS = (0.001, 0.01, 0.02, 0.04, 0.08, 0.12, 0.2)
ROBOT_STEPS = (0.001, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2)


# --- part 1: one motor ------------------------------------------------------------------------------
def motor_speed(dt: float, t_end: float, target: float, tau: float, method: str) -> float:
    """Speed of a first-order motor started from rest, after round(t_end/dt) steps."""
    w = 0.0
    alpha = dt / tau if method == "euler" else first_order_alpha(dt, tau)
    for _ in range(round(t_end / dt)):
        w += alpha * (target - w)
    return w


# --- part 2: the whole robot ------------------------------------------------------------------------
def duty_script(t: float) -> tuple[float, float]:
    """Straight for 1 s, a left arc for 1 s, a tight left turn for 1 s."""
    if t < 1.0:
        return 0.6, 0.6
    if t < 2.0:
        return 0.3, 0.7
    return -0.2, 0.5


@dataclass
class NaiveSim:
    """What most people write first: Euler for the motor and Euler for the pose."""

    params: DiffDriveParams
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def __post_init__(self) -> None:
        self.w = np.zeros(2)
        self.duty = np.zeros(2)

    def set_duty(self, left: float, right: float) -> None:
        self.duty = np.array([left, right], dtype=float)

    def step(self, dt: float) -> None:
        p = self.params
        target = deadband_speed(self.duty, p.duty_deadband, p.max_wheel_speed_rad_s)
        v_wheels = self.w * p.wheel_radius_m  # speeds at the START of the step
        v = 0.5 * (v_wheels[0] + v_wheels[1])
        omega = (v_wheels[1] - v_wheels[0]) / p.wheel_separation_m
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.w = self.w + dt / p.motor_time_constant_s * (target - self.w)


def run_script(make: Callable[[], object], dt: float, duration: float = 3.0) -> tuple[float, float, float]:
    sim = make()
    for k in range(round(duration / dt)):
        sim.set_duty(*duty_script(k * dt))
        sim.step(dt)
    if isinstance(sim, DiffDriveSim):
        return sim.pose.x, sim.pose.y, sim.pose.theta
    return sim.x, sim.y, sim.theta


def robot_errors(dt: float, reference: tuple[float, float, float], params: DiffDriveParams) -> dict[str, tuple[float, float]]:
    """Position error (m) and heading error (rad) of both simulators against the reference."""
    course = run_script(lambda: DiffDriveSim(World(), params, SensorParams.ideal(), seed=0), dt)
    naive = run_script(lambda: NaiveSim(params), dt)
    out = {}
    for name, (x, y, th) in (("course", course), ("naive", naive)):
        out[name] = (math.hypot(x - reference[0], y - reference[1]), abs(math.remainder(th - reference[2], 2 * math.pi)))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plot", help="save a log-log plot of robot position error vs dt")
    args = ap.parse_args()

    d = load_config().drive
    tau = d.motor_time_constant_s
    target = float(deadband_speed(np.array([0.6]), d.duty_deadband, d.max_wheel_speed_rad_s)[0])
    exact = target * (1.0 - math.exp(-0.24 / tau))
    print(f"Part 1 — motor, tau = {tau} s, target {target:.2f} rad/s, true speed at t = 0.24 s: {exact:.3f} rad/s")
    print(f"{'dt (s)':>7} {'dt/tau':>7} {'Euler @0.24s':>13} {'exact @0.24s':>13} {'Euler @1s':>10}")
    for dt in MOTOR_STEPS:
        print(f"{dt:7.3f} {dt / tau:7.2f} {motor_speed(dt, 0.24, target, tau, 'euler'):13.3f} "
              f"{motor_speed(dt, 0.24, target, tau, 'exact'):13.3f} {motor_speed(dt, 1.0, target, tau, 'euler'):10.2f}")

    params = DiffDriveParams.ideal()
    reference = run_script(lambda: DiffDriveSim(World(), params, SensorParams.ideal(), seed=0), 0.0001)
    print(f"\nPart 2 — robot, 3 s script; reference pose at dt = 0.1 ms: "
          f"({reference[0]:.4f}, {reference[1]:.4f}, {math.degrees(reference[2]):.2f} deg)")
    print(f"{'dt (s)':>7} {'rate (Hz)':>9} {'course pos (mm)':>16} {'course hdg (deg)':>17} {'naive pos (mm)':>15} {'naive hdg (deg)':>16}")
    table = {}
    for dt in ROBOT_STEPS:
        e = robot_errors(dt, reference, params)
        table[dt] = e
        print(f"{dt:7.3f} {1 / dt:9.0f} {e['course'][0] * 1000:16.2f} {math.degrees(e['course'][1]):17.2f} "
              f"{e['naive'][0] * 1000:15.1f} {math.degrees(e['naive'][1]):16.2f}")

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(6, 4))
        for name in ("course", "naive"):
            ax.loglog(ROBOT_STEPS, [max(table[dt][name][0], 1e-7) * 1000 for dt in ROBOT_STEPS], "o-", label=name)
        ax.set_xlabel("time step dt (s)")
        ax.set_ylabel("position error after 3 s (mm)")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print(f"saved {args.plot}")


if __name__ == "__main__":
    main()
