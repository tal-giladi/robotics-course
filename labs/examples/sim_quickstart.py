#!/usr/bin/env python3
"""Drive a 1 m square in the simulated apartment and plot truth vs. wheel odometry.

    python labs/examples/sim_quickstart.py                 # writes sim_quickstart.png
    python labs/examples/sim_quickstart.py out/square.png --ideal

The robot code below only uses the DifferentialBase interface (set_wheel_velocity, read), so
it would drive the real robot unchanged; the plot uses the simulator's ground truth.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # no display needed

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))  # run without pip install

import matplotlib.pyplot as plt  # noqa: E402

from robotlab.config import KarmelConfig, load_config  # noqa: E402
from robotlab.geometry import SE2  # noqa: E402
from robotlab.hal import BaseState, DifferentialBase  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World, arc_update, viz  # noqa: E402

START = SE2(1.0, 1.3, 0.0)
SIDE_M = 1.0
WHEEL_SPEED = 8.0  # rad/s


def wheel_travel(state: BaseState, start: BaseState, cfg: KarmelConfig) -> tuple[float, float]:
    """Meters each wheel travelled since ``start``, from encoder ticks."""
    m_per_tick = cfg.drive.meters_per_tick
    return (state.left_ticks - start.left_ticks) * m_per_tick, (state.right_ticks - start.right_ticks) * m_per_tick


class Recorder:
    """Logs ground truth and a simple wheel-odometry estimate after every telemetry sample."""

    def __init__(self, sim: DiffDriveSim, cfg: KarmelConfig) -> None:
        self.sim, self.cfg = sim, cfg
        self.truth: list[SE2] = [sim.pose]
        self.odom: list[SE2] = [sim.pose]
        self._last_ticks = sim.ticks

    def record(self, state: BaseState) -> None:
        m = self.cfg.drive.meters_per_tick
        d_left = (state.left_ticks - self._last_ticks[0]) * m
        d_right = (state.right_ticks - self._last_ticks[1]) * m
        self._last_ticks = (state.left_ticks, state.right_ticks)
        self.odom.append(arc_update(self.odom[-1], d_left, d_right, self.cfg.drive.wheel_separation_m))
        self.truth.append(self.sim.pose)


def drive_square(base: DifferentialBase, cfg: KarmelConfig, on_state) -> None:
    """Four sides and four left turns, each ended by encoder distance (like lesson 01.12)."""
    b = cfg.drive.wheel_separation_m
    for _ in range(4):
        for left, right, done in [
            (WHEEL_SPEED, WHEEL_SPEED, lambda dl, dr: (dl + dr) / 2 >= SIDE_M),
            (-WHEEL_SPEED / 2, WHEEL_SPEED / 2, lambda dl, dr: (dr - dl) / b >= math.pi / 2),
        ]:
            start = base.read()
            state = start
            while not done(*wheel_travel(state, start, cfg)):
                base.set_wheel_velocity(left, right)  # re-send every cycle: the watchdog is on
                state = base.read()
                on_state(state)
    base.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("output", nargs="?", default="sim_quickstart.png", help="PNG path (default: ./sim_quickstart.png)")
    parser.add_argument("--ideal", action="store_true", help="perfect robot instead of a realistic one")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args()

    cfg = load_config()
    world = World.apartment()
    make = "ideal" if args.ideal else "realistic"
    sim = DiffDriveSim(
        world, getattr(DiffDriveParams, make)(cfg), getattr(SensorParams, make)(cfg), pose=START, seed=args.seed
    )
    base = SimBase(sim)

    recorder = Recorder(sim, cfg)
    drive_square(base, cfg, recorder.record)
    truth, odom = recorder.truth, recorder.odom
    base.close()

    fig, ax = viz.new_axes(world, title=f"1 m square, {make} robot — final error {truth[-1].distance_to(odom[-1]) * 100:.1f} cm")
    viz.draw_trajectory(ax, truth, color="tab:blue", label="ground truth")
    viz.draw_trajectory(ax, odom, color="tab:orange", linestyle="--", label="wheel odometry")
    viz.draw_scan(ax, sim.lidar_pose, sim.lidar_scan(), size=3.0)
    viz.draw_robot(ax, truth[-1], radius=sim.params.robot_radius_m)
    ax.legend(loc="upper right")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"simulated {sim.t:.1f} s; truth {truth[-1]}; odometry {odom[-1]}")
    print(f"saved {output.resolve()}")


if __name__ == "__main__":
    main()
