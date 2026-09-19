"""Generate ``logged_runs.json`` for exercise 09.05 from the course simulator.

    python labs/exercises/09.05/make_logs.py            # rewrites logged_runs.json next to this file

The simulated robot has hidden wheel radii and wheel separation that differ from the nominal
values. It drives the experiments you would drive on the floor, closed-loop on its own
(uncalibrated) odometry, and logs what a real robot logs:

* ``cw`` / ``ccw``   UMBmark squares, 5 runs each direction, side ``square_side_m``
* ``straight``       straight runs, then you measure the distance with a tape
* ``spin``           spins in place of about two turns, heading change measured with a protractor mark

Each run stores the encoder counts (reset to zero at the start, sampled at ``sample_rate_hz``)
and the final pose "measured by hand" = simulator truth + measurement noise (3 mm, 0.3 deg).
The file is self-describing: ticks per revolution and the nominal geometry are inside.
"""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path

import numpy as np

from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World

HERE = Path(__file__).resolve().parent
OUTPUT = HERE / "logged_runs.json"

# The hidden truth (the checker compares your calibration with these scales).
TRUE_SCALES = {"wheel_radius_scale_left": 0.992, "wheel_radius_scale_right": 1.006, "wheel_separation_scale": 1.05}
SQUARE_SIDE_M = 2.0
DT = 0.02
LOG_EVERY = 2  # 25 Hz


class OdometryPose:
    """Nominal-parameter odometry the robot uses to decide when a leg or a turn is done."""

    def __init__(self, radius: float, separation: float, ticks_per_rev: int) -> None:
        self.m_per_tick = 2 * math.pi * radius / ticks_per_rev
        self.separation = separation
        self.x = self.y = self.theta = 0.0  # theta is NOT wrapped: easier turn targets
        self.last = (0, 0)

    def update(self, ticks: tuple[int, int]) -> None:
        dl = (ticks[0] - self.last[0]) * self.m_per_tick
        dr = (ticks[1] - self.last[1]) * self.m_per_tick
        self.last = ticks
        ds, dtheta = (dl + dr) / 2, (dr - dl) / self.separation
        mid = self.theta + dtheta / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.theta += dtheta


def run_segments(sim: DiffDriveSim, odom: OdometryPose, segments: list[tuple[str, float]]) -> list[list[int]]:
    """Drive ('line', meters) and ('turn', radians) segments closed-loop on odometry; log ticks."""
    log = [list(sim.ticks)]
    step = 0

    def tick() -> None:
        nonlocal step
        sim.step(DT)
        odom.update(sim.ticks)
        step += 1
        if step % LOG_EVERY == 0:
            log.append(list(sim.ticks))

    for kind, amount in segments:
        start = (odom.x, odom.y, odom.theta)
        while True:
            if kind == "line":
                remaining = amount - math.hypot(odom.x - start[0], odom.y - start[1])
                if remaining <= 0.001:
                    break
                w = max(1.0, min(6.0, 25.0 * remaining))
                sim.set_velocity(w, w)
            else:
                remaining = amount - (odom.theta - start[2])
                if abs(remaining) <= math.radians(0.2):
                    break
                w = math.copysign(max(0.6, min(4.0, 12.0 * abs(remaining))), remaining)
                sim.set_velocity(-w, w)
            tick()
        sim.set_velocity(0.0, 0.0)
        for _ in range(15):  # 0.3 s to come to rest
            tick()
    return log


def main() -> None:
    cfg = load_config()
    d = cfg.drive
    params = dataclasses.replace(DiffDriveParams.ideal(cfg), slip_std=0.01, **TRUE_SCALES)
    rng = np.random.default_rng(905)
    runs = []
    plans: list[tuple[str, list[tuple[str, float]]]] = []
    for direction, sign in (("cw", -1.0), ("ccw", 1.0)):
        for _ in range(5):
            plans.append((direction, [("line", SQUARE_SIDE_M), ("turn", sign * math.pi / 2)] * 4))
    for _ in range(3):
        plans.append(("straight", [("line", 2.5)]))
    for sign in (1.0, -1.0, 1.0):
        plans.append(("spin", [("turn", sign * 4 * math.pi)]))

    for i, (kind, segments) in enumerate(plans):
        sim = DiffDriveSim(World(), params, SensorParams.ideal(cfg), seed=1000 + i)
        odom = OdometryPose(d.wheel_radius_m, d.wheel_separation_m, d.ticks_per_wheel_rev)
        ticks = run_segments(sim, odom, segments)
        truth = sim.pose
        # the true heading change of a spin is ~2 turns: take the whole turns from odometry and the
        # fraction from the (wrapped) truth; odometry is wrong by far less than half a turn
        turns = round((odom.theta - truth.theta) / (2 * math.pi))
        unwrapped = truth.theta + 2 * math.pi * turns
        run = {
            "name": f"{kind}-{sum(r['kind'] == kind for r in runs) + 1}",
            "kind": kind,
            "ticks": ticks,
            "measured_pose": [
                round(truth.x + rng.normal(0, 0.003), 4),
                round(truth.y + rng.normal(0, 0.003), 4),
                round(math.remainder(truth.theta + rng.normal(0, math.radians(0.3)), 2 * math.pi), 5),
            ],
        }
        if kind == "straight":
            run["measured_distance_m"] = round(math.hypot(truth.x, truth.y) + rng.normal(0, 0.003), 4)
        if kind == "spin":
            run["measured_heading_change_rad"] = round(unwrapped + rng.normal(0, math.radians(0.3)), 5)
        runs.append(run)

    data = {
        "description": "karmel-like robot, encoder counts reset at the start of every run; see make_logs.py",
        "ticks_per_rev": d.ticks_per_wheel_rev,
        "nominal": {"wheel_radius_m": d.wheel_radius_m, "wheel_separation_m": d.wheel_separation_m},
        "square_side_m": SQUARE_SIDE_M,
        "sample_rate_hz": round(1 / (DT * LOG_EVERY)),
        "runs": runs,
    }
    OUTPUT.write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")
    print(f"wrote {OUTPUT} ({OUTPUT.stat().st_size / 1000:.0f} kB, {len(runs)} runs)")


if __name__ == "__main__":
    main()
