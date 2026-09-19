"""Shared helpers for the module-11 scripts: drive a tour of the apartment and log what a robot sees.

The tour is driven by a scripted pilot that steers with the *true* pose (think of it as a human
with a joystick). The mapping code never sees that pose unless a script says "known poses"
(11.02). Everything else only gets what karmel would have: encoder ticks and LiDAR scans.

    from slam_common import record_tour
    log = record_tour(realistic=True, seed=3)
    log.truth[k], log.odom[k], log.scans[k], log.ticks[k]   # one entry per 10 Hz scan
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
OUT = HERE / "out"
if str(ROOT / "labs" / "python") not in sys.path:  # robotlab without `pip install -e labs/python`
    sys.path.insert(0, str(ROOT / "labs" / "python"))

from robotlab.config import load_config  # noqa: E402
from robotlab.geometry import SE2, angle_diff  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, LaserScan, SensorParams, World, arc_update  # noqa: E402

START = SE2(1.0, 1.3, 0.0)

# A loop through all four rooms of World.apartment(), counter-clockwise around the central wall,
# ending where it started (so there is a loop to close in 11.05).
APARTMENT_TOUR: list[tuple[float, float]] = [
    (2.0, 1.65), (3.0, 1.65), (3.9, 1.65),  # living room -> kitchen doorway (x = 3.5, y 1.2-2.1)
    (4.3, 2.25), (4.9, 2.3),  # kitchen, above the table
    (4.9, 3.2),  # kitchen -> bedroom doorway (y = 2.8, x 4.5-5.3)
    (3.95, 3.25), (3.95, 4.0),  # bedroom, left of the bed
    (3.0, 4.0),  # bedroom -> study doorway (x = 3.5, y 3.6-4.4)
    (2.05, 3.75),  # study
    (2.05, 2.7),  # study -> living room doorway (y = 3.2, x 1.6-2.5)
    (1.3, 1.6), (1.0, 1.3),  # back to the start
]


@dataclass
class TourLog:
    """What happened on one tour, one entry per LiDAR scan (10 Hz)."""

    world: World
    truth: list[SE2] = field(default_factory=list)  # ground-truth pose of base_link
    odom: list[SE2] = field(default_factory=list)  # wheel odometry with nominal karmel.yaml values
    scans: list[LaserScan] = field(default_factory=list)  # in the LiDAR frame
    ticks: list[tuple[int, int]] = field(default_factory=list)  # cumulative encoder counts (left, right)
    lidar_offset: SE2 = SE2()  # base_link -> LiDAR (karmel: identity)

    def keyframes(self, min_distance: float = 0.3, min_heading: float = 0.35) -> list[int]:
        """Indices of scans kept as graph nodes: a new one after the robot (by odometry) moved
        ``min_distance`` m or turned ``min_heading`` rad. Same idea as slam_toolbox's
        ``minimum_travel_distance`` / ``minimum_travel_heading``."""
        keep = [0]
        for k in range(1, len(self.odom)):
            last = self.odom[keep[-1]]
            if last.distance_to(self.odom[k]) >= min_distance or abs(angle_diff(self.odom[k].theta, last.theta)) >= min_heading:
                keep.append(k)
        if keep[-1] != len(self.odom) - 1:
            keep.append(len(self.odom) - 1)
        return keep


def wheel_speeds(v: float, omega: float, radius: float, separation: float) -> tuple[float, float]:
    """Body velocity (m/s, rad/s) -> wheel speeds (rad/s)."""
    return (v - omega * separation / 2.0) / radius, (v + omega * separation / 2.0) / radius


def record_tour(
    realistic: bool = True,
    seed: int = 0,
    waypoints: list[tuple[float, float]] | None = None,
    start: SE2 = START,
    world: World | None = None,
    speed: float = 0.25,
    dt: float = 0.02,
    scan_every: int = 5,
    max_time: float = 300.0,
) -> TourLog:
    """Drive through ``waypoints`` and log truth, wheel odometry and a scan every ``scan_every`` steps."""
    cfg = load_config()
    world = world if world is not None else World.apartment()
    params = DiffDriveParams.realistic(cfg) if realistic else DiffDriveParams.ideal(cfg)
    sensors = SensorParams.realistic(cfg) if realistic else SensorParams.ideal(cfg)
    sim = DiffDriveSim(world, params, sensors, pose=start, seed=seed)
    r, b = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    rad_per_tick = 2.0 * math.pi / cfg.drive.ticks_per_wheel_rev
    log = TourLog(world, lidar_offset=SE2(sensors.lidar_x_m, sensors.lidar_y_m, 0.0))

    odom = start
    prev_ticks = sim.ticks
    goals = list(waypoints if waypoints is not None else APARTMENT_TOUR)
    step = 0
    while goals and sim.t < max_time:
        gx, gy = goals[0]
        pose = sim.pose
        dist = math.hypot(gx - pose.x, gy - pose.y)
        if dist < 0.08:
            goals.pop(0)
            continue
        err = angle_diff(math.atan2(gy - pose.y, gx - pose.x), pose.theta)
        omega = max(-1.5, min(1.5, 2.5 * err))
        v = 0.0 if abs(err) > 0.6 else min(speed, 0.8 * dist + 0.05) * math.cos(err)
        sim.set_velocity(*wheel_speeds(v, omega, r, b))
        sim.step(dt)
        step += 1

        ticks = sim.ticks
        d_left = (ticks[0] - prev_ticks[0]) * rad_per_tick * r
        d_right = (ticks[1] - prev_ticks[1]) * rad_per_tick * r
        prev_ticks = ticks
        odom = arc_update(odom, d_left, d_right, b)

        if step % scan_every == 0:
            log.truth.append(sim.pose)
            log.odom.append(odom)
            log.scans.append(sim.lidar_scan())
            log.ticks.append(ticks)
    sim.stop()
    return log


def load_tour(realistic: bool = True, seed: int = 0) -> TourLog:
    """``record_tour`` with a cache in ``out/`` (the tour takes several seconds to simulate).
    Delete ``out/tour_*.pkl`` after changing the simulator or ``karmel.yaml``."""
    import pickle

    path = OUT / f"tour_{'realistic' if realistic else 'ideal'}_seed{seed}.pkl"
    if path.exists():
        with path.open("rb") as f:
            return pickle.load(f)
    log = record_tour(realistic=realistic, seed=seed)
    OUT.mkdir(exist_ok=True)
    with path.open("wb") as f:
        pickle.dump(log, f)
    return log


def trajectory_array(poses: list[SE2]) -> np.ndarray:
    """``(N, 3)`` array of ``(x, y, theta)``."""
    return np.array([p.as_tuple() for p in poses], dtype=float).reshape(-1, 3)


def position_rmse(estimate: list[SE2] | np.ndarray, truth: list[SE2] | np.ndarray) -> float:
    """Root-mean-square position error (the absolute trajectory error, without re-alignment:
    every run starts at the same known pose)."""
    e = trajectory_array(list(estimate)) if not isinstance(estimate, np.ndarray) else estimate
    t = trajectory_array(list(truth)) if not isinstance(truth, np.ndarray) else truth
    return float(np.sqrt(np.mean(np.sum((e[:, :2] - t[:, :2]) ** 2, axis=1))))


def headless_pyplot():
    """Import pyplot with the Agg backend so scripts run without a display."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


if __name__ == "__main__":
    import time

    t0 = time.perf_counter()
    log = record_tour(realistic=True, seed=0)
    n = len(log.scans)
    end_truth, end_odom = log.truth[-1], log.odom[-1]
    length = sum(log.truth[k].distance_to(log.truth[k + 1]) for k in range(n - 1))
    print(f"{n} scans, {n / 10:.1f} s simulated, path {length:.2f} m, {time.perf_counter() - t0:.2f} s wall")
    print(f"final truth {end_truth.x:.3f} {end_truth.y:.3f} {math.degrees(end_truth.theta):.1f} deg")
    print(f"final odom  {end_odom.x:.3f} {end_odom.y:.3f} {math.degrees(end_odom.theta):.1f} deg")
    print(f"odometry end error {end_truth.distance_to(end_odom):.3f} m, "
          f"{math.degrees(abs(angle_diff(end_odom.theta, end_truth.theta))):.1f} deg; "
          f"RMSE {position_rmse(log.odom, log.truth):.3f} m; keyframes {len(log.keyframes())}")
