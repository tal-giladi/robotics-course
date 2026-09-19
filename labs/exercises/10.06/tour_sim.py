"""Provided helper for 10.06-10.10: karmel tours the simulated apartment and logs every sensor.

    from tour_sim import drive_tour
    log = drive_tour(seed=1, observe_every=10, scan_every=5)

The "driver" steers toward waypoints using the TRUE pose, like a person with a joystick; the
estimators you write only see ``ticks``, ``gyro``, ``landmarks`` and ``scans``. ``truth`` is there
to grade them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from robotlab.config import KarmelConfig, load_config
from robotlab.geometry import SE2, wrap_angle
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World

START = (1.0, 1.3, 0.0)  # living room, facing +x (World.apartment's suggested start)

# living room -> kitchen -> back -> study -> back to the sofa (waypoints in the map frame, meters).
TOUR = [(2.9, 1.65), (4.0, 1.9), (4.9, 2.25), (4.1, 2.0), (2.9, 1.65), (2.05, 2.6), (2.05, 4.0),
        (2.05, 2.6), (1.0, 1.9), (1.0, 1.3)]


@dataclass(frozen=True)
class TourLog:
    t: NDArray[np.floating]      # (T,) seconds
    truth: NDArray[np.floating]  # (T, 3) true poses
    ticks: NDArray[np.int64]     # (T, 2) encoder counts
    gyro: NDArray[np.floating]   # (T,) gyro z sample at each step (rad/s); gyro[0] = nan
    landmarks: list[tuple[int, list]]  # (step index, [LandmarkObservation])
    scans: list[tuple[int, object]]    # (step index, LaserScan) — empty unless scan_every is set
    world: World
    dt: float


def drive_tour(
    seed: int = 1,
    dt: float = 0.02,
    waypoints: list[tuple[float, float]] | None = None,
    start: tuple[float, float, float] = START,
    speed: float = 0.25,
    observe_every: int = 10,
    scan_every: int | None = None,
    realistic: bool = True,
    kidnap_at_s: float | None = None,
    kidnap_to: tuple[float, float, float] = (4.6, 2.2, math.pi),
    config: KarmelConfig | None = None,
) -> TourLog:
    """Drive through ``waypoints`` (default TOUR) in the apartment and log every sensor.

    ``observe_every`` / ``scan_every``: take a landmark observation / LiDAR scan every that many
    steps (dt = 0.02 s: 10 -> 5 Hz, 5 -> 10 Hz). ``kidnap_at_s`` teleports the robot (the encoders
    notice nothing) and the driver continues to the remaining waypoints from there.
    """
    cfg = config or load_config()
    world = World.apartment()
    params = (DiffDriveParams.realistic if realistic else DiffDriveParams.ideal)(cfg)
    sensors = (SensorParams.realistic if realistic else SensorParams.ideal)(cfg)
    sim = DiffDriveSim(world, params, sensors, pose=start, seed=seed, config=cfg)
    r, b = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    route = list(TOUR if waypoints is None else waypoints)
    t, truth, ticks, gyro, landmarks, scans = [0.0], [sim.pose.as_tuple()], [sim.ticks], [math.nan], [], []
    kidnapped, i, k = False, 0, 0
    while i < len(route) and k < 20_000:
        if kidnap_at_s is not None and not kidnapped and sim.t >= kidnap_at_s - 1e-9:
            sim.pose = SE2.from_tuple(kidnap_to)
            kidnapped = True
        x, y, th = sim.pose.as_tuple()
        gx, gy = route[i]
        dist = math.hypot(gx - x, gy - y)
        if dist < 0.12:
            i += 1
            continue
        err = wrap_angle(math.atan2(gy - y, gx - x) - th)
        v = 0.0 if abs(err) > 0.6 else speed * min(1.0, dist / 0.3 + 0.3)
        w = max(-1.5, min(1.5, 2.5 * err))
        sim.set_velocity((v - w * b / 2.0) / r, (v + w * b / 2.0) / r)
        sim.step(dt)
        k += 1
        t.append(sim.t)
        truth.append(sim.pose.as_tuple())
        ticks.append(sim.ticks)
        gyro.append(sim.gyro_z())
        if k % observe_every == 0:
            landmarks.append((k, sim.observe_landmarks()))
        if scan_every and k % scan_every == 0:
            scans.append((k, sim.lidar_scan()))
    return TourLog(np.array(t), np.array(truth), np.array(ticks, dtype=np.int64), np.array(gyro),
                   landmarks, scans, world, dt)


def wheel_travels(ticks: NDArray[np.int64], config: KarmelConfig | None = None) -> NDArray[np.floating]:
    """(T-1, 2) left/right wheel travel per step in meters, from a tick log (nominal radius)."""
    cfg = config or load_config()
    return np.diff(ticks, axis=0) * cfg.drive.meters_per_tick
