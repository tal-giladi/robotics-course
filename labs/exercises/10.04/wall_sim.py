"""Provided (not an exercise): karmel drives toward a wall and records encoders + front ToF.

The room is ``[0, 4] x [0, 3]`` m. The robot starts at ``(0.5, 1.5)`` facing the wall at x = 4 and
drives toward it with a speed profile (accelerate, cruise, slow down, stop). Every ``dt`` it logs:

* ``controls[k]``     = -(distance travelled since the last step, from the wheel encoders): the
  predicted change of the distance to the wall,
* ``measurements[k]`` = the front ToF reading (m), or ``None`` when it has no valid reading,
* ``truth[k]``        = the true perpendicular distance from the ToF sensor to the wall.

The 1D state is "distance from the sensor to the wall": prediction subtracts what odometry says we
drove, the ToF measures it directly (H = 1).
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from robotlab.config import KarmelConfig, load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

ROOM_W, ROOM_H = 4.0, 3.0
START = (0.5, 1.5, 0.0)


@dataclass(frozen=True)
class WallRun:
    t: NDArray[np.floating]
    controls: list[float]
    measurements: list[float | None]
    truth: NDArray[np.floating]
    velocity: NDArray[np.floating]  # true forward speed (m/s), for 10.05
    wheel_speed: NDArray[np.floating]  # encoder speed estimate (m/s), mean of both wheels


def speed_profile(t: float) -> float:
    """Forward speed command (m/s): ramp up, cruise, slow, cruise slowly, stop."""
    if t < 2.0:
        return 0.15 * t
    if t < 6.0:
        return 0.30
    if t < 8.0:
        return 0.30 - 0.1 * (t - 6.0)
    if t < 10.5:
        return 0.10
    return 0.0


def drive_to_wall(
    seed: int,
    realistic: bool = True,
    range_noise_std_m: float | None = None,
    dt: float = 0.05,
    duration_s: float = 12.0,
    config: KarmelConfig | None = None,
) -> WallRun:
    cfg = config or load_config()
    params = DiffDriveParams.realistic(cfg) if realistic else DiffDriveParams.ideal(cfg)
    sensors = SensorParams.realistic(cfg) if realistic else SensorParams.ideal(cfg)
    if range_noise_std_m is not None:
        sensors = dataclasses.replace(sensors, range_noise_std_m=range_noise_std_m)
    sim = DiffDriveSim(World.rectangle_room(ROOM_W, ROOM_H), params, sensors, pose=START, seed=seed, config=cfg)
    base = SimBase(sim, dt=dt)
    r, m_per_tick = cfg.drive.wheel_radius_m, cfg.drive.meters_per_tick
    state = base.read()
    last = (state.left_ticks, state.right_ticks)
    ts, controls, measurements, truth, velocity, wheel_speed = [], [], [], [], [], []
    for _ in range(round(duration_s / dt)):
        w = speed_profile(sim.t) / r
        base.set_wheel_velocity(w, w)
        state = base.read()
        ds = ((state.left_ticks - last[0]) + (state.right_ticks - last[1])) / 2.0 * m_per_tick
        last = (state.left_ticks, state.right_ticks)
        sensor_x = sim.pose.x + sensors.range_x_m * np.cos(sim.pose.theta)
        ts.append(state.t)
        controls.append(-ds)
        measurements.append(state.range_m)
        truth.append(ROOM_W - sensor_x)
        velocity.append(float(np.mean(sim.wheel_rad_s * [params.true_wheel_radius_left_m, params.true_wheel_radius_right_m])))
        wheel_speed.append((state.left_rad_s + state.right_rad_s) / 2.0 * r)
    return WallRun(np.array(ts), controls, measurements, np.array(truth), np.array(velocity), np.array(wheel_speed))
