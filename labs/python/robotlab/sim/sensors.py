"""Sensor models for the simulator: 2D LiDAR, front ToF range, landmark range-bearing, gyro.

Each model is a plain function of (world, true pose, parameters, random generator) so a lesson
can read one in isolation. :class:`robotlab.sim.DiffDriveSim` wires them to the robot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from robotlab.geometry import SE2, wrap_angle
from robotlab.sim.world import World


@dataclass(frozen=True)
class LaserScan:
    """One 2D LiDAR sweep, with ``sensor_msgs/LaserScan`` semantics (REP-117).

    ``ranges[i]`` is measured along ``angle_min + i * angle_increment`` in the sensor frame
    (x forward, counter-clockwise positive). Special values:

    * ``+inf`` — no return within ``range_max``
    * ``-inf`` — object closer than ``range_min``
    * ``nan``  — dropout / invalid measurement
    """

    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: NDArray[np.floating]
    stamp: float = 0.0

    @property
    def angle_max(self) -> float:
        return self.angle_min + (len(self.ranges) - 1) * self.angle_increment

    @property
    def angles(self) -> NDArray[np.floating]:
        """Beam angles in the sensor frame, shape ``(N,)``."""
        return self.angle_min + np.arange(len(self.ranges)) * self.angle_increment

    @property
    def valid(self) -> NDArray[np.bool_]:
        """Mask of finite ranges within ``[range_min, range_max]``."""
        r = self.ranges
        with np.errstate(invalid="ignore"):
            return np.isfinite(r) & (r >= self.range_min) & (r <= self.range_max)

    def points(self) -> NDArray[np.floating]:
        """Valid returns as ``(K, 2)`` points in the sensor frame."""
        mask = self.valid
        a, r = self.angles[mask], self.ranges[mask]
        return np.column_stack([r * np.cos(a), r * np.sin(a)])


@dataclass(frozen=True)
class LandmarkObservation:
    """Range and bearing to a landmark, measured from base_link (bearing CCW from +x)."""

    id: int
    range_m: float
    bearing_rad: float


def lidar_scan(
    world: World,
    sensor_pose: SE2,
    samples: int,
    range_min: float,
    range_max: float,
    noise_std: float,
    dropout_prob: float,
    rng: np.random.Generator,
    stamp: float = 0.0,
) -> LaserScan:
    """Cast ``samples`` beams over 360 degrees from ``sensor_pose`` (world frame)."""
    increment = 2.0 * math.pi / samples
    angles = -math.pi + np.arange(samples) * increment
    ranges = world.raycast((sensor_pose.x, sensor_pose.y), angles + sensor_pose.theta, np.inf)
    if noise_std > 0.0:
        ranges = ranges + rng.normal(0.0, noise_std, samples)
    ranges[ranges > range_max] = np.inf
    ranges[ranges < range_min] = -np.inf
    if dropout_prob > 0.0:
        ranges[rng.random(samples) < dropout_prob] = np.nan
    return LaserScan(-math.pi, increment, range_min, range_max, ranges, stamp)


def cone_range(
    world: World,
    sensor_pose: SE2,
    fov_rad: float,
    rays: int,
    max_range: float,
    noise_std: float,
    rng: np.random.Generator,
) -> float | None:
    """A ToF/ultrasonic reading: nearest hit among ``rays`` spread over the cone, plus noise.

    Returns ``None`` when nothing is within ``max_range`` (the sensor has no valid reading).
    """
    offsets = np.linspace(-fov_rad / 2.0, fov_rad / 2.0, rays) if rays > 1 else np.zeros(1)
    nearest = float(world.raycast((sensor_pose.x, sensor_pose.y), offsets + sensor_pose.theta).min())
    if noise_std > 0.0 and math.isfinite(nearest):
        nearest += float(rng.normal(0.0, noise_std))
    if not math.isfinite(nearest) or nearest > max_range:
        return None
    return max(nearest, 0.0)


def observe_landmarks(
    world: World,
    pose: SE2,
    max_range: float,
    fov_rad: float,
    range_std: float,
    bearing_std: float,
    rng: np.random.Generator,
    occlusion_tolerance_m: float = 0.05,
) -> list[LandmarkObservation]:
    """Landmarks inside the field of view, within range and not hidden behind an obstacle."""
    if len(world.landmarks) == 0:
        return []
    delta = world.landmarks - (pose.x, pose.y)
    ranges = np.hypot(delta[:, 0], delta[:, 1])
    world_angles = np.arctan2(delta[:, 1], delta[:, 0])
    bearings = wrap_angle(world_angles - pose.theta)
    free_path = world.raycast((pose.x, pose.y), world_angles)
    visible = (ranges <= max_range) & (np.abs(bearings) <= fov_rad / 2.0)
    visible &= free_path >= ranges - occlusion_tolerance_m
    idx = np.flatnonzero(visible)
    noisy_r = ranges[idx] + rng.normal(0.0, range_std, idx.size) if range_std > 0 else ranges[idx]
    noisy_b = bearings[idx] + rng.normal(0.0, bearing_std, idx.size) if bearing_std > 0 else bearings[idx]
    return [
        LandmarkObservation(int(world.landmark_ids[i]), float(r), float(wrap_angle(b)))
        for i, r, b in zip(idx, noisy_r, noisy_b)
    ]


def gyro_z(true_rate: float, bias: float, noise_std: float, rng: np.random.Generator) -> float:
    """Yaw-rate gyro: truth + constant bias + white noise."""
    noise = float(rng.normal(0.0, noise_std)) if noise_std > 0.0 else 0.0
    return true_rate + bias + noise
