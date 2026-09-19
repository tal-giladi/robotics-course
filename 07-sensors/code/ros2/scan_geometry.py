"""LaserScan geometry without ROS: angles, polar->Cartesian, filtering, sector minima.

Pure numpy, so it runs on any machine (`py -m pytest 07-sensors/code/ros2`). The ROS nodes in
this folder use exactly these functions, so what you test here is what runs on the robot.

Conventions (sensor_msgs/LaserScan, REP-103): angles are measured about +z (counter-clockwise),
angle 0 is +x = straight ahead, ranges are metres, and `ranges[i]` belongs to
`angle_min + i * angle_increment`.

Lessons: 07.07 (2D LiDAR), 07.10 (sensor data done right), 07.11 (troubleshooting).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

__all__ = [
    "ScanSpec",
    "scan_angles",
    "scan_to_points",
    "valid_mask",
    "sector_min",
    "gap_at_range",
    "range_for_gap",
    "scan_rate_smear",
]


@dataclass(frozen=True)
class ScanSpec:
    """The header-ish fields of a LaserScan, without ROS types.

    `n` is the number of beams; `angle_increment` is derived so that the beams cover
    `[angle_min, angle_max]` inclusive, which is what `ros2 topic echo` shows for a driver that
    fills `angle_max = angle_min + (n - 1) * angle_increment`.
    """

    n: int
    angle_min: float = -np.pi
    angle_max: float = np.pi
    range_min: float = 0.05
    range_max: float = 12.0
    scan_time: float = 0.1
    frame_id: str = "laser"
    wrap: bool = field(default=True)
    """True for a 360-degree scanner: the last beam does not repeat the first."""

    @property
    def angle_increment(self) -> float:
        span = self.angle_max - self.angle_min
        return span / self.n if self.wrap else span / (self.n - 1)

    @property
    def time_increment(self) -> float:
        return self.scan_time / self.n


def scan_angles(spec: ScanSpec) -> np.ndarray:
    """Beam angles in radians, one per range, in the sensor frame."""
    return spec.angle_min + np.arange(spec.n, dtype=float) * spec.angle_increment


def valid_mask(ranges: np.ndarray, spec: ScanSpec) -> np.ndarray:
    """True where a range is a real measurement.

    A LaserScan marks "no return" with a value outside [range_min, range_max] -- usually inf, but
    some drivers use 0.0. NaN means "beam fired, result invalid". All three must be dropped before
    you do arithmetic, or one inf turns a mean into inf and one 0.0 puts a phantom obstacle on top
    of the robot.
    """
    r = np.asarray(ranges, dtype=float)
    return np.isfinite(r) & (r >= spec.range_min) & (r <= spec.range_max)


def scan_to_points(ranges: np.ndarray, spec: ScanSpec) -> np.ndarray:
    """Valid beams as an (N, 2) array of (x, y) in the sensor frame, metres."""
    r = np.asarray(ranges, dtype=float)
    keep = valid_mask(r, spec)
    a = scan_angles(spec)[keep]
    r = r[keep]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def sector_min(ranges: np.ndarray, spec: ScanSpec, centre: float, width: float) -> float:
    """Smallest valid range in a sector of `width` radians centred on `centre` radians.

    This is the one-line obstacle check a reactive controller runs at 10 Hz. Returns `inf` when the
    sector has no valid return (all beams saw nothing), which is the honest answer -- never 0.0.
    """
    a = scan_angles(spec)
    # wrap the angular difference into [-pi, pi] so a sector across +/-pi still works
    d = np.arctan2(np.sin(a - centre), np.cos(a - centre))
    sector = np.abs(d) <= width / 2.0
    keep = sector & valid_mask(ranges, spec)
    return float(np.min(np.asarray(ranges, dtype=float)[keep])) if np.any(keep) else float("inf")


def gap_at_range(spec: ScanSpec, distance_m: float) -> float:
    """Arc length in metres between two neighbouring beams at `distance_m`.

    Angular resolution becomes a *spatial* resolution that gets worse with distance: this is why a
    360-beam LiDAR sees a chair leg at 1 m and misses the same leg at 5 m.
    """
    return float(distance_m * spec.angle_increment)


def range_for_gap(spec: ScanSpec, object_width_m: float, beams: int = 2) -> float:
    """Farthest distance at which an object of `object_width_m` still gets `beams` returns."""
    return float(object_width_m / (beams * spec.angle_increment))


def scan_rate_smear(scan_hz: float, speed_m_s: float, yaw_rate_rad_s: float = 0.0,
                    range_m: float = 2.0) -> tuple[float, float]:
    """How far the robot moves during one scan revolution, in metres and in metres of arc.

    Returns (translation_m, arc_m): the linear displacement over one revolution, and the sideways
    smear of a point at `range_m` caused by rotating during the sweep. A 2D LiDAR does not take a
    snapshot; it sweeps, and every beam is taken from a slightly different pose.
    """
    period = 1.0 / scan_hz
    return speed_m_s * period, abs(yaw_rate_rad_s) * period * range_m
