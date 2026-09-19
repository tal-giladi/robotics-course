"""sensor_msgs/Imu conventions without ROS: quaternions, covariance, mounting remap, sanity checks.

Pure numpy so it runs anywhere (`py -m pytest 07-sensors/code/ros2`). The ROS nodes in this folder
import these functions, so the maths you test here is the maths that runs on the robot.

Conventions: REP-103 (x forward, y left, z up; right-handed; SI units) and REP-145 (IMU driver
conventions: m/s^2, rad/s, tesla; covariance element 0 = -1 means "this field is not reported").

Lessons: 07.06 (the IMU in ROS 2), 07.10 (sensor data done right), 07.11 (troubleshooting).
"""

from __future__ import annotations

import math

import numpy as np

__all__ = [
    "G",
    "quaternion_from_rpy",
    "rpy_from_quaternion",
    "diag_covariance",
    "NOT_REPORTED",
    "is_not_reported",
    "variance_from_sigma",
    "variance_from_noise_density",
    "axis_remap_matrix",
    "remap_vector",
    "gravity_check",
    "tilt_from_accel",
]

G = 9.80665
"""Standard gravity, m/s^2 (ISO 80000-3). A resting accelerometer reads +G on its up axis."""

NOT_REPORTED = -1.0
"""Value REP-145 puts in covariance[0] to say "this field is not reported -- ignore it"."""


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) for the ROS convention: intrinsic Z-Y-X, i.e. yaw then pitch then roll.

    Returned in the order geometry_msgs/Quaternion stores them (x, y, z, w) -- not (w, x, y, z),
    which is what most papers and most numpy code use. Getting this backwards is the single most
    common IMU bug in ROS.
    """
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Roll, pitch, yaw (rad) from a (x, y, z, w) quaternion. Pitch is clamped at +/-90 deg."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def diag_covariance(sigma_x: float, sigma_y: float | None = None,
                    sigma_z: float | None = None) -> list[float]:
    """A row-major 3x3 covariance as the flat list sensor_msgs/Imu wants, from standard deviations.

    Covariance holds *variances* (sigma squared), in the message's own units squared: (rad/s)^2 for
    angular_velocity_covariance, (m/s^2)^2 for linear_acceleration_covariance, rad^2 for
    orientation_covariance. Pass one sigma for an isotropic sensor, three for a datasheet that
    differs per axis.
    """
    sy = sigma_x if sigma_y is None else sigma_y
    sz = sigma_x if sigma_z is None else sigma_z
    return [sigma_x**2, 0.0, 0.0, 0.0, sy**2, 0.0, 0.0, 0.0, sz**2]


def is_not_reported(cov: list[float] | np.ndarray) -> bool:
    """True when the field carries REP-145's "not reported" marker in element 0."""
    return float(np.asarray(cov).reshape(-1)[0]) == NOT_REPORTED


def variance_from_sigma(sigma: float) -> float:
    """sigma -> variance. Trivial, and named because people put sigma in the message instead."""
    return float(sigma) ** 2


def variance_from_noise_density(density: float, rate_hz: float) -> float:
    """Per-sample variance from a datasheet noise density and the output rate.

    Datasheets quote gyro noise as an *angular random walk* / rate noise density in
    units/sqrt(Hz) -- e.g. 0.014 (deg/s)/sqrt(Hz). The variance of one sample at an output
    bandwidth of `rate_hz / 2` is density^2 * (rate_hz / 2). Use that for the covariance, not the
    density itself, which is not even in the right units.
    """
    return float(density) ** 2 * (float(rate_hz) / 2.0)


# Bosch's P0..P7 style axis remaps, and the general case: a signed permutation matrix.
_AXIS = {"x": 0, "y": 1, "z": 2}


def axis_remap_matrix(spec: str) -> np.ndarray:
    """A 3x3 signed permutation matrix from a spec like "x y z", "y -x z" or "-y x z".

    The spec says: sensor axis i, expressed in the robot frame. `"y -x z"` means "the sensor's x
    axis points along the robot's +y, and the sensor's y axis points along the robot's -x" -- what
    you get when you bolt the breakout down rotated 90 degrees. Use this only for the rotations
    that are exact multiples of 90 degrees; anything else belongs in the URDF as a real rotation.
    """
    parts = spec.split()
    if len(parts) != 3:
        raise ValueError(f"axis remap needs three axes, got {spec!r}")
    m = np.zeros((3, 3))
    used = set()
    for row, token in enumerate(parts):
        sign = -1.0 if token.startswith("-") else 1.0
        name = token.lstrip("+-").lower()
        if name not in _AXIS:
            raise ValueError(f"bad axis {token!r} in {spec!r}")
        if name in used:
            raise ValueError(f"axis {name!r} used twice in {spec!r}")
        used.add(name)
        m[row, _AXIS[name]] = sign
    return m


def remap_vector(v: np.ndarray, spec: str) -> np.ndarray:
    """Apply `axis_remap_matrix(spec)` to a 3-vector (or an (N, 3) array of them)."""
    m = axis_remap_matrix(spec)
    v = np.asarray(v, dtype=float)
    return v @ m.T if v.ndim > 1 else m @ v


def gravity_check(accel: np.ndarray, tolerance: float = 0.5) -> tuple[bool, float]:
    """Is |a| within `tolerance` m/s^2 of standard gravity? Returns (ok, magnitude).

    The 10-second test for any new IMU: put it down, read the magnitude. If it is not near 9.81 the
    driver has the units wrong (g instead of m/s^2 gives 1.0), the scale is wrong, or the sensor is
    not what you think it is. Run it before you debug anything else.
    """
    mag = float(np.linalg.norm(np.asarray(accel, dtype=float)))
    return abs(mag - G) <= tolerance, mag


def tilt_from_accel(accel: np.ndarray) -> tuple[float, float]:
    """Roll and pitch (rad) from a gravity vector, valid only while the robot is not accelerating."""
    ax, ay, az = (float(c) for c in np.asarray(accel, dtype=float))
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    return roll, pitch
