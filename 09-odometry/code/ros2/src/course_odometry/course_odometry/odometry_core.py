"""Pure-Python core of the lesson-09.07 odometry node (no ROS imports: unit-testable anywhere).

    pose:        exact-arc integration of wheel joint angles (rad) -> (x, y, yaw) in the odom frame
    covariance:  first-order propagation of per-wheel noise, var = k * |wheel travel| (09.06),
                 so the uncertainty GROWS with distance instead of being a made-up constant
    twist:       body velocities from the same deltas, expressed in the child (base) frame
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation by ``yaw`` about +z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def planar_to_ros_covariance(cov3: np.ndarray, planar_variance: float = 1e-6) -> list[float]:
    """3x3 covariance of (x, y, yaw) -> the row-major 6x6 float64[36] of geometry_msgs.

    ROS order is (x, y, z, rot_x, rot_y, rot_z). z, roll and pitch get a tiny variance: a planar
    robot "knows" them, and a zero variance makes some filters divide by zero.
    """
    out = [0.0] * 36
    index = (0, 1, 5)  # where x, y, yaw live in the 6-vector
    for i, row in enumerate(index):
        for j, col in enumerate(index):
            out[row * 6 + col] = float(cov3[i, j])
    for k in (2, 3, 4):
        out[k * 6 + k] = planar_variance
    return out


@dataclass
class DiffDriveOdometry:
    """Wheel odometry with per-wheel radii and covariance growth."""

    wheel_radius_left: float
    wheel_radius_right: float
    wheel_separation: float
    k_left: float = 1e-5  # wheel travel variance per meter travelled (m^2/m = m)
    k_right: float = 1e-5
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    v: float = 0.0  # last body velocities (base frame)
    omega: float = 0.0
    _last_angles: tuple[float, float] | None = None
    _last_time: float | None = None

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self.x, self.y, self.yaw = x, y, wrap(yaw)
        self.covariance = np.zeros((3, 3))
        self._last_angles = None
        self._last_time = None

    def update(self, left_angle: float, right_angle: float, stamp_s: float) -> bool:
        """Feed cumulative wheel joint angles (rad) measured at ``stamp_s``. False on the first call."""
        if self._last_angles is None or self._last_time is None:
            self._last_angles, self._last_time = (left_angle, right_angle), stamp_s
            return False
        dt = stamp_s - self._last_time
        if dt <= 0.0:  # duplicate or out-of-order message: ignore it
            return False
        d_left = (left_angle - self._last_angles[0]) * self.wheel_radius_left
        d_right = (right_angle - self._last_angles[1]) * self.wheel_radius_right
        self._last_angles, self._last_time = (left_angle, right_angle), stamp_s

        ds = (d_left + d_right) / 2.0
        dyaw = (d_right - d_left) / self.wheel_separation
        self.covariance = self._propagate(ds, dyaw, d_left, d_right)
        if abs(dyaw) < 1e-9:
            self.x += ds * math.cos(self.yaw)
            self.y += ds * math.sin(self.yaw)
        else:
            radius = ds / dyaw
            self.x += radius * (math.sin(self.yaw + dyaw) - math.sin(self.yaw))
            self.y -= radius * (math.cos(self.yaw + dyaw) - math.cos(self.yaw))
        self.yaw = wrap(self.yaw + dyaw)
        self.v, self.omega = ds / dt, dyaw / dt
        return True

    def _propagate(self, ds: float, dyaw: float, d_left: float, d_right: float) -> np.ndarray:
        """cov' = F cov F^T + G Q G^T, linearized at the midpoint heading (lesson 09.06)."""
        b = self.wheel_separation
        mid = self.yaw + dyaw / 2.0
        c, s = math.cos(mid), math.sin(mid)
        F = np.array([[1.0, 0.0, -ds * s], [0.0, 1.0, ds * c], [0.0, 0.0, 1.0]])
        G = np.array([
            [c / 2 + ds * s / (2 * b), c / 2 - ds * s / (2 * b)],
            [s / 2 - ds * c / (2 * b), s / 2 + ds * c / (2 * b)],
            [-1.0 / b, 1.0 / b],
        ])
        Q = np.diag([self.k_left * abs(d_left), self.k_right * abs(d_right)])
        return F @ self.covariance @ F.T + G @ Q @ G.T


def pose_difference(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float]:
    """(position distance [m], |heading difference| [rad]) between two planar poses."""
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(wrap(a[2] - b[2]))
