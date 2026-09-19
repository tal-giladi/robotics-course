"""Angles and 2D rigid-body transforms (SE(2)).

Conventions: radians, angles wrapped to (-pi, pi], REP-103 frames (x forward, y left).
``SE2(x, y, theta)`` is the pose of a child frame expressed in a parent frame, so
``world_T_robot @ robot_T_sensor == world_T_sensor`` and ``pose.apply(p_child) == p_parent``.
"""

from __future__ import annotations

import math
from collections.abc import Iterator
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

TWO_PI = 2.0 * math.pi


def _scalar_or_array(values: NDArray[np.floating]) -> Any:
    return float(values) if np.ndim(values) == 0 else values


def wrap_angle(angle: float | ArrayLike) -> Any:
    """Wrap an angle (float -> float) or an array of angles (array -> array) to (-pi, pi]."""
    return _scalar_or_array(math.pi - np.mod(math.pi - np.asarray(angle, dtype=float), TWO_PI))


def angle_diff(a: float | ArrayLike, b: float | ArrayLike) -> Any:
    """Smallest signed rotation that takes heading ``b`` to heading ``a``: ``wrap_angle(a - b)``."""
    return wrap_angle(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))


def deg2rad(degrees: float | ArrayLike) -> Any:
    """Degrees to radians (not wrapped)."""
    return _scalar_or_array(np.deg2rad(np.asarray(degrees, dtype=float)))


def rad2deg(radians: float | ArrayLike) -> Any:
    """Radians to degrees (not wrapped)."""
    return _scalar_or_array(np.rad2deg(np.asarray(radians, dtype=float)))


def rotation_matrix(theta: float) -> NDArray[np.floating]:
    """2x2 matrix of a counter-clockwise rotation by ``theta``."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


@dataclass(frozen=True)
class SE2:
    """A 2D pose / rigid transform: translation ``(x, y)`` and heading ``theta``.

    >>> a = SE2(1.0, 0.0, math.pi / 2)
    >>> a @ SE2(1.0, 0.0, 0.0)          # 1 m forward from a
    SE2(x=1.0, y=1.0, theta=1.5707963267948966)
    >>> x, y, theta = a                 # unpacks like a tuple
    """

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", float(self.x))
        object.__setattr__(self, "y", float(self.y))
        object.__setattr__(self, "theta", wrap_angle(float(self.theta)))

    # --- construction ------------------------------------------------------------------------
    @classmethod
    def identity(cls) -> SE2:
        return cls(0.0, 0.0, 0.0)

    @classmethod
    def from_tuple(cls, pose: SE2 | ArrayLike) -> SE2:
        """Accept an ``SE2`` or any ``(x, y, theta)`` sequence."""
        if isinstance(pose, SE2):
            return pose
        x, y, theta = np.asarray(pose, dtype=float).reshape(3)
        return cls(x, y, theta)

    @classmethod
    def from_matrix(cls, matrix: ArrayLike) -> SE2:
        """Build from a 3x3 homogeneous transform."""
        m = np.asarray(matrix, dtype=float)
        if m.shape != (3, 3):
            raise ValueError(f"expected a 3x3 matrix, got shape {m.shape}")
        return cls(m[0, 2], m[1, 2], math.atan2(m[1, 0], m[0, 0]))

    # --- conversion --------------------------------------------------------------------------
    def as_matrix(self) -> NDArray[np.floating]:
        """3x3 homogeneous transform ``[[R, t], [0, 0, 1]]``."""
        m = np.eye(3)
        m[:2, :2] = rotation_matrix(self.theta)
        m[:2, 2] = (self.x, self.y)
        return m

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    def as_array(self) -> NDArray[np.floating]:
        return np.array([self.x, self.y, self.theta])

    @property
    def translation(self) -> NDArray[np.floating]:
        return np.array([self.x, self.y])

    def __iter__(self) -> Iterator[float]:
        return iter(self.as_tuple())

    # --- algebra -----------------------------------------------------------------------------
    def compose(self, other: SE2) -> SE2:
        """``self @ other``: the pose ``other`` (expressed in this frame) in the parent frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return SE2(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            self.theta + other.theta,
        )

    def __matmul__(self, other: SE2) -> SE2:
        if not isinstance(other, SE2):
            return NotImplemented
        return self.compose(other)

    def inverse(self) -> SE2:
        """The transform that undoes this one: ``pose @ pose.inverse() == SE2.identity()``."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return SE2(-c * self.x - s * self.y, s * self.x - c * self.y, -self.theta)

    def between(self, other: SE2) -> SE2:
        """Pose of ``other`` as seen from ``self``: ``self.inverse() @ other``."""
        return self.inverse() @ other

    def apply(self, points: ArrayLike) -> NDArray[np.floating]:
        """Transform points from this frame into the parent frame (vectorized).

        ``points`` has shape ``(2,)`` or ``(N, 2)``; the result has the same shape.
        """
        p = np.asarray(points, dtype=float)
        if p.shape[-1:] != (2,):
            raise ValueError(f"points must have shape (2,) or (N, 2), got {p.shape}")
        return p @ rotation_matrix(self.theta).T + self.translation

    def distance_to(self, other: SE2) -> float:
        """Euclidean distance between the two positions."""
        return math.hypot(other.x - self.x, other.y - self.y)
