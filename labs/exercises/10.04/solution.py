"""Reference solution for 10.04 — The Kalman filter in 1D.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray


def kalman_gain(P_pred: float, R: float) -> float:
    """K = P_pred / (P_pred + R): how much of the innovation to believe (0 = none, 1 = all)."""
    if P_pred < 0.0 or R < 0.0 or P_pred + R <= 0.0:
        raise ValueError("variances must be non-negative and not both zero")
    return P_pred / (P_pred + R)


def steady_state_gain(Q: float, R: float) -> float:
    """The gain a constant-Q, constant-R random-walk filter converges to.

    P_pred = (Q + sqrt(Q^2 + 4 Q R)) / 2 solves P_pred = P_pred R / (P_pred + R) + Q.
    """
    P_pred = (Q + math.sqrt(Q * Q + 4.0 * Q * R)) / 2.0
    return kalman_gain(P_pred, R)


class KalmanFilter1D:
    """A scalar Kalman filter for x_k = x_{k-1} + u_k + w (var Q),  z_k = x_k + v (var R)."""

    def __init__(self, x0: float, P0: float) -> None:
        if P0 < 0.0:
            raise ValueError("P0 is a variance and cannot be negative")
        self.x = float(x0)
        self.P = float(P0)
        self.K: float | None = None   # last gain
        self.nu: float | None = None  # last innovation z - x_pred
        self.S: float | None = None   # last innovation variance P_pred + R

    def predict(self, u: float = 0.0, Q: float = 0.0) -> tuple[float, float]:
        if Q < 0.0:
            raise ValueError("Q is a variance and cannot be negative")
        self.x = self.x + u
        self.P = self.P + Q
        return self.x, self.P

    def update(self, z: float, R: float) -> tuple[float, float]:
        if R <= 0.0:
            raise ValueError("R must be positive (a perfect sensor is a different problem)")
        self.nu = z - self.x
        self.S = self.P + R
        self.K = self.P / self.S
        self.x = self.x + self.K * self.nu
        self.P = (1.0 - self.K) * self.P
        return self.x, self.P


def run_kf_1d(
    x0: float,
    P0: float,
    controls: Sequence[float],
    measurements: Sequence[float | None],
    Q: float,
    R: float,
) -> tuple[NDArray[np.floating], NDArray[np.floating]]:
    """Predict with ``controls[k]``, update with ``measurements[k]`` unless it is ``None``."""
    if len(controls) != len(measurements):
        raise ValueError("controls and measurements must have the same length")
    kf = KalmanFilter1D(x0, P0)
    xs, Ps = [], []
    for u, z in zip(controls, measurements):
        kf.predict(u, Q)
        if z is not None:
            kf.update(z, R)
        xs.append(kf.x)
        Ps.append(kf.P)
    return np.array(xs), np.array(Ps)
