"""10.06 — The Extended Kalman Filter for a differential-drive robot (reference solution).

Notation: 10-localization/code/NOTATION.md. State x = [x, y, theta] in the map frame.
Control u = [d_left, d_right]: wheel travels (m) since the last predict, from the encoder ticks.
Measurement z = [r, phi]: range and bearing from base_link to a landmark at a known map position.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

Vector = NDArray[np.floating]
Matrix = NDArray[np.floating]


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.pi - (math.pi - a) % (2.0 * math.pi)


# --- motion model -------------------------------------------------------------------------------
def motion_model(x: ArrayLike, u: ArrayLike, b: float) -> tuple[Vector, Matrix, Matrix]:
    """Midpoint odometry model. Returns ``(x_pred, F, G)``.

    ds = (dL + dR)/2, dth = (dR - dL)/b, th_m = th + dth/2
    x' = x + ds cos th_m,  y' = y + ds sin th_m,  th' = wrap(th + dth)
    F = d f / d x (3x3),  G = d f / d u (3x2)
    """
    px, py, th = (float(v) for v in np.asarray(x, dtype=float).reshape(3))
    dl, dr = (float(v) for v in np.asarray(u, dtype=float).reshape(2))
    ds, dth = 0.5 * (dl + dr), (dr - dl) / b
    th_m = th + 0.5 * dth
    c, s = math.cos(th_m), math.sin(th_m)
    x_pred = np.array([px + ds * c, py + ds * s, wrap_angle(th + dth)])
    F = np.array([[1.0, 0.0, -ds * s],
                  [0.0, 1.0, ds * c],
                  [0.0, 0.0, 1.0]])
    # d th_m / d dL = -1/(2b), d th_m / d dR = +1/(2b)
    G = np.array([[0.5 * c + ds * s / (2.0 * b), 0.5 * c - ds * s / (2.0 * b)],
                  [0.5 * s - ds * c / (2.0 * b), 0.5 * s + ds * c / (2.0 * b)],
                  [-1.0 / b, 1.0 / b]])
    return x_pred, F, G


def wheel_noise(u: ArrayLike, k: float) -> Matrix:
    """M = diag(k^2 |dL|, k^2 |dR|): each wheel's travel variance grows with the distance it rolled."""
    dl, dr = np.abs(np.asarray(u, dtype=float).reshape(2))
    return np.diag([k * k * dl, k * k * dr])


# --- measurement model --------------------------------------------------------------------------
def range_bearing(x: ArrayLike, landmark: ArrayLike) -> tuple[Vector, Matrix]:
    """Expected ``z_hat = [r, phi]`` to ``landmark`` (x, y) and ``H = d h / d x`` (2x3)."""
    px, py, th = np.asarray(x, dtype=float).reshape(3)
    lx, ly = np.asarray(landmark, dtype=float).reshape(2)
    dx, dy = lx - px, ly - py
    q = dx * dx + dy * dy
    r = math.sqrt(q)
    if r < 1e-9:
        raise ValueError("the robot is on top of the landmark: bearing is undefined")
    z_hat = np.array([r, wrap_angle(math.atan2(dy, dx) - th)])
    H = np.array([[-dx / r, -dy / r, 0.0],
                  [dy / q, -dx / q, -1.0]])
    return z_hat, H


def numerical_jacobian(
    func: Callable[[Vector], Vector], x: ArrayLike, eps: float = 1e-6, angle_rows: Sequence[int] = ()
) -> Matrix:
    """Central differences, one column per input. Rows listed in ``angle_rows`` are angles:
    their differences are wrapped so a jump across +-pi doesn't produce a 2*pi/eps spike."""
    x = np.asarray(x, dtype=float).reshape(-1)
    columns = []
    for j in range(x.size):
        step = np.zeros_like(x)
        step[j] = eps
        diff = np.asarray(func(x + step), dtype=float) - np.asarray(func(x - step), dtype=float)
        for i in angle_rows:
            diff[i] = wrap_angle(diff[i])
        columns.append(diff / (2.0 * eps))
    return np.column_stack(columns)


# --- the filter ---------------------------------------------------------------------------------
class EKF:
    """EKF localization with odometry prediction and range-bearing landmark updates."""

    def __init__(self, x0: ArrayLike, P0: ArrayLike) -> None:
        self.x = np.asarray(x0, dtype=float).reshape(3).copy()
        self.x[2] = wrap_angle(self.x[2])
        self.P = np.asarray(P0, dtype=float).reshape(3, 3).copy()
        if not np.allclose(self.P, self.P.T):
            raise ValueError("P0 must be symmetric")
        self.nu: Vector | None = None
        self.S: Matrix | None = None
        self.K: Matrix | None = None

    def predict(self, u: ArrayLike, b: float, M: ArrayLike) -> tuple[Vector, Matrix]:
        """x = f(x, u);  P = F P F^T + G M G^T."""
        self.x, F, G = motion_model(self.x, u, b)
        self.P = F @ self.P @ F.T + G @ np.asarray(M, dtype=float) @ G.T
        return self.x, self.P

    def innovation(self, z: ArrayLike, landmark: ArrayLike, R: ArrayLike) -> tuple[Vector, Matrix, Matrix]:
        """``(nu, S, H)`` for a range-bearing measurement, without changing the state.
        The bearing innovation is wrapped to (-pi, pi]."""
        z_hat, H = range_bearing(self.x, landmark)
        nu = np.asarray(z, dtype=float).reshape(2) - z_hat
        nu[1] = wrap_angle(nu[1])
        S = H @ self.P @ H.T + np.asarray(R, dtype=float)
        return nu, S, H

    def update(self, z: ArrayLike, landmark: ArrayLike, R: ArrayLike) -> float:
        """Fuse one range-bearing measurement (Joseph form). Returns its NIS."""
        R = np.asarray(R, dtype=float)
        nu, S, H = self.innovation(z, landmark, R)
        K = np.linalg.solve(S, H @ self.P).T
        self.x = self.x + K @ nu
        self.x[2] = wrap_angle(self.x[2])
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.nu, self.S, self.K = nu, S, K
        return float(nu @ np.linalg.solve(S, nu))


def associate(ekf: EKF, z: ArrayLike, landmarks: ArrayLike, R: ArrayLike, gate: float = 5.991) -> int | None:
    """Index of the landmark that best explains ``z`` (smallest NIS), or ``None`` if even the best
    one has NIS >= ``gate`` (chi-square 95% for 2 dof)."""
    best, best_d2 = None, gate
    for i, landmark in enumerate(np.asarray(landmarks, dtype=float).reshape(-1, 2)):
        nu, S, _ = ekf.innovation(z, landmark, R)
        d2 = float(nu @ np.linalg.solve(S, nu))
        if d2 < best_d2:
            best, best_d2 = i, d2
    return best
