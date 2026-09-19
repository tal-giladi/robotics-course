"""Reference solution for 10.05 — The multivariate Kalman filter.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

Vector = NDArray[np.floating]
Matrix = NDArray[np.floating]


def constant_velocity_model(dt: float, accel_std: float) -> tuple[Matrix, Matrix]:
    """F and Q for the state [position, velocity] with random (white-noise) acceleration.

    p_k = p + v dt + a dt^2 / 2,  v_k = v + a dt,  a ~ N(0, accel_std^2) constant over the step,
    so the noise enters through G = [dt^2/2, dt]^T and Q = G G^T accel_std^2.
    """
    F = np.array([[1.0, dt], [0.0, 1.0]])
    G = np.array([[0.5 * dt * dt], [dt]])
    Q = G @ G.T * accel_std**2
    return F, Q


class KalmanFilter:
    """Linear Kalman filter. ``x`` is the (n,) state estimate, ``P`` its (n, n) covariance."""

    def __init__(self, x0: ArrayLike, P0: ArrayLike) -> None:
        self.x = np.asarray(x0, dtype=float).reshape(-1).copy()
        self.P = np.asarray(P0, dtype=float).copy()
        n = self.x.size
        if self.P.shape != (n, n):
            raise ValueError(f"P0 must be {n}x{n}, got {self.P.shape}")
        if not np.allclose(self.P, self.P.T):
            raise ValueError("P0 must be symmetric")
        self.nu: Vector | None = None
        self.S: Matrix | None = None
        self.K: Matrix | None = None

    def predict(self, F: ArrayLike, Q: ArrayLike, B: ArrayLike | None = None, u: ArrayLike | None = None) -> tuple[Vector, Matrix]:
        F = np.asarray(F, dtype=float)
        self.x = F @ self.x
        if B is not None and u is not None:
            self.x = self.x + np.asarray(B, dtype=float) @ np.atleast_1d(np.asarray(u, dtype=float))
        self.P = F @ self.P @ F.T + np.asarray(Q, dtype=float)
        return self.x, self.P

    def update(self, z: ArrayLike, H: ArrayLike, R: ArrayLike) -> tuple[Vector, Matrix]:
        z = np.atleast_1d(np.asarray(z, dtype=float))
        H = np.atleast_2d(np.asarray(H, dtype=float))
        R = np.atleast_2d(np.asarray(R, dtype=float))
        if H.shape != (z.size, self.x.size) or R.shape != (z.size, z.size):
            raise ValueError(f"shapes: z {z.shape}, H {H.shape} (want {(z.size, self.x.size)}), R {R.shape}")
        self.nu = z - H @ self.x
        self.S = H @ self.P @ H.T + R
        self.K = np.linalg.solve(self.S, H @ self.P).T  # = P H^T S^-1, since P and S are symmetric
        self.x = self.x + self.K @ self.nu
        I_KH = np.eye(self.x.size) - self.K @ H
        self.P = I_KH @ self.P @ I_KH.T + self.K @ R @ self.K.T  # Joseph form: stays symmetric PSD
        return self.x, self.P


def nees(x_true: ArrayLike, x_est: ArrayLike, P: ArrayLike) -> float:
    """Normalized estimation error squared: e^T P^-1 e with e = x_true - x_est."""
    e = np.atleast_1d(np.asarray(x_true, dtype=float) - np.asarray(x_est, dtype=float))
    return float(e @ np.linalg.solve(np.atleast_2d(np.asarray(P, dtype=float)), e))


def nis(nu: ArrayLike, S: ArrayLike) -> float:
    """Normalized innovation squared: nu^T S^-1 nu (no ground truth needed)."""
    nu = np.atleast_1d(np.asarray(nu, dtype=float))
    return float(nu @ np.linalg.solve(np.atleast_2d(np.asarray(S, dtype=float)), nu))
