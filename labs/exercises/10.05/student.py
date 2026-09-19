"""10.05 — The multivariate Kalman filter.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 10.05``.
Only numpy is needed.

Notation (10-localization/code/NOTATION.md), n = state size, m = measurement size:

    x (n,)    state estimate               P (n, n)  its covariance
    F (n, n)  state transition             Q (n, n)  process noise covariance
    B (n, k)  control matrix               u (k,)    control input
    z (m,)    measurement                  H (m, n)  measurement matrix
    R (m, m)  measurement noise covariance
    nu (m,)   innovation z - H x           S (m, m)  innovation covariance H P H^T + R
    K (n, m)  Kalman gain P H^T S^-1

Use ``@`` for matrix products and ``np.linalg.solve`` instead of ``np.linalg.inv`` where you can.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

Vector = NDArray[np.floating]
Matrix = NDArray[np.floating]


def constant_velocity_model(dt: float, accel_std: float) -> tuple[Matrix, Matrix]:
    """Return ``(F, Q)`` for the state [position, velocity] over a time step ``dt``.

    Motion: p_k = p + v dt + a dt^2/2,  v_k = v + a dt, with a random acceleration
    a ~ N(0, accel_std^2) held constant during the step.
    F is the 2x2 matrix of the noise-free part. The noise enters through G = [[dt^2/2], [dt]],
    so Q = G G^T accel_std^2.
    """
    # TODO(student)
    raise NotImplementedError("constant_velocity_model")


class KalmanFilter:
    """A linear Kalman filter. ``x`` (n,) and ``P`` (n, n) always hold the current estimate.

    After each ``update``, ``nu``, ``S`` and ``K`` hold that update's innovation, innovation
    covariance and gain (``None`` before the first update).
    """

    def __init__(self, x0: ArrayLike, P0: ArrayLike) -> None:
        # TODO(student): store float COPIES (x as a flat (n,) array, P as (n, n)).
        #   Raise ValueError if P0 is not n x n or not symmetric (np.allclose(P, P.T)).
        #   Set nu, S, K to None.
        raise NotImplementedError("KalmanFilter.__init__")

    def predict(self, F: ArrayLike, Q: ArrayLike, B: ArrayLike | None = None, u: ArrayLike | None = None) -> tuple[Vector, Matrix]:
        """x = F x (+ B u when both are given);  P = F P F^T + Q.  Return ``(x, P)``."""
        # TODO(student)
        raise NotImplementedError("KalmanFilter.predict")

    def update(self, z: ArrayLike, H: ArrayLike, R: ArrayLike) -> tuple[Vector, Matrix]:
        """Fuse measurement ``z`` (m,) with model ``H`` (m, n) and noise ``R`` (m, m). Return ``(x, P)``.

        nu = z - H x;  S = H P H^T + R;  K = P H^T S^-1;  x = x + K nu
        P  = (I - K H) P (I - K H)^T + K R K^T      <- the "Joseph form": same value as (I - K H) P
                                                       but stays symmetric despite rounding.
        Accept a scalar/list ``z`` and nested lists for H and R (np.atleast_1d / np.atleast_2d).
        Raise ValueError if the shapes don't fit together.
        """
        # TODO(student): store self.nu, self.S, self.K, then update self.x and self.P.
        raise NotImplementedError("KalmanFilter.update")


def nees(x_true: ArrayLike, x_est: ArrayLike, P: ArrayLike) -> float:
    """Normalized estimation error squared: e^T P^-1 e, e = x_true - x_est.

    For a consistent filter it averages n (the state size). Needs ground truth: simulation only.
    """
    # TODO(student)
    raise NotImplementedError("nees")


def nis(nu: ArrayLike, S: ArrayLike) -> float:
    """Normalized innovation squared: nu^T S^-1 nu.

    For a consistent filter it averages m (the measurement size). Needs no ground truth: use it
    on the real robot.
    """
    # TODO(student)
    raise NotImplementedError("nis")
