"""10.04 — The Kalman filter in 1D.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 10.04``.
Only the standard library (``math``) and numpy are needed.

The model (notation used through module 10, see 10-localization/code/NOTATION.md):

    predict:  x_k = x_{k-1} + u_k + w,   w ~ N(0, Q)     (u = what odometry says changed)
    measure:  z_k = x_k + v,             v ~ N(0, R)     (the sensor reads the state directly)

``x`` is the estimate (m), ``P`` its variance (m^2). ``Q``, ``R``, ``P`` are VARIANCES (sigma^2),
never standard deviations.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray


def kalman_gain(P_pred: float, R: float) -> float:
    """K = P_pred / (P_pred + R).

    Raise ``ValueError`` if either variance is negative, or both are zero.
    """
    # TODO(student)
    raise NotImplementedError("kalman_gain")


def steady_state_gain(Q: float, R: float) -> float:
    """The gain K that a filter with constant Q and R settles to after many predict/update cycles.

    In steady state the predicted variance M repeats itself: M = M*R/(M + R) + Q
    (update shrinks M to M*R/(M+R), predict adds Q). Solve that quadratic for M > 0, then return
    kalman_gain(M, R).
    """
    # TODO(student): multiply out M(M + R) = M R + Q (M + R) and use the quadratic formula.
    raise NotImplementedError("steady_state_gain")


class KalmanFilter1D:
    """A scalar Kalman filter. Attributes ``x`` and ``P`` always hold the current estimate.

    After each ``update`` the attributes ``K`` (gain), ``nu`` (innovation z - x_pred) and ``S``
    (innovation variance P_pred + R) hold that update's values; before the first update they are
    ``None``.
    """

    def __init__(self, x0: float, P0: float) -> None:
        # TODO(student): store x0 and P0 as floats; reject a negative P0 with ValueError;
        #   set K, nu, S to None.
        raise NotImplementedError("KalmanFilter1D.__init__")

    def predict(self, u: float = 0.0, Q: float = 0.0) -> tuple[float, float]:
        """Move the estimate by ``u`` and grow its variance by ``Q`` (reject Q < 0).
        Return ``(x, P)``."""
        # TODO(student): two lines of math.
        raise NotImplementedError("KalmanFilter1D.predict")

    def update(self, z: float, R: float) -> tuple[float, float]:
        """Fuse measurement ``z`` with variance ``R`` (reject R <= 0). Return ``(x, P)``.

        nu = z - x;  S = P + R;  K = P / S;  x = x + K nu;  P = (1 - K) P
        """
        # TODO(student): store nu, S, K on self, then update x and P.
        raise NotImplementedError("KalmanFilter1D.update")


def run_kf_1d(
    x0: float,
    P0: float,
    controls: Sequence[float],
    measurements: Sequence[float | None],
    Q: float,
    R: float,
) -> tuple[NDArray[np.floating], NDArray[np.floating]]:
    """Run the filter over a log.

    For every step k: ``predict(controls[k], Q)``, then ``update(measurements[k], R)`` unless the
    measurement is ``None`` (no valid reading this step). Return two numpy arrays: the estimate x
    and variance P after each step. Raise ``ValueError`` if the two sequences differ in length.
    """
    # TODO(student)
    raise NotImplementedError("run_kf_1d")
