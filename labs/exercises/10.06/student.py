"""10.06 — The Extended Kalman Filter for a differential-drive robot.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 10.06``.
Only numpy and math are needed.

Notation (10-localization/code/NOTATION.md):

    x = [x, y, theta]      robot pose in the map frame (m, m, rad), theta wrapped to (-pi, pi]
    u = [d_left, d_right]  wheel travels since the last predict (m), from encoder ticks
    b                      wheel separation (m)
    F = df/dx (3x3)        motion Jacobian w.r.t. the state
    G = df/du (3x2)        motion Jacobian w.r.t. the wheel travels
    M (2x2)                covariance of the wheel-travel errors;  Q = G M G^T
    z = [r, phi]           range (m) and bearing (rad, CCW from base_link +x) to a landmark
    H = dh/dx (2x3)        measurement Jacobian;  R (2x2) measurement noise covariance
    nu, S, K               innovation (bearing wrapped!), its covariance, Kalman gain
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

Vector = NDArray[np.floating]
Matrix = NDArray[np.floating]


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]. Provided."""
    return math.pi - (math.pi - a) % (2.0 * math.pi)


# --- motion model -------------------------------------------------------------------------------
def motion_model(x: ArrayLike, u: ArrayLike, b: float) -> tuple[Vector, Matrix, Matrix]:
    """Midpoint odometry model. Return ``(x_pred, F, G)``.

    ds = (dL + dR) / 2,  dth = (dR - dL) / b,  th_m = th + dth / 2
    x' = x + ds cos(th_m),  y' = y + ds sin(th_m),  th' = wrap(th + dth)

    F is the 3x3 matrix of partial derivatives of (x', y', th') w.r.t. (x, y, th);
    G is the 3x2 matrix of partial derivatives w.r.t. (dL, dR). Remember th_m depends on dL and dR.
    """
    # TODO(student)
    raise NotImplementedError("motion_model")


def wheel_noise(u: ArrayLike, k: float) -> Matrix:
    """Return M = diag(k^2 |dL|, k^2 |dR|): a wheel's travel variance grows with the distance it rolled
    (``k`` has units m^0.5; k = 0.01 means sigma = 1 cm after 1 m)."""
    # TODO(student)
    raise NotImplementedError("wheel_noise")


# --- measurement model --------------------------------------------------------------------------
def range_bearing(x: ArrayLike, landmark: ArrayLike) -> tuple[Vector, Matrix]:
    """Return the expected measurement ``z_hat = [r, phi]`` of ``landmark`` (x, y) seen from pose ``x``,
    with phi wrapped to (-pi, pi], and the 2x3 Jacobian ``H = dh/dx``.

    Raise ValueError if the robot is (numerically) on top of the landmark.
    """
    # TODO(student)
    raise NotImplementedError("range_bearing")


def numerical_jacobian(
    func: Callable[[Vector], Vector], x: ArrayLike, eps: float = 1e-6, angle_rows: Sequence[int] = ()
) -> Matrix:
    """Central-difference Jacobian of ``func`` at ``x``: column j = (func(x + eps e_j) - func(x - eps e_j)) / (2 eps).

    Output rows listed in ``angle_rows`` are angles: wrap their differences before dividing, so a
    value that jumps from +pi to -pi between the two evaluations doesn't give a 2*pi/eps spike.
    """
    # TODO(student)
    raise NotImplementedError("numerical_jacobian")


# --- the filter ---------------------------------------------------------------------------------
class EKF:
    """EKF localization: odometry predict, range-bearing landmark update.

    ``x`` (3,) and ``P`` (3, 3) hold the current estimate. After ``update``, ``nu``, ``S`` and ``K``
    hold that update's innovation, innovation covariance and gain (``None`` before the first one).
    """

    def __init__(self, x0: ArrayLike, P0: ArrayLike) -> None:
        # TODO(student): store float COPIES, wrap x[2], raise ValueError if P0 is not symmetric,
        #   set nu, S, K to None.
        raise NotImplementedError("EKF.__init__")

    def predict(self, u: ArrayLike, b: float, M: ArrayLike) -> tuple[Vector, Matrix]:
        """x = f(x, u);  P = F P F^T + G M G^T  (F and G evaluated at the OLD x). Return ``(x, P)``."""
        # TODO(student)
        raise NotImplementedError("EKF.predict")

    def innovation(self, z: ArrayLike, landmark: ArrayLike, R: ArrayLike) -> tuple[Vector, Matrix, Matrix]:
        """Return ``(nu, S, H)`` for measurement ``z`` of ``landmark`` without changing the state.

        nu = z - h(x) with the bearing component wrapped to (-pi, pi];  S = H P H^T + R.
        """
        # TODO(student)
        raise NotImplementedError("EKF.innovation")

    def update(self, z: ArrayLike, landmark: ArrayLike, R: ArrayLike) -> float:
        """Fuse one range-bearing measurement: K = P H^T S^-1, x = x + K nu (wrap theta),
        Joseph-form P. Store nu, S, K. Return the NIS  nu^T S^-1 nu."""
        # TODO(student)
        raise NotImplementedError("EKF.update")


def associate(ekf: EKF, z: ArrayLike, landmarks: ArrayLike, R: ArrayLike, gate: float = 5.991) -> int | None:
    """Data association without ids: return the index of the landmark (rows of ``landmarks``, shape
    (L, 2)) whose predicted measurement explains ``z`` best — smallest Mahalanobis distance squared
    nu^T S^-1 nu — or ``None`` if even the best one is >= ``gate`` (chi-square 95%, 2 dof)."""
    # TODO(student)
    raise NotImplementedError("associate")
