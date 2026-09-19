"""15.06 — visual servoing: the three Jacobians that turn an image error into joint velocities.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.06``.
Only the standard library and numpy are needed.

The loop you are building one piece at a time:

    s, s*   ->  e = s - s*                            the error, in normalised image coordinates
            ->  L = interaction_matrix(s, Z)          how the image moves per unit camera twist
            ->  v_cam = -gain * L^+ e                 the camera twist that shrinks the error
            ->  v_tool = camera_twist_to_tool_twist   the same motion, at the tool point
            ->  q_dot = J^+ v_tool                    14.06's arm Jacobian finishes the job

Frames: the optical frame is x right, y down, z forward (REP-104). A twist is written
v = (vx, vy, vz, wx, wy, wz) — linear part first.

The reference implementation lives at ``15-manipulation/code/visual_servo.py``.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given: projection and small SE(3) helpers ----------------------------------------------
def project(points_cam: ArrayLike, fx: float = 525.0, fy: float = 525.0,
            cx: float = 319.5, cy: float = 239.5) -> Array:
    """(N, 3) points in the optical frame -> (N, 2) pixels."""
    p = np.asarray(points_cam, dtype=float).reshape(-1, 3)
    if np.any(p[:, 2] <= 1e-6):
        raise ValueError("a feature is at or behind the image plane (Z <= 0)")
    return np.column_stack((fx * p[:, 0] / p[:, 2] + cx, fy * p[:, 1] / p[:, 2] + cy))


def skew(v: ArrayLike) -> Array:
    x, y, z = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rot_of(rotvec: ArrayLike) -> Array:
    """Rodrigues: exp of an axis-angle 3-vector."""
    r = np.asarray(rotvec, dtype=float).reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = skew(k)
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def rotvec_of(R: ArrayLike) -> Array:
    """log of a rotation matrix: the axis-angle 3-vector theta * u."""
    R = np.asarray(R, dtype=float)
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    theta = math.acos(c)
    if theta < 1e-9:
        return np.zeros(3)
    if math.pi - theta < 1e-6:
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        k = int(np.argmax(axis))
        axis = A[:, k] / axis[k]
        return axis / float(np.linalg.norm(axis)) * theta
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (theta / (2.0 * math.sin(theta)))


def inv_T(T: ArrayLike) -> Array:
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


# --- implement these -------------------------------------------------------------------------
def interaction_matrix_point(x: float, y: float, Z: float) -> Array:
    """The 2x6 image Jacobian of ONE normalised point feature (x, y) at depth Z.

        [ -1/Z   0    x/Z    x*y      -(1 + x^2)   y  ]
        [  0   -1/Z   y/Z   1 + y^2     -x*y      -x  ]

    Row 1 is d(x)/d(twist), row 2 is d(y)/d(twist), for the camera twist
    (vx, vy, vz, wx, wy, wz) expressed in the optical frame.

    Raise ``ValueError`` if ``Z`` is not positive: a feature at Z = 0 has no interaction matrix,
    and silently returning infinities is how a servo loop learns to lunge.
    """
    raise NotImplementedError("interaction_matrix_point")  # TODO(student)


def interaction_matrix(features: ArrayLike, depths: ArrayLike) -> Array:
    """Stack ``interaction_matrix_point`` for N normalised features -> a (2N, 6) matrix.

    ``features`` is (N, 2) normalised image coordinates, ``depths`` is N depths in metres.
    Raise ``ValueError`` when the two lengths disagree.
    """
    raise NotImplementedError("interaction_matrix")  # TODO(student)


def damped_pinv(L: ArrayLike, damping: float = 0.0) -> Array:
    """Damped pseudo-inverse, the same idea as 14.06's ``dls_solve`` but returned as a matrix.

    For a wide L (rows <= columns):   L^T (L L^T + lambda^2 I)^-1
    For a tall L (rows  > columns):   (L^T L + lambda^2 I)^-1 L^T

    With ``damping = 0`` this must equal ``np.linalg.pinv(L)`` for a full-rank L.
    """
    raise NotImplementedError("damped_pinv")  # TODO(student)


def ibvs_velocity(features: ArrayLike, features_star: ArrayLike, depths: ArrayLike, *,
                  gain: float = 0.8, damping: float = 0.0) -> Array:
    """The image-based control law: ``v_cam = -gain * L^+ (s - s*)``.

    ``features`` and ``features_star`` are (N, 2) normalised image coordinates; flatten the error
    to a 2N vector in the same row-major order the interaction matrix is stacked in.
    Returns the 6-vector camera twist.
    """
    raise NotImplementedError("ibvs_velocity")  # TODO(student)


def pbvs_velocity(T_cam_obj: ArrayLike, T_cam_obj_star: ArrayLike, *, gain: float = 0.8) -> Array:
    """The position-based control law, straight out of the pose error.

    With ``T_err = inv(T_cam_obj_star) @ T_cam_obj``, the camera must move by exactly that pose,
    so the twist is ``gain * (t_err, rotvec_of(R_err))`` — note the **plus** sign: unlike IBVS,
    the error here is already "the motion the camera should make".
    """
    raise NotImplementedError("pbvs_velocity")  # TODO(student)


def camera_twist_to_tool_twist(v_cam: ArrayLike, T_base_cam: ArrayLike,
                               T_base_tool: ArrayLike) -> Array:
    """Re-express a camera twist as the tool twist the arm Jacobian of 14.06 can serve.

    Two changes at once, and forgetting the second is the classic bug:

    1. Rotate both halves into base coordinates: ``omega = R_base_cam @ v_cam[3:]`` and
       ``R_base_cam @ v_cam[:3]``.
    2. Move the reference point from the camera's optical centre to the tool point:
       add ``omega x (p_tool - p_cam)`` to the linear part.

    Returns the 6-vector (linear first) in **base** coordinates, which is what
    ``SerialChain.geometric_jacobian`` maps joint velocities to.
    """
    raise NotImplementedError("camera_twist_to_tool_twist")  # TODO(student)
