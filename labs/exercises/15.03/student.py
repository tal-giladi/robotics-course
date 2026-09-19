"""15.03 — hand-eye calibration: SE(3) log/exp, the AX = XB pairs, and the Park-Martin solver.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.03``.
Only the standard library and numpy are needed.

Notation (the same as 05-frames-and-transforms and 14-robotic-arm/code/arm_kinematics.py):

    T_a_b   pose of frame b expressed in frame a; T_a_c = T_a_b @ T_b_c

Eye-in-hand:  camera bolted to the gripper, target fixed on the table.
    unknown X = T_gripper_cam,  known g_i = T_base_gripper (FK),  measured c_i = T_cam_target (PnP)
Eye-to-hand: camera bolted to the table, target on the gripper.
    unknown X = T_base_cam

The reference implementation lives at ``15-manipulation/code/hand_eye.py``.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given --------------------------------------------------------------------------------------
def rot_xyz(roll: float, pitch: float, yaw: float) -> Array:
    """R = Rz(yaw) Ry(pitch) Rx(roll) — the URDF/ROS convention."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def make_T(R: ArrayLike, t: ArrayLike) -> Array:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_T(T: ArrayLike) -> Array:
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rotation_angle_deg(R: ArrayLike) -> float:
    """The angle of the rotation, in degrees."""
    return math.degrees(float(np.linalg.norm(rotvec_of(R))))


def pose_error(T_est: ArrayLike, T_true: ArrayLike) -> tuple[float, float]:
    """(translation error in mm, rotation error in degrees) between two poses."""
    d = inv_T(np.asarray(T_true, float)) @ np.asarray(T_est, float)
    return float(np.linalg.norm(d[:3, 3]) * 1000.0), rotation_angle_deg(d[:3, :3])


# --- implement these ------------------------------------------------------------------------------
def rotvec_of(R: ArrayLike) -> Array:
    """log of a rotation matrix: the axis-angle 3-vector (Rodrigues, without OpenCV).

    ``theta = acos((trace(R) - 1) / 2)``, clamped into [-1, 1] before the acos.

    Three cases:
    * ``theta`` near 0            -> return the zero vector.
    * ``theta`` near pi           -> the usual formula divides by ``sin(theta)`` ~ 0. Use the
      symmetric part instead: with ``A = (R + I) / 2``, the axis components satisfy
      ``axis_k^2 = A[k, k]``; take the largest, fix the signs from that column of ``A``, and
      normalise.
    * otherwise                   -> ``w = (R21 - R12, R02 - R20, R10 - R01)`` scaled by
      ``theta / (2 sin theta)``.
    """
    raise NotImplementedError("rotvec_of")  # TODO(student)


def rot_of(rotvec: ArrayLike) -> Array:
    """exp of an axis-angle 3-vector (Rodrigues' formula).

    With ``theta = |r|`` and ``k = r / theta`` and ``K`` the skew-symmetric matrix of ``k``:

        R = I + sin(theta) K + (1 - cos(theta)) K^2

    Return the identity for a (near-)zero vector.
    """
    raise NotImplementedError("rot_of")  # TODO(student)


def motion_pairs(T_base_grippers: list[Array], T_cam_targets: list[Array],
                 *, eye_in_hand: bool = True) -> tuple[list[Array], list[Array]]:
    """Turn absolute poses into the relative motions of AX = XB.

    Eye-in-hand (target fixed): ``g_i X c_i`` is the same pose for every i, so

        A_ij = g_j^-1 g_i ,  B_ij = c_j c_i^-1 ,  A X = X B  with  X = T_gripper_cam

    Eye-to-hand (target on the gripper): ``g_i^-1 X c_i`` is constant, giving

        A_ij = g_j g_i^-1 ,  B_ij = c_j c_i^-1 ,  X = T_base_cam

    Use **all** C(n, 2) pairs with i < j, in that order. Raise ``ValueError`` if the two lists have
    different lengths or fewer than 3 entries.
    """
    raise NotImplementedError("motion_pairs")  # TODO(student)


def solve_ax_xb_park(A: list[Array], B: list[Array]) -> Array:
    """Park & Martin (1994) closed form for A X = X B. Returns the 4x4 X.

    Rotation: with alpha_i = log(R_Ai) and beta_i = log(R_Bi), the least-squares rotation is

        M = sum_i outer(beta_i, alpha_i) ,   R_X = (M^T M)^(-1/2) M^T

    Compute ``(M^T M)^(-1/2)`` from the symmetric eigen-decomposition of ``M^T M`` (``np.linalg.eigh``),
    flooring the eigenvalues at a small positive number. Then **re-orthonormalise** ``R_X`` with an
    SVD (``R = U V^T``, flipping the last column of ``V^T`` if the determinant came out negative) —
    without this it is a near-rotation with a determinant like 0.9997, which poisons everything
    downstream.

    Translation: R_Ai t_X + t_Ai = R_X t_Bi + t_X, i.e. (R_Ai - I) t_X = R_X t_Bi - t_Ai, stacked
    over all pairs (3 rows each) and solved with ``np.linalg.lstsq``.
    """
    raise NotImplementedError("solve_ax_xb_park")  # TODO(student)


def calibrate_eye_in_hand(T_base_grippers: list[Array], T_cam_targets: list[Array]) -> Array:
    """X = T_gripper_cam — the camera's pose in the gripper frame. (Two lines, given the above.)"""
    raise NotImplementedError("calibrate_eye_in_hand")  # TODO(student)


def calibrate_eye_to_hand(T_base_grippers: list[Array], T_cam_targets: list[Array]) -> Array:
    """X = T_base_cam — the fixed camera's pose in the robot base frame."""
    raise NotImplementedError("calibrate_eye_to_hand")  # TODO(student)


def consistency_residual(T_base_grippers: list[Array], T_cam_targets: list[Array],
                         X: Array) -> tuple[float, float]:
    """The check you can run **without ground truth**, on the real robot.

    The target does not move, so ``g_i @ X @ c_i`` must be the same pose for every i. Returns
    (translation spread in **mm**, rotation spread in **degrees**):

    * translation: the RMS distance of the poses' origins from their mean, times 1000;
    * rotation: the RMS of ``rotation_angle_deg(R_0^T R_i)`` over all i, using the FIRST pose's
      rotation as the reference.
    """
    raise NotImplementedError("consistency_residual")  # TODO(student)


def rotation_spread_deg(T_base_grippers: list[Array]) -> float:
    """Mean relative rotation angle over all i < j pairs — the 'is my dataset varied enough' number.

    Below about 40 degrees the dataset is degenerate and the solver will be confidently wrong.
    """
    raise NotImplementedError("rotation_spread_deg")  # TODO(student)
