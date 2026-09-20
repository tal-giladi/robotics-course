"""14.04 — reference solution. Read it after you have tried ``student.py`` yourself."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Joint:
    """One URDF joint: where the child frame sits, and (if revolute) what it turns about."""

    name: str
    type: str                       # "revolute" or "fixed"
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    lower: float = -math.pi
    upper: float = math.pi


def se3(R: ArrayLike | None = None, t: ArrayLike = (0.0, 0.0, 0.0)) -> Array:
    """A 4x4 homogeneous transform from a 3x3 rotation and a translation."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(t, dtype=float)
    return T


def rot_x(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# The SO-101 follower arm, base_link -> gripper_frame_link (so101_new_calib.urdf, Apache-2.0).
# The URDF writes pi as 3.14159; that rounding is kept on purpose so the numbers match the
# published model exactly (it moves the tool by less than 0.01 mm).
SO101_JOINTS: tuple[Joint, ...] = (
    Joint("shoulder_pan", "revolute", "base_link", "shoulder_link",
          (0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159),
          (0.0, 0.0, 1.0), -1.91986, 1.91986),
    Joint("shoulder_lift", "revolute", "shoulder_link", "upper_arm_link",
          (-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0),
          (0.0, 0.0, 1.0), -1.74533, 1.74533),
    Joint("elbow_flex", "revolute", "upper_arm_link", "lower_arm_link",
          (-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708),
          (0.0, 0.0, 1.0), -1.69, 1.69),
    Joint("wrist_flex", "revolute", "lower_arm_link", "wrist_link",
          (-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708),
          (0.0, 0.0, 1.0), -1.65806, 1.65806),
    Joint("wrist_roll", "revolute", "wrist_link", "gripper_link",
          (5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159),
          (0.0, 0.0, 1.0), -2.74385, 2.84121),
    Joint("gripper_frame_joint", "fixed", "gripper_link", "gripper_frame_link",
          (-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0),
          (0.0, 0.0, 1.0), 0.0, 0.0),
)


# --- solution ------------------------------------------------------------------------------------
def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Array:
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def axis_angle_to_matrix(axis: ArrayLike, angle: float) -> Array:
    k = np.asarray(axis, dtype=float).reshape(3)
    n = float(np.linalg.norm(k))
    if n < 1e-12:
        raise ValueError("rotation axis must be non-zero")
    kx, ky, kz = k / n
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def joint_transform(joint: Joint, q: float = 0.0) -> Array:
    origin = se3(rpy_to_matrix(*joint.rpy), joint.xyz)
    if joint.type == "fixed":
        return origin
    if joint.type != "revolute":
        raise ValueError(f"joint type {joint.type!r} not supported")
    return origin @ se3(axis_angle_to_matrix(joint.axis, q))     # axis is in the CHILD frame


def chain_frames(joints: Sequence[Joint], q: ArrayLike) -> list[tuple[str, Array]]:
    for a, b in zip(joints, joints[1:]):
        if a.child != b.parent:
            raise ValueError(f"broken chain: {a.name} ends at {a.child}, {b.name} starts at {b.parent}")
    q = np.asarray(q, dtype=float).reshape(-1)
    n_dof = sum(1 for j in joints if j.type == "revolute")
    if q.size != n_dof:
        raise ValueError(f"chain has {n_dof} revolute joints, got {q.size} values")
    T = np.eye(4)
    out: list[tuple[str, Array]] = []
    k = 0
    for j in joints:
        if j.type == "revolute":
            T = T @ joint_transform(j, float(q[k]))
            k += 1
        else:
            T = T @ joint_transform(j)
        out.append((j.child, T.copy()))
    return out


def fk(joints: Sequence[Joint], q: ArrayLike) -> Array:
    return chain_frames(joints, q)[-1][1]


def approach_pitch(T_base_tool: ArrayLike) -> float:
    a = np.asarray(T_base_tool, dtype=float)[:3, 2]
    return math.atan2(-a[2], math.hypot(a[0], a[1]))
