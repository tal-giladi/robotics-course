"""14.04 — Forward kinematics of a URDF-style serial chain, from scratch.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 14.04``.
Only the standard library and numpy are needed — no ROS, no URDF parser.

URDF semantics, which is all there is to forward kinematics::

    T_parent_child(q) = T(xyz, rpy) @ Rot(axis, q)        revolute joint
    T_parent_child    = T(xyz, rpy)                       fixed joint
    T_base_tool(q)    = T_base_1(q1) @ T_1_2(q2) @ ... @ T_n_tool

``xyz``/``rpy`` describe the joint's *origin*: where the child frame sits when ``q = 0``.
``axis`` is a unit vector **in the child frame**, so the rotation is applied on the right.
Units: metres and radians. ``rpy`` is the ROS convention ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.

``SO101_JOINTS`` below is the real SO-101 follower arm, copied from
``14-robotic-arm/code/data/so101_kinematics.yaml`` (itself copied verbatim from the published
URDF). Five revolute joints plus one fixed joint to the tool frame.
"""

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


# --- TODO(student) -------------------------------------------------------------------------------
def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Array:
    """URDF / ROS fixed-axis convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    ``rot_x``, ``rot_y`` and ``rot_z`` are given above. The only thing to get right is the order:
    reverse it and every pose is wrong except the ones where two of the three angles are zero,
    which is most of the poses you would check by eye.
    """
    raise NotImplementedError  # TODO(student)


def axis_angle_to_matrix(axis: ArrayLike, angle: float) -> Array:
    """Rodrigues' formula: rotation by ``angle`` [rad] about the unit vector ``axis``.

        R = I + sin(angle) K + (1 - cos(angle)) K^2,  K = skew(axis_hat)

    Normalise ``axis`` yourself (a URDF is allowed to write ``[0 0 2]``), and raise
    ``ValueError`` for a zero-length axis rather than dividing by zero.
    """
    raise NotImplementedError  # TODO(student)


def joint_transform(joint: Joint, q: float = 0.0) -> Array:
    """``T_parent_child`` for one joint: ``origin @ Rot(axis, q)`` (revolute) or ``origin`` (fixed).

    The origin is ``se3(rpy_to_matrix(*joint.rpy), joint.xyz)``.

    The multiplication order is the whole exercise. ``axis`` is expressed in the CHILD frame, so
    the rotation goes on the RIGHT. ``Rot(axis, q) @ origin`` gives the identical answer at
    ``q = 0`` and drifts as soon as the joint moves, which is why this bug survives a first test.

    Raise ``ValueError`` for any joint type other than "revolute" or "fixed": a prismatic joint
    would silently be treated as rigid.
    """
    raise NotImplementedError  # TODO(student)


def chain_frames(joints: Sequence[Joint], q: ArrayLike) -> list[tuple[str, Array]]:
    """``[(child link name, T_base_child)]`` for every joint in the chain, in order.

    Walk the chain accumulating ``T = T @ joint_transform(joint, ...)``, consuming one value from
    ``q`` per **revolute** joint and none for a fixed one. The returned frame of a revolute joint
    has its origin **on that joint's axis**, which is what makes it useful for Jacobians (14.06)
    and for drawing the arm.

    Raise ``ValueError`` when ``q`` does not have exactly one value per revolute joint, and when
    the chain is broken (some joint's ``child`` is not the next joint's ``parent``) — a silently
    mis-ordered joint list produces a plausible arm that is not this arm.

    Return copies: handing out the same array object you keep mutating is a classic aliasing bug.
    """
    raise NotImplementedError  # TODO(student)


def fk(joints: Sequence[Joint], q: ArrayLike) -> Array:
    """``T_base_tool(q)``: the 4x4 pose of the last child frame in the chain. One line."""
    raise NotImplementedError  # TODO(student)


def approach_pitch(T_base_tool: ArrayLike) -> float:
    """How far the tool's approach axis (its own z axis, column 2) points below the horizontal.

    0 = horizontal, +pi/2 = straight down at the table, -pi/2 = straight up. In terms of the
    approach vector ``a = T[:3, 2]``::

        pitch = atan2(-a_z, hypot(a_x, a_y))

    Use ``atan2``, not ``asin``: it stays well-conditioned when the tool points straight down,
    which is exactly the pose every pick uses.
    """
    raise NotImplementedError  # TODO(student)
