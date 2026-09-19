"""14.08 — The joint map and the JointTrajectory builder.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 14.08``.
Only the standard library and numpy are needed — no ROS, so this runs anywhere.

Two jobs, both of which are where arms get broken:

1. **Units.** The URDF speaks radians from the URDF's zero pose, positive along each joint's
   ``<axis>``. LeRobot v0.6.1 (``use_degrees=True``) speaks degrees from the MIDDLE of each
   joint's calibrated range, with a per-joint sign that depends on how the servo was mounted, and
   reports the gripper as 0-100 %. Neither zero is the other's zero.

2. **The message.** ``trajectory_msgs/JointTrajectory`` is a list of points, each with positions
   and a ``time_from_start``. Here it is plain dicts, so the tests need no ROS; the field names
   are the real ones.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
GRIPPER_JOINT = "gripper"
ALL_JOINTS = (*ARM_JOINTS, GRIPPER_JOINT)

# URDF position limits [rad], from so101.urdf.xacro.
URDF_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}


# --- TODO(student) -------------------------------------------------------------------------------
@dataclass(frozen=True)
class JointCalibration:
    """How one arm joint's LeRobot reading maps onto its URDF angle.

    ``offset_deg`` is the LeRobot reading when the joint is at the URDF's zero;
    ``sign`` is +1 or -1 and must be rejected in ``__post_init__`` if it is anything else.

        urdf_rad = sign * radians(lerobot_deg - offset_deg)

    Watch the order: subtract the offset FIRST, then apply the sign. Doing it the other way round
    passes a round-trip test and puts the arm in the wrong place.
    """

    sign: int = 1
    offset_deg: float = 0.0

    def __post_init__(self) -> None:
        raise NotImplementedError  # TODO(student)

    def to_urdf_rad(self, lerobot_deg: float) -> float:
        raise NotImplementedError  # TODO(student)

    def to_lerobot_deg(self, urdf_rad: float) -> float:
        raise NotImplementedError  # TODO(student)


@dataclass(frozen=True)
class GripperCalibration:
    """The gripper is reported as a percentage, so it needs a span, not an offset and a sign.

    Linear between (closed_percent -> closed_rad) and (open_percent -> open_rad); values outside
    the calibrated span extrapolate rather than clip (clipping is ``clamp_to_limits``' job).
    Reject a degenerate span (equal percents, or equal radians) with ``ValueError``.
    """

    closed_percent: float = 0.0
    open_percent: float = 100.0
    closed_rad: float = -0.174533
    open_rad: float = 1.74533

    def __post_init__(self) -> None:
        raise NotImplementedError  # TODO(student)

    def to_urdf_rad(self, percent: float) -> float:
        raise NotImplementedError  # TODO(student)

    def to_lerobot_percent(self, urdf_rad: float) -> float:
        raise NotImplementedError  # TODO(student)


def clamp_to_limits(urdf: Mapping[str, float],
                    limits: Mapping[str, tuple[float, float]] | None = None,
                    ) -> tuple[dict[str, float], list[str]]:
    """Clamp URDF radians into the URDF limits.

    Returns ``(clamped, notes)`` where ``notes`` has one human-readable line per joint that was
    actually clamped, naming the joint. An unknown joint name is a ``KeyError``, never a silent
    pass-through: a typo that reaches the servos is how an arm meets the table.
    """
    raise NotImplementedError  # TODO(student)


def measure_calibration(zero_pose_lerobot: Mapping[str, float],
                        probe_lerobot: Mapping[str, float],
                        probe_urdf_rad: Mapping[str, float]) -> dict[str, JointCalibration]:
    """Build the five arm calibrations from two measured poses (exercise 14.08-E1).

    ``zero_pose_lerobot``  readings with the arm held by hand in the URDF's zero pose.
    ``probe_lerobot``      readings after moving each joint to a known positive URDF angle.
    ``probe_urdf_rad``     those known URDF angles.

    offset = the zero-pose reading. sign = +1 when the servo reading moved the same way as the
    URDF angle, -1 otherwise.

    Raise ``ValueError`` naming the joint when either movement is smaller than 5 degrees: you
    cannot infer a sign from noise, and guessing one drives the arm into the table.
    """
    raise NotImplementedError  # TODO(student)


def build_joint_trajectory(joint_names: Sequence[str], t: ArrayLike, q: ArrayLike,
                           qd: ArrayLike | None = None) -> dict:
    """A trajectory_msgs/JointTrajectory as plain dicts.

        {"joint_names": [...],
         "points": [{"positions": [...], "velocities": [...], "time_from_start": 0.02}, ...]}

    ``t`` is (N,) seconds, ``q`` is (N, n) radians, ``qd`` is (N, n) or None (then the points
    carry no "velocities" key at all — not an empty list: an empty list means "no velocity" to
    ros2_control, while a list of zeros means "hold still", and they behave differently).

    ``time_from_start`` comes from ``t``, never from the sample index. Raise ``ValueError`` on a
    shape mismatch between ``joint_names``, ``t``, ``q`` and ``qd``.
    """
    raise NotImplementedError  # TODO(student)


def check_trajectory(trajectory: Mapping,
                     limits: Mapping[str, tuple[float, float]] | None = None) -> list[str]:
    """Everything wrong with a trajectory, as human-readable strings ([] = it is fine).

    Check, in this order:
      1. every joint name is known (an unknown name is reported, not raised, here)
      2. every point's ``positions`` has one value per joint name
      3. ``time_from_start`` is strictly increasing and the first is >= 0
      4. every position is inside its URDF limit

    This is the last check before a goal leaves your process. It must never raise.
    """
    raise NotImplementedError  # TODO(student)
