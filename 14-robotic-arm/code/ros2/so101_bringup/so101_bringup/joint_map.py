"""joint_map — the conversion between the URDF's radians and LeRobot's servo units (lesson 14.08).

This is the single most error-prone file in the whole arm stack, and it contains no ROS at all so
that it can be unit-tested without one (``labs/exercises/14.08``).

Two worlds:

* **URDF / ROS 2 / MoveIt**: radians, zero = the URDF's zero pose, positive = the direction of the
  joint's ``<axis>``, limits from the URDF (``so101.urdf.xacro``).
* **LeRobot v0.6.1 with ``use_degrees=True``** (14.02, 14.03): degrees measured from the MIDDLE of
  each joint's calibrated range, sign decided by how the servo happens to be mounted, and the
  gripper reported as 0-100 % instead of an angle.

Neither zero is the other's zero and neither sign is guaranteed to match, so every joint needs

    urdf_rad = sign * radians(lerobot_deg - offset_deg)

with ``sign`` and ``offset_deg`` **measured**, not assumed (14.08 exercise E1 measures them). The
defaults below are all sign = +1, offset = 0, which is deliberately wrong for a real arm: it makes
the very first RViz run show the mismatch instead of hiding it.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
GRIPPER_JOINT = "gripper"
ALL_JOINTS = (*ARM_JOINTS, GRIPPER_JOINT)

# URDF position limits [rad], copied from so101.urdf.xacro (which copies the upstream URDF).
URDF_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}


@dataclass(frozen=True)
class JointCalibration:
    """How one joint's LeRobot reading maps onto its URDF angle."""

    sign: int = 1               # +1 or -1
    offset_deg: float = 0.0     # the LeRobot reading at the URDF's zero pose

    def __post_init__(self) -> None:
        if self.sign not in (1, -1):
            raise ValueError(f"sign must be +1 or -1, got {self.sign}")

    def to_urdf_rad(self, lerobot_deg: float) -> float:
        return self.sign * math.radians(lerobot_deg - self.offset_deg)

    def to_lerobot_deg(self, urdf_rad: float) -> float:
        return math.degrees(urdf_rad) / self.sign + self.offset_deg


@dataclass(frozen=True)
class GripperCalibration:
    """The gripper is reported as 0-100 %, so it needs a span, not an offset and a sign."""

    closed_percent: float = 0.0
    open_percent: float = 100.0
    closed_rad: float = -0.174533        # URDF lower limit
    open_rad: float = 1.74533            # URDF upper limit

    def __post_init__(self) -> None:
        if self.open_percent == self.closed_percent:
            raise ValueError("open_percent and closed_percent must differ")
        if self.open_rad == self.closed_rad:
            raise ValueError("open_rad and closed_rad must differ")

    def to_urdf_rad(self, percent: float) -> float:
        f = (percent - self.closed_percent) / (self.open_percent - self.closed_percent)
        return self.closed_rad + f * (self.open_rad - self.closed_rad)

    def to_lerobot_percent(self, urdf_rad: float) -> float:
        f = (urdf_rad - self.closed_rad) / (self.open_rad - self.closed_rad)
        return self.closed_percent + f * (self.open_percent - self.closed_percent)


@dataclass(frozen=True)
class JointMap:
    """The whole arm's mapping, plus the URDF limits it must never hand out a value beyond."""

    joints: Mapping[str, JointCalibration] = field(
        default_factory=lambda: {name: JointCalibration() for name in ARM_JOINTS})
    gripper: GripperCalibration = field(default_factory=GripperCalibration)
    limits: Mapping[str, tuple[float, float]] = field(default_factory=lambda: dict(URDF_LIMITS))

    # --- servo units -> ROS ----------------------------------------------------------------
    def to_urdf(self, lerobot: Mapping[str, float]) -> dict[str, float]:
        """LeRobot readings (degrees, and % for the gripper) -> URDF radians."""
        out: dict[str, float] = {}
        for name, value in lerobot.items():
            if name == GRIPPER_JOINT:
                out[name] = self.gripper.to_urdf_rad(value)
            elif name in self.joints:
                out[name] = self.joints[name].to_urdf_rad(value)
            else:
                raise KeyError(f"unknown joint {name!r}")
        return out

    # --- ROS -> servo units ----------------------------------------------------------------
    def to_lerobot(self, urdf: Mapping[str, float]) -> dict[str, float]:
        """URDF radians -> LeRobot readings. Always clamp first; see ``clamp``."""
        out: dict[str, float] = {}
        for name, value in urdf.items():
            if name == GRIPPER_JOINT:
                out[name] = self.gripper.to_lerobot_percent(value)
            elif name in self.joints:
                out[name] = self.joints[name].to_lerobot_deg(value)
            else:
                raise KeyError(f"unknown joint {name!r}")
        return out

    def clamp(self, urdf: Mapping[str, float]) -> tuple[dict[str, float], list[str]]:
        """Clamp URDF-radian values into the URDF limits. Returns (clamped, what was clamped).

        ros2_control's ``<command_interface><param name="min"/>`` does this too, and MoveIt checks
        it again, and 14.03's ``clamp_goals`` does it once more in servo units. Three redundant
        clamps is the correct number: each one is in a different process.
        """
        clamped: dict[str, float] = {}
        notes: list[str] = []
        for name, value in urdf.items():
            lo, hi = self.limits[name]
            new = min(hi, max(lo, value))
            if new != value:
                notes.append(f"{name}: {math.degrees(value):.1f} deg clamped to "
                             f"{math.degrees(new):.1f} deg")
            clamped[name] = new
        return clamped, notes

    def round_trip_error_rad(self, urdf: Mapping[str, float]) -> dict[str, float]:
        """|to_urdf(to_lerobot(q)) - q| per joint. Must be ~1e-12; anything else is a bug."""
        back = self.to_urdf(self.to_lerobot(urdf))
        return {name: abs(back[name] - value) for name, value in urdf.items()}

    # --- persistence -----------------------------------------------------------------------
    def to_json(self) -> str:
        return json.dumps({
            "joints": {n: {"sign": c.sign, "offset_deg": c.offset_deg} for n, c in self.joints.items()},
            "gripper": {
                "closed_percent": self.gripper.closed_percent,
                "open_percent": self.gripper.open_percent,
                "closed_rad": self.gripper.closed_rad,
                "open_rad": self.gripper.open_rad,
            },
        }, indent=2)

    @classmethod
    def from_json(cls, text: str) -> JointMap:
        data = json.loads(text)
        joints = {n: JointCalibration(sign=int(c["sign"]), offset_deg=float(c["offset_deg"]))
                  for n, c in data["joints"].items()}
        missing = set(ARM_JOINTS) - set(joints)
        if missing:
            raise ValueError(f"calibration is missing joints: {sorted(missing)}")
        return cls(joints=joints, gripper=GripperCalibration(**data["gripper"]))

    @classmethod
    def load(cls, path: Path | str) -> JointMap:
        return cls.from_json(Path(path).read_text(encoding="utf-8"))

    def save(self, path: Path | str) -> None:
        Path(path).write_text(self.to_json(), encoding="utf-8")


def measure_calibration(zero_pose_lerobot: Mapping[str, float],
                        positive_probe_lerobot: Mapping[str, float],
                        probe_urdf_rad: Mapping[str, float]) -> JointMap:
    """Build a JointMap from two measured poses — the procedure of exercise 14.08-E1.

    1. Put the arm in the URDF's zero pose by hand and record LeRobot's readings:
       that is ``offset_deg`` for every joint.
    2. Move one joint at a time to a known URDF angle (read it off RViz with the slider matched
       by eye, or measure it with a protractor) and record the readings again. The sign of
       (reading - offset) against the sign of the URDF angle gives ``sign``.

    A joint that did not move between the two poses raises: you cannot infer its sign, and
    guessing is how an arm drives itself into the table.
    """
    joints: dict[str, JointCalibration] = {}
    for name in ARM_JOINTS:
        offset = float(zero_pose_lerobot[name])
        delta_deg = float(positive_probe_lerobot[name]) - offset
        delta_rad = float(probe_urdf_rad[name])
        if abs(delta_deg) < 5.0 or abs(delta_rad) < math.radians(5.0):
            raise ValueError(f"{name}: move it at least 5 deg to determine its sign "
                             f"(saw {delta_deg:.2f} deg on the servo, "
                             f"{math.degrees(delta_rad):.2f} deg in the URDF)")
        joints[name] = JointCalibration(sign=1 if delta_deg * delta_rad > 0 else -1,
                                        offset_deg=offset)
    return JointMap(joints=joints)
