"""14.08 — reference solution. Read it after you have tried ``student.py`` yourself."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
GRIPPER_JOINT = "gripper"
ALL_JOINTS = (*ARM_JOINTS, GRIPPER_JOINT)

URDF_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

MIN_PROBE_DEG = 5.0


# --- solution ------------------------------------------------------------------------------------
@dataclass(frozen=True)
class JointCalibration:
    sign: int = 1
    offset_deg: float = 0.0

    def __post_init__(self) -> None:
        if self.sign not in (1, -1):
            raise ValueError(f"sign must be +1 or -1, got {self.sign!r}")

    def to_urdf_rad(self, lerobot_deg: float) -> float:
        return self.sign * math.radians(lerobot_deg - self.offset_deg)

    def to_lerobot_deg(self, urdf_rad: float) -> float:
        return math.degrees(urdf_rad) / self.sign + self.offset_deg


@dataclass(frozen=True)
class GripperCalibration:
    closed_percent: float = 0.0
    open_percent: float = 100.0
    closed_rad: float = -0.174533
    open_rad: float = 1.74533

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


def clamp_to_limits(urdf: Mapping[str, float],
                    limits: Mapping[str, tuple[float, float]] | None = None,
                    ) -> tuple[dict[str, float], list[str]]:
    limits = URDF_LIMITS if limits is None else limits
    clamped: dict[str, float] = {}
    notes: list[str] = []
    for name, value in urdf.items():
        if name not in limits:
            raise KeyError(f"no URDF limits for joint {name!r}")
        lo, hi = limits[name]
        new = min(hi, max(lo, value))
        if new != value:
            notes.append(f"{name}: {math.degrees(value):.1f} deg clamped to {math.degrees(new):.1f} deg")
        clamped[name] = new
    return clamped, notes


def measure_calibration(zero_pose_lerobot: Mapping[str, float],
                        probe_lerobot: Mapping[str, float],
                        probe_urdf_rad: Mapping[str, float]) -> dict[str, JointCalibration]:
    out: dict[str, JointCalibration] = {}
    for name in ARM_JOINTS:
        offset = float(zero_pose_lerobot[name])
        delta_deg = float(probe_lerobot[name]) - offset
        delta_urdf_deg = math.degrees(float(probe_urdf_rad[name]))
        if abs(delta_deg) < MIN_PROBE_DEG or abs(delta_urdf_deg) < MIN_PROBE_DEG:
            raise ValueError(
                f"{name}: move it at least {MIN_PROBE_DEG:.0f} deg to determine its sign "
                f"(saw {delta_deg:.2f} deg on the servo, {delta_urdf_deg:.2f} deg in the URDF)")
        out[name] = JointCalibration(sign=1 if delta_deg * delta_urdf_deg > 0 else -1,
                                     offset_deg=offset)
    return out


def build_joint_trajectory(joint_names: Sequence[str], t: ArrayLike, q: ArrayLike,
                           qd: ArrayLike | None = None) -> dict:
    names = list(joint_names)
    t = np.asarray(t, dtype=float).reshape(-1)
    q = np.atleast_2d(np.asarray(q, dtype=float))
    if q.shape != (t.size, len(names)):
        raise ValueError(f"q must be ({t.size}, {len(names)}), got {q.shape}")
    if qd is not None:
        qd = np.atleast_2d(np.asarray(qd, dtype=float))
        if qd.shape != q.shape:
            raise ValueError(f"qd must have the same shape as q, got {qd.shape} vs {q.shape}")
    points = []
    for i in range(t.size):
        point: dict[str, object] = {"positions": [float(v) for v in q[i]]}
        if qd is not None:
            point["velocities"] = [float(v) for v in qd[i]]
        point["time_from_start"] = float(t[i])
        points.append(point)
    return {"joint_names": names, "points": points}


def check_trajectory(trajectory: Mapping,
                     limits: Mapping[str, tuple[float, float]] | None = None) -> list[str]:
    limits = URDF_LIMITS if limits is None else limits
    problems: list[str] = []
    names = list(trajectory.get("joint_names", []))
    points = list(trajectory.get("points", []))
    for name in names:
        if name not in limits:
            problems.append(f"unknown joint {name!r}")

    for i, point in enumerate(points):
        positions = list(point.get("positions", []))
        if len(positions) != len(names):
            problems.append(f"point {i}: {len(positions)} positions for {len(names)} joints")

    times = [float(p.get("time_from_start", 0.0)) for p in points]
    if times and times[0] < 0.0:
        problems.append(f"point 0: time_from_start {times[0]:.3f} is negative")
    for i in range(1, len(times)):
        if times[i] <= times[i - 1]:
            problems.append(f"point {i}: time_from_start {times[i]:.3f} does not advance "
                            f"past {times[i - 1]:.3f}")

    for i, point in enumerate(points):
        positions = list(point.get("positions", []))
        for name, value in zip(names, positions):
            if name not in limits:
                continue
            lo, hi = limits[name]
            if not lo <= value <= hi:
                problems.append(f"point {i}: {name} {math.degrees(value):.1f} deg outside "
                                f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}] deg")
    return problems
