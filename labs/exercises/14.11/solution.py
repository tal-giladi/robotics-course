"""14.11 — reference solution. Read it after you have tried ``student.py`` yourself.

The safety supervisor: state machine, watchdog, gate, and the numbers behind the limits.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 14.11``.
Only the standard library is needed — no numpy, no ROS, no hardware.

This is the layer that decides whether the arm is allowed to move. It sits above 14.03's servo
safety and below whatever decided to move: your code, a planner, or a learned policy. Everything
in it must be boring, independent, and easy to read at 2 a.m.

The one thing it CANNOT do is remove energy from the servos. That is the e-stop, in hardware.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import Enum

# STS3215 at 7.4 V (HARDWARE.md), and the SO-101's maximum reach.
STALL_TORQUE_NM = 16.5 / 10.197        # 1.618 N.m
NO_LOAD_SPEED_DEG_S = 300.0
MAX_REACH_M = 0.546


# --- given ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class JointLimits:
    lower: float
    upper: float

    def contains(self, value: float) -> bool:
        return self.lower <= value <= self.upper

    def clamp(self, value: float) -> float:
        return min(self.upper, max(self.lower, value))


@dataclass(frozen=True)
class Box:
    """A workspace box in the arm's base frame, metres."""

    x: tuple[float, float] = (0.10, 0.34)
    y: tuple[float, float] = (-0.22, 0.22)
    z: tuple[float, float] = (0.02, 0.32)

    def violations(self, point: Sequence[float]) -> list[str]:
        out = []
        for name, value, (low, high) in zip("xyz", point, (self.x, self.y, self.z)):
            if value < low:
                out.append(f"{name} = {value:.3f} m is below the box minimum {low:.3f} m")
            elif value > high:
                out.append(f"{name} = {value:.3f} m is above the box maximum {high:.3f} m")
        return out


class SafetyError(RuntimeError):
    """Raised instead of moving. The caller decides what to do about it."""



# --- solution ------------------------------------------------------------------------------------
def tip_speed(joint_speed_deg_s: float, radius_m: float = MAX_REACH_M) -> float:
    return radius_m * math.radians(joint_speed_deg_s)


def kinetic_energy(joint_speed_deg_s: float, moving_mass_kg: float = 0.30,
                   radius_m: float = MAX_REACH_M) -> float:
    v = tip_speed(joint_speed_deg_s, radius_m)
    return 0.5 * moving_mass_kg * v * v


def pinch_force(lever_m: float, torque_percent: float = 100.0) -> float:
    if lever_m <= 0:
        raise ValueError("lever must be positive")
    return STALL_TORQUE_NM * (torque_percent / 100.0) / lever_m


class ArmState(str, Enum):
    DISABLED = "disabled"
    ENABLED = "enabled"
    STOPPED = "stopped"


@dataclass
class SafetySupervisor:
    limits: Mapping[str, JointLimits]
    box: Box = field(default_factory=Box)
    watchdog_s: float = 0.5
    max_joint_step_deg: float = 30.0
    clock: Callable[[], float] = time.monotonic
    state: ArmState = ArmState.DISABLED
    _last_beat: float = field(default=0.0, init=False)
    _fault: str = field(default="", init=False)

    @property
    def fault(self) -> str:
        return self._fault

    def heartbeat(self) -> None:
        self._last_beat = self.clock()

    @property
    def watchdog_expired(self) -> bool:
        return (self.clock() - self._last_beat) > self.watchdog_s

    def enable(self) -> None:
        if self.state is ArmState.STOPPED:
            raise SafetyError(f"stopped ({self._fault}); call clear_fault() deliberately first")
        self.heartbeat()
        self.state = ArmState.ENABLED

    def stop(self, reason: str) -> None:
        self._fault = reason
        self.state = ArmState.STOPPED

    def clear_fault(self) -> None:
        self._fault = ""
        self.state = ArmState.DISABLED          # DISABLED, not ENABLED: recovery is two acts

    def check_move(self, goals_deg: Mapping[str, float], tool_point: Sequence[float] | None = None,
                   present_deg: Mapping[str, float] | None = None) -> list[str]:
        problems: list[str] = []
        if self.state is not ArmState.ENABLED:
            problems.append(f"arm is {self.state.value}"
                            + (f" ({self._fault})" if self._fault else ""))
        if self.watchdog_expired:
            problems.append(f"watchdog: no heartbeat for more than {self.watchdog_s:.2f} s")
        if tool_point is not None:
            problems.extend(f"workspace box: {v}" for v in self.box.violations(tool_point))
        for joint, value in goals_deg.items():
            limits = self.limits.get(joint)
            if limits is None:
                problems.append(f"{joint}: no software limits configured")
            elif not limits.contains(value):
                problems.append(f"{joint}: {value:.1f} deg outside "
                                f"[{limits.lower:.1f}, {limits.upper:.1f}] deg")
        if present_deg is not None:
            for joint, value in goals_deg.items():
                if joint in present_deg:
                    step = abs(value - present_deg[joint])
                    if step > self.max_joint_step_deg:
                        problems.append(f"{joint}: {step:.1f} deg step exceeds "
                                        f"{self.max_joint_step_deg:.1f} deg per request")
        return problems
