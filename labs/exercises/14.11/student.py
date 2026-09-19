"""14.11 — The safety supervisor: state machine, watchdog, gate, and the numbers behind the limits.

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


# --- TODO(student) -------------------------------------------------------------------------------
def tip_speed(joint_speed_deg_s: float, radius_m: float = MAX_REACH_M) -> float:
    """Worst-case tool speed [m/s]: the whole arm swinging about the base at full reach, v = r * w.

    Remember that the input is degrees per second and the formula needs radians per second.
    """
    raise NotImplementedError  # TODO(student)


def kinetic_energy(joint_speed_deg_s: float, moving_mass_kg: float = 0.30,
                   radius_m: float = MAX_REACH_M) -> float:
    """Rough kinetic energy [J] at that speed limit: 0.5 * m * v^2 with v from ``tip_speed``.

    A lumped-mass estimate, not a dynamics model. It exists to make the RATIO between two speed
    limits visible — which is quadratic, and that is the whole point.
    """
    raise NotImplementedError  # TODO(student)


def pinch_force(lever_m: float, torque_percent: float = 100.0) -> float:
    """Force [N] the arm can apply at ``lever_m`` from a joint axis: F = tau / r.

    Reject a non-positive lever with ``ValueError``. The interesting consequence: at the tool the
    arm manages a few newtons, and a centimetre from a joint axis the same torque becomes 162 N.
    """
    raise NotImplementedError  # TODO(student)


class ArmState(str, Enum):
    """The session state machine. Motion is allowed in exactly one of these."""

    DISABLED = "disabled"       # no torque; the arm is limp and will fall if unsupported
    ENABLED = "enabled"         # torque on, holding, allowed to move
    STOPPED = "stopped"         # a latched fault: refuses everything until cleared DELIBERATELY


@dataclass
class SafetySupervisor:
    """Guards every motion request and owns the enable/disable state.

    Implement:

    * ``heartbeat`` / ``watchdog_expired`` — the watchdog. Expired means STRICTLY more than
      ``watchdog_s`` since the last heartbeat.
    * ``enable`` — only from DISABLED. From STOPPED it must raise, because recovery is two
      deliberate acts, not one.
    * ``stop`` — latch a fault: record the reason and go to STOPPED.
    * ``clear_fault`` — clear the reason and go to **DISABLED**, not ENABLED.
    * ``check_move`` — every reason this motion should not happen, as strings; [] means go.
    """

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
        raise NotImplementedError  # TODO(student)

    @property
    def watchdog_expired(self) -> bool:
        raise NotImplementedError  # TODO(student)

    def enable(self) -> None:
        """Go to ENABLED and beat the watchdog. Raise ``SafetyError`` if the state is STOPPED."""
        raise NotImplementedError  # TODO(student)

    def stop(self, reason: str) -> None:
        raise NotImplementedError  # TODO(student)

    def clear_fault(self) -> None:
        raise NotImplementedError  # TODO(student)

    def check_move(self, goals_deg: Mapping[str, float], tool_point: Sequence[float] | None = None,
                   present_deg: Mapping[str, float] | None = None) -> list[str]:
        """Every reason to refuse, cheapest and most dangerous first:

        1. the session state (mention the fault reason when there is one)
        2. the watchdog
        3. the workspace box, when ``tool_point`` is given (prefix each with "workspace box: ")
        4. the per-joint software limits — **a joint with no configured limits is a PROBLEM**,
           never a joint without limits
        5. the step from ``present_deg``, when it is given, against ``max_joint_step_deg``

        Return every problem found, not just the first: a caller that fixes one and retries
        should not discover the next one by moving the arm.
        """
        raise NotImplementedError  # TODO(student)
