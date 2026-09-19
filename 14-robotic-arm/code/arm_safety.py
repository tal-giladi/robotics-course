"""arm_safety — the layer that decides whether the arm is allowed to move (lesson 14.11).

Everything here is deliberately boring, independent of everything else, and testable with no
hardware. It sits between "your code decided to move" and 14.03's ``servo_tools``:

    your code / a planner / a learned policy
        |
        v
    SafetySupervisor        <- this file: workspace box, speed and torque caps, watchdog,
        |                      session state machine, parked shutdown
        v
    servo_tools (14.03)     <- clamp, rate limit, load abort
        |
        v
    the servo's own registers, then the E-STOP in hardware

The one thing software cannot do is remove energy from the servos. That is the e-stop, and it is
why this file's shutdown path PARKS the arm before it de-energises it: an arm that loses torque
does not stop, it falls.

    py arm_safety.py                 # a no-hardware demo of every refusal and the parked shutdown
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import Enum

from numpy.typing import ArrayLike

from reach import WorkspaceBox
from servo_tools import (
    SO101_JOINTS,
    FakeArmBus,
    JointLimits,
    SafetyConfig,
    SafetyError,
    health_problems,
    move_smoothly,
    read_telemetry,
    safe_enable_torque,
)

# STS3215 at 7.4 V, vendor figures (HARDWARE.md): 16.5 kgf.cm stall, ~300 deg/s no load.
STALL_TORQUE_NM = 16.5 / 10.197
NO_LOAD_SPEED_DEG_S = 300.0
MAX_REACH_M = 0.546                 # the furthest gripper_frame_link gets from base_link


# ============================================================================================
# The numbers that justify the limits
# ============================================================================================
def tip_speed(joint_speed_deg_s: float, radius_m: float = MAX_REACH_M) -> float:
    """Worst-case tool speed [m/s] for a joint speed limit: the whole arm swinging at full reach."""
    return radius_m * math.radians(joint_speed_deg_s)


def kinetic_energy(joint_speed_deg_s: float, moving_mass_kg: float = 0.30,
                   radius_m: float = MAX_REACH_M) -> float:
    """Rough kinetic energy [J] of the moving part of the arm at that speed limit.

    A crude lumped-mass estimate, not a dynamics model: it is here to make the ratio between two
    speed limits visible, not to certify anything.
    """
    v = tip_speed(joint_speed_deg_s, radius_m)
    return 0.5 * moving_mass_kg * v * v


def pinch_force(lever_m: float, torque_percent: float = 100.0) -> float:
    """Force [N] the arm can apply at ``lever_m`` from the joint axis.

    The number that surprises people: at the tool the arm is weak (a few newtons), but a finger
    caught 1-2 cm from a joint axis sees tens to hundreds of newtons, because the same torque acts
    on a twentieth of the lever.
    """
    if lever_m <= 0:
        raise ValueError("lever must be positive")
    return STALL_TORQUE_NM * (torque_percent / 100.0) / lever_m


# ============================================================================================
# The session state machine
# ============================================================================================
class ArmState(str, Enum):
    DISABLED = "disabled"       # no torque; the arm is limp and must be supported
    ENABLED = "enabled"         # torque on, holding, allowed to move
    STOPPED = "stopped"         # a fault or an e-stop: refuses to move until explicitly re-enabled


@dataclass
class SafetySupervisor:
    """Guards every motion request and owns the session's enable/disable state.

    ``watchdog_s`` is the important one: if nobody calls :meth:`heartbeat` for that long, the
    supervisor refuses further motion. The servos keep holding their last goal — software cannot
    de-energise anything by not running — which is exactly why the hardware e-stop exists.
    """

    box: WorkspaceBox = field(default_factory=WorkspaceBox)
    config: SafetyConfig = field(default_factory=SafetyConfig)
    watchdog_s: float = 0.5
    max_joint_step_deg: float = 30.0        # per request, not per tick
    park_pose_deg: Mapping[str, float] = field(
        default_factory=lambda: {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0,
                                 "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 30.0})
    clock: Callable[[], float] = time.monotonic
    state: ArmState = ArmState.DISABLED
    _last_beat: float = field(default=0.0, init=False)
    _fault: str = field(default="", init=False)

    # --- lifecycle ---------------------------------------------------------------------
    def heartbeat(self) -> None:
        self._last_beat = self.clock()

    @property
    def watchdog_expired(self) -> bool:
        return (self.clock() - self._last_beat) > self.watchdog_s

    def enable(self, bus) -> dict[str, float]:
        """Torque on, the 14.03 way, but only from DISABLED and only if the arm is healthy."""
        if self.state is ArmState.STOPPED:
            raise SafetyError(f"stopped ({self._fault}); call clear_fault() deliberately before enabling")
        problems = health_problems(read_telemetry(bus), self.config)
        if problems:
            raise SafetyError("refusing to enable: " + "; ".join(problems))
        present = safe_enable_torque(bus, self.config)
        self.heartbeat()
        self.state = ArmState.ENABLED
        return present

    def stop(self, reason: str) -> None:
        """Latch a fault. Motion stays refused until ``clear_fault`` is called deliberately."""
        self._fault = reason
        self.state = ArmState.STOPPED

    def clear_fault(self) -> None:
        """The software analogue of twisting the e-stop button: an explicit, separate action."""
        self._fault = ""
        self.state = ArmState.DISABLED

    @property
    def fault(self) -> str:
        return self._fault

    # --- the gate ----------------------------------------------------------------------
    def check_move(self, goals_deg: Mapping[str, float], tool_point: ArrayLike | None = None,
                   present_deg: Mapping[str, float] | None = None) -> list[str]:
        """Every reason this motion should not happen. Empty list = go ahead.

        Checks, cheapest and most dangerous first:
          1. the session state and the watchdog
          2. the workspace box, if you told us where the tool would end up
          3. the per-joint software limits
          4. the size of the step from where the arm is now
        """
        problems: list[str] = []
        if self.state is not ArmState.ENABLED:
            problems.append(f"arm is {self.state.value}"
                            + (f" ({self._fault})" if self._fault else ""))
        if self.watchdog_expired:
            problems.append(f"watchdog: no heartbeat for more than {self.watchdog_s:.2f} s")
        if tool_point is not None:
            problems.extend(f"workspace box: {v}" for v in self.box.violations(tool_point))
        for joint, value in goals_deg.items():
            limits = self.config.limits.get(joint)
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

    def move(self, bus, goals_deg: Mapping[str, float], tool_point: ArrayLike | None = None,
             sleep: Callable[[float], None] | None = None) -> dict[str, float]:
        """Check, then move through 14.03's rate-limited, load-aborting mover. Latches on fault."""
        problems = self.check_move(goals_deg, tool_point, bus.read_positions())
        if problems:
            raise SafetyError("; ".join(problems))
        try:
            return move_smoothly(bus, goals_deg, self.config,
                                 sleep=sleep or getattr(bus, "advance", None))
        except SafetyError as exc:
            self.stop(str(exc))
            raise

    # --- shutdown ----------------------------------------------------------------------
    def park_and_disable(self, bus, sleep: Callable[[float], None] | None = None) -> dict[str, float]:
        """Move to a pose that is safe to drop from, THEN cut torque.

        An arm is not a wheeled base: removing torque does not stop it, it lets it fall. Parking
        first means the fall is a few centimetres onto its own base instead of a metre onto
        whatever is below. Call this at the end of every session, and in a ``finally:``.
        """
        parked: dict[str, float] = bus.read_positions()
        if self.state is ArmState.ENABLED:
            goals, _ = _clamped(self.park_pose_deg, self.config)
            try:
                parked = move_smoothly(bus, goals, self.config,
                                       sleep=sleep or getattr(bus, "advance", None))
            except SafetyError as exc:                 # blocked on the way home: stop where we are
                self.stop(f"could not park: {exc}")
        bus.disable_torque()
        if self.state is not ArmState.STOPPED:
            self.state = ArmState.DISABLED
        return parked


def _clamped(goals: Mapping[str, float], config: SafetyConfig) -> tuple[dict[str, float], list[str]]:
    out, notes = {}, []
    for joint, value in goals.items():
        limits = config.limits.get(joint)
        if limits is None:
            raise SafetyError(f"no software limits configured for {joint}")
        new = limits.clamp(value)
        if new != value:
            notes.append(f"{joint}: {value:.1f} -> {new:.1f}")
        out[joint] = new
    return out, notes


# ============================================================================================
# The pre-flight checklist
# ============================================================================================
PRE_FLIGHT = (
    "Arm base clamped to the table, clamp tight.",
    "0.5 m clear in every direction: no mugs, no cables, no keyboard, no pets, no children.",
    "Servo PSU switch (or the e-stop button) within one second's reach of your hand.",
    "E-stop tested THIS SESSION: press it while the arm holds, confirm the arm goes limp.",
    "Face and hands outside the workspace; nothing you care about inside it.",
    "Software limits loaded from your recorded limits file, not the defaults.",
    "Torque limit at 30 % and speed at 30 deg/s for the first run of any new code.",
    "You know what the arm will do when the script ends: it goes LIMP. Support it.",
    "Someone else in the building knows you are running the arm.",
)


def print_checklist() -> None:
    print("PRE-FLIGHT (14.11) — tick every line out loud before you energise the arm:")
    for i, item in enumerate(PRE_FLIGHT, 1):
        print(f"  [ ] {i}. {item}")


if __name__ == "__main__":
    print_checklist()
    print()
    print("the numbers that justify the limits")
    for limit in (NO_LOAD_SPEED_DEG_S, 60.0, 30.0):
        print(f"  {limit:5.0f} deg/s -> tip {tip_speed(limit):5.2f} m/s, "
              f"kinetic energy {kinetic_energy(limit):6.3f} J")
    print()
    for lever, label in ((MAX_REACH_M, "at the tool, fully extended"), (0.05, "at the gripper jaw tip"),
                         (0.02, "2 cm from a joint axis"), (0.01, "1 cm from a joint axis")):
        print(f"  {label:28s}: {pinch_force(lever):6.1f} N at full torque, "
              f"{pinch_force(lever, 30.0):5.1f} N at 30 %")

    print()
    print("a no-hardware session")
    limits = {j: JointLimits(-40.0, 40.0) for j in SO101_JOINTS}
    limits["gripper"] = JointLimits(5.0, 60.0)
    _clock = [0.0]                                    # a fake clock, so the watchdog demo is instant
    supervisor = SafetySupervisor(config=SafetyConfig(limits=limits), clock=lambda: _clock[0])
    bus = FakeArmBus()

    try:
        supervisor.move(bus, {"wrist_flex": 10.0})
    except SafetyError as exc:
        print("  refused before enabling:  ", exc)

    supervisor.enable(bus)
    print("  enabled, state =", supervisor.state.value)
    print("  refused, outside the box:", supervisor.check_move({"wrist_flex": 10.0},
                                                               tool_point=[0.25, 0.0, -0.05]))
    print("  refused, outside limits: ", supervisor.check_move({"wrist_flex": 80.0}))
    print("  refused, step too big:   ", supervisor.check_move({"wrist_flex": 35.0},
                                                               present_deg={"wrist_flex": 0.0}))
    supervisor.max_joint_step_deg = 40.0
    supervisor.move(bus, {"wrist_flex": 20.0})
    print("  moved wrist_flex to      ", round(bus.read_positions()["wrist_flex"], 1), "deg")

    _clock[0] += 5.0                                  # simulate five seconds of a hung control loop
    print("  refused, watchdog:       ", supervisor.check_move({"wrist_flex": 10.0}))
    supervisor.heartbeat()

    bus.servos["elbow_flex"].blocked_at = 5.0
    try:
        supervisor.move(bus, {"elbow_flex": 30.0})
    except SafetyError as exc:
        print("  blocked joint -> STOPPED:", exc)
    print("  state =", supervisor.state.value, "| fault =", supervisor.fault)
    print("  refuses to re-enable:    ", end=" ")
    try:
        supervisor.enable(bus)
    except SafetyError as exc:
        print(exc)

    bus.servos["elbow_flex"].blocked_at = None        # you removed whatever was in the way
    supervisor.clear_fault()
    supervisor.enable(bus)
    parked = supervisor.park_and_disable(bus)
    print("  parked at                ", {k: round(v, 1) for k, v in parked.items()})
    print("  final state =", supervisor.state.value, "(the arm is now LIMP)")
