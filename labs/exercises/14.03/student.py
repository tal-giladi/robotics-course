"""14.03 — Safe bus-servo control: register units, a safe start-up, and a rate-limited move.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 14.03``.
Only the standard library is needed — the tests drive a simulated six-servo bus, so nothing here
can break a real arm. It can, however, teach you the order of operations that keeps one intact.

Register units (Feetech STS memory table, as used by the SO-101's STS3215 servos):

* position: 4096 steps per turn
* ``Goal_Velocity``: steps/s (0 means "as fast as you like" — the factory default, and the reason
  arms snap)
* ``Acceleration``: 100 steps/s^2 per unit, 1..254 (0 again means maximum)
* ``Torque_Limit`` and ``Present_Load``: 0.1 % of stall torque, so 1000 = 100 %
* ``Present_Voltage``: 0.1 V; ``Present_Temperature``: deg C; ``Present_Current``: 6.5 mA per unit

Positions are LeRobot's (v0.6.1, ``use_degrees=True``): degrees from the middle of each joint's
calibrated range, and 0-100 % for the gripper.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Protocol

# --- given ---------------------------------------------------------------------------------------
STEPS_PER_REV = 4096
DEG_PER_STEP = 360.0 / STEPS_PER_REV               # 0.0879 deg
ACCEL_UNIT_STEPS_S2 = 100.0                        # one Acceleration unit
CURRENT_UNIT_A = 0.0065                            # one Present_Current unit
SO101_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def steps_to_deg(steps: float, center: float = 2047.5) -> float:
    """Raw position (0..4095) to degrees from ``center`` (LeRobot uses the middle of the range)."""
    return (steps - center) * DEG_PER_STEP


def deg_to_steps(deg: float, center: float = 2047.5) -> int:
    return int(round(center + deg / DEG_PER_STEP))


@dataclass(frozen=True)
class JointLimits:
    lower: float
    upper: float

    def __post_init__(self) -> None:
        if self.lower >= self.upper:
            raise ValueError(f"lower {self.lower} must be below upper {self.upper}")

    def contains(self, value: float) -> bool:
        return self.lower <= value <= self.upper

    def clamp(self, value: float) -> float:
        return min(self.upper, max(self.lower, value))


def first_run_limits() -> dict[str, JointLimits]:
    """Deliberately narrow limits for the very first powered session (LeRobot units)."""
    arm = {name: JointLimits(-30.0, 30.0) for name in SO101_JOINTS[:-1]}
    arm["gripper"] = JointLimits(5.0, 60.0)
    return arm


@dataclass(frozen=True)
class SafetyConfig:
    limits: Mapping[str, JointLimits] = field(default_factory=first_run_limits)
    torque_percent: float = 30.0          # of stall torque; raise gradually once things work
    speed_deg_s: float = 30.0             # servo-side speed limit
    accel_deg_s2: float = 60.0            # servo-side acceleration
    max_step_deg: float = 1.0             # software rate limit per control tick
    max_load_percent: float = 25.0        # abort a move above this |Present_Load|
    max_temperature_c: int = 55           # abort / refuse above this
    voltage_window_v: tuple[float, float] = (4.5, 8.4)   # 7.4 V servos on a 5 V supply

    def __post_init__(self) -> None:
        if self.max_load_percent >= self.torque_percent:
            raise ValueError("max_load_percent must be below torque_percent: a servo capped at "
                             f"{self.torque_percent} % can never report more, so the check would never fire")


class SafetyError(RuntimeError):
    """Raised instead of moving when a check fails. The caller decides whether to disable torque."""


class ArmBus(Protocol):
    joint_names: tuple[str, ...]

    def read_positions(self) -> dict[str, float]: ...
    def write_goal_positions(self, goals: Mapping[str, float]) -> None: ...
    def enable_torque(self) -> None: ...
    def disable_torque(self) -> None: ...
    def read_register(self, register: str, joint: str) -> int: ...
    def write_register(self, register: str, joint: str, value: int) -> None: ...


@dataclass
class Telemetry:
    position: float
    load_percent: float
    voltage_v: float
    temperature_c: int
    current_a: float


def read_telemetry(bus: ArmBus) -> dict[str, Telemetry]:
    """One decoded snapshot of every joint (given — it is just the unit table applied)."""
    positions = bus.read_positions()
    return {
        j: Telemetry(
            position=positions[j],
            load_percent=bus.read_register("Present_Load", j) / 10.0,
            voltage_v=bus.read_register("Present_Voltage", j) / 10.0,
            temperature_c=bus.read_register("Present_Temperature", j),
            current_a=bus.read_register("Present_Current", j) * CURRENT_UNIT_A,
        )
        for j in bus.joint_names
    }


# --- TODO(student): the four register conversions --------------------------------------------------
def speed_register(deg_per_s: float) -> int:
    """``Goal_Velocity`` value for a speed limit in deg/s.

    The register counts steps per second. Round to the nearest integer, but never return 0: 0 in
    this register means "no limit", so a rounding-down bug turns your speed cap into full speed.
    Raise ``ValueError`` for a non-positive request, for the same reason.
    """
    raise NotImplementedError  # TODO(student)


def acceleration_register(deg_per_s2: float) -> int:
    """``Acceleration`` value (1..254) for an acceleration in deg/s^2.

    One unit is ``ACCEL_UNIT_STEPS_S2`` steps/s^2. Clamp into 1..254 — again, 0 means maximum, and
    255 does not exist. Raise ``ValueError`` for a non-positive request.
    """
    raise NotImplementedError  # TODO(student)


def torque_register(percent_of_stall: float) -> int:
    """``Torque_Limit`` value (0..1000) for a percentage of stall torque (1000 = 100 %).

    Raise ``ValueError`` outside 0..100: a percentage above 100 silently becomes a number the
    servo reads as something else entirely.
    """
    raise NotImplementedError  # TODO(student)


def decode_sign_magnitude(raw: int, sign_bit: int) -> int:
    """Feetech signed values: bit ``sign_bit`` is the sign, the bits below it are the magnitude.

    ``Present_Load`` uses bit 10, ``Present_Velocity`` and ``Goal_Velocity`` bit 15. So raw 1036
    is 1024 + 12 = "negative 12", not "+103.6 %". Two's complement this is not.
    """
    raise NotImplementedError  # TODO(student)


# --- TODO(student): the safety logic ---------------------------------------------------------------
def clamp_goals(goals: Mapping[str, float], limits: Mapping[str, JointLimits]) -> tuple[dict[str, float], list[str]]:
    """Clamp every goal into its joint's limits.

    Returns ``(clamped goals, notes)`` where each note reads ``"<joint>: <asked> clamped to
    <value>"`` (one decimal place each) and is produced **only** for goals that actually changed.

    A joint with no configured limits is a ``SafetyError``, not a pass-through. "I did not know
    the limit so I sent it anyway" is how arms meet tables.
    """
    raise NotImplementedError  # TODO(student)


def health_problems(telemetry: Mapping[str, Telemetry], cfg: SafetyConfig) -> list[str]:
    """Everything that should stop a session, as human-readable strings (empty list = healthy).

    Check, per joint: the supply voltage inside ``cfg.voltage_window_v`` (a 12 V servo on the 5 V
    profile, or the other way round, is a wiring mistake you want to catch before torque);
    temperature at or below ``cfg.max_temperature_c``; and ``|load|`` at or below
    ``cfg.max_load_percent``. Mention the joint name and the offending number in every string so
    the message is useful on its own.
    """
    raise NotImplementedError  # TODO(student)


def safe_enable_torque(bus: ArmBus, cfg: SafetyConfig) -> dict[str, float]:
    """Enable torque without the arm jumping. Returns the pose it now holds.

    A servo whose ``Goal_Position`` register still holds yesterday's value will drive there at
    full speed the instant torque comes on. The order is the whole point:

    1. ``disable_torque()`` — limp, so nothing moves while you configure.
    2. ``read_positions()``; raise ``SafetyError`` if any joint is outside its software limits
       (a person moves it back by hand — do not "fix" it by driving there), and raise if
       ``health_problems`` finds anything.
    3. Write ``Torque_Limit``, ``Acceleration`` and ``Goal_Velocity`` for every joint.
    4. ``write_goal_positions(present)`` — give the servo nowhere to go.
    5. ``enable_torque()``, last.

    Swap steps 4 and 5 and the arm still ends up in the right place, so a test that only looks at
    the final pose passes. The tests here look at the order.
    """
    raise NotImplementedError  # TODO(student)


def move_smoothly(
    bus: ArmBus,
    goals: Mapping[str, float],
    cfg: SafetyConfig,
    rate_hz: float = 50.0,
    tolerance: float = 1.0,
    timeout_s: float = 20.0,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, float]:
    """Walk the goal toward ``goals`` in steps of at most ``cfg.max_step_deg`` per tick.

    Clamp the target with ``clamp_goals`` first and ``print("limit:", note)`` for each note.
    Then, at ``rate_hz`` starting from the present position, each tick:

    1. move each commanded value at most ``cfg.max_step_deg`` toward its target,
    2. ``write_goal_positions`` the commanded values,
    3. ``sleep(1 / rate_hz)`` — the tests pass the simulator's ``advance`` here, so this is what
       makes simulated time pass,
    4. read the positions back, and for every commanded joint read ``Present_Load``: if
       ``|load| > cfg.max_load_percent``, write the *present* positions as the goal (stop where
       you are, do not keep pushing) and raise ``SafetyError``
       ``"<joint>: load <NN> % during move — stopped at <pos>"``,
    5. return the present positions once every joint is within ``tolerance`` of its target.

    Give up after ``timeout_s`` with a ``SafetyError``: a move that never finishes means a blocked
    joint or limits that make the target unreachable, and silently looping forever hides both.

    Rate-limiting in software is defence in depth: the servo's own ``Goal_Velocity`` and
    ``Acceleration`` are the other layer, and neither is enough alone.
    """
    raise NotImplementedError  # TODO(student)


# --- given: a fake bus so none of this needs hardware ----------------------------------------------
@dataclass
class _FakeServo:
    position: float
    goal: float
    torque: bool = False
    torque_limit: int = 1000
    acceleration: int = 0
    velocity: int = 0
    temperature_c: int = 30
    blocked_at: float | None = None      # simulate an obstacle: the joint can't pass this value


class FakeArmBus:
    """Six simulated STS3215 servos, including the behaviour that hurts people.

    ``advance(dt)`` moves every torqued servo toward its goal at the ``Goal_Velocity`` limit; the
    tests pass ``bus.advance`` as ``move_smoothly``'s ``sleep`` so simulated time follows the loop.
    ``log`` records torque changes and register writes, in order. ``max_jump_deg`` records the
    largest distance any joint was thrown when torque came on.
    """

    def __init__(self, positions: Mapping[str, float] | None = None, voltage_v: float = 5.0,
                 goal_register: Mapping[str, float] | None = None) -> None:
        self.joint_names = SO101_JOINTS
        start = {j: 0.0 for j in SO101_JOINTS}
        start["gripper"] = 30.0
        start.update(positions or {})
        goals = dict(start)
        goals.update(goal_register or {})           # stale goals from a previous session
        self.servos = {j: _FakeServo(position=start[j], goal=goals[j]) for j in SO101_JOINTS}
        self.voltage_v = voltage_v
        self.log: list[str] = []
        self.max_jump_deg = 0.0

    def read_positions(self) -> dict[str, float]:
        return {j: s.position for j, s in self.servos.items()}

    def write_goal_positions(self, goals: Mapping[str, float]) -> None:
        self.log.append("goals " + ",".join(sorted(goals)))
        for j, g in goals.items():
            self.servos[j].goal = float(g)

    def enable_torque(self) -> None:
        self.log.append("torque on")
        for s in self.servos.values():
            if not s.torque:
                # a real STS3215 heads for its goal register immediately, at full speed if
                # Goal_Velocity is 0 — record how far that would throw the joint
                self.max_jump_deg = max(self.max_jump_deg, abs(s.goal - s.position))
                s.position = s.goal if s.blocked_at is None else s.position
            s.torque = True

    def disable_torque(self) -> None:
        self.log.append("torque off")
        for s in self.servos.values():
            s.torque = False

    def read_register(self, register: str, joint: str) -> int:
        s = self.servos[joint]
        if register == "Present_Load":
            if not s.torque:
                return 0
            pushing = s.blocked_at is not None and abs(s.position - s.blocked_at) < 1e-9 and (
                (s.goal - s.position) * (s.blocked_at - s.position) >= 0 and abs(s.goal - s.position) > 0.5)
            if pushing:
                return min(1000, s.torque_limit)       # pushing against the obstacle
            return 80                                  # 8 %: holding against gravity
        if register == "Present_Voltage":
            return int(round(self.voltage_v * 10))
        if register == "Present_Temperature":
            return s.temperature_c
        if register == "Present_Current":
            return 60 if s.torque else 5
        if register == "Torque_Limit":
            return s.torque_limit
        if register == "Acceleration":
            return s.acceleration
        if register == "Goal_Velocity":
            return s.velocity
        raise KeyError(register)

    def write_register(self, register: str, joint: str, value: int) -> None:
        s = self.servos[joint]
        self.log.append(f"{joint}.{register}={value}")
        if register == "Torque_Limit":
            s.torque_limit = int(value)
        elif register == "Acceleration":
            s.acceleration = int(value)
        elif register == "Goal_Velocity":
            s.velocity = int(value)
        else:
            raise KeyError(register)

    def advance(self, dt: float) -> None:
        for s in self.servos.values():
            if not s.torque:
                continue
            speed = (s.velocity or 3400) * DEG_PER_STEP          # deg/s; 0 = maximum ~300 deg/s
            step = max(-speed * dt, min(speed * dt, s.goal - s.position))
            new = s.position + step
            if s.blocked_at is not None:
                if s.position <= s.blocked_at <= new or new <= s.blocked_at <= s.position:
                    new = s.blocked_at
            s.position = new
