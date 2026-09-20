"""14.03 — reference solution. Read it after you have tried ``student.py`` yourself."""

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


# --- solution: the four register conversions -------------------------------------------------------
def speed_register(deg_per_s: float) -> int:
    if deg_per_s <= 0:
        raise ValueError("speed limit must be positive (0 in the register means full speed)")
    return max(1, int(round(deg_per_s / DEG_PER_STEP)))


def acceleration_register(deg_per_s2: float) -> int:
    if deg_per_s2 <= 0:
        raise ValueError("acceleration must be positive (0 in the register means maximum)")
    steps_s2 = deg_per_s2 / DEG_PER_STEP
    return int(min(254, max(1, round(steps_s2 / ACCEL_UNIT_STEPS_S2))))


def torque_register(percent_of_stall: float) -> int:
    if not 0 <= percent_of_stall <= 100:
        raise ValueError("torque limit is a percentage 0..100")
    return int(round(percent_of_stall * 10))


def decode_sign_magnitude(raw: int, sign_bit: int) -> int:
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if raw & (1 << sign_bit) else magnitude


# --- solution: the safety logic ---------------------------------------------------------------------
def clamp_goals(goals: Mapping[str, float], limits: Mapping[str, JointLimits]) -> tuple[dict[str, float], list[str]]:
    clamped, notes = {}, []
    for j, g in goals.items():
        if j not in limits:
            raise SafetyError(f"no software limits configured for {j}")
        c = limits[j].clamp(g)
        if c != g:
            notes.append(f"{j}: {g:.1f} clamped to {c:.1f}")
        clamped[j] = c
    return clamped, notes


def health_problems(telemetry: Mapping[str, Telemetry], cfg: SafetyConfig) -> list[str]:
    problems = []
    lo, hi = cfg.voltage_window_v
    for j, t in telemetry.items():
        if not lo <= t.voltage_v <= hi:
            problems.append(f"{j}: supply {t.voltage_v:.1f} V outside {lo}-{hi} V (wrong PSU or servo version?)")
        if t.temperature_c > cfg.max_temperature_c:
            problems.append(f"{j}: {t.temperature_c} °C > {cfg.max_temperature_c} °C — let it cool")
        if abs(t.load_percent) > cfg.max_load_percent:
            problems.append(f"{j}: load {t.load_percent:.0f} % > {cfg.max_load_percent:.0f} % — blocked or overloaded")
    return problems


def safe_enable_torque(bus: ArmBus, cfg: SafetyConfig) -> dict[str, float]:
    bus.disable_torque()                                     # 1. limp first
    present = bus.read_positions()                           # 2. where are we, and is it sane?
    outside = [f"{j} at {present[j]:.1f} (limits {cfg.limits[j].lower}..{cfg.limits[j].upper})"
               for j in bus.joint_names if not cfg.limits[j].contains(present[j])]
    if outside:
        raise SafetyError("refusing to enable torque, joints outside limits: " + "; ".join(outside))
    problems = health_problems(read_telemetry(bus), cfg)
    if problems:
        raise SafetyError("refusing to enable torque: " + "; ".join(problems))
    for j in bus.joint_names:                                # 3. limits into the servos
        bus.write_register("Torque_Limit", j, torque_register(cfg.torque_percent))
        bus.write_register("Acceleration", j, acceleration_register(cfg.accel_deg_s2))
        bus.write_register("Goal_Velocity", j, speed_register(cfg.speed_deg_s))
    bus.write_goal_positions(present)                        # 4. nowhere to go
    bus.enable_torque()                                      # 5. only now
    return present


def move_smoothly(
    bus: ArmBus,
    goals: Mapping[str, float],
    cfg: SafetyConfig,
    rate_hz: float = 50.0,
    tolerance: float = 1.0,
    timeout_s: float = 20.0,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, float]:
    target, notes = clamp_goals(goals, cfg.limits)
    for note in notes:
        print("limit:", note)
    command = bus.read_positions()
    dt = 1.0 / rate_hz
    for _ in range(int(timeout_s * rate_hz)):
        for j, g in target.items():
            err = g - command[j]
            command[j] += max(-cfg.max_step_deg, min(cfg.max_step_deg, err))
        bus.write_goal_positions({j: command[j] for j in target})
        sleep(dt)
        present = bus.read_positions()
        for j in target:
            load = bus.read_register("Present_Load", j) / 10.0
            if abs(load) > cfg.max_load_percent:
                bus.write_goal_positions({k: present[k] for k in target})   # stop where we are
                raise SafetyError(f"{j}: load {load:.0f} % during move — stopped at {present[j]:.1f}")
        if all(abs(target[j] - present[j]) <= tolerance for j in target):
            return present
    raise SafetyError(f"move did not finish within {timeout_s} s (blocked joint or limits too tight?)")


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
