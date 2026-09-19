"""Shared helpers for the module-09 scripts: find the repo, import exercises, pace a robot loop.

The scripts in this folder use YOUR exercise code by default (``labs/exercises/<id>/student.py``)
so you see your own implementation drive the robot. Pass ``--solution`` to use the reference.
"""

from __future__ import annotations

import importlib.util
import math
import re
import sys
from collections.abc import Callable
from pathlib import Path
from types import ModuleType

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
EXERCISES = ROOT / "labs" / "exercises"

if str(ROOT / "labs" / "python") not in sys.path:  # robotlab without `pip install -e labs/python`
    sys.path.insert(0, str(ROOT / "labs" / "python"))

from robotlab.hal import BaseState, DifferentialBase  # noqa: E402


def load_exercise(lesson_id: str, solution: bool = False) -> ModuleType:
    """Import ``student.py`` (or ``solution.py``) of ``labs/exercises/<lesson_id>``."""
    kind = "solution" if solution else "student"
    path = EXERCISES / lesson_id / f"{kind}.py"
    safe_id = re.sub(r"\W", "_", lesson_id)
    name = f"module09_{safe_id}_{kind}"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def next_state(base: DifferentialBase, previous: BaseState | None) -> BaseState:
    """The next telemetry sample.

    ``SimBase.read()`` advances simulated time by one step. ``SerialBase.read()`` returns the newest
    sample immediately, so on the real robot wait for a NEW one: the loop then runs at the
    telemetry rate (50 Hz) and never integrates the same encoder counts twice.
    """
    wait = getattr(base, "wait_for_telemetry", None)
    if wait is not None and previous is not None:
        return wait(timeout_s=1.0, newer_than=previous.t)
    return base.read()


def open_base(port: str | None, realistic: bool = True, seed: int = 0, pose=(0.0, 0.0, 0.0)):
    """A real robot on ``port`` (SerialBase), or the course simulator in an empty world."""
    if port:
        from robotlab.serial_base import SerialBase

        return SerialBase(port)
    from robotlab.config import load_config
    from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

    cfg = load_config()
    params = DiffDriveParams.realistic(cfg) if realistic else DiffDriveParams.ideal(cfg)
    sensors = SensorParams.realistic(cfg) if realistic else SensorParams.ideal(cfg)
    return SimBase(DiffDriveSim(World(), params, sensors, pose=pose, seed=seed))


class WheelOdometry:
    """Exact-arc wheel odometry with one radius per wheel (what 09.04 builds, plus 09.05's knobs).

    ``heading`` is unwrapped (it keeps counting past +-pi) so turn targets are easy; ``pose``
    reports it wrapped.
    """

    def __init__(self, radius_left: float, radius_right: float, separation: float, ticks_per_rev: int) -> None:
        self.rad_per_tick = 2.0 * math.pi / ticks_per_rev
        self.radius_left, self.radius_right, self.separation = radius_left, radius_right, separation
        self.x = self.y = self.heading = 0.0
        self.distance = 0.0  # total path length of base_link (m)
        self._last: tuple[int, int] | None = None

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, math.atan2(math.sin(self.heading), math.cos(self.heading))

    def update(self, left_ticks: int, right_ticks: int) -> tuple[float, float, float]:
        if self._last is None:
            self._last = (left_ticks, right_ticks)
            return self.pose
        dl = (left_ticks - self._last[0]) * self.rad_per_tick * self.radius_left
        dr = (right_ticks - self._last[1]) * self.rad_per_tick * self.radius_right
        self._last = (left_ticks, right_ticks)
        ds, dtheta = (dl + dr) / 2.0, (dr - dl) / self.separation
        if abs(dtheta) < 1e-9:
            self.x += ds * math.cos(self.heading)
            self.y += ds * math.sin(self.heading)
        else:
            radius = ds / dtheta
            self.x += radius * (math.sin(self.heading + dtheta) - math.sin(self.heading))
            self.y -= radius * (math.cos(self.heading + dtheta) - math.cos(self.heading))
        self.heading += dtheta
        self.distance += abs(ds)
        return self.pose


Segment = tuple[str, float]  # ("line", meters) or ("turn", radians, + = left)


def drive_segments(
    base: DifferentialBase,
    odom: WheelOdometry,
    segments: list[Segment],
    wheel_speed: float = 5.0,
    turn_wheel_speed: float = 3.0,
    on_state: Callable[[BaseState], None] | None = None,
    motor_time_constant_s: float = 0.08,
) -> BaseState:
    """Drive straight lines and turns in place, closed-loop on ``odom``; slow down near each target.

    SAFETY: this moves the robot. On hardware, clear the floor, keep the power switch in reach.
    Returns the last telemetry sample. ``on_state`` sees every sample (for logging).
    """
    state = next_state(base, None)
    odom.update(state.left_ticks, state.right_ticks)

    def step(left: float, right: float) -> None:
        nonlocal state
        base.set_wheel_velocity(left, right)
        state = next_state(base, state)
        odom.update(state.left_ticks, state.right_ticks)
        if on_state is not None:
            on_state(state)

    def coast(kind: str) -> float:
        """How far the wheels will still roll after a stop: speed x motor time constant."""
        wheel = (abs(state.left_rad_s) + abs(state.right_rad_s)) / 2.0
        travel = odom.radius_left * wheel * motor_time_constant_s
        return travel if kind == "line" else 2.0 * travel / odom.separation

    def remaining(kind: str, amount: float, x0: float, y0: float, h0: float) -> float:
        if kind == "line":  # signed distance still to go along the segment's start heading
            along = (odom.x - x0) * math.cos(h0) + (odom.y - y0) * math.sin(h0)
            return amount - along
        if kind == "turn":
            return amount - (odom.heading - h0)
        raise ValueError(f"unknown segment {kind!r}")

    for kind, amount in segments:
        x0, y0, h0 = odom.x, odom.y, odom.heading
        tolerance = 0.003 if kind == "line" else math.radians(0.5)
        for _attempt in range(4):  # drive, let the wheels coast to rest, correct any over/undershoot
            while abs(error := remaining(kind, amount, x0, y0, h0)) > tolerance / 2 + coast(kind):
                if kind == "line":
                    w = math.copysign(max(1.0, min(wheel_speed, 20.0 * abs(error))), error)
                    hold = 20.0 * (h0 - odom.heading)  # keep the heading the line started with
                    step(w - hold, w + hold)
                else:
                    w = math.copysign(max(0.4, min(turn_wheel_speed, 8.0 * abs(error))), error)
                    step(-w, w)
            for _ in range(15):  # stand still ~0.3 s: coasting ends
                step(0.0, 0.0)
            if abs(remaining(kind, amount, x0, y0, h0)) <= tolerance:
                break
    return state
