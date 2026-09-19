"""Shared helpers for the module-08 experiments: open a base, run a fixed-rate loop on the Pi, measure.

Every script in this folder runs the same way on three targets:

    python 08-control/code/p_control.py                       # the course simulator (realistic preset)
    python 08-control/code/p_control.py --fake                # a fake Pico over a socket (whole serial stack)
    python 08-control/code/p_control.py --port /dev/ttyACM0   # the real robot, WHEELS IN THE AIR

In lessons 08.01-08.06 the controller runs HERE, on the Pi, at the 50 Hz telemetry rate, and sends
open-loop duty (`set_wheel_duty`). That is deliberately the slow, laggy place for a control loop:
you can see every signal and change code in seconds. Lesson 08.12 moves the loop onto the Pico
(`labs/firmware/pico/velocity.py`, 100 Hz), which is where it belongs on a finished robot.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import re
import sys
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
EXERCISES = ROOT / "labs" / "exercises"
IMAGES = ROOT / "08-control" / "images"

if str(ROOT / "labs" / "python") not in sys.path:  # robotlab without `pip install -e labs/python`
    sys.path.insert(0, str(ROOT / "labs" / "python"))

from robotlab.config import KarmelConfig, load_config  # noqa: E402
from robotlab.hal import BaseState, DifferentialBase  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World  # noqa: E402

LOOP_HZ = 50.0  # the Pi-side loop runs at the telemetry rate
SIM_DT = 0.01  # the simulated firmware runs at 100 Hz, like the Pico

Duties = tuple[float, float]
StepFn = Callable[[float, float, BaseState, dict[str, float]], Duties]


# --- exercises ------------------------------------------------------------------------------------
def load_exercise(lesson_id: str, solution: bool = False) -> ModuleType:
    """Import ``student.py`` (or ``solution.py``) of ``labs/exercises/<lesson_id>``."""
    kind = "solution" if solution else "student"
    path = EXERCISES / lesson_id / f"{kind}.py"
    safe_id = re.sub(r"\W", "_", lesson_id)
    name = f"module08_{safe_id}_{kind}"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# --- bases ----------------------------------------------------------------------------------------
def make_sim_base(realistic: bool = True, seed: int = 0, config: KarmelConfig | None = None, **overrides: Any) -> SimBase:
    """The course simulator in an empty world, with the firmware at 100 Hz.

    ``overrides`` replace DiffDriveParams fields, e.g. ``motor_time_constant_s=0.15``.
    """
    cfg = config or load_config()
    params = DiffDriveParams.realistic(cfg) if realistic else DiffDriveParams.ideal(cfg)
    if overrides:
        from dataclasses import replace

        params = replace(params, **overrides)
    sensors = SensorParams.realistic(cfg) if realistic else SensorParams.ideal(cfg)
    return SimBase(DiffDriveSim(World(), params, sensors, seed=seed), dt=SIM_DT)


def add_target_args(parser: argparse.ArgumentParser) -> None:
    group = parser.add_argument_group("where to run")
    group.add_argument("--port", help="the real robot: serial device, e.g. /dev/ttyACM0 or COM5")
    group.add_argument("--fake", action="store_true", help="a fake Pico (simulator) behind the real serial stack")
    group.add_argument("--ideal", action="store_true", help="simulator: ideal preset instead of realistic")
    group.add_argument("--seed", type=int, default=0, help="simulator random seed")
    group.add_argument("--yes", action="store_true", help="skip the wheels-in-the-air confirmation")
    group.add_argument("--plot", help="save the plot to this PNG file")


class Target:
    """Context manager giving a DifferentialBase for the command-line options; always stops the motors."""

    def __init__(self, args: argparse.Namespace, **sim_overrides: Any) -> None:
        self.args = args
        self.sim_overrides = sim_overrides
        self.base: Any = None
        self._server: Any = None

    def __enter__(self) -> Any:
        a = self.args
        if a.port or a.fake:
            from robotlab.serial_base import SerialBase

            if a.port:
                confirm_wheels_in_the_air(a.yes)
                url = a.port
            else:
                from robotlab.fake_pico import FakePico, FakePicoServer, default_sim

                self._server = FakePicoServer(FakePico(default_sim("empty", realistic=not a.ideal, seed=a.seed)), port=0)
                self._server.start()
                url = self._server.url
            self.base = SerialBase(url)
        else:
            self.base = make_sim_base(realistic=not a.ideal, seed=a.seed, **self.sim_overrides)
        return self.base

    def __exit__(self, *exc: object) -> None:
        try:
            if self.base is not None:
                self.base.close()  # SAFETY: close() brakes the motors
        finally:
            if self._server is not None:
                self._server.stop()


def confirm_wheels_in_the_air(yes: bool) -> None:
    print("SAFETY: the wheels will spin. Put the robot on a stand with BOTH wheels off the table,")
    print("        keep a hand on the battery switch. Ctrl-C stops the motors.")
    if not yes and input("Wheels in the air? Type yes: ").strip().lower() != "yes":
        sys.exit("aborted")


def is_sim(base: Any) -> bool:
    return isinstance(base, SimBase)


# --- the fixed-rate loop ----------------------------------------------------------------------------
@dataclass
class Log:
    """Rows of named floats; ``log["t"]`` is a numpy column."""

    rows: list[dict[str, float]] = field(default_factory=list)

    def append(self, row: dict[str, float]) -> None:
        self.rows.append(row)

    def __getitem__(self, name: str) -> np.ndarray:
        return np.array([r.get(name, math.nan) for r in self.rows], dtype=float)

    def __len__(self) -> int:
        return len(self.rows)

    def save_csv(self, path: str | Path) -> None:
        names = list(dict.fromkeys(k for r in self.rows for k in r))
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=names)
            writer.writeheader()
            writer.writerows(self.rows)


def next_state(base: Any, previous: BaseState | None, rate_hz: float) -> BaseState:
    """Wait one loop period and return the newest telemetry sample.

    SimBase: step the simulated firmware (100 Hz) for one period. SerialBase: block until a sample
    at least one period newer than ``previous`` arrives (the robot's clock, not the Pi's).
    """
    period = 1.0 / rate_hz
    if is_sim(base):
        state = base.read()
        for _ in range(max(1, round(period / base.dt)) - 1):
            state = base.read()
        return state
    if previous is None:
        return base.wait_for_telemetry(timeout_s=1.0)
    state = base.wait_for_telemetry(timeout_s=1.0, newer_than=previous.t)
    while state.t - previous.t < period - 0.005:
        state = base.wait_for_telemetry(timeout_s=1.0, newer_than=state.t)
    return state


def run_loop(
    base: Any,
    step: StepFn,
    duration_s: float,
    rate_hz: float = LOOP_HZ,
    delay_steps: int = 0,
    command: str = "duty",
) -> Log:
    """Call ``step(t, dt, state, extra)`` every period and send what it returns to the motors.

    * ``step`` returns (left, right): duty -1..1 (``command="duty"``) or rad/s (``"velocity"``).
      It may put extra values to log into the ``extra`` dict (e.g. the setpoint).
    * ``delay_steps`` holds each command back for that many periods: extra dead time, like a busy
      Pi or a loop running on a laptop over Wi-Fi.
    * Every row logs: t, measured wheel speeds (firmware estimate), ticks, the duty actually sent,
      battery voltage, and in simulation the TRUE wheel speeds.
    """
    send = base.set_wheel_duty if command == "duty" else base.set_wheel_velocity
    pending: deque[Duties] = deque([(0.0, 0.0)] * delay_steps)
    log = Log()
    state = next_state(base, None, rate_hz)
    t0, previous_t = state.t, state.t - 1.0 / rate_hz
    try:
        while state.t - t0 < duration_s - 1e-9:
            dt = state.t - previous_t
            extra: dict[str, float] = {}
            wanted = step(state.t - t0, dt, state, extra)
            pending.append(wanted)
            left, right = pending.popleft()
            send(left, right)  # SAFETY: the wheels move
            row = {
                "t": state.t - t0,
                "left_rad_s": state.left_rad_s,
                "right_rad_s": state.right_rad_s,
                "left_ticks": float(state.left_ticks),
                "right_ticks": float(state.right_ticks),
                "left_cmd": left,
                "right_cmd": right,
                "battery_v": state.battery_v if state.battery_v is not None else math.nan,
            }
            if is_sim(base):
                row["left_true"], row["right_true"] = (float(w) for w in base.sim.wheel_rad_s)
            row.update(extra)
            log.append(row)
            previous_t = state.t
            state = next_state(base, state, rate_hz)
    finally:
        base.stop()
    return log


def let_wheels_stop(base: Any, seconds: float = 0.5) -> None:
    """Brake and wait (keeps the simulated watchdog quiet)."""
    base.stop()
    if is_sim(base):
        for _ in range(round(seconds / base.dt)):
            base.read()
    else:
        import time

        time.sleep(seconds)


# --- response metrics -----------------------------------------------------------------------------
def steady_value(t: np.ndarray, y: np.ndarray, last_s: float = 0.5) -> float:
    """Mean of ``y`` over the last ``last_s`` seconds."""
    return float(np.mean(y[t >= t[-1] - last_s + 1e-9]))


def settling_time(t: np.ndarray, y: np.ndarray, target: float, band: float, t_start: float = 0.0) -> float:
    """Time after ``t_start`` until ``y`` enters ``target ± band`` for good (inf if it never does)."""
    mask = t >= t_start - 1e-9
    tt, yy = t[mask], y[mask]
    outside = np.nonzero(np.abs(yy - target) > band)[0]
    if len(outside) == 0:
        return 0.0
    if outside[-1] == len(yy) - 1:
        return math.inf
    return float(tt[outside[-1] + 1] - t_start)


def overshoot_percent(y: np.ndarray, start: float, target: float) -> float:
    """How far past the target the response went, as % of the step size."""
    step = target - start
    peak = np.max(y) if step > 0 else np.min(y)
    return float(max(0.0, (peak - target) / step * 100.0))


def oscillation_std(t: np.ndarray, y: np.ndarray, last_s: float = 1.0) -> float:
    """Standard deviation over the last ``last_s`` seconds: ~0.05 rad/s = quiet, >0.5 = oscillating."""
    return float(np.std(y[t >= t[-1] - last_s + 1e-9]))


def savefig(fig: Any, path: str | None) -> None:
    if path:
        fig.savefig(path, dpi=90)
        print(f"saved {path}")


def plt_headless() -> Any:
    """matplotlib.pyplot with a non-interactive backend (works over SSH and in tests)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt
