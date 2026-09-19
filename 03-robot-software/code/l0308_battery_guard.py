"""Lesson 03.08 — one safety behaviour, written so it can be tested at every level.

    python 03-robot-software/code/l0308_battery_guard.py sim://room --seconds 6
    python 03-robot-software/code/l0308_battery_guard.py fake://room --low 12.30 --critical 12.24

The behaviour (lesson 01.14 measures the battery; this is what you *do* about it): while driving,
watch the pack voltage. Below ``low_v`` for ``consecutive`` samples in a row, warn. Below
``critical_v`` for ``consecutive`` samples, stop and stay stopped. Lose the reading entirely for
``consecutive`` samples, also stop -- a robot that cannot see its battery is not safe to drive.

The point for 03.08 is the SHAPE, not the rule:

* :class:`BatteryGuard` is a pure state machine. Feed it ``(t, volts)``, get a
  :class:`Decision`. No robot, no clock, no I/O -- so it is tested exhaustively in microseconds.
* :func:`drive_with_guard` is the thin I/O shell: read, decide, act. It knows only
  ``DifferentialBase`` (lesson 03.05), so the same code is tested on a fake, on the simulator,
  against a fake Pico over TCP, and on the robot.

That split is what makes a test pyramid possible. Logic that is tangled with I/O can only be
tested at the slowest level.

Nothing here needs hardware. Tests: test_l0308_levels.py
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import sys
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "labs" / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from robotlab.config import load_config  # noqa: E402
from robotlab.hal import BaseState, DifferentialBase  # noqa: E402

log = logging.getLogger(__name__)


class Action(Enum):
    """What the guard wants the caller to do, right now."""

    OK = "ok"
    WARN = "warn"      # keep driving, but say so (and stop starting new work)
    STOP = "stop"      # stop, and stay stopped: this decision is latched


@dataclass(frozen=True)
class Decision:
    action: Action
    reason: str
    t: float
    volts: float | None

    @property
    def should_stop(self) -> bool:
        return self.action is Action.STOP


@dataclass
class BatteryGuard:
    """Debounced, latching battery supervision. Pure logic: no I/O, no clock of its own.

    ``consecutive`` exists because a single sample means nothing: motor inrush current drops the
    pack voltage for tens of milliseconds every time you start moving (lesson 02.02). Acting on
    one sample would stop the robot every time it accelerates.

    Latching exists because a battery that recovers after the load is removed is still nearly
    empty. Once STOP is decided, only a human (a new guard) resumes driving.
    """

    low_v: float
    critical_v: float
    consecutive: int = 5
    _low_streak: int = field(default=0, init=False)
    _critical_streak: int = field(default=0, init=False)
    _missing_streak: int = field(default=0, init=False)
    _latched: str | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if not self.critical_v < self.low_v:
            raise ValueError(f"critical_v ({self.critical_v}) must be below low_v ({self.low_v})")
        if self.consecutive < 1:
            raise ValueError("consecutive must be at least 1")

    def update(self, t: float, volts: float | None) -> Decision:
        if self._latched is not None:
            return Decision(Action.STOP, self._latched, t, volts)

        if volts is None:
            self._missing_streak += 1
            self._low_streak = self._critical_streak = 0
            if self._missing_streak >= self.consecutive:
                return self._latch(t, volts, f"no battery reading for {self._missing_streak} samples")
            return Decision(Action.OK, "no reading yet", t, volts)

        self._missing_streak = 0
        if volts < self.critical_v:
            self._critical_streak += 1
            self._low_streak += 1
            if self._critical_streak >= self.consecutive:
                return self._latch(t, volts, f"{volts:.3f} V below critical {self.critical_v:.3f} V")
            return Decision(Action.WARN, "below critical, debouncing", t, volts)

        self._critical_streak = 0
        if volts < self.low_v:
            self._low_streak += 1
            if self._low_streak >= self.consecutive:
                return Decision(Action.WARN, f"{volts:.3f} V below low {self.low_v:.3f} V", t, volts)
            return Decision(Action.OK, "below low, debouncing", t, volts)

        self._low_streak = 0
        return Decision(Action.OK, "nominal", t, volts)

    def _latch(self, t: float, volts: float | None, reason: str) -> Decision:
        self._latched = reason
        return Decision(Action.STOP, reason, t, volts)

    @property
    def latched_reason(self) -> str | None:
        return self._latched


@dataclass(frozen=True)
class GuardedRun:
    """What happened during :func:`drive_with_guard`."""

    samples: int
    stopped_by_guard: bool
    reason: str | None
    warnings: int
    last_state: BaseState | None

    @property
    def ok(self) -> bool:
        return not self.stopped_by_guard


def drive_with_guard(base: DifferentialBase, guard: BatteryGuard, left_rad_s: float,
                     right_rad_s: float, max_samples: int = 500) -> GuardedRun:
    """Drive at a constant wheel speed until the guard says stop or ``max_samples`` is reached.

    The whole I/O shell: read, decide, act. ``base.stop()`` runs on EVERY exit path.
    """
    samples = 0
    warnings = 0
    state: BaseState | None = None
    reason: str | None = None
    stopped = False
    try:
        for _ in range(max_samples):
            base.set_wheel_velocity(left_rad_s, right_rad_s)  # every cycle: feeds the watchdog
            state = base.read()
            samples += 1
            decision = guard.update(state.t, state.battery_v)
            if decision.action is Action.WARN:
                warnings += 1
            elif decision.should_stop:
                reason, stopped = decision.reason, True
                break
    except BaseException:
        # SAFETY: stop on the failure path too -- but quietly. On a link that just died, stop()
        # itself raises, and a `finally: base.stop()` would replace "no telemetry for 0.2 s" with
        # "not connected" and throw away the only useful error. The firmware watchdog is the
        # backstop when even this cannot get through.
        stop_quietly(base)
        raise
    base.stop()  # normal exit: a failure to stop here IS the news, so let it propagate
    return GuardedRun(samples, stopped, reason, warnings, state)


def stop_quietly(base: DifferentialBase) -> None:
    """Best-effort stop for an exception path: never let cleanup mask the original error."""
    try:
        base.stop()
    except Exception:  # noqa: BLE001 — deliberate: the caller is already handling something worse
        log.warning("could not stop the base while unwinding", exc_info=True)


@dataclass
class FailingBatteryBase:
    """A ``DifferentialBase`` decorator whose battery sensor dies after ``after`` reads.

    A test double for a failure you cannot ask a real robot to produce on demand (lesson 03.05:
    a decorator is still a base, because the protocol is structural).
    """

    inner: DifferentialBase
    after: int
    _reads: int = field(default=0, init=False)

    def set_wheel_duty(self, left: float, right: float) -> None:
        self.inner.set_wheel_duty(left, right)

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        self.inner.set_wheel_velocity(left_rad_s, right_rad_s)

    def stop(self) -> None:
        self.inner.stop()

    def read(self) -> BaseState:
        state = self.inner.read()
        self._reads += 1
        if self._reads > self.after:
            return dataclasses.replace(state, battery_v=None)
        return state

    def close(self) -> None:
        self.inner.close()

    def __getattr__(self, name: str):
        return getattr(self.inner, name)


def main(argv: list[str] | None = None) -> int:
    from l0305_hal_demo import open_base  # the composition root from lesson 03.05

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("url", nargs="?", default="sim://room", help="sim://, fake://, socket:// or a device")
    parser.add_argument("--low", type=float, help="warn below this pack voltage (default: battery.low_warning_v)")
    parser.add_argument("--critical", type=float, help="stop below this (default: battery.cutoff_v)")
    parser.add_argument("--consecutive", type=int, default=5)
    parser.add_argument("--speed", type=float, default=8.0, help="wheel speed, rad/s")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--battery-fails-after", type=int, metavar="N",
                        help="simulate a dead battery sensor after N reads")
    args = parser.parse_args(argv)

    battery = load_config().battery
    guard = BatteryGuard(args.low if args.low is not None else battery.low_warning_v,
                         args.critical if args.critical is not None else battery.cutoff_v,
                         args.consecutive)
    with open_base(args.url) as raw:
        base: DifferentialBase = raw if args.battery_fails_after is None else FailingBatteryBase(raw, args.battery_fails_after)
        run = drive_with_guard(base, guard, args.speed, args.speed, max_samples=int(args.seconds / 0.02))
    volts = None if run.last_state is None else run.last_state.battery_v
    print(f"{args.url}: samples={run.samples} warnings={run.warnings} "
          f"stopped_by_guard={run.stopped_by_guard} reason={run.reason} last_battery={volts}")
    return 1 if run.stopped_by_guard else 0


if __name__ == "__main__":
    raise SystemExit(main())
