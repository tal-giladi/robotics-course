"""One behaviour, three robots: dependency injection with a composition root and decorators.

Lesson 03.05 (hardware abstraction). `approach_wall` only knows `DifferentialBase`. Where the base
comes from is decided in ONE place, `open_base(url)`, from a string you can put in config or on
the command line:

    sim://apartment?realistic=1&seed=3     the course simulator (lockstep, instant)
    fake://room                            a fake Pico over TCP in this process (real SerialBase stack)
    socket://localhost:5760                a fake Pico you started: python -m robotlab.fake_pico
    /dev/karmel  (or COM5)                 the real robot

    python 03-robot-software/code/l0305_hal_demo.py sim://room
    python 03-robot-software/code/l0305_hal_demo.py fake://room --stop-at 0.5
    python 03-robot-software/code/l0305_hal_demo.py /dev/karmel --stop-at 0.5 --speed 3

SAFETY (real robot): the robot drives forward until the front sensor reads --stop-at metres.
Aim it at a soft obstacle, start slow, hand near the battery switch.
"""

from __future__ import annotations

import argparse
import contextlib
import math
import sys
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol, runtime_checkable
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "labs" / "python"))

from robotlab.hal import BaseState, DifferentialBase  # noqa: E402


# --- an optional capability, as its own small protocol ------------------------------------------------------
@runtime_checkable
class TelemetryStream(Protocol):
    """Bases that can block until a NEW sample arrives (SerialBase). SimBase doesn't need it:
    each of its read() calls already advances time by one step."""

    def wait_for_telemetry(self, timeout_s: float = 1.0, newer_than: float | None = None) -> BaseState: ...


def next_state(base: DifferentialBase, previous: BaseState | None) -> BaseState:
    """The next sample, exactly once: wait for new telemetry if the base streams it, else read()."""
    if previous is not None and isinstance(base, TelemetryStream):
        return base.wait_for_telemetry(timeout_s=0.5, newer_than=previous.t)
    return base.read()


# --- decorators: still DifferentialBase, add one responsibility each -----------------------------------------
class SpeedLimitedBase:
    """Clamp wheel speeds and duties before they reach the wrapped base (e.g. 'demo mode')."""

    def __init__(self, inner: DifferentialBase, max_rad_s: float, max_duty: float = 1.0) -> None:
        self.inner, self.max_rad_s, self.max_duty = inner, max_rad_s, max_duty

    def set_wheel_duty(self, left: float, right: float) -> None:
        clamp = lambda d: max(-self.max_duty, min(self.max_duty, d))  # noqa: E731
        self.inner.set_wheel_duty(clamp(left), clamp(right))

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        clamp = lambda w: max(-self.max_rad_s, min(self.max_rad_s, w))  # noqa: E731
        self.inner.set_wheel_velocity(clamp(left_rad_s), clamp(right_rad_s))

    def stop(self) -> None:
        self.inner.stop()

    def read(self) -> BaseState:
        return self.inner.read()

    def close(self) -> None:
        self.inner.close()

    def __getattr__(self, name: str):  # pass optional capabilities (wait_for_telemetry, sim, scan) through
        return getattr(self.inner, name)


@dataclass
class RecordingBase:
    """Record every command and state that passes through (a flight recorder / test spy)."""

    inner: DifferentialBase
    commands: list[tuple[str, float, float]] = field(default_factory=list)
    states: list[BaseState] = field(default_factory=list)

    def set_wheel_duty(self, left: float, right: float) -> None:
        self.commands.append(("duty", left, right))
        self.inner.set_wheel_duty(left, right)

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        self.commands.append(("velocity", left_rad_s, right_rad_s))
        self.inner.set_wheel_velocity(left_rad_s, right_rad_s)

    def stop(self) -> None:
        self.commands.append(("stop", 0.0, 0.0))
        self.inner.stop()

    def read(self) -> BaseState:
        state = self.inner.read()
        self.states.append(state)
        return state

    def close(self) -> None:
        self.inner.close()

    def __getattr__(self, name: str):
        return getattr(self.inner, name)


# --- the composition root -----------------------------------------------------------------------------
@contextlib.contextmanager
def open_base(url: str) -> Iterator[DifferentialBase]:
    """Build the right DifferentialBase for ``url`` and always close it (and any fake server)."""
    parsed = urlparse(url)
    options = {k: v[-1] for k, v in parse_qs(parsed.query).items()}
    realistic = options.get("realistic", "0") == "1"
    seed = int(options.get("seed", "0"))
    world = parsed.netloc or "room"
    if parsed.scheme == "sim":
        from robotlab.fake_pico import default_sim
        from robotlab.sim import SimBase

        base: DifferentialBase = SimBase(default_sim(world, realistic=realistic, seed=seed), dt=0.02)
        server = None
    else:
        from robotlab.serial_base import SerialBase

        server = None
        port = url
        if parsed.scheme == "fake":
            from robotlab.fake_pico import FakePico, FakePicoServer, default_sim

            server = FakePicoServer(FakePico(default_sim(world, realistic=realistic, seed=seed)), port=0).start()
            port = server.url
        base = SerialBase(port)
    try:
        yield base
    finally:
        base.close()  # SAFETY: close() stops the motors
        if server is not None:
            server.stop()


# --- business logic: knows only the interface ---------------------------------------------------------------
@dataclass(frozen=True)
class ApproachResult:
    stopped_at_m: float | None  # last front range reading
    driven_s: float  # robot time spent driving
    reason: str  # "target", "no_reading" or "timeout"


def approach_wall(base: DifferentialBase, stop_at_m: float, speed_rad_s: float = 4.0,
                  reaction_s: float = 0.2, wheel_radius_m: float = 0.045, timeout_s: float = 20.0) -> ApproachResult:
    """Drive straight until the front range is within stop_at_m (+ the distance covered while reacting)."""
    threshold = stop_at_m + speed_rad_s * wheel_radius_m * reaction_s
    state = next_state(base, None)
    start = state.t
    try:
        while True:
            if state.range_m is None:
                return ApproachResult(None, state.t - start, "no_reading")  # unknown is not free
            if state.range_m <= threshold:
                return ApproachResult(state.range_m, state.t - start, "target")
            if state.t - start > timeout_s:
                return ApproachResult(state.range_m, state.t - start, "timeout")
            base.set_wheel_velocity(speed_rad_s, speed_rad_s)
            state = next_state(base, state)
    finally:
        base.stop()


def main(argv: list[str] | None = None) -> ApproachResult:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("url", nargs="?", default="sim://room")
    parser.add_argument("--stop-at", type=float, default=0.5, help="stop this far from the wall, m")
    parser.add_argument("--speed", type=float, default=4.0, help="wheel speed, rad/s")
    parser.add_argument("--limit", type=float, help="wrap the base in SpeedLimitedBase(max rad/s)")
    args = parser.parse_args(argv)
    with open_base(args.url) as raw:
        base: DifferentialBase = SpeedLimitedBase(raw, args.limit) if args.limit else raw
        result = approach_wall(base, args.stop_at, args.speed)
        truth = getattr(getattr(raw, "sim", None), "pose", None)
    extra = f"  (sim truth x={truth.x:.3f} m)" if truth is not None else ""
    print(f"{args.url}: {result.reason}, range {result.stopped_at_m} m after {result.driven_s:.2f} s{extra}")
    return result


_ = math

if __name__ == "__main__":
    main()
