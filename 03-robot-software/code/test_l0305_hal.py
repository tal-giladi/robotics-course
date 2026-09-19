"""Tests for the lesson 03.05 HAL demo — no hardware, no network beyond localhost.

    python -m pytest 03-robot-software/code/test_l0305_hal.py

Only the lockstep ``sim://`` bases are used here, so the whole file runs in a couple of seconds.
The ``fake://`` path (a real SerialBase stack against a fake Pico over TCP) runs in real time and
is covered by ``labs/python/tests/test_fake_pico.py``.
"""

from __future__ import annotations

import pytest

import l0305_hal_demo as demo
from robotlab.hal import FLAG_WATCHDOG, BaseState, DifferentialBase


class StuckBase:
    """Accepts every command, never moves, always reports the same range. Structurally a base."""

    def __init__(self, range_m: float | None = 2.0) -> None:
        self.t = 0.0
        self.range_m = range_m
        self.stopped = 0
        self.commands: list[tuple[float, float]] = []

    def set_wheel_duty(self, left: float, right: float) -> None: ...

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        self.commands.append((left_rad_s, right_rad_s))

    def stop(self) -> None:
        self.stopped += 1

    def read(self) -> BaseState:
        self.t += 0.02
        return BaseState(self.t, 0, 0, 0.0, 0.0, 12.0, self.range_m, 0)

    def close(self) -> None: ...


def test_a_plain_class_is_a_differential_base() -> None:
    assert isinstance(StuckBase(), DifferentialBase)  # structural typing, no inheritance


def test_speed_limited_base_clamps_both_command_kinds() -> None:
    inner = demo.RecordingBase(StuckBase())
    limited = demo.SpeedLimitedBase(inner, max_rad_s=4.0, max_duty=0.5)
    limited.set_wheel_velocity(100.0, -100.0)
    limited.set_wheel_duty(0.9, -0.9)
    limited.set_wheel_velocity(1.0, -1.0)  # inside the limit: untouched
    assert inner.commands == [
        ("velocity", 4.0, -4.0),
        ("duty", 0.5, -0.5),
        ("velocity", 1.0, -1.0),
    ]
    assert isinstance(limited, DifferentialBase)


def test_recording_base_is_a_spy_and_passes_everything_through() -> None:
    stuck = StuckBase()
    rec = demo.RecordingBase(stuck)
    rec.set_wheel_velocity(2.0, 2.0)
    state = rec.read()
    rec.stop()
    assert rec.commands == [("velocity", 2.0, 2.0), ("stop", 0.0, 0.0)]
    assert rec.states == [state]
    assert stuck.stopped == 1


def test_decorators_forward_unknown_attributes() -> None:
    stuck = StuckBase()
    wrapped = demo.SpeedLimitedBase(demo.RecordingBase(stuck), max_rad_s=1.0)
    assert wrapped.range_m == 2.0  # reached through two __getattr__ hops
    # ...but the protocol isinstance check looks at the CLASS, so it does not see it (lesson KC 7).
    assert not isinstance(wrapped, demo.TelemetryStream)


def test_next_state_uses_read_when_the_base_does_not_stream() -> None:
    stuck = StuckBase()
    first = demo.next_state(stuck, None)
    second = demo.next_state(stuck, first)
    assert second.t > first.t


def test_approach_wall_stops_at_the_threshold_on_the_simulator() -> None:
    with demo.open_base("sim://room") as base:
        result = demo.approach_wall(base, stop_at_m=0.5, speed_rad_s=4.0)
        truth_x = base.sim.pose.x
    assert result.reason == "target"
    # threshold = 0.5 + 4.0 * 0.045 * 0.2 = 0.536 m; one 0.02 s cycle covers 3.6 mm.
    assert result.stopped_at_m == pytest.approx(0.536, abs=0.01)
    assert truth_x == pytest.approx(3.34, abs=0.05)


def test_approach_wall_is_deterministic_across_realistic_seeds() -> None:
    ranges = []
    for seed in (1, 2, 3):
        with demo.open_base(f"sim://room?realistic=1&seed={seed}") as base:
            ranges.append(demo.approach_wall(base, stop_at_m=0.5).stopped_at_m)
    assert all(r is not None and 0.50 <= r <= 0.56 for r in ranges), ranges


def test_approach_wall_treats_a_missing_reading_as_a_reason_to_stop() -> None:
    blind = StuckBase(range_m=None)
    result = demo.approach_wall(blind, stop_at_m=0.5)
    assert result.reason == "no_reading" and result.stopped_at_m is None
    assert blind.stopped == 1, "SAFETY: stop() runs on every exit path"


def test_approach_wall_times_out_and_still_stops() -> None:
    stuck = StuckBase(range_m=2.0)
    result = demo.approach_wall(stuck, stop_at_m=0.5, timeout_s=0.2)
    assert result.reason == "timeout"
    assert stuck.stopped == 1
    assert len(stuck.commands) >= 5, "a command must go out every cycle (it feeds the watchdog)"


def test_commanding_only_once_trips_the_simulated_watchdog() -> None:
    """The behaviour every rung of the fidelity ladder must reproduce (knowledge check 3)."""
    with demo.open_base("sim://room") as base:
        base.set_wheel_velocity(6.0, 6.0)
        states = [base.read() for _ in range(50)]  # 1 s without re-commanding
    assert not states[5].flags & FLAG_WATCHDOG
    assert states[-1].flags & FLAG_WATCHDOG
    assert abs(states[-1].left_rad_s) < 0.5


def test_main_runs_the_demo(capsys: pytest.CaptureFixture[str]) -> None:
    result = demo.main(["sim://room", "--stop-at", "0.8"])
    assert result.reason == "target"
    assert "sim truth" in capsys.readouterr().out
