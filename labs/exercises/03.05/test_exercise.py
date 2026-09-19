"""Checker for 03.05 — A fake DifferentialBase that passes the conformance suite.

Run: ``python course.py check 03.05`` (or ``--solution`` to see the reference pass).

The CONFORMANCE tests below are written against the DifferentialBase contract only. They run
against your FakeBase *and* against the course simulator (SimBase), which proves the suite
describes the interface and not one implementation. Then drive_distance — business logic that
only knows the interface — must work unchanged on both, and fail safely on a broken robot.
"""

from __future__ import annotations

import math

import pytest

from robotlab.config import load_config
from robotlab.hal import FLAG_VELOCITY_MODE, FLAG_WATCHDOG, BaseState, DifferentialBase
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

CFG = load_config()
TPR = CFG.drive.ticks_per_wheel_rev
MAX_SPEED = CFG.drive.max_wheel_speed_rad_s


def make_fake(impl) -> DifferentialBase:
    return impl.FakeBase(ticks_per_rev=TPR, max_wheel_speed_rad_s=MAX_SPEED, dt=0.02, watchdog_s=0.3)


def make_sim(impl) -> DifferentialBase:
    sim = DiffDriveSim(World(), DiffDriveParams.ideal(CFG), SensorParams.ideal(CFG), pose=(0.0, 0.0, 0.0), seed=0)
    return SimBase(sim, dt=0.02, watchdog_s=0.3)


IMPLEMENTATIONS = {"your_fake": make_fake, "simbase": make_sim}


@pytest.fixture(params=list(IMPLEMENTATIONS))
def base(request, impl):
    b = IMPLEMENTATIONS[request.param](impl)
    yield b
    try:
        b.close()
    except NotImplementedError:
        pass  # student close() not written yet: the test itself is already reported as skipped


def run(base: DifferentialBase, seconds: float, command=None) -> list[BaseState]:
    """Read for `seconds` of robot time (dt = 0.02), re-sending `command` every cycle if given."""
    states = []
    for _ in range(round(seconds / 0.02)):
        if command is not None:
            command()
        states.append(base.read())
    return states


# --- CONFORMANCE: the contract every DifferentialBase must honour -------------------------------------
def test_is_a_differential_base(base):
    assert isinstance(base, DifferentialBase)


def test_read_returns_base_state_with_increasing_time(base):
    states = run(base, 0.2)
    assert all(isinstance(s, BaseState) for s in states)
    times = [s.t for s in states]
    assert all(b > a for a, b in zip(times, times[1:])), "BaseState.t must strictly increase"
    assert all(isinstance(s.left_ticks, int) and isinstance(s.right_ticks, int) for s in states)


def test_ticks_follow_the_commanded_wheel_speed(base):
    run(base, 0.5, lambda: base.set_wheel_velocity(5.0, -5.0))  # settle
    a = base.read()
    b = run(base, 1.0, lambda: base.set_wheel_velocity(5.0, -5.0))[-1]
    expected = 5.0 * (b.t - a.t) / (2 * math.pi) * TPR  # rad -> revolutions -> ticks (~1980)
    assert b.left_ticks - a.left_ticks == pytest.approx(expected, rel=0.03), "left: +5 rad/s forward"
    assert b.right_ticks - a.right_ticks == pytest.approx(-expected, rel=0.03), "right: -5 rad/s backward"
    assert b.left_rad_s == pytest.approx(5.0, abs=0.5) and b.right_rad_s == pytest.approx(-5.0, abs=0.5)


def test_velocity_mode_flag(base):
    base.set_wheel_velocity(3.0, 3.0)
    assert base.read().flags & FLAG_VELOCITY_MODE
    base.set_wheel_duty(0.3, 0.3)
    assert not base.read().flags & FLAG_VELOCITY_MODE


def test_speeds_are_clamped_to_the_max_wheel_speed(base):
    states = run(base, 1.0, lambda: base.set_wheel_velocity(100.0, -100.0))
    # The fake clamps exactly; the simulated motors can overshoot the config limit a little (the
    # no-load speed at a full battery is ~21.5 rad/s), so the contract allows 30 % margin.
    assert abs(states[-1].left_rad_s) <= MAX_SPEED * 1.3, "100 rad/s must be limited, not passed through"
    assert abs(states[-1].right_rad_s) <= MAX_SPEED * 1.3


def test_watchdog_stops_a_base_that_is_not_commanded(base):
    base.set_wheel_duty(0.5, 0.5)
    states = run(base, 1.0)  # nobody re-sends the command
    assert not states[5].flags & FLAG_WATCHDOG, "0.12 s after the command: still valid"
    assert states[-1].flags & FLAG_WATCHDOG, "1 s without a command: the watchdog must have tripped"
    assert abs(states[-1].left_rad_s) < 0.5 and abs(states[-1].right_rad_s) < 0.5, "and the wheels stopped"
    base.set_wheel_duty(0.5, 0.5)
    assert not base.read().flags & FLAG_WATCHDOG, "a new command clears the flag"


def test_stop_stops_the_wheels(base):
    run(base, 0.5, lambda: base.set_wheel_velocity(8.0, 8.0))
    base.stop()
    last = run(base, 1.0)[-1]  # a real (or simulated) wheel needs time to spin down
    assert abs(last.left_rad_s) < 0.2 and abs(last.right_rad_s) < 0.2
    ticks = (last.left_ticks, last.right_ticks)
    assert (base.read().left_ticks, base.read().right_ticks) == pytest.approx(ticks, abs=2)


def test_close_is_idempotent_and_final(base):
    base.set_wheel_velocity(2.0, 2.0)
    base.read()
    base.close()
    base.close()
    with pytest.raises(RuntimeError):
        base.read()
    with pytest.raises(RuntimeError):
        base.set_wheel_velocity(1.0, 1.0)


def test_context_manager_closes(impl):
    for make in IMPLEMENTATIONS.values():
        with make(impl) as b:
            b.read()
        with pytest.raises(RuntimeError):
            b.read()


# --- FakeBase specifics: exact, because a fake should be predictable -----------------------------------
def test_fake_one_revolution_per_second_is_exact(impl):
    fake = impl.FakeBase(ticks_per_rev=2464, dt=0.02)
    for _ in range(50):
        fake.set_wheel_velocity(2 * math.pi, 2 * math.pi)
        state = fake.read()
    assert state.t == pytest.approx(1.0)
    assert (state.left_ticks, state.right_ticks) == (2464, 2464)


def test_fake_duty_maps_linearly_to_speed(impl):
    fake = impl.FakeBase(max_wheel_speed_rad_s=17.0)
    fake.set_wheel_duty(0.5, -2.0)  # -2.0 clamps to -1.0
    state = fake.read()
    assert (state.left_rad_s, state.right_rad_s) == pytest.approx((8.5, -17.0))


def test_fake_records_commands_for_assertions(impl):
    fake = impl.FakeBase()
    fake.set_wheel_velocity(1.0, 2.0)
    fake.set_wheel_duty(0.1, 0.2)
    fake.stop()
    assert fake.commands == [("velocity", 1.0, 2.0), ("duty", 0.1, 0.2), ("stop", 0.0, 0.0)]


def test_fake_reports_configured_sensors(impl):
    state = impl.FakeBase(battery_v=11.1, range_m=0.42).read()
    assert (state.battery_v, state.range_m) == (11.1, 0.42)


def test_fake_rejects_nan_commands(impl):
    fake = impl.FakeBase()
    with pytest.raises(ValueError):
        fake.set_wheel_velocity(float("nan"), 0.0)


# --- business logic written against the interface --------------------------------------------------------
@pytest.mark.parametrize("which", list(IMPLEMENTATIONS))
@pytest.mark.parametrize("distance", [0.5, -0.3])
def test_drive_distance_on_any_base(impl, which, distance):
    base = IMPLEMENTATIONS[which](impl)
    travelled = impl.drive_distance(base, distance, 6.0, CFG.drive.wheel_radius_m, TPR)
    assert travelled == pytest.approx(distance, abs=0.02)  # 6 rad/s * 0.045 m * 0.02 s = 5.4 mm per cycle
    after = run(base, 0.2)[-1]  # 0.2 s < the 0.3 s watchdog: only stop() can have stopped it
    assert abs(after.left_rad_s) < 1.0, "drive_distance must stop the base when done (not 6 rad/s)"
    if which == "simbase":
        assert base.sim.pose.x == pytest.approx(distance, abs=0.03), "the simulated robot really moved"
    base.close()


def test_drive_distance_uses_the_interface_only(impl):
    fake = impl.FakeBase()
    impl.drive_distance(fake, 0.1, 5.0, 0.045, 2464)
    kinds = [c[0] for c in fake.commands]
    assert kinds[-1] == "stop"
    assert kinds.count("velocity") >= 2, "re-send the command every cycle (it feeds the watchdog)"


class StuckBase:
    """A robot whose wheels are blocked: commands are accepted, ticks never change."""

    def __init__(self) -> None:
        self.t = 0.0
        self.stopped = False

    def set_wheel_duty(self, left: float, right: float) -> None: ...
    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None: ...

    def stop(self) -> None:
        self.stopped = True

    def read(self) -> BaseState:
        self.t += 0.02
        return BaseState(self.t, 0, 0, 0.0, 0.0, 12.0, None, 0)

    def close(self) -> None: ...


def test_drive_distance_times_out_and_still_stops(impl):
    stuck = StuckBase()
    assert isinstance(stuck, DifferentialBase)  # structural typing: no inheritance needed
    with pytest.raises(TimeoutError):
        impl.drive_distance(stuck, 1.0, 5.0, 0.045, 2464, timeout_s=0.5)
    assert stuck.stopped, "SAFETY: stop() must run even when drive_distance raises"


def test_drive_distance_validates_arguments(impl):
    fake = impl.FakeBase()
    assert impl.drive_distance(fake, 0.0, 5.0, 0.045, 2464) == 0.0
    with pytest.raises(ValueError):
        impl.drive_distance(fake, 0.5, -5.0, 0.045, 2464)
