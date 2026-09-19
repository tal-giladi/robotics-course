"""Lesson 03.08 — ONE behaviour (the battery guard), tested at every level of the pyramid.

    python -m pytest 03-robot-software/code/test_l0308_levels.py -v --durations=0

Read this file top to bottom: each class is one rung, and the classes get slower and less
deterministic as you go down. The last one needs a robot and skips itself without one.

    Level 0  pure logic          BatteryGuard alone                  microseconds, exhaustive
    Level 1  unit with a fake    drive_with_guard on FakeBase        microseconds, exact
    Level 2  contract            the same assertions on both bases   milliseconds
    Level 3  simulation          SimBase + ground truth              ~1 s, physical tolerances
    Level 4  HIL-lite            SerialBase -> fake Pico over TCP    seconds, real time
    Level 5  hardware            the robot                           skipped unless KARMEL_PORT

The rule the file demonstrates: a bug should be caught by the HIGHEST class in this list that
can see it, because those tests are the ones you can afford to run on every save.
"""

from __future__ import annotations

import math
import os
from typing import Iterator

import pytest

from l0308_battery_guard import Action, BatteryGuard, FailingBatteryBase, drive_with_guard
from robotlab.config import load_config
from robotlab.hal import FLAG_VELOCITY_MODE, BaseState, DifferentialBase

CFG = load_config()
TPR = CFG.drive.ticks_per_wheel_rev


# =====================================================================================================
# Level 0 — pure logic. No robot, no clock, no I/O. Exhaustive, and it costs nothing to run.
# =====================================================================================================
class TestLevel0PureLogic:
    def test_nominal_voltage_is_ok(self) -> None:
        guard = BatteryGuard(low_v=10.5, critical_v=9.9, consecutive=3)
        for i in range(100):
            assert guard.update(0.02 * i, 11.5).action is Action.OK

    def test_a_single_dip_is_debounced_away(self) -> None:
        """Motor inrush drops the pack for one sample. Stopping for that would be a bug."""
        guard = BatteryGuard(low_v=10.5, critical_v=9.9, consecutive=3)
        guard.update(0.00, 11.0)
        assert guard.update(0.02, 9.0).action is Action.WARN   # below critical, but only once
        assert guard.update(0.04, 11.0).action is Action.OK    # recovered: streak reset
        assert guard.latched_reason is None

    def test_three_in_a_row_below_critical_latches_stop(self) -> None:
        guard = BatteryGuard(low_v=10.5, critical_v=9.9, consecutive=3)
        actions = [guard.update(0.02 * i, 9.5).action for i in range(4)]
        assert actions == [Action.WARN, Action.WARN, Action.STOP, Action.STOP]
        assert "below critical" in (guard.latched_reason or "")

    def test_stop_is_latched_even_when_the_battery_recovers(self) -> None:
        """A pack that bounces back once the load is removed is still nearly empty."""
        guard = BatteryGuard(low_v=10.5, critical_v=9.9, consecutive=2)
        for _ in range(2):
            guard.update(0.0, 9.0)
        assert guard.update(1.0, 12.6).action is Action.STOP

    def test_low_but_not_critical_warns_forever_without_stopping(self) -> None:
        guard = BatteryGuard(low_v=10.5, critical_v=9.9, consecutive=2)
        actions = [guard.update(0.02 * i, 10.2).action for i in range(10)]
        assert actions[0] is Action.OK and set(actions[1:]) == {Action.WARN}
        assert guard.latched_reason is None

    def test_losing_the_reading_stops_the_robot(self) -> None:
        """Unknown is not safe. A robot that cannot see its battery does not drive."""
        guard = BatteryGuard(low_v=10.5, critical_v=9.9, consecutive=3)
        actions = [guard.update(0.02 * i, None).action for i in range(3)]
        assert actions == [Action.OK, Action.OK, Action.STOP]

    def test_a_reading_that_comes_back_resets_the_missing_streak(self) -> None:
        guard = BatteryGuard(low_v=10.5, critical_v=9.9, consecutive=3)
        guard.update(0.00, None)
        guard.update(0.02, None)
        assert guard.update(0.04, 11.5).action is Action.OK
        assert guard.update(0.06, None).action is Action.OK, "the streak restarted at 1"

    def test_thresholds_are_validated(self) -> None:
        with pytest.raises(ValueError):
            BatteryGuard(low_v=9.9, critical_v=10.5)          # the wrong way round
        with pytest.raises(ValueError):
            BatteryGuard(low_v=10.5, critical_v=9.9, consecutive=0)

    def test_the_boundary_is_strict(self) -> None:
        """Exactly at the threshold is NOT below it — a test that pins down one character of code."""
        guard = BatteryGuard(low_v=10.5, critical_v=9.9, consecutive=1)
        assert guard.update(0.0, 10.5).action is Action.OK
        assert guard.update(0.02, 10.4999).action is Action.WARN


# =====================================================================================================
# Level 1 — unit tests with a fake robot. Still microseconds, still exact, now exercising the
# I/O shell (drive_with_guard) as well as the logic.
# =====================================================================================================
class ScriptedBase:
    """The smallest possible DifferentialBase: ideal wheels and a scripted battery."""

    def __init__(self, volts: list[float | None], dt: float = 0.02) -> None:
        self.volts = volts
        self.dt = dt
        self.t = 0.0
        self.angle = 0.0
        self.speed = 0.0
        self.index = 0
        self.stops = 0
        self.commands: list[tuple[float, float]] = []

    def set_wheel_duty(self, left: float, right: float) -> None:
        self.speed = left * CFG.drive.max_wheel_speed_rad_s

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        self.commands.append((left_rad_s, right_rad_s))
        self.speed = left_rad_s

    def stop(self) -> None:
        self.stops += 1
        self.speed = 0.0

    def read(self) -> BaseState:
        self.t += self.dt
        self.angle += self.speed * self.dt
        volts = self.volts[min(self.index, len(self.volts) - 1)]
        self.index += 1
        ticks = round(self.angle / (2 * math.pi) * TPR)
        return BaseState(self.t, ticks, ticks, self.speed, self.speed, volts, None,
                         FLAG_VELOCITY_MODE if self.speed else 0)

    def close(self) -> None:
        self.stop()


class TestLevel1UnitWithAFake:
    def test_a_healthy_battery_runs_to_the_end(self) -> None:
        base = ScriptedBase([11.5])
        run = drive_with_guard(base, BatteryGuard(10.5, 9.9, 3), 6.0, 6.0, max_samples=50)
        assert run.ok and run.samples == 50 and run.reason is None
        assert base.stops == 1, "stop() runs once, at the end"

    def test_the_guard_stops_the_robot_and_says_why(self) -> None:
        base = ScriptedBase([11.5] * 10 + [9.0])
        run = drive_with_guard(base, BatteryGuard(10.5, 9.9, 3), 6.0, 6.0, max_samples=200)
        assert run.stopped_by_guard and run.samples == 13, "10 healthy + 3 to debounce"
        assert "below critical" in (run.reason or "")
        assert base.stops == 1 and base.speed == 0.0

    def test_stop_runs_even_when_the_base_raises(self) -> None:
        class Exploding(ScriptedBase):
            def read(self) -> BaseState:
                raise RuntimeError("cable pulled")

        base = Exploding([11.5])
        with pytest.raises(RuntimeError):
            drive_with_guard(base, BatteryGuard(10.5, 9.9, 3), 6.0, 6.0)
        assert base.stops == 1, "SAFETY: try/finally, not a happy-path stop()"

    def test_a_command_goes_out_every_cycle(self) -> None:
        """The firmware watchdog is fed by commands, not by good intentions (lesson 03.04)."""
        base = ScriptedBase([11.5])
        drive_with_guard(base, BatteryGuard(10.5, 9.9, 3), 6.0, 6.0, max_samples=30)
        assert len(base.commands) == 30

    def test_a_dead_battery_sensor_stops_the_robot(self) -> None:
        base = FailingBatteryBase(ScriptedBase([11.5]), after=10)
        run = drive_with_guard(base, BatteryGuard(10.5, 9.9, 3), 6.0, 6.0, max_samples=200)
        assert run.stopped_by_guard and "no battery reading" in (run.reason or "")
        assert run.samples == 13


# =====================================================================================================
# Level 2 — contract tests: the SAME assertions against two implementations. If one passes and the
# other fails, either the test describes an implementation or an implementation is lying.
# =====================================================================================================
def make_scripted() -> DifferentialBase:
    return ScriptedBase([11.5])


def make_sim() -> DifferentialBase:
    from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

    sim = DiffDriveSim(World.rectangle_room(6.0, 3.0), DiffDriveParams.ideal(CFG),
                       SensorParams.ideal(CFG), pose=(1.0, 1.5, 0.0), seed=5)
    return SimBase(sim, dt=0.02, watchdog_s=0.3)


@pytest.fixture(params=["scripted", "simbase"])
def base(request: pytest.FixtureRequest) -> Iterator[DifferentialBase]:
    b = {"scripted": make_scripted, "simbase": make_sim}[request.param]()
    yield b
    b.close()


class TestLevel2Contract:
    def test_a_healthy_run_finishes_on_any_base(self, base: DifferentialBase) -> None:
        run = drive_with_guard(base, BatteryGuard(10.5, 9.9, 5), 6.0, 6.0, max_samples=100)
        assert run.ok and run.samples == 100
        assert run.last_state is not None and run.last_state.left_ticks > 0, "it really drove"

    def test_a_dead_sensor_stops_any_base(self, base: DifferentialBase) -> None:
        run = drive_with_guard(FailingBatteryBase(base, after=20), BatteryGuard(10.5, 9.9, 5),
                               6.0, 6.0, max_samples=200)
        assert run.stopped_by_guard and run.samples == 25

    def test_every_base_reports_velocity_mode_while_driving(self, base: DifferentialBase) -> None:
        run = drive_with_guard(base, BatteryGuard(10.5, 9.9, 5), 6.0, 6.0, max_samples=30)
        assert run.last_state is not None and run.last_state.flags & FLAG_VELOCITY_MODE


# =====================================================================================================
# Level 3 — simulation against ground truth. Seconds, and the tolerances describe PHYSICS.
# =====================================================================================================
class TestLevel3Simulation:
    def test_the_robot_really_stops_where_the_guard_said(self) -> None:
        base = make_sim()
        guarded = FailingBatteryBase(base, after=50)          # sensor dies at t = 1.0 s
        run = drive_with_guard(guarded, BatteryGuard(10.5, 9.9, 5), 8.0, 8.0, max_samples=300)
        x_at_stop = base.sim.pose.x
        for _ in range(50):                                   # 1 s of simulated time, uncommanded
            base.read()
        assert run.stopped_by_guard and run.samples == 55
        # 8 rad/s x 0.045 m = 0.36 m/s; the motor time constant is 0.08 s, so the robot coasts a
        # few centimetres after stop(). It must NOT keep driving.
        assert base.sim.pose.x - x_at_stop < 0.05, "stop() must actually stop the wheels"
        base.close()

    def test_the_guard_does_not_fire_on_a_good_simulated_battery(self) -> None:
        base = make_sim()
        run = drive_with_guard(base, BatteryGuard(CFG.battery.low_warning_v, CFG.battery.cutoff_v, 5),
                               8.0, 8.0, max_samples=250)
        assert run.ok and run.warnings == 0
        assert run.last_state is not None and run.last_state.battery_v is not None
        assert run.last_state.battery_v > CFG.battery.low_warning_v
        base.close()

    def test_the_result_is_identical_for_the_same_seed(self) -> None:
        """Determinism is a feature you have to test, or tolerances quietly hide regressions."""
        results = []
        for _ in range(2):
            base = make_sim()
            drive_with_guard(base, BatteryGuard(10.5, 9.9, 5), 8.0, 8.0, max_samples=100)
            results.append((base.sim.pose.x, base.sim.pose.y, base.sim.pose.theta))
            base.close()
        assert results[0] == results[1]


# =====================================================================================================
# Level 4 — HIL-lite: the REAL host stack (framing, checksums, acks, reader thread, keep-alive)
# against a fake Pico over TCP. Real time, so these are the expensive ones.
# =====================================================================================================
class TestLevel4FakePicoOverTcp:
    def test_the_guard_works_through_the_whole_serial_stack(self) -> None:
        from robotlab.fake_pico import FakePico, FakePicoServer, default_sim
        from robotlab.serial_base import SerialBase

        with FakePicoServer(FakePico(default_sim("room")), port=0) as server:
            with SerialBase(server.url) as base:
                run = drive_with_guard(FailingBatteryBase(base, after=20),
                                       BatteryGuard(10.5, 9.9, 5), 6.0, 6.0, max_samples=100)
        assert run.stopped_by_guard and "no battery reading" in (run.reason or "")
        # Unlike SimBase, read() here returns the NEWEST sample and may repeat it, so the sample
        # count is a property of wall-clock timing, not of the simulation. Assert the behaviour,
        # not the count (lesson 03.04).
        assert 20 < run.samples <= 100

    def test_a_disconnected_robot_is_an_error_not_a_silent_stop(self) -> None:
        """A failure you cannot stage on the simulator at all, and must never fail silently."""
        from robotlab.fake_pico import FakePico, FakePicoServer, default_sim
        from robotlab.serial_base import SerialBase, SerialBaseError

        with FakePicoServer(FakePico(default_sim("room")), port=0) as server:
            base = SerialBase(server.url, max_telemetry_age_s=0.2, reconnect=False)
            try:
                server.disconnect_client()                    # "the USB cable was pulled"
                with pytest.raises(SerialBaseError) as excinfo:
                    drive_with_guard(base, BatteryGuard(10.5, 9.9, 5), 6.0, 6.0, max_samples=100)
                # Which SerialBaseError depends on whether the write or the read notices first —
                # NotConnectedError from set_wheel_velocity, or TelemetryTimeoutError from read().
                # Assert the contract ("it raises, names the port, and the link is down"), not
                # the race between the two.
                assert server.url in str(excinfo.value)
                assert not base.connected
            finally:
                base.close()


# =====================================================================================================
# Level 5 — the robot. Skipped everywhere except on a bench with a robot on a stand.
#     KARMEL_PORT=/dev/karmel python -m pytest 03-robot-software/code/test_l0308_levels.py -k Level5
# =====================================================================================================
@pytest.mark.skipif(not os.environ.get("KARMEL_PORT"),
                    reason="hardware-in-the-loop: set KARMEL_PORT (wheels OFF THE GROUND)")
class TestLevel5Hardware:
    def test_the_real_robot_reports_a_plausible_battery(self) -> None:
        from robotlab.serial_base import SerialBase

        with SerialBase(os.environ["KARMEL_PORT"]) as base:
            state = base.read()
        assert state.battery_v is not None, "no battery reading from the real robot"
        assert CFG.battery.cutoff_v < state.battery_v < CFG.battery.full_v + 0.3

    def test_the_guard_stops_the_real_robot_when_the_sensor_dies(self) -> None:
        """SAFETY: wheels off the ground. The robot turns its wheels for about half a second."""
        from robotlab.serial_base import SerialBase

        with SerialBase(os.environ["KARMEL_PORT"]) as base:
            run = drive_with_guard(FailingBatteryBase(base, after=20),
                                   BatteryGuard(CFG.battery.low_warning_v, CFG.battery.cutoff_v, 5),
                                   3.0, 3.0, max_samples=100)
            assert run.stopped_by_guard
            settled = [base.read() for _ in range(10)]
        assert abs(settled[-1].left_rad_s) < 0.5, "the wheels must have stopped"
