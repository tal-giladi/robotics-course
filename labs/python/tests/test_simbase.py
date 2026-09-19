from __future__ import annotations

import time

import pytest

from robotlab.hal import FLAG_LOW_BATTERY, FLAG_VELOCITY_MODE, FLAG_WATCHDOG, BaseState, DifferentialBase
from robotlab.sim import DiffDriveSim, SimBase, World


def make_base(ideal, **kwargs) -> SimBase:
    return SimBase(DiffDriveSim(World.apartment(), *ideal, pose=(1.0, 1.3, 0.0), seed=0), **kwargs)


def test_implements_protocol(ideal):
    assert isinstance(make_base(ideal), DifferentialBase)


def test_protocol_flag_values():
    assert (FLAG_WATCHDOG, FLAG_LOW_BATTERY, FLAG_VELOCITY_MODE) == (1, 2, 8)


def test_lockstep_advances_dt_per_read(ideal):
    base = make_base(ideal, dt=0.05)
    times = [base.read().t for _ in range(4)]
    assert times == pytest.approx([0.05, 0.10, 0.15, 0.20])


def test_read_reports_telemetry(ideal):
    base = make_base(ideal)
    for _ in range(50):
        base.set_wheel_velocity(8.0, 8.0)
        state = base.read()
    assert isinstance(state, BaseState)
    assert state.left_ticks > 0 and state.right_ticks > 0
    assert state.left_rad_s == pytest.approx(8.0, abs=1.0)
    assert state.flags & FLAG_VELOCITY_MODE
    assert state.battery_v is not None and state.battery_v > 10.0
    assert state.range_m is not None and 0.0 < state.range_m < 4.0
    assert base.sim.pose.x > 1.0  # ground truth available


def test_duty_mode_clears_velocity_flag(ideal):
    base = make_base(ideal)
    base.set_wheel_velocity(3.0, 3.0)
    base.read()
    base.set_wheel_duty(0.5, 0.5)
    assert not base.read().flags & FLAG_VELOCITY_MODE


def test_watchdog_stops_motors(ideal):
    base = make_base(ideal)
    base.set_wheel_duty(0.6, 0.6)
    flags = [base.read().flags for _ in range(40)]  # 0.8 s without a new command
    assert not flags[10] & FLAG_WATCHDOG
    assert flags[-1] & FLAG_WATCHDOG
    x = base.sim.pose.x
    for _ in range(25):
        base.read()
    assert base.sim.pose.x == pytest.approx(x, abs=1e-3)  # coasted to a stop
    base.set_wheel_duty(0.6, 0.6)
    assert not base.read().flags & FLAG_WATCHDOG


def test_watchdog_can_be_disabled(ideal):
    base = make_base(ideal, watchdog_s=None)
    base.set_wheel_duty(0.6, 0.6)
    for _ in range(100):
        state = base.read()
    assert not state.flags & FLAG_WATCHDOG and base.sim.wheel_rad_s[0] > 1.0


def test_low_battery_flag(ideal):
    import dataclasses

    params = dataclasses.replace(ideal[0], battery_initial_soc=0.1)
    base = SimBase(DiffDriveSim(World(), params, ideal[1]))
    assert base.read().flags & FLAG_LOW_BATTERY


def test_extra_sensors(ideal):
    base = make_base(ideal)
    base.read()
    assert len(base.scan().ranges) == base.sim.sensor_params.lidar_samples
    assert isinstance(base.gyro_z(), float)
    assert isinstance(base.landmarks(), list)
    assert base.true_pose == base.sim.pose


def test_close_stops_and_rejects_commands(ideal):
    base = make_base(ideal)
    base.set_wheel_duty(0.5, 0.5)
    base.read()
    base.close()
    assert list(base.sim.duty) == [0.0, 0.0]
    with pytest.raises(RuntimeError):
        base.read()
    base.close()  # idempotent


def test_context_manager(ideal):
    with make_base(ideal) as base:
        base.read()
    with pytest.raises(RuntimeError):
        base.set_wheel_duty(0.1, 0.1)


def test_realtime_mode_tracks_wall_clock(ideal):
    base = make_base(ideal, realtime=True)
    start = time.monotonic()
    for _ in range(10):
        base.read()
    assert time.monotonic() - start >= 0.18
