from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from robotlab.geometry import SE2
from robotlab.sim import DiffDriveSim, World, arc_update

DT = 0.02


def run(sim: DiffDriveSim, seconds: float, *, duty=None, velocity=None) -> None:
    for _ in range(round(seconds / DT)):
        if duty is not None:
            sim.set_duty(*duty)
        if velocity is not None:
            sim.set_velocity(*velocity)
        sim.step(DT)


def test_arc_update_quarter_circle():
    pose = arc_update(SE2(), 0.9 * math.pi / 2, 1.1 * math.pi / 2, 0.2)
    assert pose.as_tuple() == pytest.approx((1.0, 1.0, math.pi / 2))


def test_straight_line_with_equal_duty(ideal):
    sim = DiffDriveSim(World(), *ideal, pose=(0.0, 0.0, 0.0))
    run(sim, 2.0, duty=(0.6, 0.6))
    assert sim.pose.x > 0.5
    assert abs(sim.pose.y) < 1e-9 and abs(sim.pose.theta) < 1e-9
    left, right = sim.ticks
    assert left == right > 0


def test_motor_mismatch_curves_toward_weaker_motor(ideal):
    params = dataclasses.replace(ideal[0], motor_gain_right=0.9)
    sim = DiffDriveSim(World(), params, ideal[1])
    run(sim, 2.0, duty=(0.6, 0.6))
    assert sim.pose.theta < -0.05 and sim.pose.y < 0.0


def test_velocity_mode_holds_setpoint_despite_mismatch(realistic):
    sim = DiffDriveSim(World(), *realistic, seed=3)
    run(sim, 2.0, velocity=(10.0, 10.0))
    assert sim.wheel_rad_s == pytest.approx([10.0, 10.0], abs=0.6)
    assert sim.wheel_velocity_estimate == pytest.approx((10.0, 10.0), abs=1.5)


def test_velocity_mode_unreachable_setpoint_reaches_top_speed(ideal):
    """Anti-windup fills the duty headroom (same rule as labs/firmware/pico/velocity.py): a pure PID
    asked for 25 rad/s ends at duty 1 and the motor's top speed, not stranded below it."""
    params = dataclasses.replace(ideal[0], velocity_kp=0.1, velocity_ki=2.0, velocity_ff=0.0)
    sim = DiffDriveSim(World(), params, ideal[1])
    for _ in range(300):
        sim.set_velocity(25.0, 25.0)
        sim.step(0.01)
    top = params.max_wheel_speed_rad_s * sim.battery_v / params.battery_nominal_v
    assert sim.duty == pytest.approx([1.0, 1.0])
    assert sim.wheel_rad_s == pytest.approx([top, top], rel=0.02)


def test_steady_state_speed_and_time_constant(ideal):
    params = dataclasses.replace(ideal[0], battery_initial_soc=1.0)
    sim = DiffDriveSim(World(), params, ideal[1])
    sim.set_duty(1.0, 1.0)
    tau = params.motor_time_constant_s
    sim.advance(tau, DT)
    after_tau = sim.wheel_rad_s[0]
    sim.advance(1.0, DT)
    final = sim.wheel_rad_s[0]
    expected = params.max_wheel_speed_rad_s * sim.battery_v / params.battery_nominal_v
    assert final == pytest.approx(expected, rel=0.02)
    assert after_tau / final == pytest.approx(1 - math.exp(-1), abs=0.05)


def test_rotation_in_place(ideal):
    params = ideal[0]
    sim = DiffDriveSim(World(), *ideal, pose=(1.0, 2.0, 0.0))
    run(sim, 0.3, duty=(-0.5, 0.5))
    assert (sim.pose.x, sim.pose.y) == pytest.approx((1.0, 2.0), abs=1e-9)
    assert 0.3 < sim.pose.theta < math.pi  # counter-clockwise
    expected_rate = 2 * params.wheel_radius_m * sim.wheel_rad_s[1] / params.wheel_separation_m
    assert sim.yaw_rate == pytest.approx(expected_rate, rel=0.01)


def test_rotation_in_place_velocity_mode(ideal):
    sim = DiffDriveSim(World(), *ideal, pose=(1.0, 2.0, 0.0))
    run(sim, 2.0, velocity=(-5.0, 5.0))
    assert sim.pose.distance_to(sim.pose.from_tuple((1.0, 2.0, 0.0))) < 1e-3
    assert sim.wheel_rad_s == pytest.approx([-5.0, 5.0], abs=0.3)


def test_deadband(ideal):
    params = ideal[0]
    below = DiffDriveSim(World(), *ideal)
    run(below, 1.0, duty=(params.duty_deadband * 0.9, -params.duty_deadband * 0.9))
    assert below.ticks == (0, 0) and below.pose == SE2()
    above = DiffDriveSim(World(), *ideal)
    run(above, 1.0, duty=(params.duty_deadband + 0.1, params.duty_deadband + 0.1))
    assert above.ticks[0] > 0 and above.pose.x > 0.0


def test_saturation(ideal):
    a, b = DiffDriveSim(World(), *ideal), DiffDriveSim(World(), *ideal)
    run(a, 1.0, duty=(1.0, 1.0))
    run(b, 1.0, duty=(5.0, 5.0))
    assert a.pose.x == pytest.approx(b.pose.x)


def test_collision_stops_the_robot(ideal):
    params = ideal[0]
    sim = DiffDriveSim(World.rectangle_room(2.0, 2.0), *ideal, pose=(1.0, 1.0, 0.0))
    run(sim, 5.0, duty=(0.8, 0.8))
    assert sim.collided
    assert sim.pose.x <= 2.0 - params.robot_radius_m + 1e-9
    assert sim.pose.x > 2.0 - params.robot_radius_m - 0.05
    ticks_at_wall = sim.ticks
    run(sim, 0.5, duty=(0.8, 0.8))
    assert sim.ticks == ticks_at_wall  # stalled wheels don't count
    run(sim, 1.0, duty=(-0.8, -0.8))  # backing away is allowed
    assert not sim.collided and sim.pose.x < 2.0 - params.robot_radius_m - 0.1


def test_encoder_tick_math_matches_config(cfg, ideal):
    sim = DiffDriveSim(World(), *ideal)
    run(sim, 3.0, velocity=(6.0, 6.0))
    distance = sim.pose.x
    ticks_per_rev = cfg.drive.ticks_per_wheel_rev
    expected_ticks = distance / (2 * math.pi * cfg.drive.wheel_radius_m) * ticks_per_rev
    left, right = sim.ticks
    assert isinstance(left, int)
    assert abs(left - expected_ticks) <= 1 and abs(right - expected_ticks) <= 1
    assert sim.params.ticks_per_wheel_rev == ticks_per_rev


def test_slip_is_invisible_to_encoders(ideal):
    params = dataclasses.replace(ideal[0], slip_std=0.2)
    sim = DiffDriveSim(World(), params, ideal[1], seed=5)
    run(sim, 2.0, duty=(0.5, 0.5))
    assert sim.ticks[0] == sim.ticks[1]  # same rotation...
    assert abs(sim.pose.theta) > 1e-3  # ...but the ground travel differed


def test_encoder_rollover(ideal):
    params = dataclasses.replace(ideal[0], encoder_bits=10)
    sim = DiffDriveSim(World(), params, ideal[1])
    run(sim, 3.0, duty=(1.0, 1.0))
    left, _ = sim.ticks
    assert -512 <= left < 512


def test_wrong_nominal_geometry_is_true_geometry(ideal):
    params = dataclasses.replace(ideal[0], wheel_radius_scale_left=1.1, wheel_radius_scale_right=1.1)
    nominal, bigger = DiffDriveSim(World(), *ideal), DiffDriveSim(World(), params, ideal[1])
    run(nominal, 1.0, duty=(0.7, 0.7))
    run(bigger, 1.0, duty=(0.7, 0.7))
    assert bigger.ticks == nominal.ticks
    assert bigger.pose.x == pytest.approx(1.1 * nominal.pose.x)


def test_battery_sags_under_load_and_drains(realistic):
    sim = DiffDriveSim(World(), *realistic, seed=0)
    resting = sim.battery_v
    run(sim, 1.0, duty=(1.0, 1.0))
    loaded = sim.battery_v
    assert loaded < resting
    sim.stop()
    sim.step(DT)
    assert loaded < sim.battery_v < resting  # recovers, minus the charge used


def test_determinism_with_seed(realistic):
    def trace(seed: int):
        sim = DiffDriveSim(World.apartment(), *realistic, pose=(1.0, 1.3, 0.0), seed=seed)
        out = []
        for i in range(200):
            sim.set_duty(0.5, 0.45)
            sim.step(DT)
            out.append((*sim.pose, *sim.ticks, sim.gyro_z(), sim.front_range() or -1.0))
        return np.array(out), sim.lidar_scan().ranges

    a, scan_a = trace(7)
    b, scan_b = trace(7)
    c, _ = trace(8)
    assert np.array_equal(a, b)
    assert np.array_equal(scan_a, scan_b, equal_nan=True)
    assert not np.array_equal(a, c)


def test_reset(ideal):
    sim = DiffDriveSim(World(), *ideal)
    run(sim, 1.0, duty=(0.5, 0.5))
    sim.reset((1.0, 2.0, 0.5))
    assert sim.pose == SE2(1.0, 2.0, 0.5) and sim.ticks == (0, 0) and sim.t == 0.0
