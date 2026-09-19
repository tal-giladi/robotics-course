"""Checker for 09.03 — Dead reckoning: Euler, midpoint and exact-arc integration.

Run: ``python course.py check 09.03`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import pytest

from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

approx = pytest.approx
QUARTER = math.pi / 2
ORIGIN = (0.0, 0.0, 0.0)


# --- one big step: v = 1 m/s, omega = pi/2 rad/s for 1 s (a quarter circle of radius 2/pi) ------
def test_euler_quarter_turn(impl):
    # Euler drives 1 m along the OLD heading (x), then turns.
    assert impl.euler_step(ORIGIN, 1.0, QUARTER, 1.0) == approx((1.0, 0.0, QUARTER))


def test_midpoint_quarter_turn(impl):
    # Midpoint drives 1 m along the heading at half time: 45 degrees.
    s = math.sqrt(0.5)
    assert impl.midpoint_step(ORIGIN, 1.0, QUARTER, 1.0) == approx((s, s, QUARTER))


def test_exact_quarter_turn(impl):
    # Arc of radius R = v/omega = 2/pi ends at (R, R).
    r = 2 / math.pi
    assert impl.exact_step(ORIGIN, 1.0, QUARTER, 1.0) == approx((r, r, QUARTER), abs=1e-12)


@pytest.mark.parametrize("name", ["euler_step", "midpoint_step", "exact_step"])
def test_straight_line_is_exact_for_everyone(impl, name):
    step = getattr(impl, name)
    assert step((1.0, 2.0, QUARTER), 0.5, 0.0, 2.0) == approx((1.0, 3.0, QUARTER))


@pytest.mark.parametrize("name", ["euler_step", "midpoint_step", "exact_step"])
def test_heading_is_wrapped(impl, name):
    _, _, theta = getattr(impl, name)((0.0, 0.0, 3.0), 0.0, 1.0, 0.5)
    assert theta == approx(3.5 - 2 * math.pi)


def test_exact_step_with_tiny_turn_rate_is_finite(impl):
    pose = impl.exact_step(ORIGIN, 1.0, 1e-13, 1.0)
    assert pose == approx((1.0, 0.0, 0.0), abs=1e-9)


def test_exact_spin_in_place(impl):
    assert impl.exact_step((0.3, 0.4, 0.0), 0.0, 2.0, 0.25) == approx((0.3, 0.4, 0.5))


def test_integrate_applies_each_sample(impl):
    twists = [(1.0, 0.0), (0.0, QUARTER), (1.0, 0.0)]  # 1 m, turn left 90 deg, 1 m
    assert impl.integrate(impl.exact_step, ORIGIN, twists, 1.0) == approx((1.0, 1.0, QUARTER), abs=1e-12)


def test_exact_step_does_not_care_about_the_time_step(impl):
    one = impl.exact_step(ORIGIN, 0.5, 2.0, 5.0)
    many = impl.integrate(impl.exact_step, ORIGIN, [(0.5, 2.0)] * 500, 0.01)
    assert many == approx(one, abs=1e-9), "for a constant twist, 500 exact small arcs = 1 exact big arc"


# --- the time-step experiment -------------------------------------------------------------------
def test_constant_twist_error_values(impl):
    # 5 s at 0.5 m/s and 2 rad/s (radius 0.25 m, about 1.6 turns), dt = 0.1 s.
    assert impl.constant_twist_error(impl.euler_step, 0.5, 2.0, 5.0, 0.1) == approx(0.04797, rel=1e-3)
    assert impl.constant_twist_error(impl.midpoint_step, 0.5, 2.0, 5.0, 0.1) == approx(0.000800, rel=1e-3)
    assert impl.constant_twist_error(impl.exact_step, 0.5, 2.0, 5.0, 0.1) == approx(0.0, abs=1e-9)


def test_euler_is_first_order(impl):
    coarse = impl.constant_twist_error(impl.euler_step, 0.5, 2.0, 5.0, 0.1)
    fine = impl.constant_twist_error(impl.euler_step, 0.5, 2.0, 5.0, 0.05)
    assert coarse / fine == approx(2.0, rel=0.1), "halving dt should halve Euler's error"


def test_midpoint_is_second_order(impl):
    coarse = impl.constant_twist_error(impl.midpoint_step, 0.5, 2.0, 5.0, 0.1)
    fine = impl.constant_twist_error(impl.midpoint_step, 0.5, 2.0, 5.0, 0.05)
    assert coarse / fine == approx(4.0, rel=0.1), "halving dt should quarter the midpoint error"


# --- against the simulator ----------------------------------------------------------------------
PLAN = [(2.0, (8.0, 8.0)), (3.0, (3.0, 10.0)), (1.5, (-6.0, 6.0)), (3.0, (10.0, 4.0))]


def twists_from_simulator(every: int) -> tuple[list[tuple[float, float]], float, tuple[float, float, float]]:
    """Drive PLAN on the ideal simulator; return (v, omega) from encoder ticks every `every` reads."""
    cfg = load_config()
    d = cfg.drive
    base = SimBase(DiffDriveSim(World(), DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg), seed=0))
    m_per_tick = 2 * math.pi * d.wheel_radius_m / d.ticks_per_wheel_rev
    last = base.read()
    dt = base.dt * every
    twists, count = [], 0
    for seconds, (left, right) in PLAN:
        for _ in range(round(seconds / base.dt)):
            base.set_wheel_velocity(left, right)
            state = base.read()
            count += 1
            if count % every == 0:
                dl = (state.left_ticks - last.left_ticks) * m_per_tick
                dr = (state.right_ticks - last.right_ticks) * m_per_tick
                twists.append(((dl + dr) / 2 / dt, (dr - dl) / d.wheel_separation_m / dt))
                last = state
    truth = base.sim.pose
    return twists, dt, (truth.x, truth.y, truth.theta)


def position_error(pose, truth) -> float:
    return math.hypot(pose[0] - truth[0], pose[1] - truth[1])


def test_exact_integration_of_encoder_twists_matches_simulator(impl):
    twists, dt, truth = twists_from_simulator(every=1)  # 50 Hz, like karmel's telemetry
    pose = impl.integrate(impl.exact_step, ORIGIN, twists, dt)
    assert position_error(pose, truth) < 0.005, "at 50 Hz exact arcs should match a perfect robot within 5 mm"


def test_low_rate_euler_is_worse_than_midpoint(impl):
    twists, dt, truth = twists_from_simulator(every=5)  # 10 Hz: bigger steps
    euler = position_error(impl.integrate(impl.euler_step, ORIGIN, twists, dt), truth)
    midpoint = position_error(impl.integrate(impl.midpoint_step, ORIGIN, twists, dt), truth)
    exact = position_error(impl.integrate(impl.exact_step, ORIGIN, twists, dt), truth)
    assert euler > 3 * midpoint, f"at 10 Hz Euler ({euler:.3f} m) should be far worse than midpoint ({midpoint:.3f} m)"
    assert exact < 0.005
