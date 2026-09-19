"""Checker for 14.07 — trajectory generation.

Run: ``python course.py check 14.07`` (or ``--solution`` to see the reference pass).

Limits used throughout: v_max = 60 deg/s, a_max = 120 deg/s^2, in radians.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

VMAX = math.radians(60.0)
AMAX = math.radians(120.0)
BREAK_EVEN = VMAX * VMAX / AMAX          # 30 deg: triangle below, trapezoid above


def sampled(impl, profile, dt: float = 0.002):
    t = np.arange(0.0, profile.duration + dt / 2, dt)
    if t.size:
        t[-1] = profile.duration
    return t, *impl.sample_profile(profile, t)


# --- trapezoid_profile ---------------------------------------------------------------------------
def test_ninety_degree_move_by_hand(impl):
    p = impl.trapezoid_profile(0.0, math.radians(90.0), VMAX, AMAX)
    assert p.t_accel == pytest.approx(0.5)
    assert p.t_cruise == pytest.approx(1.0)
    assert p.duration == pytest.approx(2.0)
    assert p.v_peak == pytest.approx(VMAX)
    assert not p.is_triangular


def test_short_move_becomes_a_triangle(impl):
    p = impl.trapezoid_profile(0.0, math.radians(20.0), VMAX, AMAX)
    assert p.is_triangular
    assert p.duration == pytest.approx(2.0 * math.sqrt(math.radians(20.0) / AMAX))
    assert p.duration == pytest.approx(0.8165, abs=1e-4)
    assert math.degrees(p.v_peak) == pytest.approx(48.99, abs=0.01)


def test_the_break_even_distance(impl):
    """Exactly v_max^2 / a_max: the cruise phase has just vanished, peak speed is exactly v_max."""
    p = impl.trapezoid_profile(0.0, BREAK_EVEN, VMAX, AMAX)
    assert p.v_peak == pytest.approx(VMAX, rel=1e-9)
    assert p.t_cruise == pytest.approx(0.0, abs=1e-9)
    assert p.duration == pytest.approx(1.0)


def test_backwards_move(impl):
    forward = impl.trapezoid_profile(0.0, math.radians(90.0), VMAX, AMAX)
    backward = impl.trapezoid_profile(math.radians(90.0), 0.0, VMAX, AMAX)
    assert backward.duration == pytest.approx(forward.duration)


def test_zero_distance_move(impl):
    p = impl.trapezoid_profile(0.7, 0.7, VMAX, AMAX)
    assert p.duration == pytest.approx(0.0)


def test_rejects_nonpositive_limits(impl):
    with pytest.raises(ValueError):
        impl.trapezoid_profile(0.0, 1.0, 0.0, AMAX)
    with pytest.raises(ValueError):
        impl.trapezoid_profile(0.0, 1.0, VMAX, -1.0)


# --- sample_profile ------------------------------------------------------------------------------
@pytest.mark.parametrize("degrees", [5.0, 20.0, 30.0, 90.0, -75.0])
def test_endpoints_are_exact(impl, degrees):
    q1 = math.radians(degrees)
    p = impl.trapezoid_profile(0.3, 0.3 + q1, VMAX, AMAX)
    q, qd, qdd = impl.sample_profile(p, np.array([0.0, p.duration]))
    assert q[0] == pytest.approx(0.3, abs=1e-12)
    assert q[-1] == pytest.approx(0.3 + q1, abs=1e-9)
    assert abs(qd[0]) < 1e-9 and abs(qd[-1]) < 1e-9


@pytest.mark.parametrize("degrees", [5.0, 20.0, 90.0, -110.0])
def test_velocity_integrates_to_the_distance(impl, degrees):
    p = impl.trapezoid_profile(0.0, math.radians(degrees), VMAX, AMAX)
    t, q, qd, _ = sampled(impl, p)
    area = float(np.sum((qd[1:] + qd[:-1]) / 2.0 * np.diff(t)))      # trapezoidal rule, numpy 1.x safe
    assert area == pytest.approx(math.radians(degrees), abs=1e-5)


@pytest.mark.parametrize("degrees", [5.0, 20.0, 90.0, -110.0])
def test_samples_never_exceed_the_limits(impl, degrees):
    p = impl.trapezoid_profile(0.0, math.radians(degrees), VMAX, AMAX)
    _, _, qd, qdd = sampled(impl, p)
    assert np.abs(qd).max() <= VMAX + 1e-9
    assert np.abs(qdd).max() <= AMAX + 1e-9


def test_position_is_monotonic_and_matches_the_integral(impl):
    p = impl.trapezoid_profile(0.0, math.radians(90.0), VMAX, AMAX)
    t, q, qd, _ = sampled(impl, p)
    assert np.all(np.diff(q) >= -1e-12)
    integrated = np.concatenate([[0.0], np.cumsum((qd[1:] + qd[:-1]) / 2 * np.diff(t))])
    assert np.abs(integrated - q).max() < 1e-5


def test_sampling_beyond_the_end_holds_the_goal(impl):
    p = impl.trapezoid_profile(0.0, math.radians(45.0), VMAX, AMAX)
    q, qd, _ = impl.sample_profile(p, np.array([p.duration + 5.0]))
    assert q[0] == pytest.approx(math.radians(45.0), abs=1e-9)
    assert abs(qd[0]) < 1e-9


# --- profile_with_duration -----------------------------------------------------------------------
def test_with_duration_hits_the_requested_time(impl):
    p = impl.profile_with_duration(0.0, math.radians(90.0), 3.0, AMAX)
    assert p.duration == pytest.approx(3.0, abs=1e-9)
    q, _, _ = impl.sample_profile(p, np.array([3.0]))
    assert q[0] == pytest.approx(math.radians(90.0), abs=1e-9)
    assert p.v_peak < VMAX          # stretched: it cruises more slowly


def test_with_duration_picks_the_smaller_root(impl):
    """The larger root overshoots the goal and comes back. Peak speed must be the low one."""
    D = math.radians(90.0)
    T = 3.0
    p = impl.profile_with_duration(0.0, D, T, AMAX)
    big_root = (AMAX * T + math.sqrt(AMAX * AMAX * T * T - 4 * AMAX * D)) / 2.0
    assert p.v_peak < big_root


def test_with_duration_refuses_the_impossible(impl):
    D = math.radians(90.0)
    too_short = 0.5 * 2.0 * math.sqrt(D / AMAX)      # half the full-acceleration triangle time
    with pytest.raises(ValueError):
        impl.profile_with_duration(0.0, D, too_short, AMAX)


# --- polynomials ---------------------------------------------------------------------------------
def test_cubic_matches_its_boundary_conditions(impl):
    T = 2.25
    c = impl.cubic_coefficients(0.2, 0.2 + math.radians(90.0), T)
    q, qd, _ = impl.polynomial_sample(c, np.array([0.0, T]))
    assert q[0] == pytest.approx(0.2, abs=1e-12)
    assert q[1] == pytest.approx(0.2 + math.radians(90.0), abs=1e-12)
    assert abs(qd[0]) < 1e-12 and abs(qd[1]) < 1e-12


def test_cubic_with_nonzero_end_velocities(impl):
    T = 1.5
    c = impl.cubic_coefficients(0.0, 1.0, T, v0=0.3, v1=-0.2)
    q, qd, _ = impl.polynomial_sample(c, np.array([0.0, T]))
    assert q[0] == pytest.approx(0.0, abs=1e-12)
    assert q[1] == pytest.approx(1.0, abs=1e-12)
    assert qd[0] == pytest.approx(0.3, abs=1e-12)
    assert qd[1] == pytest.approx(-0.2, abs=1e-12)


def test_cubic_peaks_match_the_closed_forms(impl):
    D, T = math.radians(90.0), 2.25
    c = impl.cubic_coefficients(0.0, D, T)
    t = np.linspace(0.0, T, 2001)
    _, qd, qdd = impl.polynomial_sample(c, t)
    assert np.abs(qd).max() == pytest.approx(1.5 * D / T, rel=1e-4)
    assert np.abs(qdd).max() == pytest.approx(6.0 * D / T ** 2, rel=1e-4)


def test_polynomial_sample_returns_arrays_shaped_like_t(impl):
    c = impl.cubic_coefficients(0.0, 1.0, 2.0)
    t = np.linspace(0.0, 2.0, 7)
    q, qd, qdd = impl.polynomial_sample(c, t)
    assert q.shape == t.shape and qd.shape == t.shape and qdd.shape == t.shape


def test_min_durations(impl):
    D = math.radians(90.0)
    assert impl.min_duration_cubic(D, VMAX, AMAX) == pytest.approx(2.25, abs=1e-3)
    assert impl.min_duration_quintic(D, VMAX, AMAX) == pytest.approx(2.8125, abs=1e-3)
    short = math.radians(10.0)
    assert impl.min_duration_cubic(short, VMAX, AMAX) == pytest.approx(0.7071, abs=1e-3)
    assert impl.min_duration_quintic(short, VMAX, AMAX) == pytest.approx(0.6938, abs=1e-3)


def test_quintic_beats_cubic_only_when_acceleration_binds(impl):
    short, long = math.radians(10.0), math.radians(90.0)
    assert impl.min_duration_quintic(short, VMAX, AMAX) < impl.min_duration_cubic(short, VMAX, AMAX)
    assert impl.min_duration_quintic(long, VMAX, AMAX) > impl.min_duration_cubic(long, VMAX, AMAX)


def test_min_duration_respects_the_limit_it_reports(impl):
    D = math.radians(45.0)
    T = impl.min_duration_cubic(D, VMAX, AMAX)
    _, qd, qdd = impl.polynomial_sample(impl.cubic_coefficients(0.0, D, T), np.linspace(0, T, 2001))
    assert np.abs(qd).max() <= VMAX + 1e-6
    assert np.abs(qdd).max() <= AMAX + 1e-6


# --- synchronisation ------------------------------------------------------------------------------
def test_all_joints_share_one_duration(impl):
    q0 = np.zeros(5)
    q1 = np.radians([60.0, 30.0, 20.0, 5.0, 0.0])
    profiles = impl.synchronize(q0, q1, [VMAX] * 5, [AMAX] * 5)
    durations = [p.duration for p in profiles]
    assert len(profiles) == 5
    assert max(durations) - min(durations) < 1e-9
    assert durations[0] == pytest.approx(1.5, abs=1e-6)     # the 60 deg joint sets the pace


def test_synchronisation_does_not_break_any_limit(impl):
    q0 = np.zeros(4)
    q1 = np.radians([90.0, -45.0, 12.0, 3.0])
    profiles = impl.synchronize(q0, q1, [VMAX] * 4, [AMAX] * 4)
    for p in profiles:
        _, _, qd, qdd = sampled(impl, p)
        assert np.abs(qd).max() <= VMAX + 1e-9
        assert np.abs(qdd).max() <= AMAX + 1e-9


def test_every_joint_reaches_its_goal(impl):
    q0 = np.radians([10.0, -20.0, 5.0])
    q1 = np.radians([70.0, 15.0, 5.0])
    profiles = impl.synchronize(q0, q1, [VMAX] * 3, [AMAX] * 3)
    T = profiles[0].duration
    for p, goal in zip(profiles, q1):
        q, _, _ = impl.sample_profile(p, np.array([T]))
        assert q[0] == pytest.approx(goal, abs=1e-9)


def test_slow_joints_are_stretched_not_clipped(impl):
    """The 5 deg joint would take 0.408 s alone; synchronised it must cruise much more slowly."""
    profiles = impl.synchronize(np.zeros(2), np.radians([60.0, 5.0]), [VMAX] * 2, [AMAX] * 2)
    assert profiles[1].v_peak < 0.2 * profiles[0].v_peak


# --- the pre-flight check --------------------------------------------------------------------------
def test_violations_is_empty_for_a_legal_trajectory(impl):
    profiles = impl.synchronize(np.zeros(3), np.radians([60.0, 30.0, 10.0]), [VMAX] * 3, [AMAX] * 3)
    dt = 0.01
    t = np.arange(0.0, profiles[0].duration + dt / 2, dt)
    qd = np.stack([impl.sample_profile(p, t)[1] for p in profiles], axis=1)
    qdd = np.stack([impl.sample_profile(p, t)[2] for p in profiles], axis=1)
    assert impl.violations(t, qd, qdd, VMAX, AMAX) == []


def test_violations_catches_a_truncated_deceleration(impl):
    """The bug of exercise 14.07-E5: cruise until the goal, then stop dead."""
    dt = 0.02
    t = np.arange(0.0, 0.70 + dt / 2, dt)
    qd = np.full((t.size, 1), VMAX)
    qd[-1, 0] = 0.0                       # from full speed to zero in one 20 ms sample
    qdd = np.vstack([[0.0], np.diff(qd, axis=0) / dt])
    found = impl.violations(t, qd, qdd, VMAX, AMAX)
    assert len(found) == 1 and "0" in found[0]


def test_violations_reports_the_right_joint_and_both_kinds(impl):
    t = np.linspace(0.0, 1.0, 50)
    qd = np.zeros((50, 3))
    qdd = np.zeros((50, 3))
    qd[10, 1] = 3.0 * VMAX
    qdd[20, 2] = 2.0 * AMAX
    found = impl.violations(t, qd, qdd, VMAX, AMAX)
    assert len(found) == 2
    assert "1" in found[0] and "2" in found[1]      # velocity violations first
