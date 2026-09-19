"""Checker for 08.11 — rotate exactly 90°: trapezoidal profile + wrap-safe heading control.

Run: ``python course.py check 08.11`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import math

import pytest

from robotlab.config import load_config
from robotlab.geometry import angle_diff, wrap_angle
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

approx = pytest.approx

V_MAX, A_MAX = 1.5, 3.0  # rad/s, rad/s^2 — gentle enough not to slip the wheels
KP, KI = 4.0, 1.0
LOOP_DT = 0.02


# --- wrapping -------------------------------------------------------------------------------------
def test_heading_error_takes_the_short_way(impl):
    d = math.radians
    assert impl.heading_error(d(90), d(0)) == approx(d(90))
    assert impl.heading_error(d(-179), d(179)) == approx(d(2), abs=1e-9), "+2 deg, not -358 deg"
    assert impl.heading_error(d(179), d(-179)) == approx(d(-2), abs=1e-9)
    assert impl.heading_error(d(10), d(350)) == approx(d(20), abs=1e-9)
    assert abs(impl.heading_error(d(0), d(180))) == approx(math.pi), "exactly opposite: +-pi"


def test_heading_error_agrees_with_robotlab(impl):
    for target in (-3.0, -0.1, 0.0, 1.2, 3.1, 6.5, -7.0):
        for measured in (-3.1, 0.0, 2.9, 4.0):
            assert impl.heading_error(target, measured) == approx(angle_diff(target, measured), abs=1e-12)


# --- the profile ----------------------------------------------------------------------------------
def test_trapezoid_by_hand(impl):
    """90 deg = 1.5708 rad at v_max 1.5, a_max 3.0: ramp 0.5 s, cruise 0.547 s, total 1.547 s."""
    p = impl.TrapezoidalProfile(math.radians(90), V_MAX, A_MAX)
    assert p.peak_v == approx(1.5)
    assert p.t_accel == approx(0.5)
    assert p.t_cruise == approx(0.5472, abs=1e-3)
    assert p.duration == approx(1.5472, abs=1e-3)
    assert p.velocity(0.25) == approx(0.75), "half way up the ramp"
    assert p.position(0.5) == approx(0.375), "0.5 * a * t^2 = 0.5 * 3 * 0.25"
    assert p.position(p.duration) == approx(math.radians(90))
    assert p.velocity(p.duration + 0.1) == 0.0 and p.position(p.duration + 0.1) == approx(math.radians(90))
    assert p.velocity(-0.1) == 0.0 and p.position(-0.1) == 0.0


def test_short_turn_is_triangular(impl):
    """10 deg never reaches v_max: peak = sqrt(a * d) = sqrt(3 * 0.1745) = 0.7235 rad/s."""
    p = impl.TrapezoidalProfile(math.radians(10), V_MAX, A_MAX)
    assert p.peak_v == approx(0.7235, abs=1e-3)
    assert p.t_cruise == approx(0.0)
    assert p.duration == approx(2 * 0.7235 / A_MAX, abs=1e-3)
    assert p.position(p.duration) == approx(math.radians(10))


def test_negative_distance_turns_the_other_way(impl):
    p = impl.TrapezoidalProfile(-math.radians(90), V_MAX, A_MAX)
    assert p.duration == approx(impl.TrapezoidalProfile(math.radians(90), V_MAX, A_MAX).duration)
    assert p.velocity(0.25) == approx(-0.75)
    assert p.position(p.duration) == approx(-math.radians(90))


def test_position_is_the_integral_of_velocity(impl):
    """The feedforward and the feedback must agree, so these two cannot drift apart."""
    for degrees in (5, 45, 90, 180, -120):
        p = impl.TrapezoidalProfile(math.radians(degrees), V_MAX, A_MAX)
        steps, h = 4000, p.duration / 4000
        integral = sum(p.velocity((k + 0.5) * h) for k in range(steps)) * h
        assert integral == approx(p.distance, abs=1e-4), f"{degrees} deg: profile does not close"
        for k in range(0, steps, 137):
            running = sum(p.velocity((j + 0.5) * h) for j in range(k)) * h
            assert p.position(k * h) == approx(running, abs=2e-4)


def test_zero_turn_is_harmless(impl):
    p = impl.TrapezoidalProfile(0.0, V_MAX, A_MAX)
    assert p.duration == approx(0.0) and p.velocity(0.0) == 0.0 and p.position(1.0) == approx(0.0)


# --- the controller -------------------------------------------------------------------------------
def test_on_profile_the_output_is_pure_feedforward(impl):
    p = impl.TrapezoidalProfile(math.radians(90), V_MAX, A_MAX)
    c = impl.TurnController(p, 0.0, KP, KI)
    out = c.update(0.25, p.position(0.25), LOOP_DT)
    assert c.error == approx(0.0, abs=1e-12)
    assert c.feedforward == approx(p.velocity(0.25))
    assert out == approx(p.velocity(0.25)), "no error: the profile alone commands the robot"


def test_error_adds_proportional_action(impl):
    p = impl.TrapezoidalProfile(math.radians(90), V_MAX, A_MAX)
    c = impl.TurnController(p, 0.0, KP, KI)
    behind = p.position(0.25) - math.radians(5)
    out = c.update(0.25, behind, LOOP_DT)
    e = math.radians(5)
    assert c.error == approx(e)
    assert out == approx(p.velocity(0.25) + KP * e + KI * e * LOOP_DT)


def test_the_target_starts_from_the_current_heading(impl):
    """Starting at +170 deg, a +90 deg turn ends at -100 deg — the controller must not unwind."""
    p = impl.TrapezoidalProfile(math.radians(90), V_MAX, A_MAX)
    c = impl.TurnController(p, math.radians(170), KP, KI)
    assert c.target(0.0) == approx(math.radians(170))
    assert c.target(p.duration) == approx(wrap_angle(math.radians(260)))
    assert math.degrees(c.target(p.duration)) == approx(-100.0, abs=1e-6)
    assert abs(c.update(p.duration, math.radians(-100), LOOP_DT)) < 1e-9, "already there: command nothing"


def test_output_is_clamped_and_the_integral_does_not_wind_up(impl):
    p = impl.TrapezoidalProfile(math.radians(90), V_MAX, A_MAX)
    c = impl.TurnController(p, 0.0, KP, KI, max_yaw_rate_rad_s=1.0)
    for k in range(200):  # the robot never moves: a 4 s stall at full command
        out = c.update(k * LOOP_DT, 0.0, LOOP_DT)
    assert out == approx(1.0), "clamped to max_yaw_rate_rad_s"
    assert c.integral <= 1.0 + 1e-9, f"integral wound up to {c.integral:.2f}"
    assert c.update(4.0, math.radians(120), LOOP_DT) < 1.0, "past the target: the command must reverse fast"


def test_finished_needs_both_the_profile_and_the_tolerance(impl):
    p = impl.TrapezoidalProfile(math.radians(90), V_MAX, A_MAX)
    c = impl.TurnController(p, 0.0, KP, KI)
    assert not c.finished(0.5, math.radians(90)), "the profile has not finished yet"
    assert not c.finished(p.duration + 0.5, math.radians(85)), "5 deg away is not finished"
    assert c.finished(p.duration + 0.5, math.radians(90.5), tolerance_rad=math.radians(1.0))


# --- end to end on the simulator -------------------------------------------------------------------
def turn_once(impl, degrees: float, seed: int, settle_s: float = 0.6) -> float:
    """Turn in place on the realistic simulator, heading from the (bias-corrected) gyro.

    Returns the TRUE final heading in degrees.
    """
    cfg = load_config()
    sim = DiffDriveSim(World(), DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg), seed=seed)
    base = SimBase(sim, dt=0.01)
    radius, separation = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m

    bias_samples = []
    for _ in range(50):  # 1 s standing still: measure the gyro bias
        base.set_wheel_velocity(0.0, 0.0)
        base.read()
        base.read()
        bias_samples.append(base.gyro_z())
    bias = sum(bias_samples) / len(bias_samples)

    profile = impl.TrapezoidalProfile(math.radians(degrees), V_MAX, A_MAX)
    controller = impl.TurnController(profile, 0.0, KP, KI)
    heading, t = 0.0, 0.0
    while t < profile.duration + settle_s:
        yaw = controller.update(t, heading, LOOP_DT)
        differential = yaw * separation / (2.0 * radius)
        base.set_wheel_velocity(-differential, differential)
        base.read()
        base.read()
        heading = wrap_angle(heading + (base.gyro_z() - bias) * LOOP_DT)
        t += LOOP_DT
    true_heading = math.degrees(base.sim.pose.theta)
    base.close()
    return true_heading


def test_ninety_degrees_within_two_degrees_over_ten_trials(impl):
    """The lesson's acceptance criterion: +-2 deg, repeatably."""
    errors = [turn_once(impl, 90.0, seed=s) - 90.0 for s in range(10)]
    worst = max(abs(e) for e in errors)
    mean = sum(errors) / len(errors)
    assert worst <= 2.0, f"worst trial was {worst:.2f} deg off (errors: {[round(e, 2) for e in errors]})"
    assert abs(mean) <= 1.0, f"a systematic {mean:.2f} deg bias means the profile or the gain is wrong"


def test_turns_both_ways_and_across_the_wrap(impl):
    assert turn_once(impl, -90.0, seed=3) == approx(-90.0, abs=2.0), "clockwise must work too"
    # 170 deg: the profile's target crosses +pi is not needed, but 200 deg lands at -160 deg
    end = turn_once(impl, 200.0, seed=4)
    assert angle_diff(math.radians(-160.0), math.radians(end)) == approx(0.0, abs=math.radians(3.0)), \
        f"a 200 deg turn must end at -160 deg, got {end:.1f} deg"


def test_it_does_not_overshoot(impl):
    """A profile that decelerates in time should not sail past the target and come back."""
    cfg = load_config()
    sim = DiffDriveSim(World(), DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg), seed=7)
    base = SimBase(sim, dt=0.01)
    radius, separation = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    profile = impl.TrapezoidalProfile(math.radians(90), V_MAX, A_MAX)
    controller = impl.TurnController(profile, 0.0, KP, KI)
    heading, t, peak = 0.0, 0.0, 0.0
    while t < profile.duration + 0.6:
        yaw = controller.update(t, heading, LOOP_DT)
        differential = yaw * separation / (2.0 * radius)
        base.set_wheel_velocity(-differential, differential)
        base.read()
        base.read()
        heading = wrap_angle(heading + base.gyro_z() * LOOP_DT)
        peak = max(peak, math.degrees(base.sim.pose.theta))
        t += LOOP_DT
    base.close()
    assert peak < 95.0, f"overshot to {peak:.1f} deg: the deceleration ramp is not doing its job"
