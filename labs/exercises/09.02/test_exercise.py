"""Checker for 09.02 — cmd_vel to wheel speeds (with saturation).

Run: ``python course.py check 09.02`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math
import random

import pytest

from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

approx = pytest.approx
R, L = 0.05, 0.2  # round numbers for the hand-computed cases


# --- kinematics ---------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("v", "omega", "expected"),
    [
        (0.3, 0.0, (6.0, 6.0)),  # straight: 0.3 / 0.05
        (0.0, 1.0, (-2.0, 2.0)),  # spin left: rim speeds -+0.1 m/s
        (0.2, 1.0, (2.0, 6.0)),  # arc left: (0.2 - 0.1)/0.05, (0.2 + 0.1)/0.05
        (-0.2, 1.0, (-6.0, -2.0)),  # reversing while turning left
    ],
)
def test_twist_to_wheel_speeds(impl, v, omega, expected):
    assert impl.twist_to_wheel_speeds(v, omega, R, L) == approx(expected)


@pytest.mark.parametrize(
    ("left", "right", "expected"),
    [(6.0, 6.0, (0.3, 0.0)), (-2.0, 2.0, (0.0, 1.0)), (2.0, 6.0, (0.2, 1.0))],
)
def test_wheel_speeds_to_twist(impl, left, right, expected):
    assert impl.wheel_speeds_to_twist(left, right, R, L) == approx(expected)


def test_round_trip(impl):
    rng = random.Random(7)
    for _ in range(200):
        v, omega = rng.uniform(-1, 1), rng.uniform(-4, 4)
        left, right = impl.twist_to_wheel_speeds(v, omega, 0.045, 0.2)
        assert impl.wheel_speeds_to_twist(left, right, 0.045, 0.2) == approx((v, omega)), (
            "forward and inverse kinematics must undo each other"
        )


# --- limits -------------------------------------------------------------------------------------
def test_limit_twist_scales_both(impl):
    v, omega = impl.limit_twist(0.7, 3.0, 0.5, 2.5)
    assert (v, omega) == approx((0.5, 3.0 * 0.5 / 0.7))  # v is the tighter limit: factor 0.714
    assert v / omega == approx(0.7 / 3.0), "the arc radius v/omega must not change"


def test_limit_twist_negative_and_untouched(impl):
    assert impl.limit_twist(-1.0, 0.5, 0.5, 2.5) == approx((-0.5, 0.25))
    assert impl.limit_twist(0.2, -1.0, 0.5, 2.5) == approx((0.2, -1.0))


@pytest.mark.parametrize(
    ("left", "right", "limit", "expected"),
    [
        (8.0, 20.0, 16.0, (6.4, 16.0)),  # factor 16/20 = 0.8
        (-20.0, 10.0, 16.0, (-16.0, 8.0)),  # the negative wheel is the fast one
        (5.0, -7.0, 16.0, (5.0, -7.0)),  # within limits: unchanged
    ],
)
def test_scale_to_limit(impl, left, right, limit, expected):
    assert impl.scale_to_limit(left, right, limit) == approx(expected)


@pytest.mark.parametrize(
    ("left", "right", "limit", "expected"),
    [
        (8.0, 20.0, 16.0, (4.0, 16.0)),  # mean 14, diff 6 -> mean clamped to 10
        (-20.0, -10.0, 16.0, (-16.0, -6.0)),  # mean -15, diff 5 -> mean clamped to -11
        (-30.0, 30.0, 16.0, (-16.0, 16.0)),  # pure spin too fast: spin at the limit
        (10.0, -25.0, 16.0, (16.0, -16.0)),  # |diff| 17.5 > 16: spin right at the limit
        (5.0, -7.0, 16.0, (5.0, -7.0)),  # within limits: unchanged
    ],
)
def test_keep_rotation_to_limit(impl, left, right, limit, expected):
    assert impl.keep_rotation_to_limit(left, right, limit) == approx(expected)


def test_keep_rotation_keeps_omega(impl):
    left, right = impl.cmd_vel_to_wheels(0.6, 2.0, 0.045, 0.2, 12.0, keep_rotation=True)
    v, omega = impl.wheel_speeds_to_twist(left, right, 0.045, 0.2)
    assert omega == approx(2.0), "keep_rotation must preserve the yaw rate"
    assert max(abs(left), abs(right)) == approx(12.0)
    assert v < 0.6


def test_pipeline_scales_curvature_preserving(impl):
    left, right = impl.cmd_vel_to_wheels(0.7, 3.0, 0.045, 0.2, 17.0)
    # raw wheels 8.889 and 22.222 rad/s -> factor 17/22.222 = 0.765
    assert (left, right) == approx((6.8, 17.0))


def test_pipeline_applies_twist_limits_first(impl):
    left, right = impl.cmd_vel_to_wheels(0.7, 3.0, 0.045, 0.2, 17.0, max_linear=0.5, max_angular=2.5)
    v, omega = impl.wheel_speeds_to_twist(left, right, 0.045, 0.2)
    assert (v, omega) == approx((0.5, 3.0 * 0.5 / 0.7))


# --- against the simulator ----------------------------------------------------------------------
def driven_radius(impl, v: float, omega: float, keep_rotation: bool = False) -> float:
    """Command (v, omega) through the student's pipeline for 4 s; return the radius actually driven."""
    cfg = load_config()
    d = cfg.drive
    base = SimBase(DiffDriveSim(World(), DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg), seed=0))
    poses = []
    for step in range(200):
        left, right = impl.cmd_vel_to_wheels(
            v, omega, d.wheel_radius_m, d.wheel_separation_m, d.max_wheel_speed_rad_s, keep_rotation=keep_rotation
        )
        base.set_wheel_velocity(left, right)
        base.read()
        if step >= 50:  # skip the first second: motors spinning up
            poses.append(base.sim.pose)
    arc_length = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:]))
    turned = sum(abs(math.remainder(b.theta - a.theta, 2 * math.pi)) for a, b in zip(poses, poses[1:]))
    return arc_length / turned


def test_saturated_arc_keeps_its_radius_in_simulation(impl):
    # 0.7 m/s at 3 rad/s asks the right wheel for 22 rad/s; the motors top out at 17.
    radius = driven_radius(impl, 0.7, 3.0)
    assert radius == approx(0.7 / 3.0, rel=0.05), (
        f"asked for a {0.7 / 3.0:.3f} m radius, drove {radius:.3f} m: "
        "scale both wheels together instead of clipping one"
    )
