import math

import pytest

from karmel_base.kinematics import (integrate_odometry, Pose2D, ramp, twist_to_wheels, wheels_to_twist,
                                    wrap_angle, yaw_to_quaternion)

R, L = 0.045, 0.200


def test_straight_line():
    left, right = twist_to_wheels(0.45, 0.0, R, L, 100.0)
    assert left == pytest.approx(10.0) and right == pytest.approx(10.0)


def test_spin_in_place():
    left, right = twist_to_wheels(0.0, 1.0, R, L, 100.0)
    assert left == pytest.approx(-right)
    assert right == pytest.approx(0.1 / R)


def test_round_trip():
    for v, w in [(0.3, 0.5), (-0.2, 1.3), (0.0, -2.0)]:
        assert wheels_to_twist(*twist_to_wheels(v, w, R, L, 1e9), R, L) == pytest.approx((v, w))


def test_saturation_preserves_curvature():
    v, w = 1.0, 2.0
    left, right = twist_to_wheels(v, w, R, L, 17.0)
    assert max(abs(left), abs(right)) == pytest.approx(17.0)
    v2, w2 = wheels_to_twist(left, right, R, L)
    assert v2 / w2 == pytest.approx(v / w)


def test_ramp():
    assert ramp(0.0, 1.0, 2.0, 0.1) == pytest.approx(0.2)
    assert ramp(1.0, 0.0, 2.0, 0.1) == pytest.approx(0.8)
    assert ramp(0.0, 0.05, 2.0, 0.1) == pytest.approx(0.05)
    assert ramp(0.0, 5.0, 0.0, 0.1) == 5.0     # disabled


def test_odometry_straight_and_quarter_circle():
    pose = integrate_odometry(Pose2D(), 1.0, 1.0, L)
    assert (pose.x, pose.y, pose.theta) == pytest.approx((1.0, 0.0, 0.0))
    # quarter circle of radius 0.5 m to the left, in 100 steps
    radius, steps = 0.5, 100
    arc = math.pi / 2 * radius / steps
    d_left, d_right = arc * (radius - L / 2) / radius, arc * (radius + L / 2) / radius
    pose = Pose2D()
    for _ in range(steps):
        pose = integrate_odometry(pose, d_left, d_right, L)
    assert (pose.x, pose.y, pose.theta) == pytest.approx((0.5, 0.5, math.pi / 2), abs=1e-9)


def test_wrap_angle():
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


def test_quaternion():
    x, y, z, w = yaw_to_quaternion(math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sqrt(0.5)) and w == pytest.approx(math.sqrt(0.5))
