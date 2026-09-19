"""Tests for odometry_core (no ROS needed): py -m pytest 09-odometry/code"""
from __future__ import annotations

import math

import numpy as np
import pytest

from course_odometry.odometry_core import (
    DiffDriveOdometry,
    planar_to_ros_covariance,
    pose_difference,
    quaternion_to_yaw,
    yaw_to_quaternion,
)

R, B = 0.045, 0.2


def make(**kwargs) -> DiffDriveOdometry:
    return DiffDriveOdometry(wheel_radius_left=R, wheel_radius_right=R, wheel_separation=B, **kwargs)


def test_first_update_only_stores():
    odom = make()
    assert odom.update(10.0, 20.0, 1.0) is False
    assert (odom.x, odom.y, odom.yaw) == (0.0, 0.0, 0.0)


def test_straight_and_twist():
    odom = make()
    odom.update(0.0, 0.0, 0.0)
    assert odom.update(2.0, 2.0, 0.5) is True  # 2 rad * 0.045 m = 0.09 m in 0.5 s
    assert (odom.x, odom.y, odom.yaw) == pytest.approx((0.09, 0.0, 0.0))
    assert (odom.v, odom.omega) == pytest.approx((0.18, 0.0))


def test_spin_quarter_turn():
    odom = make()
    odom.update(0.0, 0.0, 0.0)
    wheel = (B / 2) * (math.pi / 2) / R  # wheel angle for a quarter turn in place
    odom.update(-wheel, wheel, 1.0)
    assert (odom.x, odom.y, odom.yaw) == pytest.approx((0.0, 0.0, math.pi / 2), abs=1e-12)


def test_ignores_old_or_duplicate_stamps():
    odom = make()
    odom.update(0.0, 0.0, 1.0)
    assert odom.update(5.0, 5.0, 1.0) is False
    assert odom.update(5.0, 5.0, 0.9) is False
    assert odom.x == 0.0


def test_covariance_grows_like_the_09_06_prediction():
    k = 2e-6
    odom = make(k_left=k, k_right=k)
    step = 0.005 / R  # 5 mm per update
    odom.update(0.0, 0.0, 0.0)
    for i in range(1, 1001):  # 5 m straight
        odom.update(i * step, i * step, i * 0.02)
    d = 5.0
    q = 2 * k / B**2
    assert math.sqrt(odom.covariance[2, 2]) == pytest.approx(math.sqrt(q * d), rel=1e-3)
    assert math.sqrt(odom.covariance[1, 1]) == pytest.approx(math.sqrt(q * d**3 / 3), rel=0.01)
    assert math.sqrt(odom.covariance[0, 0]) == pytest.approx(math.sqrt(k * d / 2), rel=0.01)


def test_ros_covariance_layout():
    cov3 = np.array([[1.0, 2.0, 3.0], [2.0, 4.0, 5.0], [3.0, 5.0, 6.0]])
    out = planar_to_ros_covariance(cov3)
    assert len(out) == 36
    assert out[0] == 1.0 and out[7] == 4.0 and out[35] == 6.0  # x, y, yaw variances
    assert out[5] == 3.0 and out[30] == 3.0  # x-yaw correlation, both triangles
    assert out[14] == out[21] == out[28] == 1e-6  # z, roll, pitch


@pytest.mark.parametrize("yaw", [0.0, 0.5, -2.0, math.pi - 1e-6])
def test_quaternion_round_trip(yaw):
    assert quaternion_to_yaw(*yaw_to_quaternion(yaw)) == pytest.approx(yaw)


def test_pose_difference_wraps():
    dist, dyaw = pose_difference((1.0, 1.0, math.pi - 0.01), (1.0, 2.0, -math.pi + 0.01))
    assert dist == pytest.approx(1.0)
    assert dyaw == pytest.approx(0.02)
