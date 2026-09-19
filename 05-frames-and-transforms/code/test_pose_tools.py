"""Tests for pose_tools.py (lesson 05.02)."""

import math

import pytest

from pose_tools import Pose2D, bearing_to, circular_mean, heading_error, wrap_angle


def test_wrap_angle_edges() -> None:
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)
    assert wrap_angle(7.0) == pytest.approx(0.7168146928)


def test_bottle_bearing_and_error() -> None:
    robot = Pose2D(2.0, 1.0, math.radians(90))
    assert math.degrees(bearing_to(robot, 2.05, 1.70)) == pytest.approx(85.9144, abs=1e-4)
    assert math.degrees(heading_error(robot, 2.05, 1.70)) == pytest.approx(-4.0856, abs=1e-4)
    assert math.degrees(heading_error(robot, 1.0, 0.0)) == pytest.approx(135.0)


def test_pose_wraps_theta() -> None:
    assert Pose2D(0, 0, math.radians(450)).theta == pytest.approx(math.pi / 2)


def test_circular_mean() -> None:
    angles = [math.radians(a) for a in (350, 10, 20)]
    assert math.degrees(circular_mean(angles)) == pytest.approx(6.705, abs=1e-3)
    assert abs(math.degrees(circular_mean([math.radians(170), math.radians(-170)]))) == pytest.approx(180.0)
