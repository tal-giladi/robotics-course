"""Unit tests for course_tf_examples.tf_math — no ROS needed.

    colcon test --packages-select course_tf_examples      (in the ROS workspace)
    py -m pytest 05-frames-and-transforms/code            (on any machine)
"""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from course_tf_examples.tf_math import (KARMEL_STATIC, apply, circle_pose, quaternion_from_euler,
                                        quaternion_to_matrix, transform_msg_to_matrix,
                                        yaw_from_quaternion)


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def test_quaternion_from_euler_matches_rpy_matrix():
    rng = np.random.default_rng(1)
    for _ in range(50):
        r, p, y = rng.uniform(-3, 3), rng.uniform(-1.5, 1.5), rng.uniform(-3, 3)
        q = quaternion_from_euler(r, p, y)
        assert quaternion_to_matrix(*q) == pytest.approx(rpy_matrix(r, p, y))


def test_known_quaternions():
    s = math.sqrt(0.5)
    assert quaternion_from_euler(0, 0, math.pi / 2) == pytest.approx((0, 0, s, s))
    assert quaternion_from_euler(-math.pi / 2, 0, -math.pi / 2) == pytest.approx((-0.5, 0.5, -0.5, 0.5))
    assert yaw_from_quaternion(0, 0, s, s) == pytest.approx(math.pi / 2)


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError):
        quaternion_to_matrix(0.0, 0.0, 0.0, 0.0)


def make_transform(x, y, z, roll, pitch, yaw):
    q = quaternion_from_euler(roll, pitch, yaw)
    return SimpleNamespace(translation=SimpleNamespace(x=x, y=y, z=z),
                           rotation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))


def test_bottle_chain():
    static = {(p, c): transform_msg_to_matrix(make_transform(*v)) for p, c, *v in KARMEL_STATIC}
    T_map_odom = transform_msg_to_matrix(make_transform(2.0, 1.0, 0, 0, 0, math.pi / 2))
    T_odom_fp = np.eye(4)
    T_base_opt = static[('base_link', 'camera_link')] @ static[('camera_link', 'camera_optical_frame')]
    T_map_opt = T_map_odom @ T_odom_fp @ static[('base_footprint', 'base_link')] @ T_base_opt
    assert apply(T_base_opt, (0.05, -0.02, 0.60)) == pytest.approx([0.70, -0.05, 0.12])
    assert apply(T_map_opt, (0.05, -0.02, 0.60)) == pytest.approx([2.05, 1.70, 0.165])


def test_circle_pose():
    x, y, yaw = circle_pose(math.pi / 0.2, 0.1, 0.2)       # half a circle of radius 0.5 m
    assert (x, y, yaw) == pytest.approx((0.0, 1.0, math.pi), abs=1e-9)
    assert circle_pose(3.0, 0.1, 0.0) == pytest.approx((0.3, 0.0, 0.0))
