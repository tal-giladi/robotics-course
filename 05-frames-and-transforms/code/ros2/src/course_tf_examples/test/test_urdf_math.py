"""Tests for course_tf_examples.urdf_math, fake_world and tf_debug — no ROS needed.

    py -m pytest 05-frames-and-transforms/code
"""

import math

import numpy as np
import pytest

from course_tf_examples.fake_world import beam_angles, raycast_room
from course_tf_examples.tf_debug import explain
from course_tf_examples.urdf_math import (box_inertia, chain_transform, cylinder_inertia,
                                          format_transform, matrix_to_quaternion, parse_urdf,
                                          root_links, sphere_inertia, transform_from_root)

# The frames of karmel's URDF as xacro renders it (numbers from labs/config/karmel.yaml, 2026-09).
MINI_KARMEL = """<?xml version="1.0"?>
<robot name="karmel">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <link name="left_wheel"/>
  <link name="laser"/>
  <link name="camera_link"/>
  <link name="camera_optical_frame"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
    <origin rpy="0 0 0" xyz="0 0 0.045"/>
  </joint>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="left_wheel"/>
    <origin rpy="-1.5707963267948966 0 0" xyz="0 0.1 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/><child link="laser"/>
    <origin rpy="0 0 0" xyz="0.0 0.0 0.12"/>
  </joint>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/><child link="camera_link"/>
    <origin rpy="0 0 0" xyz="0.1 0.0 0.1"/>
  </joint>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/><child link="camera_optical_frame"/>
    <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
  </joint>
</robot>
"""


def apply(T, p):
    return T[:3, :3] @ np.asarray(p, dtype=float) + T[:3, 3]


def test_root_and_tree():
    joints = parse_urdf(MINI_KARMEL)
    assert root_links(MINI_KARMEL) == ['base_footprint']
    assert joints['camera_optical_frame'].parent == 'camera_link'
    root, T = transform_from_root(joints, 'laser')
    assert root == 'base_footprint'
    assert T[:3, 3] == pytest.approx([0.0, 0.0, 0.165])


def test_optical_frame_matches_tf2_echo():
    T = chain_transform(parse_urdf(MINI_KARMEL), 'base_link', 'camera_optical_frame')
    assert T[:3, :3] == pytest.approx(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]), abs=1e-12)
    assert matrix_to_quaternion(T[:3, :3]) == pytest.approx((-0.5, 0.5, -0.5, 0.5))
    assert apply(T, (0.05, -0.02, 0.60)) == pytest.approx([0.70, -0.05, 0.12])
    assert '- Translation: [0.100, 0.000, 0.100]' in format_transform(T)


def test_inverse_direction():
    joints = parse_urdf(MINI_KARMEL)
    T_ab = chain_transform(joints, 'laser', 'camera_optical_frame')
    T_ba = chain_transform(joints, 'camera_optical_frame', 'laser')
    assert T_ab @ T_ba == pytest.approx(np.eye(4), abs=1e-12)
    assert T_ab[:3, 3] == pytest.approx([0.10, 0.0, -0.02])


def test_continuous_joint_rotates_about_its_axis():
    joints = parse_urdf(MINI_KARMEL)
    T0 = chain_transform(joints, 'base_link', 'left_wheel')
    T1 = chain_transform(joints, 'base_link', 'left_wheel', {'left_wheel_joint': math.pi / 2})
    # the wheel axis (child z) is +y in base_link and does not move when the wheel turns
    assert T0[:3, 2] == pytest.approx([0, 1, 0], abs=1e-12)
    assert T1[:3, 2] == pytest.approx([0, 1, 0], abs=1e-12)
    assert T1[:3, 3] == pytest.approx([0, 0.1, 0])
    # a point on the rim (x = 0.045 in the wheel frame) moves from front to top... or bottom?
    assert apply(T0, (0.045, 0, 0)) == pytest.approx([0.045, 0.1, 0.0], abs=1e-12)
    assert apply(T1, (0.045, 0, 0)) == pytest.approx([0.0, 0.1, -0.045], abs=1e-12)


def test_broken_urdfs_are_rejected():
    two_parents = MINI_KARMEL.replace('</robot>', """
  <joint name="second" type="fixed"><parent link="laser"/><child link="camera_link"/></joint>
</robot>""")
    with pytest.raises(ValueError, match='two parents'):
        parse_urdf(two_parents)
    typo = MINI_KARMEL.replace('<parent link="camera_link"/>', '<parent link="camera_lnk"/>')
    with pytest.raises(ValueError, match='unknown link'):
        parse_urdf(typo)


def test_disconnected_frames():
    orphan = MINI_KARMEL.replace('</robot>', '<link name="bumper"/></robot>')
    assert root_links(orphan) == ['base_footprint', 'bumper']
    with pytest.raises(ValueError, match='not in the same tree'):
        chain_transform(parse_urdf(orphan), 'base_link', 'bumper')


def test_inertia_formulas_match_rendered_karmel():
    # values printed by `xacro karmel.urdf.xacro use_sim:=false` (chassis 1.23 kg, 0.25 x 0.18 x 0.07 m)
    assert box_inertia(1.23, 0.25, 0.18, 0.07) == pytest.approx((0.00382325, 0.0069085, 0.00972725))
    assert cylinder_inertia(0.08, 0.045, 0.01) == pytest.approx((4.1166666666666673e-05, 4.1166666666666673e-05, 8.1e-05))
    assert sphere_inertia(0.03, 0.012) == pytest.approx((1.728e-06,) * 3)


def test_raycast_room():
    angles = beam_angles(4)                       # -180, -90, 0, +90 degrees
    r = raycast_room(2.0, 1.0, 0.0, angles)
    assert r == pytest.approx([2.0, 1.0, 2.0, 2.0])
    r = raycast_room(2.0, 1.0, math.pi / 2, angles)  # facing +y: ahead is the y = 3 wall
    assert r == pytest.approx([1.0, 2.0, 2.0, 2.0])
    assert np.isinf(raycast_room(5.0, 1.0, 0.0, angles)).all()


def test_explain_tf_errors():
    future = ('Lookup would require extrapolation into the future.  Requested time 1789605455.383672 '
              'but the latest data is at time 1789605455.361462, when looking up transform from '
              'frame [base_footprint] to frame [map]')
    d = explain(future)
    assert d.kind == 'future' and d.gap_s == pytest.approx(0.02221, abs=1e-5)
    assert d.frames == ('base_footprint', 'map') and 'Time()' in d.hint
    past = ('Lookup would require extrapolation into the past.  Requested time 1789605579.564709 '
            'but the earliest data is at time 1789605598.597879, when looking up transform from '
            'frame [base_footprint] to frame [odom]')
    assert 'older than the buffer' in explain(past).hint
    sim = ('Lookup would require extrapolation into the past.  Requested time 13838.398000 but the '
           'earliest data is at time 1789605220.975972, when looking up transform from frame [a] to frame [b]')
    assert 'two different clocks' in explain(sim).hint
    assert explain('"kitchen" passed to lookupTransform argument source_frame does not exist. ').kind == 'missing-frame'
    unconnected = ("Could not find a connection between 'map' and 'table' because they are not part "
                   "of the same tree.Tf has two or more unconnected trees.")
    assert explain(unconnected).frames == ('map', 'table')
