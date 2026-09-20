"""Checker for 14.04 — forward kinematics of a URDF-style serial chain.

Run: ``python course.py check 14.04`` (or ``--solution`` to see the reference pass).

The reference poses below come from the published SO-101 URDF
(``14-robotic-arm/code/data/so101_new_calib.urdf``, Apache-2.0) and match
``arm_kinematics.load_so101().fk(q)`` to machine precision.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

ZEROS = np.zeros(5)

# T_base_tool at q = 0. The URDF's 3.14159-for-pi rounding is why the zeros are 1e-5, not 0.
FK_ZERO = np.array([
    [8.665019265738e-06, -1.030036843928e-05, 9.999999999094e-01, 3.913614702202e-01],
    [4.866292685830e-02, 9.988152579192e-01, 9.866499961486e-06, -9.212063114603e-06],
    [-9.988152579304e-01, 4.866292676840e-02, 9.155999528757e-06, 2.264697102403e-01],
    [0.0, 0.0, 0.0, 1.0],
])

# T_base_tool at q = (20, -30, 40, 60, 0) degrees — the pose `arm_viz.py so101 --q 20 -30 40 60 0` draws.
Q_POSE = np.radians([20.0, -30.0, 40.0, 60.0, 0.0])
FK_POSE = np.array([
    [-0.865332823739, 0.384575968405, 0.321403840495, 0.207178823283],
    [0.366733162331, 0.922943192093, -0.116972867860, -0.061282525181],
    [-0.341622440432, 0.016648984764, -0.939689799614, 0.057457106621],
    [0.0, 0.0, 0.0, 1.0],
])

# Joint-axis origins in base_link at q = 0 (the "frames" column of arm_kinematics' summary).
ORIGINS_AT_ZERO = {
    "shoulder_link": (0.038835, 0.0, 0.0624),
    "upper_arm_link": (0.069235, -0.018278, 0.116600),
    "lower_arm_link": (0.097234, -0.018278, 0.229170),
    "wrist_link": (0.232134, -0.018277, 0.234370),
    "gripper_link": (0.293234, -0.000177, 0.234370),
    "gripper_frame_link": (0.391361, -0.000009, 0.226470),
}


def random_q(impl, n: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    lower = np.array([j.lower for j in impl.SO101_JOINTS if j.type == "revolute"])
    upper = np.array([j.upper for j in impl.SO101_JOINTS if j.type == "revolute"])
    return rng.uniform(lower, upper, size=(n, lower.size))


# --- rpy_to_matrix -------------------------------------------------------------------------------
def test_rpy_of_zeros_and_of_single_axes(impl):
    assert impl.rpy_to_matrix(0.0, 0.0, 0.0) == pytest.approx(np.eye(3), abs=1e-15)
    a = 0.7
    assert impl.rpy_to_matrix(a, 0.0, 0.0) == pytest.approx(impl.rot_x(a), abs=1e-15)
    assert impl.rpy_to_matrix(0.0, a, 0.0) == pytest.approx(impl.rot_y(a), abs=1e-15)
    assert impl.rpy_to_matrix(0.0, 0.0, a) == pytest.approx(impl.rot_z(a), abs=1e-15)


def test_rpy_order_is_yaw_pitch_roll(impl):
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll). The reversed order is a different rotation."""
    r, p, y = 0.3, -0.4, 1.1
    expected = impl.rot_z(y) @ impl.rot_y(p) @ impl.rot_x(r)
    got = impl.rpy_to_matrix(r, p, y)
    assert got == pytest.approx(expected, abs=1e-15)
    assert not np.allclose(got, impl.rot_x(r) @ impl.rot_y(p) @ impl.rot_z(y))
    assert got.T @ got == pytest.approx(np.eye(3), abs=1e-14)
    assert np.linalg.det(got) == pytest.approx(1.0, abs=1e-14)


# --- axis_angle_to_matrix ------------------------------------------------------------------------
def test_rodrigues_about_the_principal_axes(impl):
    a = 0.9
    assert impl.axis_angle_to_matrix((1, 0, 0), a) == pytest.approx(impl.rot_x(a), abs=1e-14)
    assert impl.axis_angle_to_matrix((0, 1, 0), a) == pytest.approx(impl.rot_y(a), abs=1e-14)
    assert impl.axis_angle_to_matrix((0, 0, 1), a) == pytest.approx(impl.rot_z(a), abs=1e-14)
    # a URDF may write [0 0 2]; the length of the axis is not part of the angle
    assert impl.axis_angle_to_matrix((0, 0, 2.0), 0.5) == pytest.approx(impl.rot_z(0.5), abs=1e-14)
    assert impl.axis_angle_to_matrix((3.0, 4.0, 0.0), 0.5) == pytest.approx(
        impl.axis_angle_to_matrix((0.6, 0.8, 0.0), 0.5), abs=1e-14)


def test_rodrigues_leaves_its_own_axis_alone(impl):
    axis = np.array([0.3, -0.5, 0.81])
    R = impl.axis_angle_to_matrix(axis, 1.234)
    assert R @ axis == pytest.approx(axis, abs=1e-14)
    assert R.T @ R == pytest.approx(np.eye(3), abs=1e-14)


def test_rodrigues_rejects_a_zero_axis(impl):
    with pytest.raises(ValueError):
        impl.axis_angle_to_matrix((0.0, 0.0, 0.0), 0.5)


# --- joint_transform -----------------------------------------------------------------------------
def test_joint_transform_at_zero_is_the_origin(impl):
    j = impl.SO101_JOINTS[1]
    expected = impl.se3(impl.rpy_to_matrix(*j.rpy), j.xyz)
    assert impl.joint_transform(j, 0.0) == pytest.approx(expected, abs=1e-15)


def test_a_turning_joint_does_not_move_its_own_origin(impl):
    """``origin @ Rot(axis, q)``, never ``Rot(axis, q) @ origin``.

    The axis is expressed in the CHILD frame, so the rotation multiplies on the right and the
    child frame's origin stays put. Build it the other way round and the answer is still exactly
    right at q = 0 — and wrong everywhere else.
    """
    for j in impl.SO101_JOINTS:
        if j.type != "revolute":
            continue
        for q in (-0.9, 0.0, 0.4, 1.3):
            assert impl.joint_transform(j, q)[:3, 3] == pytest.approx(j.xyz, abs=1e-15), j.name


def test_fixed_joints_ignore_q(impl):
    fixed = impl.SO101_JOINTS[-1]
    assert fixed.type == "fixed"
    assert impl.joint_transform(fixed, 0.0) == pytest.approx(impl.joint_transform(fixed, 1.7), abs=1e-15)


def test_an_unsupported_joint_type_raises(impl):
    prismatic = impl.Joint("slide", "prismatic", "a", "b", (0.1, 0.0, 0.0), (0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        impl.joint_transform(prismatic, 0.2)


# --- chain_frames --------------------------------------------------------------------------------
def test_chain_frames_returns_one_frame_per_joint_in_order(impl):
    frames = impl.chain_frames(impl.SO101_JOINTS, ZEROS)
    assert [name for name, _ in frames] == [j.child for j in impl.SO101_JOINTS]
    assert all(T.shape == (4, 4) for _, T in frames)


def test_chain_frames_at_zero_match_the_urdf(impl):
    for name, T in impl.chain_frames(impl.SO101_JOINTS, ZEROS):
        assert T[:3, 3] == pytest.approx(ORIGINS_AT_ZERO[name], abs=1e-6), name


def test_each_revolute_frame_sits_on_its_own_axis(impl):
    """Turning joint j must not move joint j's own frame origin — that is what "on the axis" means."""
    names = [j.child for j in impl.SO101_JOINTS]
    base = {n: T[:3, 3].copy() for n, T in impl.chain_frames(impl.SO101_JOINTS, ZEROS)}
    for k in range(5):
        q = ZEROS.copy()
        q[k] = 0.8
        moved = dict(impl.chain_frames(impl.SO101_JOINTS, q))
        assert moved[names[k]][:3, 3] == pytest.approx(base[names[k]], abs=1e-12), names[k]
        # ... and everything before it is untouched as well
        for earlier in names[:k]:
            assert moved[earlier][:3, 3] == pytest.approx(base[earlier], abs=1e-12)


def test_chain_frames_rejects_bad_input(impl):
    with pytest.raises(ValueError):
        impl.chain_frames(impl.SO101_JOINTS, np.zeros(4))
    with pytest.raises(ValueError):
        impl.chain_frames(impl.SO101_JOINTS, np.zeros(6))
    joints = list(impl.SO101_JOINTS)
    joints[2], joints[3] = joints[3], joints[2]
    with pytest.raises(ValueError):
        impl.chain_frames(tuple(joints), ZEROS)


def test_chain_frames_hands_out_independent_arrays(impl):
    frames = impl.chain_frames(impl.SO101_JOINTS, ZEROS)
    frames[0][1][0, 3] = 99.0
    assert impl.chain_frames(impl.SO101_JOINTS, ZEROS)[0][1][0, 3] != 99.0
    assert frames[1][1][0, 3] != 99.0


# --- fk ------------------------------------------------------------------------------------------
def test_fk_at_the_zero_pose(impl):
    assert impl.fk(impl.SO101_JOINTS, ZEROS) == pytest.approx(FK_ZERO, abs=1e-9)


def test_fk_at_a_real_pose(impl):
    assert impl.fk(impl.SO101_JOINTS, Q_POSE) == pytest.approx(FK_POSE, abs=1e-9)


def test_fk_is_the_last_chain_frame_and_always_a_valid_pose(impl):
    for q in random_q(impl, 100, seed=8):
        T = impl.fk(impl.SO101_JOINTS, q)
        assert T == pytest.approx(impl.chain_frames(impl.SO101_JOINTS, q)[-1][1], abs=0.0)
        assert T[3] == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-15)
        R = T[:3, :3]
        assert R.T @ R == pytest.approx(np.eye(3), abs=1e-12)
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-12)


def test_shoulder_pan_only_spins_the_arm_about_its_own_axis(impl):
    """The tool's distance to the pan axis, and its height along it, cannot depend on the pan angle.

    The axis is vertical but it does **not** pass through ``base_link``'s origin: it is 38.8 mm
    forward and 62.4 mm up. Measuring the radius from the base origin instead of from the axis is
    the mistake this test catches.
    """
    q = np.radians([0.0, -30.0, 40.0, 60.0, 0.0])
    T_shoulder = impl.chain_frames(impl.SO101_JOINTS, q)[0][1]
    axis_point, axis_dir = T_shoulder[:3, 3], T_shoulder[:3, 2]
    assert abs(abs(axis_dir[2]) - 1.0) < 1e-4, "shoulder_pan's axis should be (near) vertical"

    def radius_and_height(p):
        d = p - axis_point
        h = float(d @ axis_dir)
        return float(np.linalg.norm(d - h * axis_dir)), h

    r0, h0 = radius_and_height(impl.fk(impl.SO101_JOINTS, q)[:3, 3])
    for pan_deg in (15.0, -37.0, 90.0):
        q_pan = q.copy()
        q_pan[0] = math.radians(pan_deg)
        r, h = radius_and_height(impl.fk(impl.SO101_JOINTS, q_pan)[:3, 3])
        assert (r, h) == pytest.approx((r0, h0), abs=1e-9)


def test_the_planar_pose_of_exercise_e5(impl):
    """q = (0, -30, 40, 60, 0) deg puts the tool at (0.2180, 0.0000, 0.0575) m (14.04-E5)."""
    p = impl.fk(impl.SO101_JOINTS, np.radians([0.0, -30.0, 40.0, 60.0, 0.0]))[:3, 3]
    assert p == pytest.approx((0.2180, 0.0000, 0.0575), abs=5e-5)


# --- approach_pitch ------------------------------------------------------------------------------
def pose_with_approach(ax: float, ay: float, az: float) -> np.ndarray:
    """A 4x4 whose z axis (the approach direction) is the given unit vector."""
    z = np.array([ax, ay, az], dtype=float)
    helper = np.array([0.0, 1.0, 0.0]) if abs(z[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(helper, z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = x, np.cross(z, x), z
    return T


def test_pitch_of_a_horizontal_tool_is_zero(impl):
    assert impl.approach_pitch(pose_with_approach(1.0, 0.0, 0.0)) == pytest.approx(0.0, abs=1e-15)
    assert impl.approach_pitch(pose_with_approach(0.0, 1.0, 0.0)) == pytest.approx(0.0, abs=1e-15)
    # the SO-101's zero pose points the gripper along base +x, i.e. horizontal
    assert math.degrees(impl.approach_pitch(impl.fk(impl.SO101_JOINTS, ZEROS))) == pytest.approx(0.0, abs=0.01)


def test_pitch_straight_down_and_straight_up(impl):
    assert impl.approach_pitch(pose_with_approach(0.0, 0.0, -1.0)) == pytest.approx(math.pi / 2, abs=1e-12)
    assert impl.approach_pitch(pose_with_approach(0.0, 0.0, 1.0)) == pytest.approx(-math.pi / 2, abs=1e-12)
    half = math.sqrt(0.5)
    assert impl.approach_pitch(pose_with_approach(half, 0.0, -half)) == pytest.approx(math.pi / 4, abs=1e-12)


def test_pitch_at_a_known_pose_and_independent_of_position(impl):
    T = impl.fk(impl.SO101_JOINTS, np.radians([0.0, -30.0, 40.0, 60.0, 0.0]))
    assert math.degrees(impl.approach_pitch(T)) == pytest.approx(70.0, abs=0.01)
    shifted = T.copy()
    shifted[:3, 3] += (0.1, -0.2, 0.3)
    assert impl.approach_pitch(shifted) == pytest.approx(impl.approach_pitch(T), abs=0.0)
