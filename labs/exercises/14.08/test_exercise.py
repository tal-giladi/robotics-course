"""Checker for 14.08 — the joint map and the JointTrajectory builder.

Run: ``python course.py check 14.08`` (or ``--solution`` to see the reference pass).
No ROS and no hardware: every field name matches the real messages, but these are plain dicts.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}
# A plausible measured calibration: three joints mounted the other way round.
MEASURED = {"shoulder_pan": (1, 3.0), "shoulder_lift": (-1, -12.0), "elbow_flex": (-1, 5.5),
            "wrist_flex": (1, 0.0), "wrist_roll": (-1, 2.0)}


def calibrations(impl):
    return {n: impl.JointCalibration(sign=s, offset_deg=o) for n, (s, o) in MEASURED.items()}


# --- JointCalibration -----------------------------------------------------------------------------
def test_zero_pose_reading_maps_to_urdf_zero(impl):
    for name, (sign, offset) in MEASURED.items():
        c = impl.JointCalibration(sign=sign, offset_deg=offset)
        assert c.to_urdf_rad(offset) == pytest.approx(0.0, abs=1e-15), name


def test_sign_flips_the_direction(impl):
    positive = impl.JointCalibration(sign=1, offset_deg=10.0)
    negative = impl.JointCalibration(sign=-1, offset_deg=10.0)
    assert positive.to_urdf_rad(40.0) == pytest.approx(math.radians(30.0))
    assert negative.to_urdf_rad(40.0) == pytest.approx(math.radians(-30.0))


def test_offset_is_subtracted_before_the_sign_is_applied(impl):
    """sign * (deg - offset), not sign * deg - offset. The two agree only when offset is 0."""
    c = impl.JointCalibration(sign=-1, offset_deg=20.0)
    assert c.to_urdf_rad(0.0) == pytest.approx(math.radians(20.0))


def test_round_trip_is_exact(impl):
    rng = np.random.default_rng(1408)
    for name, c in calibrations(impl).items():
        lo, hi = LIMITS[name]
        for value in rng.uniform(lo, hi, size=100):
            back = c.to_urdf_rad(c.to_lerobot_deg(value))
            assert back == pytest.approx(value, abs=1e-12), name


def test_rejects_an_impossible_sign(impl):
    for bad in (0, 2, -2):
        with pytest.raises(ValueError):
            impl.JointCalibration(sign=bad)


# --- GripperCalibration ---------------------------------------------------------------------------
def test_gripper_endpoints(impl):
    g = impl.GripperCalibration()
    assert g.to_urdf_rad(0.0) == pytest.approx(-0.174533)
    assert g.to_urdf_rad(100.0) == pytest.approx(1.74533)
    assert g.to_lerobot_percent(-0.174533) == pytest.approx(0.0, abs=1e-9)
    assert g.to_lerobot_percent(1.74533) == pytest.approx(100.0, abs=1e-9)


def test_gripper_is_linear(impl):
    g = impl.GripperCalibration()
    middle = g.to_urdf_rad(50.0)
    assert middle == pytest.approx((-0.174533 + 1.74533) / 2.0)


def test_gripper_round_trip(impl):
    g = impl.GripperCalibration(closed_percent=5.0, open_percent=95.0)
    for percent in (5.0, 27.5, 60.0, 95.0):
        assert g.to_lerobot_percent(g.to_urdf_rad(percent)) == pytest.approx(percent, abs=1e-9)


def test_gripper_rejects_a_degenerate_span(impl):
    with pytest.raises(ValueError):
        impl.GripperCalibration(closed_percent=40.0, open_percent=40.0)
    with pytest.raises(ValueError):
        impl.GripperCalibration(closed_rad=1.0, open_rad=1.0)


# --- clamp_to_limits -------------------------------------------------------------------------------
def test_clamp_leaves_legal_values_alone(impl):
    q = {"shoulder_pan": 0.5, "elbow_flex": -1.0}
    clamped, notes = impl.clamp_to_limits(q, LIMITS)
    assert clamped == q and notes == []


def test_clamp_reports_what_it_changed(impl):
    clamped, notes = impl.clamp_to_limits({"shoulder_pan": 3.0, "elbow_flex": 0.1}, LIMITS)
    assert clamped["shoulder_pan"] == pytest.approx(1.91986)
    assert clamped["elbow_flex"] == pytest.approx(0.1)
    assert len(notes) == 1 and "shoulder_pan" in notes[0]


def test_clamp_handles_both_ends(impl):
    clamped, _ = impl.clamp_to_limits({"wrist_roll": -9.0}, LIMITS)
    assert clamped["wrist_roll"] == pytest.approx(-2.74385)


def test_clamp_raises_on_an_unknown_joint(impl):
    with pytest.raises(KeyError):
        impl.clamp_to_limits({"elbow_twist": 0.0}, LIMITS)


# --- measure_calibration ---------------------------------------------------------------------------
def probe_data(deltas_servo_deg, deltas_urdf_deg=(30.0, 30.0, 20.0, 25.0, 40.0)):
    zero = {n: o for n, (_, o) in MEASURED.items()}
    probe = {n: zero[n] + d for n, d in zip(ARM_JOINTS, deltas_servo_deg)}
    urdf = {n: math.radians(d) for n, d in zip(ARM_JOINTS, deltas_urdf_deg)}
    return zero, probe, urdf


def test_recovers_the_true_signs_and_offsets(impl):
    zero, probe, urdf = probe_data((30.0, -30.0, -20.0, 25.0, -40.0))
    got = impl.measure_calibration(zero, probe, urdf)
    for name, (sign, offset) in MEASURED.items():
        assert got[name].sign == sign, name
        assert got[name].offset_deg == pytest.approx(offset), name


def test_the_recovered_map_reproduces_the_readings(impl):
    zero, probe, urdf = probe_data((30.0, -30.0, -20.0, 25.0, -40.0))
    got = impl.measure_calibration(zero, probe, urdf)
    for name in ARM_JOINTS:
        assert got[name].to_urdf_rad(probe[name]) == pytest.approx(urdf[name], abs=1e-12), name


def test_refuses_a_joint_that_did_not_move(impl):
    zero, probe, urdf = probe_data((30.0, -30.0, 0.5, 25.0, -40.0))
    with pytest.raises(ValueError, match="elbow_flex"):
        impl.measure_calibration(zero, probe, urdf)


def test_refuses_a_probe_angle_that_is_too_small(impl):
    zero, probe, urdf = probe_data((30.0, -30.0, -20.0, 25.0, -40.0),
                                   deltas_urdf_deg=(30.0, 30.0, 20.0, 1.0, 40.0))
    with pytest.raises(ValueError, match="wrist_flex"):
        impl.measure_calibration(zero, probe, urdf)


# --- build_joint_trajectory -------------------------------------------------------------------------
def simple(impl, n_points=4, with_velocity=False):
    t = np.linspace(0.0, 0.6, n_points)
    q = np.zeros((n_points, 5))
    q[:, 0] = np.linspace(0.0, 0.5, n_points)
    qd = np.zeros_like(q) if with_velocity else None
    return impl.build_joint_trajectory(ARM_JOINTS, t, q, qd)


def test_trajectory_shape_and_names(impl):
    traj = simple(impl)
    assert traj["joint_names"] == list(ARM_JOINTS)
    assert len(traj["points"]) == 4
    assert len(traj["points"][0]["positions"]) == 5


def test_time_from_start_comes_from_t_not_from_the_index(impl):
    traj = simple(impl)
    times = [p["time_from_start"] for p in traj["points"]]
    assert times == pytest.approx([0.0, 0.2, 0.4, 0.6])


def test_velocities_are_absent_when_not_given(impl):
    traj = simple(impl, with_velocity=False)
    assert "velocities" not in traj["points"][0]


def test_velocities_are_present_when_given(impl):
    traj = simple(impl, with_velocity=True)
    assert traj["points"][0]["velocities"] == [0.0] * 5


def test_rejects_mismatched_shapes(impl):
    with pytest.raises(ValueError):
        impl.build_joint_trajectory(ARM_JOINTS, np.zeros(4), np.zeros((4, 3)))
    with pytest.raises(ValueError):
        impl.build_joint_trajectory(ARM_JOINTS, np.zeros(4), np.zeros((5, 5)))
    with pytest.raises(ValueError):
        impl.build_joint_trajectory(ARM_JOINTS, np.zeros(4), np.zeros((4, 5)), np.zeros((4, 3)))


# --- check_trajectory ------------------------------------------------------------------------------
def test_a_good_trajectory_has_no_problems(impl):
    t = np.linspace(0.0, 1.0, 20)
    q = np.zeros((20, 5))
    q[:, 2] = np.linspace(0.0, 1.0, 20)
    assert impl.check_trajectory(impl.build_joint_trajectory(ARM_JOINTS, t, q), LIMITS) == []


def test_catches_a_limit_violation_and_names_the_joint(impl):
    t = np.linspace(0.0, 1.0, 5)
    q = np.zeros((5, 5))
    q[3, 2] = 2.0                                   # elbow_flex limit is 1.69
    problems = impl.check_trajectory(impl.build_joint_trajectory(ARM_JOINTS, t, q), LIMITS)
    assert len(problems) == 1 and "elbow_flex" in problems[0] and "3" in problems[0]


def test_catches_time_that_does_not_advance(impl):
    traj = {"joint_names": list(ARM_JOINTS), "points": [
        {"positions": [0.0] * 5, "time_from_start": 0.0},
        {"positions": [0.0] * 5, "time_from_start": 0.0},
        {"positions": [0.0] * 5, "time_from_start": 0.2},
    ]}
    problems = impl.check_trajectory(traj, LIMITS)
    assert len(problems) == 1 and "1" in problems[0]


def test_catches_negative_time(impl):
    traj = {"joint_names": list(ARM_JOINTS), "points": [
        {"positions": [0.0] * 5, "time_from_start": -0.1},
        {"positions": [0.0] * 5, "time_from_start": 0.2},
    ]}
    assert any("negative" in p for p in impl.check_trajectory(traj, LIMITS))


def test_catches_a_wrong_number_of_positions(impl):
    traj = {"joint_names": list(ARM_JOINTS), "points": [
        {"positions": [0.0, 0.0], "time_from_start": 0.1},
    ]}
    assert any("2 positions" in p for p in impl.check_trajectory(traj, LIMITS))


def test_catches_an_unknown_joint_name_without_raising(impl):
    traj = {"joint_names": ["shoulder_pan", "elbow_twist"], "points": [
        {"positions": [0.0, 0.0], "time_from_start": 0.1},
    ]}
    problems = impl.check_trajectory(traj, LIMITS)
    assert any("elbow_twist" in p for p in problems)


def test_check_never_raises_on_an_empty_trajectory(impl):
    assert impl.check_trajectory({"joint_names": [], "points": []}, LIMITS) == []
