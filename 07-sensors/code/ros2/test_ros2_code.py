"""Tests for the ROS-free helpers used by the 07.06-07.11 nodes.

    py -m pytest 07-sensors/code/ros2

Runs on any machine: nothing here imports rclpy. The rclpy nodes in this folder are exercised by
hand in the lessons (see each lesson's "Expected result"), inside the course Docker image.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import imu_conventions as ic  # noqa: E402
import scan_geometry as sg  # noqa: E402
import sync_math as sm  # noqa: E402


# --- scan_geometry ---------------------------------------------------------------------------
def test_360_scan_does_not_repeat_the_first_beam():
    spec = sg.ScanSpec(n=360)
    assert spec.angle_increment == pytest.approx(math.radians(1.0))
    a = sg.scan_angles(spec)
    assert a[0] == pytest.approx(-math.pi)
    assert a[-1] == pytest.approx(math.pi - math.radians(1.0))
    assert len(a) == 360


def test_beam_zero_is_straight_ahead():
    spec = sg.ScanSpec(n=360)
    a = sg.scan_angles(spec)
    assert a[180] == pytest.approx(0.0, abs=1e-12)          # index 180 = +x = forward
    assert a[270] == pytest.approx(math.pi / 2)             # +90 deg = robot's left (REP-103)


def test_scan_to_points_hand_computed():
    spec = sg.ScanSpec(n=4, angle_min=0.0, angle_max=2 * math.pi)   # 0, 90, 180, 270 degrees
    pts = sg.scan_to_points([2.0, 3.0, 4.0, 5.0], spec)
    np.testing.assert_allclose(pts, [[2, 0], [0, 3], [-4, 0], [0, -5]], atol=1e-9)


def test_invalid_ranges_are_dropped_not_plotted_at_the_origin():
    spec = sg.ScanSpec(n=4, angle_min=0.0, angle_max=2 * math.pi, range_min=0.1, range_max=10.0)
    pts = sg.scan_to_points([2.0, float("inf"), 0.0, float("nan")], spec)
    assert pts.shape == (1, 2)          # only the 2.0 m beam survives
    np.testing.assert_allclose(pts[0], [2.0, 0.0], atol=1e-9)


def test_sector_min_wraps_across_pi_and_returns_inf_when_empty():
    spec = sg.ScanSpec(n=360)
    r = np.full(360, 5.0)
    r[0] = 1.0                                  # the beam at exactly -pi (straight behind)
    assert sg.sector_min(r, spec, math.pi, math.radians(10)) == pytest.approx(1.0)
    r[:] = float("inf")
    assert sg.sector_min(r, spec, 0.0, math.radians(10)) == float("inf")


def test_angular_resolution_becomes_a_spatial_resolution():
    spec = sg.ScanSpec(n=360)
    assert sg.gap_at_range(spec, 1.0) == pytest.approx(0.01745, abs=1e-5)   # 1.7 cm at 1 m
    assert sg.gap_at_range(spec, 5.0) == pytest.approx(0.08727, abs=1e-5)   # 8.7 cm at 5 m
    # a 4 cm chair leg gets two beams only out to ~1.1 m
    assert sg.range_for_gap(spec, 0.04, beams=2) == pytest.approx(1.146, abs=1e-3)


def test_scan_rate_smear():
    trans, arc = sg.scan_rate_smear(scan_hz=10, speed_m_s=0.5, yaw_rate_rad_s=1.0, range_m=3.0)
    assert trans == pytest.approx(0.05)     # 5 cm of travel during one revolution
    assert arc == pytest.approx(0.30)       # 30 cm of smear at 3 m while spinning at 1 rad/s


# --- imu_conventions -------------------------------------------------------------------------
def test_quaternion_round_trip_and_xyzw_order():
    q = ic.quaternion_from_rpy(0.0, 0.0, math.pi / 2)
    assert q[3] == pytest.approx(math.sqrt(0.5))        # w is LAST in geometry_msgs/Quaternion
    assert q[2] == pytest.approx(math.sqrt(0.5))
    rpy = ic.rpy_from_quaternion(*q)
    assert rpy == pytest.approx((0.0, 0.0, math.pi / 2))


def test_identity_quaternion_is_w_equals_one():
    assert ic.quaternion_from_rpy(0, 0, 0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_covariance_holds_variance_not_sigma():
    cov = ic.diag_covariance(0.02)
    assert cov[0] == pytest.approx(4e-4)
    assert cov[1] == 0.0 and len(cov) == 9
    assert ic.variance_from_sigma(0.02) == pytest.approx(cov[0])


def test_noise_density_to_variance():
    # 0.014 (deg/s)/sqrt(Hz) at 100 Hz output: sigma = 0.014 * sqrt(50) = 0.099 deg/s
    var = ic.variance_from_noise_density(0.014, 100.0)
    assert math.sqrt(var) == pytest.approx(0.09899, abs=1e-5)


def test_not_reported_marker():
    assert ic.is_not_reported([-1.0] + [0.0] * 8)
    assert not ic.is_not_reported([0.0] * 9)


def test_axis_remap_rotates_the_board_by_90_degrees():
    # board bolted down rotated +90 deg about z: the chip's +x points along the robot's +y
    m = ic.axis_remap_matrix("y -x z")
    v_robot = ic.remap_vector(np.array([1.0, 0.0, 0.0]), "y -x z")
    np.testing.assert_allclose(v_robot, [0.0, 1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(m), 1.0)      # a rotation, not a mirror


def test_axis_remap_rejects_nonsense():
    with pytest.raises(ValueError):
        ic.axis_remap_matrix("x y")
    with pytest.raises(ValueError):
        ic.axis_remap_matrix("x x z")
    with pytest.raises(ValueError):
        ic.axis_remap_matrix("x y q")


def test_gravity_check_catches_a_driver_reporting_g():
    ok, mag = ic.gravity_check([0.0, 0.0, 9.81])
    assert ok and mag == pytest.approx(9.81)
    ok, mag = ic.gravity_check([0.0, 0.0, 1.0])            # units bug: g instead of m/s^2
    assert not ok and mag == pytest.approx(1.0)


def test_tilt_from_accel():
    roll, pitch = ic.tilt_from_accel([0.0, 0.0, ic.G])
    assert (roll, pitch) == pytest.approx((0.0, 0.0))
    # nose up by 10 degrees moves -g*sin(10) onto x
    roll, pitch = ic.tilt_from_accel([-ic.G * math.sin(math.radians(10)), 0.0,
                                      ic.G * math.cos(math.radians(10))])
    assert math.degrees(pitch) == pytest.approx(10.0, abs=1e-9)


# --- sync_math -------------------------------------------------------------------------------
def test_pair_nearest_pairs_each_slow_message_once():
    scans = [0.00, 0.10, 0.20]
    imu = [t / 100 for t in range(25)]          # 100 Hz
    pairs = sm.pair_nearest(scans, imu, slop=0.05)
    assert [p[0] for p in pairs] == [0, 1, 2]
    assert all(abs(dt) <= 0.05 for _, _, dt in pairs)
    assert len({p[1] for p in pairs}) == 3      # no imu sample reused


def test_tight_slop_drops_pairs():
    scans = [0.003, 0.103, 0.203]
    imu = [t / 100 for t in range(25)]
    assert len(sm.pair_nearest(scans, imu, slop=0.05)) == 3
    assert len(sm.pair_nearest(scans, imu, slop=0.001)) == 0


def test_rate_report_finds_the_gap_the_average_hides():
    stamps = [i * 0.01 for i in range(500)]
    stamps += [stamps[-1] + 0.20 + i * 0.01 for i in range(500)]
    rr = sm.rate_report(stamps, expected_hz=100.0)
    assert rr.mean_hz == pytest.approx(98.1, abs=0.2)       # "looks like 100 Hz"
    assert rr.max_gap_ms == pytest.approx(200.0, abs=1.0)   # but there is one 200 ms hole
    assert rr.dropped_estimate == 19                        # 19 messages went missing in it


def test_stamp_age_stats_detects_a_clock_in_the_future():
    stamps = [1.0, 2.0, 3.0]
    received = [1.02, 2.02, 3.02]
    age = sm.stamp_age_stats(stamps, received)
    assert age["median_ms"] == pytest.approx(20.0)
    future = sm.stamp_age_stats([10.0, 11.0], [1.0, 2.0])
    assert future["max_ms"] < 0                             # stamps ahead of the receiver's clock
