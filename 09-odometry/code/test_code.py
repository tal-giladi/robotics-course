"""Tests for the module-09 scripts (reference exercise solutions, no hardware): py -m pytest 09-odometry/code"""

from __future__ import annotations

import math
import random

import numpy as np
import pytest

import course_code
import diff_drive_numbers as ddn
import drift_monte_carlo as mc
import integration_experiment
import saturation_demo
import umbmark_run
from robotlab.config import load_config


def test_forward_kinematics_karmel_arc():
    m = ddn.forward_kinematics(4.0, 8.0, 0.045, 0.2)
    assert (m.v, m.omega, m.radius) == pytest.approx((0.27, 0.9, 0.3))


def test_forward_kinematics_straight_and_spin():
    assert math.isinf(ddn.forward_kinematics(6.0, 6.0, 0.045, 0.2).radius)
    spin = ddn.forward_kinematics(-5.0, 5.0, 0.045, 0.2)
    assert (spin.v, spin.omega, spin.radius) == pytest.approx((0.0, 2.25, 0.0))


def test_ticks_per_period():
    assert ddn.ticks_per_period(0.3, 0.045, 2464, 0.02) == pytest.approx(52.28, abs=0.01)


def test_wheel_odometry_matches_09_04_solution():
    ref = course_code.load_exercise("09.04", solution=True)
    odom = course_code.WheelOdometry(0.045, 0.045, 0.2, 2464)
    pose = (0.0, 0.0, 0.0)
    rng = random.Random(4)
    left = right = 0
    odom.update(left, right)
    for _ in range(300):
        dl, dr = rng.randint(-30, 60), rng.randint(-30, 60)
        left, right = left + dl, right + dr
        odom.update(left, right)
        pose = ref.integrate_pose(pose, ref.ticks_to_distance(dl, 0.045, 2464), ref.ticks_to_distance(dr, 0.045, 2464), 0.2)
    assert odom.pose == pytest.approx(pose, abs=1e-9)


def test_drive_segments_closes_a_square_on_the_ideal_robot():
    d = load_config().drive
    base = course_code.open_base(None, realistic=False)
    odom = course_code.WheelOdometry(d.wheel_radius_m, d.wheel_radius_m, d.wheel_separation_m, d.ticks_per_wheel_rev)
    course_code.drive_segments(base, odom, [("line", 1.0), ("turn", math.pi / 2)] * 4)
    truth = base.sim.pose
    assert math.hypot(truth.x, truth.y) < 0.02
    assert abs(math.degrees(math.remainder(truth.theta, 2 * math.pi))) < 1.5


def test_saturation_demo_radius():
    ex = course_code.load_exercise("09.02", solution=True)
    raw = ex.twist_to_wheel_speeds(0.7, 3.0, 0.045, 0.2)
    _, radius = saturation_demo.drive(lambda: ex.scale_to_limit(*raw, 17.0))
    assert radius == pytest.approx(0.7 / 3.0, rel=0.03)


def test_integration_table_orders():
    table = integration_experiment.error_table(course_code.load_exercise("09.03", solution=True))
    assert table["euler"][2] / table["euler"][3] == pytest.approx(2.0, rel=0.05)
    assert table["midpoint"][2] / table["midpoint"][3] == pytest.approx(4.0, rel=0.05)


def test_umbmark_plans():
    plans = umbmark_run.plans(1.5, 5, extra=True)
    assert [kind for kind, _ in plans].count("cw") == 5
    assert len(plans) == 16
    assert plans[0][1][1] == ("turn", -math.pi / 2)


def test_predicted_sigmas_values():
    pred = mc.predicted_sigmas(np.array([5.0]), 2e-6, 0.2)
    assert pred["heading"][0] == pytest.approx(math.sqrt(2 * 2e-6 / 0.04 * 5))
    assert pred["lateral"][0] == pytest.approx(math.sqrt(2 * 2e-6 / 0.04 * 125 / 3))


def test_covariance_propagation_matches_closed_form():
    k, b, step = 2e-6, 0.2, 0.005
    cov = np.zeros((3, 3))
    for _ in range(1000):  # 5 m straight along x
        cov = mc.propagate_covariance((0.0, 0.0, 0.0), cov, step, step, b, k)
    pred = mc.predicted_sigmas(np.array([5.0]), k, b)
    assert math.sqrt(cov[1, 1]) == pytest.approx(pred["lateral"][0], rel=0.01)
    assert math.sqrt(cov[2, 2]) == pytest.approx(pred["heading"][0], rel=1e-6)


def test_monte_carlo_small():
    errors, dist = mc.monte_carlo(runs=4, distance=1.0, step_m=0.5)
    assert errors["nominal"].shape == (4, 2, 3)
    assert list(dist) == [0.5, 1.0]
    # the realistic preset's bigger right wheel curves the true path left: odometry lags to the right
    assert errors["nominal"][:, -1, 1].mean() < -0.01
    assert abs(errors["calibrated"][:, -1, 1].mean()) < 0.01
