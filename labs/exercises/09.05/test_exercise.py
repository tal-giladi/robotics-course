"""Checker for 09.05 — Calibrating odometry from logged runs.

Run: ``python course.py check 09.05`` (or ``--solution`` to see the reference pass).
The logged runs are a recording of one specific (simulated) robot, so the robot numbers come from
the self-describing ``logged_runs.json``, not from karmel.yaml.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
approx = pytest.approx


def load_truth() -> dict[str, float]:
    spec = importlib.util.spec_from_file_location("course_exercise_09_05_make_logs", HERE / "make_logs.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.TRUE_SCALES


@pytest.fixture(scope="module")
def data() -> dict:
    return json.loads((HERE / "logged_runs.json").read_text(encoding="utf-8"))


def runs_of(data: dict, *kinds: str) -> list[tuple[list[list[int]], tuple[float, float, float]]]:
    return [(r["ticks"], tuple(r["measured_pose"])) for r in data["runs"] if r["kind"] in kinds]


def nominal(data: dict) -> tuple[float, float, float]:
    n = data["nominal"]
    return n["wheel_radius_m"], n["wheel_radius_m"], n["wheel_separation_m"]


def umbmark(impl, data: dict, params) -> object:
    errors: dict[str, list[tuple[float, float]]] = {"cw": [], "ccw": []}
    for run in data["runs"]:
        if run["kind"] in errors:
            x, y, _ = impl.replay_odometry(run["ticks"], *params, data["ticks_per_rev"])
            errors[run["kind"]].append((run["measured_pose"][0] - x, run["measured_pose"][1] - y))
    return impl.umbmark_summary(errors["cw"], errors["ccw"])


# --- linear least squares -----------------------------------------------------------------------
def test_fit_wheel_radius(impl):
    # r = (10*0.45 + 20*0.91 + 40*1.79) / (10^2 + 20^2 + 40^2) = 94.3 / 2100
    assert impl.fit_wheel_radius([10.0, 20.0, 40.0], [0.45, 0.91, 1.79]) == approx(0.0449048, rel=1e-5)


def test_fit_wheel_radius_exact_data(impl):
    angles = [5.0, 12.5, 30.0]
    assert impl.fit_wheel_radius(angles, [0.0451 * a for a in angles]) == approx(0.0451)


def test_fit_wheel_separation(impl):
    # s = right - left = [1.0, 2.0]; b = (1*5.0 + 2*10.1) / (5.0^2 + 10.1^2) = 25.2 / 127.01
    assert impl.fit_wheel_separation([-0.5, -1.0], [0.5, 1.0], [5.0, 10.1]) == approx(0.1984096, rel=1e-5)


# --- UMBmark and Borenstein ---------------------------------------------------------------------
def test_umbmark_summary(impl):
    result = impl.umbmark_summary([(0.3, -0.4), (0.1, -0.2)], [(-0.1, 0.05), (0.1, 0.15)])
    assert result.cg_cw == approx((0.2, -0.3))
    assert result.cg_ccw == approx((0.0, 0.1))
    assert result.e_max_syst == approx(math.hypot(0.2, 0.3))


def test_borenstein_hand_computed(impl):
    # alpha = -0.1 / -8 = 0.0125 rad, beta = -0.3 / -8 = 0.0375 rad, R = 1 / sin(0.01875) = 53.34 m
    c_left, c_right, e_b = impl.borenstein_correction(-0.2, 0.1, 2.0, 0.2)
    assert (c_left, c_right, e_b) == approx((0.9981251, 1.0018749, 1.0080216), rel=1e-6)


def test_borenstein_detects_a_bigger_right_wheel(impl):
    # Return errors of a perfect-wheelbase robot whose right wheel is 0.5 % larger (2 m square).
    c_left, c_right, e_b = impl.borenstein_correction(-0.1702, 0.2313, 2.0, 0.2)
    assert c_right / c_left == approx(1.005, abs=5e-4), "E_d = D_right / D_left should come out near 1.005"
    assert e_b == approx(1.0, abs=0.01)


def test_borenstein_detects_a_wider_wheelbase(impl):
    # Return errors of a robot whose true wheelbase is 4 % wider (2 m square): both x errors equal.
    c_left, c_right, e_b = impl.borenstein_correction(-0.2252, -0.2252, 2.0, 0.2)
    assert (c_left, c_right) == approx((1.0, 1.0))
    assert e_b == approx(1.04, abs=0.005), "first-order formula: about 1.037 for a true 1.04"


# --- replaying logs -----------------------------------------------------------------------------
def test_replay_straight(impl):
    pose = impl.replay_odometry([[0, 0], [1232, 1232], [2464, 2464]], 0.045, 0.045, 0.2, 2464)
    assert pose == approx((2 * math.pi * 0.045, 0.0, 0.0), abs=1e-12)


def test_replay_spin_with_nonzero_start_counts(impl):
    # 308 ticks each way with r = 0.05: each wheel 0.03927 m, dtheta = 0.07854 / 0.2 = pi/8
    pose = impl.replay_odometry([[100, -50], [-208, 258]], 0.05, 0.05, 0.2, 2464)
    assert pose == approx((0.0, 0.0, math.pi / 8), abs=1e-12)


def test_replay_uses_separate_radii(impl):
    # Same ticks, right wheel 1 % bigger: the "straight" run curves left.
    x, y, theta = impl.replay_odometry([[0, 0], [2464, 2464]], 0.045, 0.045 * 1.01, 0.2, 2464)
    assert theta == approx(2 * math.pi * 0.045 * 0.01 / 0.2)
    assert y > 0


def test_pose_residuals_shape_and_weight(impl):
    runs = [([[0, 0], [2464, 2464]], (0.28, 0.01, 0.1))]
    r = impl.pose_residuals((0.045, 0.045, 0.2), runs, 2464, heading_weight_m=2.0)
    assert np.asarray(r) == approx([2 * math.pi * 0.045 - 0.28, -0.01, -0.2])


# --- calibration --------------------------------------------------------------------------------
def synthetic_runs(impl, true_params, ticks_per_rev=2464):
    """Noise-free logs: the 'measured' pose is the odometry of the true parameters."""
    rng = np.random.default_rng(3)
    runs = []
    for _ in range(8):
        steps = rng.integers(-40, 120, size=(60, 2))
        ticks = np.vstack([[0, 0], np.cumsum(steps, axis=0)]).tolist()
        runs.append((ticks, impl.replay_odometry(ticks, *true_params, ticks_per_rev)))
    return runs


def test_calibrate_recovers_noise_free_parameters(impl):
    truth = (0.0442, 0.0457, 0.212)
    found = impl.calibrate_least_squares(synthetic_runs(impl, truth), 2464, (0.045, 0.045, 0.2))
    assert found == approx(truth, rel=1e-4)


def test_logged_umbmark_is_bad_before_calibration(impl, data):
    assert umbmark(impl, data, nominal(data)).e_max_syst > 1.0, "uncalibrated E_max,syst should be over 1 m"


def test_borenstein_improves_the_logged_robot(impl, data):
    before = umbmark(impl, data, nominal(data))
    r_left, r_right, separation = nominal(data)
    c_left, c_right, e_b = impl.borenstein_correction(
        before.cg_cw[0], before.cg_ccw[0], data["square_side_m"], separation
    )
    after = umbmark(impl, data, (r_left * c_left, r_right * c_right, separation * e_b))
    assert after.e_max_syst < 0.6 * before.e_max_syst


def test_least_squares_calibration_of_the_logged_robot(impl, data):
    truth = load_truth()
    r0, _, b0 = nominal(data)
    found = impl.calibrate_least_squares(runs_of(data, "cw", "ccw", "straight", "spin"), data["ticks_per_rev"], nominal(data))
    expected = (r0 * truth["wheel_radius_scale_left"], r0 * truth["wheel_radius_scale_right"], b0 * truth["wheel_separation_scale"])
    assert found == approx(expected, rel=0.01), f"calibrated {found}, hidden truth {expected}"
    ratio = found[1] / found[0]
    assert ratio == approx(expected[1] / expected[0], rel=0.002), "the wheel diameter ratio is well observed"
    after = umbmark(impl, data, found)
    assert after.e_max_syst < 0.05, f"E_max,syst after calibration should be a few cm, got {after.e_max_syst:.3f} m"
