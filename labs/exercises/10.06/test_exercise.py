"""Checker for 10.06 — The Extended Kalman Filter for a differential-drive robot.

Run: ``python course.py check 10.06`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import functools
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from robotlab.config import load_config

approx = pytest.approx
HERE = Path(__file__).resolve().parent
B = 0.2  # wheel separation for the hand-computed cases


def _tour_sim():
    name = "course_exercise_10_06_tour_sim"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, HERE / "tour_sim.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


def _wrap(a):
    return math.pi - (math.pi - a) % (2.0 * math.pi)


def _central_diff(func, x, eps=1e-6, angle_rows=()):
    """The checker's own numerical Jacobian, independent of yours."""
    x = np.asarray(x, dtype=float)
    cols = []
    for j in range(x.size):
        d = np.zeros_like(x)
        d[j] = eps
        diff = np.asarray(func(x + d)) - np.asarray(func(x - d))
        for i in angle_rows:
            diff[i] = _wrap(diff[i])
        cols.append(diff / (2 * eps))
    return np.column_stack(cols)


# --- motion model -------------------------------------------------------------------------------
def test_motion_model_straight_line(impl):
    x_pred, F, G = impl.motion_model([1.0, 2.0, 0.0], [0.1, 0.1], B)
    assert x_pred == approx([1.1, 2.0, 0.0])
    # F: a heading error turns into a sideways (y) error of ds per radian
    assert F == approx(np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.1], [0.0, 0.0, 1.0]]), abs=1e-12)
    # G: both wheels push x by 1/2; the right wheel turns left: y += ds/(2b) = 0.1/0.4, theta += 1/b
    assert G == approx(np.array([[0.5, 0.5], [-0.25, 0.25], [-5.0, 5.0]]))


def test_motion_model_wraps_heading(impl):
    x_pred, _, _ = impl.motion_model([0.0, 0.0, 3.1], [-0.01, 0.01], B)  # turn left 0.1 rad in place
    assert x_pred[2] == approx(3.2 - 2 * math.pi), "theta must stay in (-pi, pi]"
    assert x_pred[:2] == approx([0.0, 0.0], abs=1e-12)


@pytest.mark.parametrize("seed", range(5))
def test_motion_jacobians_match_numerical(impl, seed):
    rng = np.random.default_rng(seed)
    x = np.array([rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-math.pi, math.pi)])
    u = rng.uniform(-0.05, 0.08, 2)
    _, F, G = impl.motion_model(x, u, B)
    F_num = _central_diff(lambda s: impl.motion_model(s, u, B)[0], x, angle_rows=[2])
    G_num = _central_diff(lambda v: impl.motion_model(x, v, B)[0], u, angle_rows=[2])
    assert F == approx(F_num, abs=1e-6), "F = df/dx disagrees with central differences"
    assert G == approx(G_num, abs=1e-6), "G = df/du disagrees with central differences (th_m depends on dL, dR)"


def test_wheel_noise(impl):
    M = impl.wheel_noise([0.04, -0.09], 0.01)
    assert M == approx(np.diag([1e-4 * 0.04, 1e-4 * 0.09])), "variance k^2 |d| per wheel, never negative"


# --- measurement model --------------------------------------------------------------------------
def test_range_bearing_by_hand(impl):
    z_hat, H = impl.range_bearing([1.0, 1.0, math.pi / 2], [2.0, 2.0])
    # dx = dy = 1: r = sqrt(2); world angle 45 deg, robot faces 90 deg -> bearing -45 deg
    assert z_hat == approx([math.sqrt(2), -math.pi / 4])
    s = 1 / math.sqrt(2)
    assert H == approx(np.array([[-s, -s, 0.0], [0.5, -0.5, -1.0]]))


def test_range_bearing_wraps(impl):
    z_hat, _ = impl.range_bearing([0.0, 0.0, 3.0], [-1.0, -0.1])
    # atan2(-0.1, -1) - 3.0 = -3.0419 - 3.0 = -6.0419 -> wrapped +0.2413
    assert z_hat[1] == approx(_wrap(math.atan2(-0.1, -1.0) - 3.0))
    assert -math.pi < z_hat[1] <= math.pi
    with pytest.raises(ValueError):
        impl.range_bearing([2.0, 2.0, 0.0], [2.0, 2.0])


@pytest.mark.parametrize("seed", range(5))
def test_measurement_jacobian_matches_numerical(impl, seed):
    rng = np.random.default_rng(100 + seed)
    x = np.array([rng.uniform(0, 6), rng.uniform(0, 5), rng.uniform(-math.pi, math.pi)])
    landmark = x[:2] + rng.uniform(0.5, 3.0) * np.array([math.cos(a := rng.uniform(-3, 3)), math.sin(a)])
    _, H = impl.range_bearing(x, landmark)
    H_num = _central_diff(lambda s: impl.range_bearing(s, landmark)[0], x, angle_rows=[1])
    assert H == approx(H_num, abs=1e-6), "H = dh/dx disagrees with central differences"


def test_your_numerical_jacobian(impl):
    J = impl.numerical_jacobian(lambda v: np.array([v[0] * v[1], math.sin(v[1])]), [2.0, 0.5])
    assert J == approx(np.array([[0.5, 2.0], [0.0, math.cos(0.5)]]), abs=1e-6)
    # An angle output that crosses +-pi between the two evaluations must not explode:
    J = impl.numerical_jacobian(lambda v: np.array([_wrap(v[0])]), [math.pi], angle_rows=[0])
    assert J == approx(np.array([[1.0]]), abs=1e-6)
    # ... and it is the tool to check YOUR Jacobians:
    x = np.array([1.0, 2.0, 3.1])
    _, H = impl.range_bearing(x, [-0.5, 2.05])
    assert H == approx(impl.numerical_jacobian(lambda s: impl.range_bearing(s, [-0.5, 2.05])[0], x, angle_rows=[1]), abs=1e-6)


# --- the filter ---------------------------------------------------------------------------------
def test_constructor(impl):
    x0, P0 = np.array([0.0, 0.0, 4.0]), np.eye(3)
    ekf = impl.EKF(x0, P0)
    assert ekf.x[2] == approx(4.0 - 2 * math.pi)
    ekf.predict([0.1, 0.1], B, np.eye(2))
    assert x0.tolist() == [0.0, 0.0, 4.0] and P0.tolist() == np.eye(3).tolist(), "copy the inputs"
    assert ekf.nu is None and ekf.S is None and ekf.K is None
    with pytest.raises(ValueError):
        impl.EKF([0, 0, 0], [[1, 0.5, 0], [0, 1, 0], [0, 0, 1]])


def test_predict_by_hand(impl):
    ekf = impl.EKF([1.0, 2.0, 0.0], np.diag([0.01, 0.01, 0.01]))
    x, P = ekf.predict([0.1, 0.1], B, np.diag([1e-4, 1e-4]))
    assert x == approx([1.1, 2.0, 0.0])
    # F P F^T: y variance 0.01 + 0.1^2 * 0.01 = 0.0101, cov(y, th) = 0.1 * 0.01 = 0.001
    # G M G^T: x += 2 * 0.25 * 1e-4 = 5e-5; y += 2 * 0.0625e-4 = 1.25e-5; th += 50e-4; cov(y, th) += 2 * 1.25e-4
    expected = np.array([[0.01005, 0.0, 0.0], [0.0, 0.0101125, 0.00125], [0.0, 0.00125, 0.015]])
    assert P == approx(expected, abs=1e-12)


def test_update_by_hand(impl):
    ekf = impl.EKF([0.0, 0.0, 0.0], np.diag([0.04, 0.04, 0.01]))
    nis = ekf.update([1.9, 0.05], [2.0, 0.0], np.diag([0.01, 0.001]))
    # z_hat = [2, 0]; H = [[-1, 0, 0], [0, -0.5, -1]]; nu = [-0.1, 0.05]; S = diag(0.05, 0.021)
    # K = P H^T S^-1 = [[-0.8, 0], [0, -0.95238], [0, -0.47619]]
    assert ekf.nu == approx([-0.1, 0.05])
    assert ekf.S == approx(np.diag([0.05, 0.021]))
    assert ekf.K == approx(np.array([[-0.8, 0.0], [0.0, -0.952381], [0.0, -0.476190]]), abs=1e-6)
    assert ekf.x == approx([0.08, -0.047619, -0.023810], abs=1e-6), "closer than expected -> x grows; landmark to the left -> y or theta shrink"
    assert ekf.P == approx(np.array([[0.008, 0, 0], [0, 0.0209524, -0.0095238], [0, -0.0095238, 0.0052381]]), abs=1e-6)
    assert nis == approx(0.1**2 / 0.05 + 0.05**2 / 0.021)


def test_update_wraps_the_bearing_innovation(impl):
    # Landmark almost straight behind: predicted bearing +3.1366, measured -3.13 (just across +-pi).
    ekf = impl.EKF([0.0, 0.0, 0.0], np.diag([0.01, 0.01, 0.01]))
    nis = ekf.update([2.0, -3.13], [-2.0, 0.01], np.diag([0.01, 0.001]))
    assert ekf.nu[1] == approx(_wrap(-3.13 - math.atan2(0.01, -2.0))), "wrap nu[1]: the real surprise is 0.017 rad, not -6.27"
    assert abs(ekf.nu[1]) < 0.05 and nis < 1.0
    assert np.abs(ekf.x).max() < 0.02 and -math.pi < ekf.x[2] <= math.pi


def test_update_keeps_covariance_symmetric_positive(impl):
    rng = np.random.default_rng(3)
    ekf = impl.EKF([1.0, 1.0, 0.3], np.diag([0.5, 0.5, 0.5]))
    for _ in range(500):
        ekf.predict(rng.uniform(0.0, 0.02, 2), B, np.diag([1e-6, 1e-6]))
        ekf.update([rng.uniform(1, 3), rng.uniform(-0.5, 0.5)], rng.uniform(0, 6, 2), np.diag([1e-6, 1e-8]))
        assert np.allclose(ekf.P, ekf.P.T, rtol=0, atol=1e-12)
    assert np.linalg.eigvalsh(ekf.P).min() > 0.0


def test_association_uses_mahalanobis_distance(impl):
    # Heading very uncertain (sigma 0.3 rad), position and range precise. The measurement (r 2.0 m,
    # bearing 0.25 rad) is a point at (1.94, 0.49). In plain meters landmark B (1.75, 0.35) is closer,
    # but B would need a 22 cm range error; A (2, 0) only needs a 0.25 rad heading error, which this
    # filter considers normal. Mahalanobis distance picks A.
    ekf = impl.EKF([0.0, 0.0, 0.0], np.diag([1e-4, 1e-4, 0.09]))
    R = np.diag([0.02**2, 0.01**2])
    z = [2.0, 0.25]
    landmarks = [[1.75, 0.35], [2.0, 0.0], [-3.0, 0.0]]
    point = np.array([2.0 * math.cos(0.25), 2.0 * math.sin(0.25)])
    assert np.linalg.norm(point - landmarks[0]) < np.linalg.norm(point - landmarks[1])
    assert impl.associate(ekf, z, landmarks, R) == 1, "use nu^T S^-1 nu, not the distance in meters"
    assert ekf.x == approx([0.0, 0.0, 0.0]), "associate must not change the state"
    assert impl.associate(ekf, [4.0, 1.2], landmarks, R) is None, "nothing explains it: reject (gate)"


# --- against the simulator ----------------------------------------------------------------------
K_WHEEL = 0.01
R_LANDMARK = np.diag([0.05**2, 0.03**2])  # SensorParams.realistic: sigma_r 5 cm, sigma_phi 0.03 rad


@functools.cache
def _tour(seed: int):
    return _tour_sim().drive_tour(seed=seed, observe_every=10)


def _localize(impl, log, use_ids: bool):
    b = load_config().drive.wheel_separation_m
    travels = _tour_sim().wheel_travels(log.ticks)
    ekf = impl.EKF(log.truth[0], np.diag([0.02**2, 0.02**2, 0.02**2]))
    observations = dict(log.landmarks)
    est, nis, nees, wrong = [ekf.x.copy()], [], [], 0
    for k in range(1, len(log.t)):
        u = travels[k - 1]
        ekf.predict(u, b, impl.wheel_noise(u, K_WHEEL))
        for obs in observations.get(k, []):
            z = [obs.range_m, obs.bearing_rad]
            if use_ids:
                i = int(np.flatnonzero(log.world.landmark_ids == obs.id)[0])
            else:
                i = impl.associate(ekf, z, log.world.landmarks, R_LANDMARK)
                if i is None:
                    continue
                wrong += int(log.world.landmark_ids[i] != obs.id)
            nis.append(ekf.update(z, log.world.landmarks[i], R_LANDMARK))
        est.append(ekf.x.copy())
        e = log.truth[k] - ekf.x
        e[2] = _wrap(e[2])
        nees.append(float(e @ np.linalg.solve(ekf.P, e)))
    est = np.array(est)
    pos_err = np.hypot(est[:, 0] - log.truth[:, 0], est[:, 1] - log.truth[:, 1])
    head_err = np.abs([_wrap(a) for a in est[:, 2] - log.truth[:, 2]])
    return {
        "rmse": float(np.sqrt(np.mean(pos_err**2))), "max": float(pos_err.max()),
        "heading_rmse_deg": math.degrees(float(np.sqrt(np.mean(head_err**2)))),
        "nis": float(np.mean(nis)), "nees": float(np.mean(nees)), "wrong": wrong,
    }


@pytest.mark.parametrize("seed", [1, 2])
def test_localizes_on_the_apartment_tour(impl, seed):
    log = _tour(seed)
    r = _localize(impl, log, use_ids=True)
    travels = _tour_sim().wheel_travels(log.ticks)
    b = load_config().drive.wheel_separation_m
    pose = np.array(log.truth[0], dtype=float)
    dr_err = [0.0]
    for u in travels:  # dead reckoning with the same model, for comparison
        pose, _, _ = impl.motion_model(pose, u, b)
        dr_err.append(math.hypot(*(pose[:2] - log.truth[len(dr_err), :2])))
    dr_rmse = float(np.sqrt(np.mean(np.square(dr_err))))
    assert r["rmse"] < 0.05, f"position RMSE {r['rmse'] * 100:.1f} cm (dead reckoning {dr_rmse * 100:.0f} cm)"
    assert r["max"] < 0.12, f"max position error {r['max'] * 100:.1f} cm"
    assert r["heading_rmse_deg"] < 4.0
    assert dr_rmse > 5 * r["rmse"], "landmarks should beat dead reckoning by far"
    assert 1.0 < r["nis"] < 3.5, f"mean NIS {r['nis']:.2f}, expected about m = 2 for a well-tuned filter"
    assert r["nees"] < 6.0, f"mean NEES {r['nees']:.2f} far above n = 3: overconfident"


def test_localizes_without_ids_using_association(impl):
    r = _localize(impl, _tour(1), use_ids=False)
    assert r["wrong"] == 0, f"{r['wrong']} observations were associated with the wrong landmark"
    assert r["rmse"] < 0.05, f"position RMSE {r['rmse'] * 100:.1f} cm"
