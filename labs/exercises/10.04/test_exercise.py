"""Checker for 10.04 — The Kalman filter in 1D.

Run: ``python course.py check 10.04`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import functools
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

approx = pytest.approx
HERE = Path(__file__).resolve().parent


def _wall_sim():
    name = "course_exercise_10_04_wall_sim"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, HERE / "wall_sim.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


@functools.cache
def _noisy_wall_run(seed: int):
    return _wall_sim().drive_to_wall(seed, realistic=True, range_noise_std_m=0.03)


# --- the gain -----------------------------------------------------------------------------------
def test_gain_is_a_trust_ratio(impl):
    assert impl.kalman_gain(0.09, 0.01) == approx(0.9), "prior sigma 30 cm, sensor sigma 10 cm: trust the sensor 90%"
    assert impl.kalman_gain(0.01, 0.01) == approx(0.5), "equal variances: split the difference"
    assert impl.kalman_gain(0.0, 0.01) == approx(0.0), "a certain prior ignores the sensor"


def test_gain_rejects_negative_variances(impl):
    with pytest.raises(ValueError):
        impl.kalman_gain(-0.01, 0.01)
    with pytest.raises(ValueError):
        impl.kalman_gain(0.0, 0.0)


def test_steady_state_gain(impl):
    # M = (Q + sqrt(Q^2 + 4QR)) / 2 = (1e-4 + sqrt(1e-8 + 4e-6)) / 2 = 1.05125e-3; K = M / (M + R)
    assert impl.steady_state_gain(1e-4, 1e-2) == approx(0.0951249, rel=1e-5)
    assert impl.steady_state_gain(1.0, 1.0) == approx(0.6180340, rel=1e-6), "Q = R gives 1/golden ratio"


# --- one filter, step by step -------------------------------------------------------------------
def test_first_update_matches_fm16(impl):
    kf = impl.KalmanFilter1D(2.0, 0.09)  # wall believed at 2.0 m +/- 0.3 m
    assert kf.K is None and kf.nu is None and kf.S is None
    x, P = kf.update(2.2, 0.01)  # ToF reads 2.2 m +/- 0.1 m
    # nu = 0.2, S = 0.1, K = 0.9, x = 2.0 + 0.9 * 0.2 = 2.18, P = 0.1 * 0.09 = 0.009
    assert (kf.nu, kf.S, kf.K) == approx((0.2, 0.1, 0.9))
    assert (x, P) == approx((2.18, 0.009))
    assert (kf.x, kf.P) == approx((2.18, 0.009)), "update must store the new estimate on self"


def test_predict_then_update(impl):
    kf = impl.KalmanFilter1D(2.18, 0.009)
    # Robot drives 10 cm toward the wall; odometry noise variance 0.0004 (sigma 2 cm)
    assert kf.predict(-0.10, 0.0004) == approx((2.08, 0.0094))
    # ToF reads 2.05: nu = -0.03, S = 0.0194, K = 0.0094 / 0.0194 = 0.48454
    # x = 2.08 - 0.48454 * 0.03 = 2.065464, P = (1 - 0.48454) * 0.0094 = 0.00484536
    x, P = kf.update(2.05, 0.01)
    assert kf.nu == approx(-0.03)
    assert kf.S == approx(0.0194)
    assert kf.K == approx(0.4845361, rel=1e-6)
    assert (x, P) == approx((2.0654639, 0.00484536), rel=1e-6)


def test_predict_defaults_and_validation(impl):
    kf = impl.KalmanFilter1D(1.0, 0.5)
    assert kf.predict() == approx((1.0, 0.5)), "no motion and no process noise changes nothing"
    with pytest.raises(ValueError):
        kf.predict(0.1, -1e-3)
    with pytest.raises(ValueError):
        kf.update(1.0, 0.0)
    with pytest.raises(ValueError):
        impl.KalmanFilter1D(0.0, -1.0)


def test_information_adds(impl):
    kf = impl.KalmanFilter1D(0.0, 1e6)  # "no idea"
    kf.update(1.0, 0.04)
    x, P = kf.update(1.2, 0.04)
    # Two equally good readings: the average, with half the variance of one reading.
    assert x == approx(1.1, abs=1e-6)
    assert P == approx(0.02, rel=1e-6)


def test_update_never_increases_variance_and_gain_stays_in_0_1(impl):
    rng = np.random.default_rng(4)
    kf = impl.KalmanFilter1D(0.0, 1.0)
    for _ in range(200):
        P_before = kf.predict(rng.normal(0, 0.1), rng.uniform(0, 0.01))[1]
        _, P_after = kf.update(rng.normal(0, 1.0), rng.uniform(1e-4, 1.0))
        assert 0.0 <= kf.K <= 1.0
        assert 0.0 < P_after <= P_before


# --- running over a log -------------------------------------------------------------------------
def test_run_skips_missing_measurements(impl):
    xs, Ps = impl.run_kf_1d(2.0, 0.09, [0.0, -0.1, -0.1], [2.2, None, 2.05], Q=0.0004, R=0.01)
    # step 0: predict (2.0, 0.0904) then update with 2.2 -> K = 0.0904/0.1004
    K0 = 0.0904 / 0.1004
    x0, P0 = 2.0 + K0 * 0.2, (1 - K0) * 0.0904
    # step 1: predict only
    x1, P1 = x0 - 0.1, P0 + 0.0004
    # step 2: predict and update with 2.05
    xp, Pp = x1 - 0.1, P1 + 0.0004
    K2 = Pp / (Pp + 0.01)
    assert xs == approx([x0, x1, xp + K2 * (2.05 - xp)])
    assert Ps == approx([P0, P1, (1 - K2) * Pp])
    with pytest.raises(ValueError):
        impl.run_kf_1d(0.0, 1.0, [0.0], [1.0, 2.0], Q=0.0, R=1.0)


def test_gain_converges_to_steady_state(impl):
    Q, R = 1e-4, 1e-2
    _, Ps = impl.run_kf_1d(0.0, 100.0, [0.0] * 200, [1.0] * 200, Q=Q, R=R)
    # After an update P = K * R, so the final P tells us the gain.
    assert Ps[-1] / R == approx(impl.steady_state_gain(Q, R), rel=1e-6)


# --- against the simulator ----------------------------------------------------------------------
@pytest.mark.parametrize("seed", [1, 2])
def test_filter_beats_the_sensor_on_the_simulated_robot(impl, seed):
    """karmel drives 2.2 m toward a wall: encoders predict, a noisy ToF (sigma 3 cm) corrects."""
    run = _noisy_wall_run(seed)
    z = np.array([np.nan if m is None else m for m in run.measurements])
    valid = ~np.isnan(z)
    R = 0.03**2
    xs, Ps = impl.run_kf_1d(float(z[valid][0]), R, run.controls, run.measurements, Q=1e-6, R=R)
    settle = 20  # ignore the first second while the filter converges
    meas_rmse = math.sqrt(np.mean((z[valid][settle:] - run.truth[valid][settle:]) ** 2))
    kf_rmse = math.sqrt(np.mean((xs[settle:] - run.truth[settle:]) ** 2))
    assert kf_rmse < 0.5 * meas_rmse, (
        f"filtered RMSE {kf_rmse * 1000:.1f} mm should be well below the ToF's {meas_rmse * 1000:.1f} mm"
    )
    assert abs(xs[-1] - run.truth[-1]) < 0.03, "the final distance to the wall should be within 3 cm"
    inside = np.abs(xs[settle:] - run.truth[settle:]) < 3.0 * np.sqrt(Ps[settle:]) + 0.01
    assert inside.mean() > 0.8, "most errors should be inside 3 sigma (+1 cm for the cos-heading effect)"


def test_filter_matches_sensor_accuracy_with_huge_Q(impl):
    """Q -> infinity means 'ignore odometry': the filter just repeats the measurements."""
    run = _noisy_wall_run(1)
    xs, _ = impl.run_kf_1d(3.0, 1.0, run.controls, run.measurements, Q=1e3, R=0.03**2)
    z = np.array([np.nan if m is None else m for m in run.measurements])
    valid = ~np.isnan(z)
    assert xs[valid] == approx(z[valid], abs=1e-5)
