"""Checker for 10.05 — The multivariate Kalman filter.

Run: ``python course.py check 10.05`` (or ``--solution`` to see the reference pass).
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

# 95% two-sided bounds for the AVERAGE NEES of a 2-state filter over 50 Monte Carlo runs:
# 50 * mean ~ chi2(100 dof) -> scipy.stats.chi2.ppf([0.025, 0.975], 100) / 50
NEES_BOUNDS_50_RUNS_2D = (1.4844, 2.5912)


def _wall_sim():
    """The wall-approach run from 10.04 (encoders, front ToF, ground truth)."""
    name = "course_exercise_10_04_wall_sim"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, HERE.parent / "10.04" / "wall_sim.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


# --- model --------------------------------------------------------------------------------------
def test_constant_velocity_model(impl):
    F, Q = impl.constant_velocity_model(0.1, 0.5)
    assert F == approx(np.array([[1.0, 0.1], [0.0, 1.0]]))
    # G = [0.005, 0.1]; Q = G G^T * 0.25 = [[2.5e-5, 5e-4], [5e-4, 1e-2]] * 0.25
    assert Q == approx(np.array([[6.25e-6, 1.25e-4], [1.25e-4, 2.5e-3]]))
    assert Q == approx(Q.T)


# --- predict ------------------------------------------------------------------------------------
def test_predict_matches_fm07(impl):
    kf = impl.KalmanFilter([1.0, 0.5], np.diag([0.04, 0.01]))
    F = np.array([[1.0, 0.1], [0.0, 1.0]])
    x, P = kf.predict(F, np.zeros((2, 2)))
    assert x == approx([1.05, 0.5])
    # F P F^T: position variance grows by dt^2 * var(v); position and velocity become correlated
    assert P == approx(np.array([[0.0401, 0.001], [0.001, 0.01]]))
    assert kf.nu is None and kf.S is None and kf.K is None


def test_predict_with_control_input(impl):
    kf = impl.KalmanFilter([0.0, 0.0], np.eye(2))
    dt = 0.1
    F = np.array([[1.0, dt], [0.0, 1.0]])
    x, P = kf.predict(F, np.zeros((2, 2)), B=[[0.5 * dt * dt], [dt]], u=[2.0])  # commanded 2 m/s^2
    assert x == approx([0.01, 0.2])
    assert P == approx(np.array([[1.01, 0.1], [0.1, 1.0]]))


def test_constructor_copies_and_validates(impl):
    x0, P0 = np.array([1.0, 2.0]), np.eye(2)
    kf = impl.KalmanFilter(x0, P0)
    kf.predict(np.eye(2) * 2, np.eye(2))
    assert x0.tolist() == [1.0, 2.0] and P0.tolist() == [[1.0, 0.0], [0.0, 1.0]], "copy the inputs"
    with pytest.raises(ValueError):
        impl.KalmanFilter([0.0, 0.0], np.eye(3))
    with pytest.raises(ValueError):
        impl.KalmanFilter([0.0, 0.0], [[1.0, 0.5], [0.0, 1.0]])


# --- update -------------------------------------------------------------------------------------
def test_position_measurement_also_corrects_velocity(impl):
    kf = impl.KalmanFilter([1.05, 0.5], [[0.0401, 0.001], [0.001, 0.01]])
    x, P = kf.update([1.10], [[1.0, 0.0]], [[0.01]])
    # nu = 0.05, S = 0.0401 + 0.01 = 0.0501, K = [0.0401, 0.001] / 0.0501 = [0.80040, 0.019960]
    assert kf.nu == approx([0.05])
    assert kf.S == approx(np.array([[0.0501]]))
    assert kf.K == approx(np.array([[0.8003992], [0.01996008]]), rel=1e-6)
    assert x == approx([1.0900200, 0.5009980], rel=1e-6), "velocity moves too, through the correlation"
    assert P == approx(np.array([[0.00800399, 0.00019960], [0.00019960, 0.00998004]]), rel=1e-5)


def test_two_sensors_at_once(impl):
    kf = impl.KalmanFilter([1.05, 0.5], [[0.04010625, 0.001125], [0.001125, 0.0125]])
    x, P = kf.update([1.10, 0.45], np.eye(2), np.diag([0.01, 0.0025]))  # ToF position + encoder speed
    assert kf.nu == approx([0.05, -0.05])
    assert kf.S == approx(np.array([[0.05010625, 0.001125], [0.001125, 0.015]]))
    assert x == approx([1.0892547, 0.4585348], rel=1e-6)
    assert P == approx(np.array([[8.000875e-3, 3.74836e-5], [3.74836e-5, 2.082631e-3]]), rel=1e-5)


def test_scalar_case_equals_the_1d_filter(impl):
    kf = impl.KalmanFilter([2.0], [[0.09]])
    x, P = kf.update(2.2, [[1.0]], [[0.01]])
    assert x == approx([2.18]) and P == approx(np.array([[0.009]]))


def test_update_rejects_wrong_shapes(impl):
    kf = impl.KalmanFilter([0.0, 0.0], np.eye(2))
    with pytest.raises(ValueError):
        kf.update([1.0], [[1.0, 0.0, 0.0]], [[1.0]])
    with pytest.raises(ValueError):
        kf.update([1.0, 2.0], [[1.0, 0.0]], [[1.0]])


def test_covariance_stays_symmetric_and_positive(impl):
    rng = np.random.default_rng(11)
    F, Q = impl.constant_velocity_model(0.02, 0.2)
    kf = impl.KalmanFilter([0.0, 0.0], np.diag([1e4, 1e4]))
    for _ in range(3000):
        kf.predict(F, Q)
        kf.update([rng.normal()], [[1.0, 0.0]], [[1e-6]])  # a very precise sensor: hard on rounding
        assert np.array_equal(kf.P, kf.P.T) or np.allclose(kf.P, kf.P.T, rtol=0, atol=1e-15)
    assert np.linalg.eigvalsh(kf.P).min() > 0.0


# --- consistency --------------------------------------------------------------------------------
def test_nees_and_nis_by_hand(impl):
    assert impl.nees([0.1, -0.2], [0.0, 0.0], np.diag([0.01, 0.04])) == approx(2.0)
    # Correlated: P^-1 = [[0.09, -0.03], [-0.03, 0.04]] / 0.0027; e = [0.2, 0.3] -> 0.0036 / 0.0027
    assert impl.nees([0.2, 0.3], [0.0, 0.0], [[0.04, 0.03], [0.03, 0.09]]) == approx(4 / 3)
    assert impl.nis([0.05], [[0.0501]]) == approx(0.05**2 / 0.0501)
    assert impl.nis(0.3, 0.09) == approx(1.0)


def _monte_carlo_nees(impl, q_scale: float, runs: int = 50, steps: int = 100) -> np.ndarray:
    """Truth follows the constant-velocity model exactly; the filter assumes Q * q_scale."""
    dt, accel_std, r_std = 0.1, 0.5, 0.05
    F, Q = impl.constant_velocity_model(dt, accel_std)
    G = np.array([0.5 * dt * dt, dt])
    rng = np.random.default_rng(2024)
    out = np.zeros((runs, steps))
    P0 = np.diag([0.1**2, 0.1**2])
    for run in range(runs):
        x = np.array([0.0, 0.3])
        kf = impl.KalmanFilter(x + rng.multivariate_normal([0.0, 0.0], P0), P0)
        for k in range(steps):
            x = F @ x + G * rng.normal(0.0, accel_std)
            kf.predict(F, Q * q_scale)
            kf.update([x[0] + rng.normal(0.0, r_std)], [[1.0, 0.0]], [[r_std**2]])
            out[run, k] = impl.nees(x, kf.x, kf.P)
    return out.mean(axis=0)  # average over runs, one value per time step


def test_correctly_tuned_filter_is_consistent(impl):
    avg = _monte_carlo_nees(impl, 1.0)
    lo, hi = NEES_BOUNDS_50_RUNS_2D
    inside = np.mean((avg > lo) & (avg < hi))
    assert avg.mean() == approx(2.0, abs=0.25), f"average NEES should be about n = 2, got {avg.mean():.2f}"
    assert inside > 0.85, f"about 95% of time steps should be inside the chi-square bounds, got {inside:.0%}"


def test_overconfident_and_underconfident_filters_are_detected(impl):
    too_small_q = _monte_carlo_nees(impl, 0.01).mean()
    too_big_q = _monte_carlo_nees(impl, 100.0).mean()
    assert too_small_q > NEES_BOUNDS_50_RUNS_2D[1], "Q 100x too small: P is too small, errors look huge (overconfident)"
    assert too_big_q < 2.0 * 0.75, "Q 100x too big: P is too big, errors look small (underconfident)"


# --- against the simulator ----------------------------------------------------------------------
@functools.cache
def _wall_run(seed: int):
    return _wall_sim().drive_to_wall(seed, realistic=True, range_noise_std_m=0.03)


@pytest.mark.parametrize("seed", [1, 2])
def test_fuses_tof_and_encoder_speed_on_the_simulated_robot(impl, seed):
    """State [x, v] of karmel driving toward a wall at x = 4 m: ToF gives x, encoders give v."""
    ws = _wall_sim()
    run = _wall_run(seed)
    range_x = 0.125  # ToF mount, karmel.yaml sensors.range_front.x_m
    x_true = ws.ROOM_W - range_x - run.truth
    dt = float(run.t[1] - run.t[0])
    F, Q = impl.constant_velocity_model(dt, 0.3)
    kf = impl.KalmanFilter([0.5, 0.0], np.diag([0.05**2, 0.05**2]))
    est, nees_values, nis_tof, tof_positions = [], [], [], []
    for k, (z, speed) in enumerate(zip(run.measurements, run.wheel_speed)):
        kf.predict(F, Q)
        # Encoder speed is low-pass filtered and uses the nominal wheel radius: its errors are
        # correlated in time, so sigma = 3 cm/s, not its per-sample 1 cm/s (see the lesson).
        kf.update([speed], [[0.0, 1.0]], [[0.03**2]])
        if z is not None:
            position = ws.ROOM_W - range_x - z  # the map turns a range into a position
            kf.update([position], [[1.0, 0.0]], [[0.03**2]])
            nis_tof.append(impl.nis(kf.nu, kf.S))
            tof_positions.append((k, position))
        est.append(kf.x.copy())
        nees_values.append(impl.nees([x_true[k], run.velocity[k]], kf.x, kf.P))
    est = np.array(est)
    settle = 20
    kf_rmse = math.sqrt(np.mean((est[settle:, 0] - x_true[settle:]) ** 2))
    tof_rmse = math.sqrt(np.mean([(p - x_true[k]) ** 2 for k, p in tof_positions if k >= settle]))
    v_rmse = math.sqrt(np.mean((est[settle:, 1] - run.velocity[settle:]) ** 2))
    assert kf_rmse < 0.5 * tof_rmse, f"position RMSE {kf_rmse * 1000:.1f} mm vs ToF {tof_rmse * 1000:.1f} mm"
    assert v_rmse < 0.02, f"velocity RMSE should be under 2 cm/s, got {v_rmse * 100:.1f} cm/s"
    assert 0.5 < np.mean(nis_tof) < 2.0, f"ToF NIS should average about 1, got {np.mean(nis_tof):.2f}"
    assert np.mean(nees_values[settle:]) < 5.99, "NEES far above n = 2 means the filter is overconfident"
