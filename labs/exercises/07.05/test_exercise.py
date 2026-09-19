"""Checker for 07.05 — IMU fundamentals: gyro bias and heading drift.

Run: ``python course.py check 07.05`` (``--solution`` runs the reference).
The ``impl`` fixture (labs/exercises/conftest.py) is student.py or solution.py.
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from robotlab.config import load_config
from robotlab.geometry import angle_diff
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World


# --- numbers ----------------------------------------------------------------------------------------
def test_heading_drift(impl):
    # 0.01 rad/s for 60 s = 0.6 rad = 34.38 deg
    assert impl.heading_drift_deg(0.01, 60.0) == pytest.approx(34.3775, abs=1e-3)
    assert impl.heading_drift_deg(-0.002, 3600.0) == pytest.approx(-412.53, abs=0.01)


def test_seconds_until_error(impl):
    # 1 deg / (0.5 deg/s) = 2 s
    assert impl.seconds_until_error(math.radians(0.5), 1.0) == pytest.approx(2.0)
    assert impl.seconds_until_error(-math.radians(0.02), 5.0) == pytest.approx(250.0)
    assert impl.seconds_until_error(0.0, 5.0) == math.inf


# --- bias ---------------------------------------------------------------------------------------------
def test_estimate_gyro_bias(impl):
    assert impl.estimate_gyro_bias([0.011, 0.009, 0.010, 0.010]) == pytest.approx(0.010)
    with pytest.raises(ValueError):
        impl.estimate_gyro_bias([])


def test_detect_still_hand_computed(impl):
    rates = [0.02, 0.021, 0.019, 0.5, 1.0, 1.0, 0.02, 0.02]
    # window 3: i=0 -> [0.02, 0.021] range 0.001 still; i=2 -> [0.021, 0.019, 0.5] moving; i=7 -> [0.02, 0.02] still
    assert impl.detect_still(rates, 3, 0.01) == [True, True, False, False, False, False, False, True]


def test_detect_still_uses_range_not_magnitude(impl):
    biased = [0.3, 0.301, 0.299, 0.3, 0.302]  # a big bias, but perfectly still
    assert all(impl.detect_still(biased, 5, 0.01))


def test_bias_from_still(impl):
    assert impl.bias_from_still([0.01, 2.0, 0.03], [True, False, True]) == pytest.approx(0.02)
    with pytest.raises(ValueError):
        impl.bias_from_still([0.01, 2.0], [False, False])
    with pytest.raises(ValueError):
        impl.bias_from_still([0.01], [True, False])


# --- integration --------------------------------------------------------------------------------------
def test_integrate_constant_rate(impl):
    times = [0.0, 0.5, 1.0, 1.5, 2.0]
    assert impl.integrate_heading(times, [0.5] * 5) == pytest.approx([0.0, 0.25, 0.5, 0.75, 1.0])


def test_integrate_subtracts_bias_and_uses_trapezoid(impl):
    # rates minus bias 0.1: 0, 1, 1 -> trapezoids 0.5*(0+1)*1 = 0.5, then 0.5*(1+1)*1 = 1.0
    assert impl.integrate_heading([0.0, 1.0, 2.0], [0.1, 1.1, 1.1], bias=0.1) == pytest.approx([0.0, 0.5, 1.5])


def test_integrate_uneven_steps_initial_and_wrap(impl):
    out = impl.integrate_heading([0.0, 0.1, 0.4], [1.0, 1.0, 1.0], initial_heading=3.0)
    assert out == pytest.approx([3.0, 3.1, 3.4 - 2 * math.pi])  # 3.1 < pi stays, 3.4 wraps
    assert impl.integrate_heading([], []) == []
    with pytest.raises(ValueError):
        impl.integrate_heading([0.0, 1.0], [0.0])


def test_long_integration_does_not_lose_turns(impl):
    # 10 full turns at 2*pi rad/s sampled at 100 Hz: must end back at 0, not stop wrapping after one turn
    times = [k * 0.01 for k in range(1001)]
    out = impl.integrate_heading(times, [2 * math.pi] * 1001)
    assert abs(angle_diff(out[-1], 0.0)) < 1e-6
    assert abs(angle_diff(out[25], math.pi / 2)) < 1e-6


# --- against the simulator ----------------------------------------------------------------------------
def record_turns(seed: int, bias: float):
    """karmel stands still 4 s, then 4 spins of ~90 deg with 1.5 s pauses; logs the gyro at 100 Hz."""
    cfg = load_config()
    sensors = dataclasses.replace(SensorParams.realistic(cfg), gyro_bias_rad_s=bias, gyro_noise_std_rad_s=0.005)
    base = SimBase(DiffDriveSim(World(), DiffDriveParams.realistic(cfg), sensors, seed=seed), dt=0.01)
    plan = [(4.0, 0.0)] + [(0.9, 4.0), (1.5, 0.0)] * 4
    times, rates, truth = [], [], 0.0
    for seconds, wheel in plan:
        for _ in range(round(seconds / base.dt)):
            base.set_wheel_velocity(-wheel, wheel)
            state = base.read()
            truth += base.sim.yaw_rate * base.dt
            times.append(state.t)
            rates.append(base.sim.gyro_z())
    return times, rates, truth


@pytest.mark.parametrize(("seed", "bias"), [(1, 0.01), (2, -0.015), (3, 0.02)])
def test_bias_correction_on_simulated_robot(impl, seed, bias):
    times, rates, truth = record_turns(seed, bias)
    # 1 s window, 0.035 rad/s: loose enough for the noise (sigma 0.005 -> range of 100 samples ~0.025),
    # strict enough to reject the slowly settling tail after each spin, which would bias the estimate
    still = impl.detect_still(rates, window=100, threshold_rad_s=0.035)
    assert sum(still) > 300, "most of the 4 s of standing still (400 samples at 100 Hz) must be detected as still"
    estimate = impl.bias_from_still(rates, still)
    assert estimate == pytest.approx(bias, abs=0.0015)
    corrected = impl.integrate_heading(times, rates, bias=estimate)
    raw = impl.integrate_heading(times, rates)
    corrected_error = abs(math.degrees(angle_diff(corrected[-1], truth)))
    raw_error = abs(math.degrees(angle_diff(raw[-1], truth)))
    assert corrected_error < 1.0, f"bias-corrected heading should be within 1 deg of truth, was {corrected_error:.2f}"
    assert raw_error > 5.0, "without correction the bias should have drifted the heading several degrees"
