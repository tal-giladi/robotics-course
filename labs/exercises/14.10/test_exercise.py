"""Checker for 14.10 — the gate, the statistics and the correction.

Run: ``python course.py check 14.10`` (or ``--solution`` to see the reference pass).
No hardware: the "arm" is a known affine distortion of the commanded points, which is what a
joint-angle calibration bias looks like in task space.
"""

from __future__ import annotations

import numpy as np
import pytest

GRID = np.array([[x, y, z]
                 for x in (0.15, 0.20, 0.25, 0.30)
                 for y in (-0.12, 0.0, 0.12)
                 for z in (0.05, 0.12, 0.20)])


def distorted(points: np.ndarray, seed: int = 0, noise_m: float = 0.0) -> np.ndarray:
    """What a miscalibrated arm 'measures': a small affine distortion, plus optional scatter."""
    rng = np.random.default_rng(seed)
    A = np.eye(3) + rng.normal(0.0, 0.02, size=(3, 3))
    b = rng.normal(0.0, 0.004, size=3)
    out = points @ A.T + b
    if noise_m:
        out = out + rng.normal(0.0, noise_m, size=out.shape)
    return out


# --- WorkspaceBox --------------------------------------------------------------------------------
def test_a_point_inside_is_accepted(impl):
    box = impl.WorkspaceBox()
    assert box.contains([0.25, 0.0, 0.12])
    assert box.violations([0.25, 0.0, 0.12]) == []


def test_each_face_is_reported_by_name(impl):
    box = impl.WorkspaceBox()
    for point, axis in (([0.50, 0.0, 0.12], "x"), ([0.05, 0.0, 0.12], "x"),
                        ([0.25, 0.9, 0.12], "y"), ([0.25, -0.9, 0.12], "y"),
                        ([0.25, 0.0, -0.05], "z"), ([0.25, 0.0, 0.90], "z")):
        problems = box.violations(point)
        assert len(problems) == 1, point
        assert problems[0].startswith(axis), problems[0]
        assert not box.contains(point)


def test_two_violations_are_both_reported(impl):
    problems = impl.WorkspaceBox().violations([0.50, 0.0, -0.05])
    assert len(problems) == 2


def test_the_boundary_counts_as_inside(impl):
    box = impl.WorkspaceBox(x=(0.10, 0.34), y=(-0.22, 0.22), z=(0.02, 0.32))
    assert box.contains([0.10, -0.22, 0.02])
    assert box.contains([0.34, 0.22, 0.32])


def test_clearance_is_positive_inside_and_negative_outside(impl):
    box = impl.WorkspaceBox()
    assert impl.WorkspaceBox().clearance([0.22, 0.0, 0.17]) > 0.0
    assert box.clearance([0.25, 0.0, 0.03]) == pytest.approx(0.01)     # 1 cm above z_min
    assert box.clearance([0.40, 0.0, 0.12]) == pytest.approx(-0.06)    # 6 cm past x_max


def test_an_impossible_box_is_rejected(impl):
    with pytest.raises(ValueError):
        impl.WorkspaceBox(z=(0.30, 0.10))


# --- accuracy and repeatability --------------------------------------------------------------------
def test_perfect_runs_give_zero_and_zero(impl):
    target = np.array([0.25, 0.0, 0.12])
    accuracy, repeatability = impl.accuracy_and_repeatability(np.tile(target, (10, 1)), target)
    assert accuracy == pytest.approx(0.0, abs=1e-12)
    assert repeatability == pytest.approx(0.0, abs=1e-12)


def test_a_pure_offset_is_accuracy_not_repeatability(impl):
    target = np.array([0.25, 0.0, 0.12])
    measured = np.tile(target + np.array([0.012, 0.0, 0.0]), (10, 1))
    accuracy, repeatability = impl.accuracy_and_repeatability(measured, target)
    assert accuracy == pytest.approx(0.012)
    assert repeatability == pytest.approx(0.0, abs=1e-12)


def test_pure_scatter_is_repeatability_not_accuracy(impl):
    target = np.array([0.25, 0.0, 0.12])
    offsets = np.array([[0.002, 0, 0], [-0.002, 0, 0], [0, 0.002, 0], [0, -0.002, 0]])
    accuracy, repeatability = impl.accuracy_and_repeatability(target + offsets, target)
    assert accuracy == pytest.approx(0.0, abs=1e-12)
    assert repeatability == pytest.approx(0.002)


def test_repeatability_is_the_worst_not_the_average(impl):
    target = np.zeros(3)
    measured = np.array([[0.0, 0, 0], [0.0, 0, 0], [0.0, 0, 0], [0.010, 0, 0]])
    _, repeatability = impl.accuracy_and_repeatability(measured, target)
    mean_offset = 0.010 / 4
    assert repeatability == pytest.approx(0.010 - mean_offset)


def test_one_measurement_has_zero_repeatability(impl):
    accuracy, repeatability = impl.accuracy_and_repeatability([[0.26, 0.0, 0.12]], [0.25, 0.0, 0.12])
    assert accuracy == pytest.approx(0.01)
    assert repeatability == pytest.approx(0.0, abs=1e-12)


def test_rejects_no_measurements(impl):
    with pytest.raises(ValueError):
        impl.accuracy_and_repeatability(np.zeros((0, 3)), [0.25, 0.0, 0.12])


# --- fit_offset -----------------------------------------------------------------------------------
def test_offset_recovers_a_pure_translation(impl):
    shift = np.array([0.004, -0.002, 0.006])
    got = np.asarray(impl.fit_offset(GRID, GRID - shift))
    np.testing.assert_allclose(got, shift, atol=1e-12)


def test_offset_of_a_perfect_arm_is_zero(impl):
    np.testing.assert_allclose(np.asarray(impl.fit_offset(GRID, GRID)), np.zeros(3), atol=1e-12)


def test_offset_rejects_a_shape_mismatch(impl):
    with pytest.raises(ValueError):
        impl.fit_offset(GRID, GRID[:5])


# --- fit_affine -----------------------------------------------------------------------------------
def test_affine_recovers_a_known_distortion_exactly(impl):
    measured = distorted(GRID, seed=3)
    A, b = impl.fit_affine(GRID, measured)
    corrected = np.asarray(impl.apply_affine(A, b, measured))
    assert np.linalg.norm(corrected - GRID, axis=1).max() < 1e-10


def test_affine_beats_a_constant_offset(impl):
    measured = distorted(GRID, seed=5)
    offset = np.asarray(impl.fit_offset(GRID, measured))
    A, b = impl.fit_affine(GRID, measured)
    offset_error = np.asarray(impl.residuals(GRID, measured + offset)).mean()
    affine_error = np.asarray(impl.residuals(GRID, impl.apply_affine(A, b, measured))).mean()
    assert affine_error < offset_error / 10.0


def test_affine_needs_at_least_four_points(impl):
    with pytest.raises(ValueError):
        impl.fit_affine(GRID[:3], distorted(GRID, seed=1)[:3])


def test_apply_affine_keeps_the_input_shape(impl):
    A, b = np.eye(3), np.array([0.001, 0.0, 0.0])
    one = np.asarray(impl.apply_affine(A, b, np.array([0.2, 0.1, 0.1])))
    many = np.asarray(impl.apply_affine(A, b, GRID))
    assert one.shape == (3,)
    assert many.shape == GRID.shape
    np.testing.assert_allclose(one, [0.201, 0.1, 0.1], atol=1e-12)


def test_residuals_are_per_point_magnitudes(impl):
    got = np.asarray(impl.residuals([[0.0, 0, 0], [0.0, 0, 0]], [[0.003, 0, 0], [0, 0.004, 0]]))
    np.testing.assert_allclose(got, [0.003, 0.004], atol=1e-12)


# --- holdout_validate ---------------------------------------------------------------------------------
def test_a_real_correction_generalises(impl):
    measured = distorted(GRID, seed=11, noise_m=0.0005)
    index = np.arange(len(GRID))
    train, test = index[::2], index[1::2]
    train_error, test_error = impl.holdout_validate(GRID, measured, train, test)
    assert test_error < 3.0 * train_error + 1e-9, "a genuine fit generalises to held-out points"
    assert test_error < 0.002


def test_fitting_noise_does_not_generalise(impl):
    """Four points, all noise: the fit is perfect on them and useless everywhere else."""
    rng = np.random.default_rng(2)
    commanded = GRID[:8]
    measured = commanded + rng.normal(0.0, 0.02, size=commanded.shape)
    train_error, test_error = impl.holdout_validate(commanded, measured, range(4), range(4, 8))
    assert train_error < 1e-9                       # 12 parameters, 4 points: exact
    assert test_error > 0.005                       # and it says nothing about the other four


def test_overlapping_indices_are_rejected(impl):
    measured = distorted(GRID, seed=1)
    with pytest.raises(ValueError):
        impl.holdout_validate(GRID, measured, [0, 1, 2, 3, 4], [4, 5, 6, 7])
