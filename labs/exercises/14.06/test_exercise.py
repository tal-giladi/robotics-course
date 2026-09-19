"""Checker for 14.06 — the Jacobian toolkit.

Run: ``python course.py check 14.06`` (or ``--solution`` to see the reference pass).

The arm is the planar model of the SO-101's upper arm + forearm: l1 = 0.116 m, l2 = 0.135 m.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

L1, L2 = 0.116, 0.135
LENGTHS3 = (0.116, 0.135, 0.080)      # a 3-link (redundant) version for the wide-J tests
G = 9.81


def numeric_jacobian(f, q, eps: float = 1e-7) -> np.ndarray:
    """Central-difference derivative of f at q — the ground truth the analytic J must match."""
    q = np.asarray(q, dtype=float)
    f0 = np.atleast_1d(np.asarray(f(q), dtype=float))
    J = np.zeros((f0.size, q.size))
    for j in range(q.size):
        dq = np.zeros_like(q)
        dq[j] = eps
        J[:, j] = (np.atleast_1d(f(q + dq)) - np.atleast_1d(f(q - dq))) / (2.0 * eps)
    return J


# --- planar_jacobian ---------------------------------------------------------------------------
def test_known_pose_by_hand(impl):
    """q = (30, 60) deg: the forearm points straight up, so joint 2 moves the tip in -x only."""
    J = impl.planar_jacobian((L1, L2), np.radians([30.0, 60.0]))
    assert J.shape == (2, 2)
    np.testing.assert_allclose(J, [[-0.19296, -0.135], [0.10046, 0.0]], atol=1e-4)


def test_matches_a_numerical_derivative(impl):
    rng = np.random.default_rng(14060)
    for _ in range(200):
        q = rng.uniform(-math.pi, math.pi, size=2)
        J = impl.planar_jacobian((L1, L2), q)
        Jn = numeric_jacobian(lambda qq: impl.planar_tip((L1, L2), qq), q)
        assert np.abs(np.asarray(J) - Jn).max() < 1e-6, f"disagrees at q = {np.degrees(q)}"


def test_works_for_three_links(impl):
    rng = np.random.default_rng(3)
    for _ in range(50):
        q = rng.uniform(-math.pi, math.pi, size=3)
        J = np.asarray(impl.planar_jacobian(LENGTHS3, q))
        assert J.shape == (2, 3)
        Jn = numeric_jacobian(lambda qq: impl.planar_tip(LENGTHS3, qq), q)
        assert np.abs(J - Jn).max() < 1e-6


def test_first_column_uses_the_whole_arm(impl):
    """Column 1 must be z x (tip - base), not z x (tip - joint_1). A classic off-by-one."""
    q = np.radians([15.0, 70.0])
    J = np.asarray(impl.planar_jacobian((L1, L2), q))
    tip = np.asarray(impl.planar_tip((L1, L2), q))
    np.testing.assert_allclose(J[:, 0], [-tip[1], tip[0]], atol=1e-12)


def test_rejects_a_mismatched_pose(impl):
    with pytest.raises(ValueError):
        impl.planar_jacobian((L1, L2), [0.1])


# --- manipulability and singular values ----------------------------------------------------------
def test_manipulability_is_l1_l2_sin_q2(impl):
    for q1_deg in (0.0, 37.0, -100.0):
        for q2_deg in (120.0, 90.0, 60.0, 30.0, 10.0, -45.0):
            q = np.radians([q1_deg, q2_deg])
            w = impl.manipulability(impl.planar_jacobian((L1, L2), q))
            assert w == pytest.approx(L1 * L2 * abs(math.sin(math.radians(q2_deg))), abs=1e-12)


def test_manipulability_is_zero_at_both_singularities(impl):
    for q2_deg in (0.0, 180.0, -180.0):
        J = impl.planar_jacobian((L1, L2), np.radians([25.0, q2_deg]))
        assert impl.manipulability(J) == pytest.approx(0.0, abs=1e-9)


def test_manipulability_never_returns_nan_for_a_wide_jacobian(impl):
    """3 joints, 2 task dimensions: det(J J^T) can go slightly negative in floating point."""
    J = impl.planar_jacobian(LENGTHS3, np.radians([10.0, 0.0, 0.0]))
    w = impl.manipulability(J)
    assert not math.isnan(w) and w >= 0.0


def test_singular_values_are_sorted_and_correct(impl):
    J = np.asarray(impl.planar_jacobian((L1, L2), np.radians([30.0, 60.0])))
    s = np.asarray(impl.singular_values(J))
    np.testing.assert_allclose(s, [0.25029, 0.05418], atol=1e-4)
    assert s[0] >= s[-1]


def test_manipulability_equals_the_product_of_singular_values(impl):
    q = np.radians([44.0, 71.0])
    J = impl.planar_jacobian((L1, L2), q)
    s = np.asarray(impl.singular_values(J))
    assert impl.manipulability(J) == pytest.approx(float(np.prod(s)), rel=1e-9)


# --- damped least squares ------------------------------------------------------------------------
def test_zero_damping_is_the_exact_inverse(impl):
    q = np.radians([30.0, 60.0])
    J = np.asarray(impl.planar_jacobian((L1, L2), q))
    v = np.array([0.05, 0.0])
    qd = np.asarray(impl.dls_velocity(J, v, 0.0))
    np.testing.assert_allclose(qd, np.linalg.solve(J, v), atol=1e-9)
    np.testing.assert_allclose(qd, [0.0, -0.37037], atol=1e-4)      # 0 and -21.22 deg/s


def test_zero_damping_gives_the_least_norm_solution_when_redundant(impl):
    """3 joints, 2 equations: infinitely many exact answers. The pseudo-inverse picks the smallest."""
    q = np.radians([20.0, 50.0, -30.0])
    J = np.asarray(impl.planar_jacobian(LENGTHS3, q))
    v = np.array([0.03, 0.02])
    qd = np.asarray(impl.dls_velocity(J, v, 0.0))
    np.testing.assert_allclose(J @ qd, v, atol=1e-9)                # still exact
    null = np.linalg.svd(J)[2][-1]                                  # a null-space direction
    assert abs(float(qd @ null)) < 1e-9, "the solution has a null-space component: not least-norm"


def test_damping_bounds_the_joint_speed_at_a_singularity(impl):
    J = np.asarray(impl.planar_jacobian((L1, L2), np.radians([30.0, 0.0])))   # exactly singular
    v = np.array([0.05, 0.0])
    for damping in (0.005, 0.01, 0.05):
        qd = np.asarray(impl.dls_velocity(J, v, damping))
        assert np.all(np.isfinite(qd))
        assert np.linalg.norm(qd) <= np.linalg.norm(v) / (2.0 * damping) + 1e-9


def test_damping_solves_the_regularised_least_squares_problem(impl):
    """The defining property: it minimises |J qd - v|^2 + lambda^2 |qd|^2."""
    q = np.radians([30.0, 4.0])
    J = np.asarray(impl.planar_jacobian((L1, L2), q))
    v = np.array([0.05, 0.0])
    lam = 0.02
    qd = np.asarray(impl.dls_velocity(J, v, lam))

    def cost(x):
        return float(np.sum((J @ x - v) ** 2) + lam ** 2 * np.sum(x ** 2))

    rng = np.random.default_rng(7)
    for _ in range(200):
        assert cost(qd) <= cost(qd + rng.normal(scale=0.05, size=qd.size)) + 1e-12


def test_damping_shrinks_the_solution_monotonically(impl):
    J = impl.planar_jacobian((L1, L2), np.radians([30.0, 2.0]))
    v = np.array([0.05, 0.0])
    norms = [np.linalg.norm(np.asarray(impl.dls_velocity(J, v, d))) for d in (0.001, 0.01, 0.05, 0.2)]
    assert norms == sorted(norms, reverse=True)


# --- speed limits ---------------------------------------------------------------------------------
def test_scale_to_limits_preserves_direction(impl):
    qd = np.array([0.2, -8.0, 3.0])
    out = np.asarray(impl.scale_to_limits(qd, 4.0))
    assert np.abs(out).max() == pytest.approx(4.0)
    np.testing.assert_allclose(out / np.linalg.norm(out), qd / np.linalg.norm(qd), atol=1e-12)


def test_scale_to_limits_leaves_a_legal_vector_alone(impl):
    qd = np.array([0.2, -1.0, 3.0])
    np.testing.assert_allclose(np.asarray(impl.scale_to_limits(qd, 4.0)), qd, atol=1e-12)


def test_scale_to_limits_rejects_a_nonpositive_limit(impl):
    with pytest.raises(ValueError):
        impl.scale_to_limits(np.array([1.0, 2.0]), 0.0)


def test_worst_case_joint_speed(impl):
    J = impl.planar_jacobian((L1, L2), np.radians([30.0, 10.0]))
    sigma_min = float(np.linalg.svd(np.asarray(J), compute_uv=False)[-1])
    assert sigma_min == pytest.approx(0.009575, abs=1e-5)
    assert impl.worst_case_joint_speed(J, 0.05) == pytest.approx(0.05 / sigma_min, rel=1e-9)
    singular = impl.planar_jacobian((L1, L2), np.radians([30.0, 0.0]))
    assert math.isinf(impl.worst_case_joint_speed(singular, 0.05)), (
        "the SVD of a singular J returns ~1e-17, not 0.0 — treat anything <= 1e-12 as a singularity")


def test_worst_case_bounds_every_commanded_direction(impl):
    q = np.radians([30.0, 25.0])
    J = np.asarray(impl.planar_jacobian((L1, L2), q))
    bound = impl.worst_case_joint_speed(J, 0.05)
    for angle in np.linspace(0, 2 * math.pi, 60):
        v = 0.05 * np.array([math.cos(angle), math.sin(angle)])
        qd = np.asarray(impl.dls_velocity(J, v, 0.0))
        assert np.linalg.norm(qd) <= bound + 1e-9


# --- statics ---------------------------------------------------------------------------------------
def test_payload_torques_at_the_hand_computed_pose(impl):
    """200 g hanging at q = (30, 60) deg: the tip is straight above the elbow, so the elbow is free."""
    J = impl.planar_jacobian((L1, L2), np.radians([30.0, 60.0]))
    tau = np.asarray(impl.payload_torques(J, [0.0, -0.2 * G]))
    np.testing.assert_allclose(tau, [-0.19711, 0.0], atol=1e-4)


def test_power_is_conserved(impl):
    """f . v = tau . qdot for any joint velocity: the same duality that makes tau = J^T f true."""
    rng = np.random.default_rng(11)
    q = np.radians([25.0, -40.0])
    J = np.asarray(impl.planar_jacobian((L1, L2), q))
    force = np.array([1.3, -2.7])
    tau = np.asarray(impl.payload_torques(J, force))
    for _ in range(20):
        qd = rng.normal(size=2)
        assert float(force @ (J @ qd)) == pytest.approx(float(tau @ qd), abs=1e-12)


def test_a_vertical_payload_never_loads_a_joint_above_it(impl):
    """Fold the arm so the tip sits exactly on the shoulder axis: gravity gives the shoulder no moment."""
    q = np.radians([0.0, 180.0])            # tip at x = l1 - l2 on the x axis, y = 0
    J = impl.planar_jacobian((L1, L2), q)
    tau = np.asarray(impl.payload_torques(J, [0.0, -0.2 * G]))
    assert abs(tau[1]) == pytest.approx(0.2 * G * L2, abs=1e-9)
