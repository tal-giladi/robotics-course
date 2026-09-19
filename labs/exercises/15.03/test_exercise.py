"""Checker for 15.03 — hand-eye calibration: SE(3) log/exp, AX = XB pairs, Park-Martin.

Run: ``python course.py check 15.03`` (or ``--solution`` to see the reference pass).
The synthetic rigs here are the same ones ``15-manipulation/code/hand_eye.py`` uses, so the
numbers in the lesson's tables are reproducible from this exercise.
"""

from __future__ import annotations

import math

import numpy as np
import pytest


# --- synthetic rigs (given to the tests; the student never sees ground truth in their own code) --
def true_X(impl):
    """Camera 35 mm above the tool point, 30 mm behind it, pitched 20 deg down, rolled -90 deg."""
    return impl.make_T(impl.rot_xyz(0.0, math.radians(20.0), 0.0) @ impl.rot_xyz(math.radians(-90), 0, 0),
                       (0.0, -0.030, 0.035))


def gripper_poses(impl, n: int, rng, *, single_axis: bool = False):
    centre = np.array([0.22, 0.0, 0.18])
    s = math.radians(35.0)
    poses = []
    for _ in range(n):
        t = centre + rng.uniform(-0.07, 0.07, size=3)
        if single_axis:
            rpy = (math.pi, 0.0, rng.uniform(-s, s))
        else:
            rpy = (math.pi + rng.uniform(-s, s), rng.uniform(-s, s), rng.uniform(-math.pi, math.pi))
        poses.append(impl.make_T(impl.rot_xyz(*rpy), t))
    return poses


def perturb(impl, T, sigma_mm, sigma_deg, rng):
    if sigma_mm <= 0.0 and sigma_deg <= 0.0:
        return T.copy()
    out = T.copy()
    out[:3, :3] = T[:3, :3] @ impl.rot_of(rng.normal(0.0, math.radians(sigma_deg), size=3))
    out[:3, 3] = T[:3, 3] + rng.normal(0.0, sigma_mm / 1000.0, size=3)
    return out


def eye_in_hand_data(impl, n, rng, *, noise=(0.0, 0.0, 0.0, 0.0), single_axis=False):
    """(g list, c list, X_true). noise = (pnp_deg, pnp_mm, fk_deg, fk_mm)."""
    X = true_X(impl)
    T_base_target = impl.make_T(impl.rot_xyz(0.0, 0.0, math.radians(12.0)), (0.25, 0.02, 0.0))
    gs, cs = [], []
    for T_bg in gripper_poses(impl, n, rng, single_axis=single_axis):
        T_ct = impl.inv_T(T_bg @ X) @ T_base_target
        cs.append(perturb(impl, T_ct, noise[1], noise[0], rng))
        gs.append(perturb(impl, T_bg, noise[3], noise[2], rng))
    return gs, cs, X


def eye_to_hand_data(impl, n, rng, *, noise=(0.0, 0.0, 0.0, 0.0)):
    """(g list, c list, T_base_cam) for a camera on a tripod, board on the gripper."""
    T_gripper_target = impl.make_T(impl.rot_xyz(math.radians(10), 0.0, math.radians(-25)), (0.0, 0.0, 0.055))
    T_base_cam = impl.make_T(impl.rot_xyz(math.radians(-150), 0.0, math.radians(25)), (0.45, -0.30, 0.40))
    gs, cs = [], []
    for T_bg in gripper_poses(impl, n, rng):
        T_ct = impl.inv_T(T_base_cam) @ T_bg @ T_gripper_target
        cs.append(perturb(impl, T_ct, noise[1], noise[0], rng))
        gs.append(perturb(impl, T_bg, noise[3], noise[2], rng))
    return gs, cs, T_base_cam


NOISE = (0.5, 1.0, 0.3, 0.5)   # pnp_deg, pnp_mm, fk_deg, fk_mm — the realistic case


# --- rotvec_of / rot_of -------------------------------------------------------------------------
def test_log_of_the_identity_is_zero(impl):
    assert np.allclose(impl.rotvec_of(np.eye(3)), 0.0)


def test_exp_of_zero_is_the_identity(impl):
    assert np.allclose(impl.rot_of([0.0, 0.0, 0.0]), np.eye(3))


def test_log_of_a_90_degree_z_rotation(impl):
    R = impl.rot_xyz(0.0, 0.0, math.pi / 2)
    assert np.allclose(impl.rotvec_of(R), [0.0, 0.0, math.pi / 2], atol=1e-12)


def test_exp_and_log_are_inverses(impl):
    rng = np.random.default_rng(0)
    for _ in range(50):
        r = rng.normal(0.0, 1.0, size=3)
        r = r / np.linalg.norm(r) * rng.uniform(0.01, 3.0)   # angles up to ~172 deg
        assert np.allclose(impl.rotvec_of(impl.rot_of(r)), r, atol=1e-9)


def test_log_survives_a_rotation_of_exactly_180_degrees(impl):
    """The naive formula divides by sin(theta) = 0 here. Use the symmetric part of R."""
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 1, 1]):
        k = np.asarray(axis, dtype=float)
        k = k / np.linalg.norm(k)
        R = impl.rot_of(k * math.pi)
        back = impl.rotvec_of(R)
        assert np.allclose(np.abs(back), np.abs(k * math.pi), atol=1e-5), f"axis {axis}"
        assert np.allclose(impl.rot_of(back), R, atol=1e-6), f"axis {axis}: log must round-trip"


def test_rot_of_produces_a_proper_rotation(impl):
    rng = np.random.default_rng(1)
    for _ in range(20):
        R = impl.rot_of(rng.normal(0.0, 1.0, size=3))
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert float(np.linalg.det(R)) == pytest.approx(1.0, abs=1e-12)


# --- motion_pairs -------------------------------------------------------------------------------
def test_pair_count_is_n_choose_2(impl):
    rng = np.random.default_rng(2)
    gs, cs, _ = eye_in_hand_data(impl, 5, rng)
    A, B = impl.motion_pairs(gs, cs)
    assert len(A) == len(B) == 10, "all C(n, 2) pairs, not just consecutive ones"


def test_too_few_poses_is_rejected(impl):
    rng = np.random.default_rng(2)
    gs, cs, _ = eye_in_hand_data(impl, 2, rng)
    with pytest.raises(ValueError):
        impl.motion_pairs(gs, cs)


def test_mismatched_lists_are_rejected(impl):
    rng = np.random.default_rng(2)
    gs, cs, _ = eye_in_hand_data(impl, 5, rng)
    with pytest.raises(ValueError):
        impl.motion_pairs(gs, cs[:4])


def test_ax_equals_xb_exactly_for_clean_eye_in_hand_data(impl):
    rng = np.random.default_rng(42)
    gs, cs, X = eye_in_hand_data(impl, 4, rng)
    A, B = impl.motion_pairs(gs, cs, eye_in_hand=True)
    for Ai, Bi in zip(A, B):
        assert np.allclose(Ai @ X, X @ Bi, atol=1e-12), (
            "the pair construction is wrong: A X must equal X B to machine precision on exact data")


def test_the_rotation_angles_of_a_and_b_match(impl):
    """‖log R_A‖ = ‖log R_B‖ for every pair — the free data-quality check of lesson 15.03."""
    rng = np.random.default_rng(42)
    gs, cs, _ = eye_in_hand_data(impl, 5, rng)
    A, B = impl.motion_pairs(gs, cs)
    for Ai, Bi in zip(A, B):
        a = float(np.linalg.norm(impl.rotvec_of(Ai[:3, :3])))
        b = float(np.linalg.norm(impl.rotvec_of(Bi[:3, :3])))
        assert a == pytest.approx(b, abs=1e-9)


def test_eye_to_hand_uses_the_other_pair_construction(impl):
    rng = np.random.default_rng(7)
    gs, cs, X = eye_to_hand_data(impl, 4, rng)
    A, B = impl.motion_pairs(gs, cs, eye_in_hand=False)
    for Ai, Bi in zip(A, B):
        assert np.allclose(Ai @ X, X @ Bi, atol=1e-12), (
            "eye-to-hand is A = g_j g_i^-1, not g_j^-1 g_i — the wrong one still 'converges'")


# --- solve_ax_xb_park ---------------------------------------------------------------------------
def test_recovers_x_exactly_from_clean_data(impl):
    rng = np.random.default_rng(42)
    gs, cs, X = eye_in_hand_data(impl, 6, rng)
    est = impl.calibrate_eye_in_hand(gs, cs)
    dt, dr = impl.pose_error(est, X)
    assert dt < 1e-6, f"translation error {dt:.3g} mm on exact data"
    assert dr < 1e-6, f"rotation error {dr:.3g} deg on exact data"


def test_the_solved_rotation_is_a_proper_rotation(impl):
    """Without the SVD re-orthonormalisation this is a near-rotation with det ~0.9997."""
    rng = np.random.default_rng(11)
    gs, cs, _ = eye_in_hand_data(impl, 12, rng, noise=NOISE)
    R = impl.calibrate_eye_in_hand(gs, cs)[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-10), "R_X must be orthonormal"
    assert float(np.linalg.det(R)) == pytest.approx(1.0, abs=1e-10)


def test_recovers_x_from_noisy_data(impl):
    """15 poses, realistic noise: the lesson reports ~1.2 mm / 0.40 deg averaged over seeds."""
    errs = []
    for seed in range(8):
        rng = np.random.default_rng(100 + seed)
        gs, cs, X = eye_in_hand_data(impl, 15, rng, noise=NOISE)
        errs.append(impl.pose_error(impl.calibrate_eye_in_hand(gs, cs), X))
    mean_t = float(np.mean([e[0] for e in errs]))
    mean_r = float(np.mean([e[1] for e in errs]))
    assert mean_t < 3.0, f"mean translation error {mean_t:.2f} mm — expected around 1.2 mm"
    assert mean_r < 1.0, f"mean rotation error {mean_r:.3f} deg — expected around 0.4 deg"


def test_more_poses_are_better(impl):
    def mean_error(n):
        return float(np.mean([impl.pose_error(
            impl.calibrate_eye_in_hand(*eye_in_hand_data(impl, n, np.random.default_rng(1000 + s),
                                                         noise=NOISE)[:2]),
            true_X(impl))[0] for s in range(8)]))

    assert mean_error(20) < mean_error(3), "3 poses is a disaster; 20 is good"


def test_eye_to_hand_recovers_the_tripod_camera(impl):
    rng = np.random.default_rng(7)
    gs, cs, T_base_cam = eye_to_hand_data(impl, 10, rng)
    dt, dr = impl.pose_error(impl.calibrate_eye_to_hand(gs, cs), T_base_cam)
    assert dt < 1e-6 and dr < 1e-6


# --- consistency_residual -----------------------------------------------------------------------
def test_the_residual_is_zero_for_a_perfect_calibration(impl):
    rng = np.random.default_rng(42)
    gs, cs, X = eye_in_hand_data(impl, 6, rng)
    t_mm, r_deg = impl.consistency_residual(gs, cs, X)
    assert t_mm < 1e-6 and r_deg < 1e-6


def test_the_residual_grows_when_x_is_wrong(impl):
    rng = np.random.default_rng(42)
    gs, cs, X = eye_in_hand_data(impl, 6, rng)
    wrong = X.copy()
    wrong[:3, 3] += np.array([0.020, 0.0, 0.0])          # 20 mm off
    t_mm, _ = impl.consistency_residual(gs, cs, wrong)
    assert t_mm > 5.0, "a 20 mm error in X must show up in the residual"


def test_the_residual_is_in_millimetres_and_degrees(impl):
    rng = np.random.default_rng(5)
    gs, cs, X = eye_in_hand_data(impl, 12, rng, noise=NOISE)
    t_mm, r_deg = impl.consistency_residual(gs, cs, impl.calibrate_eye_in_hand(gs, cs))
    assert 0.1 < t_mm < 20.0, f"expected a few mm, got {t_mm}"
    assert 0.0 <= r_deg < 10.0, f"expected well under a degree, got {r_deg}"


# --- rotation_spread_deg and the silent failure --------------------------------------------------
def test_rotation_spread_separates_good_and_degenerate_datasets(impl):
    rng = np.random.default_rng(50)
    good, _, _ = eye_in_hand_data(impl, 20, rng, noise=NOISE)
    rng = np.random.default_rng(50)
    bad, _, _ = eye_in_hand_data(impl, 20, rng, noise=NOISE, single_axis=True)
    assert impl.rotation_spread_deg(good) > 60.0
    assert impl.rotation_spread_deg(bad) < 40.0, (
        "a single-axis dataset must score low — this is the only warning you get")


def test_a_single_axis_dataset_is_badly_wrong(impl):
    """The lesson's headline trap: every A rotates about z, so (R_A - I) is singular along z and
    t_X along z is unobservable. The solver reports nothing — you must check the dataset."""
    def mean_error(single_axis):
        out = []
        for seed in range(5):
            rng = np.random.default_rng(50 + seed)
            gs, cs, X = eye_in_hand_data(impl, 20, rng, noise=NOISE, single_axis=single_axis)
            out.append(impl.pose_error(impl.calibrate_eye_in_hand(gs, cs), X)[0])
        return float(np.mean(out))

    varied, degenerate = mean_error(False), mean_error(True)
    assert varied < 3.0, f"a varied 20-pose dataset should solve to ~1 mm, got {varied:.1f} mm"
    assert degenerate > 20.0 * varied, (
        f"a z-only dataset must be far worse ({degenerate:.1f} mm vs {varied:.1f} mm) — "
        "and neither the solver nor an exception tells you")


def test_a_degenerate_dataset_breaks_the_rotation_too(impl):
    """Not just the translation: with every alpha_i and beta_i along one axis, M = sum outer(beta, alpha)
    is rank 1, so (M^T M)^(-1/2) is meaningless in its null space and R_X is arbitrary there.

    (The course module's nonlinear refinement repairs most of this and leaves ~60 mm with a ~2 mm
    residual — which is exactly what makes the failure *silent*. The closed form alone at least
    fails loudly. Neither is a substitute for checking ``rotation_spread_deg``.)
    """
    rot_errs = []
    for seed in range(5):
        rng = np.random.default_rng(50 + seed)
        gs, cs, X = eye_in_hand_data(impl, 20, rng, noise=NOISE, single_axis=True)
        rot_errs.append(impl.pose_error(impl.calibrate_eye_in_hand(gs, cs), X)[1])
    assert float(np.mean(rot_errs)) > 10.0

    M = np.zeros((3, 3))
    rng = np.random.default_rng(50)
    gs, cs, _ = eye_in_hand_data(impl, 20, rng, noise=NOISE, single_axis=True)
    A, B = impl.motion_pairs(gs, cs)
    for Ai, Bi in zip(A, B):
        M += np.outer(impl.rotvec_of(Bi[:3, :3]), impl.rotvec_of(Ai[:3, :3]))
    s = np.linalg.svd(M, compute_uv=False)
    assert s[1] < 0.05 * s[0], f"M should be nearly rank 1 for a single-axis dataset, got {s}"
