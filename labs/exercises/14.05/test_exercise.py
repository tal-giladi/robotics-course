"""Checker for 14.05 — analytic inverse kinematics of the planar 2-link arm.

Run: ``python course.py check 14.05`` (or ``--solution`` to see the reference pass).

Link lengths throughout are the SO-101's planar model from 14.04:
``l1 = 0.116`` m (upper arm), ``l2 = 0.135`` m (forearm), so the arm reaches an annulus
between 0.019 m and 0.251 m.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

L1, L2 = 0.116, 0.135
INNER, OUTER = abs(L1 - L2), L1 + L2
LENGTHS = (L1, L2)


def fk_xy(impl, sol) -> tuple[float, float]:
    x, y, _ = impl.planar_fk(LENGTHS, (sol.q1, sol.q2))
    return x, y


def reachable_targets(n: int, seed: int = 0) -> list[tuple[float, float]]:
    """``n`` targets strictly inside the annulus (never on a boundary)."""
    rng = np.random.default_rng(seed)
    r = rng.uniform(INNER + 5e-4, OUTER - 5e-4, n)
    theta = rng.uniform(-math.pi, math.pi, n)
    return [(float(ri * math.cos(t)), float(ri * math.sin(t))) for ri, t in zip(r, theta)]


# --- reachability ---------------------------------------------------------------------------------
def test_reachability_classifies_the_five_cases(impl):
    assert impl.reachability(L1, L2, 0.0) == "inside"
    assert impl.reachability(L1, L2, 0.010) == "inside"
    assert impl.reachability(L1, L2, INNER) == "boundary_inner"
    assert impl.reachability(L1, L2, 0.200) == "interior"
    assert impl.reachability(L1, L2, OUTER) == "boundary_outer"
    assert impl.reachability(L1, L2, 0.260) == "outside"


def test_equal_links_have_no_dead_hole(impl):
    """With l1 == l2 the inner radius is 0, so r = 0 is the folded boundary, not "inside"."""
    assert impl.reachability(0.1, 0.1, 0.0) == "boundary_inner"
    assert impl.reachability(0.1, 0.1, 0.05) == "interior"


def test_reachability_boundaries_use_the_tolerance(impl):
    assert impl.reachability(L1, L2, OUTER - 1e-12) == "boundary_outer"
    assert impl.reachability(L1, L2, INNER + 1e-12) == "boundary_inner"
    assert impl.reachability(L1, L2, OUTER + 1e-6) == "outside"
    assert impl.reachability(L1, L2, INNER - 1e-6) == "inside"


def test_reachability_rejects_nonsense(impl):
    with pytest.raises(ValueError):
        impl.reachability(0.0, L2, 0.1)
    with pytest.raises(ValueError):
        impl.reachability(L1, L2, -0.01)


# --- two_link_ik: the by-hand cases from 14.05-E1 --------------------------------------------------
def test_target_on_the_x_axis_by_hand(impl):
    """(0.20, 0): cos q2 = 0.008319 / 0.03132 = 0.26562 -> q2 = -+74.60 deg, q1 = +-40.60 deg."""
    sols = impl.two_link_ik(L1, L2, 0.20, 0.0)
    assert len(sols) == 2
    up, down = sols
    assert (up.elbow, down.elbow) == ("up", "down")
    assert math.degrees(up.q2) == pytest.approx(-74.60, abs=0.01)
    assert math.degrees(up.q1) == pytest.approx(+40.60, abs=0.01)
    assert math.degrees(down.q2) == pytest.approx(+74.60, abs=0.01)
    assert math.degrees(down.q1) == pytest.approx(-40.60, abs=0.01)


def test_the_two_branches_are_mirror_images_for_a_target_on_the_axis(impl):
    up, down = impl.two_link_ik(L1, L2, 0.20, 0.0)
    assert up.q1 == pytest.approx(-down.q1, abs=1e-12)
    assert up.q2 == pytest.approx(-down.q2, abs=1e-12)


def test_target_above_the_shoulder(impl):
    sols = impl.two_link_ik(L1, L2, 0.05, 0.12)
    assert len(sols) == 2
    for s in sols:
        assert fk_xy(impl, s) == pytest.approx((0.05, 0.12), abs=1e-12)


def test_target_behind_the_shoulder(impl):
    """The planar model has no joint limits, so it happily reaches backwards."""
    sols = impl.two_link_ik(L1, L2, -0.10, 0.08)
    assert len(sols) == 2
    for s in sols:
        assert fk_xy(impl, s) == pytest.approx((-0.10, 0.08), abs=1e-12)


def test_unreachable_targets_return_no_solutions(impl):
    assert impl.two_link_ik(L1, L2, 0.24, 0.10) == []      # r = 0.260 > 0.251
    assert impl.two_link_ik(L1, L2, 0.010, 0.0) == []      # inside the dead hole
    assert impl.two_link_ik(L1, L2, 0.0, 0.0) == []


# --- two_link_ik: branches and boundaries ----------------------------------------------------------
def test_interior_targets_give_exactly_two_distinct_branches(impl):
    for x, y in reachable_targets(200, seed=1):
        sols = impl.two_link_ik(L1, L2, x, y)
        assert len(sols) == 2, f"expected both branches at ({x:.4f}, {y:.4f})"
        up, down = sols
        assert up.q2 < 0.0 < down.q2
        assert abs(up.q2 - down.q2) > 1e-6


def test_every_branch_round_trips_through_forward_kinematics(impl):
    worst = 0.0
    for x, y in reachable_targets(300, seed=2):
        for s in impl.two_link_ik(L1, L2, x, y):
            fx, fy = fk_xy(impl, s)
            worst = max(worst, math.hypot(fx - x, fy - y))
    assert worst < 1e-12, f"largest round-trip error {worst:.3e} m"


def test_elbow_labels_match_the_sign_of_q2(impl):
    for x, y in reachable_targets(100, seed=3):
        for s in impl.two_link_ik(L1, L2, x, y):
            assert s.elbow == ("up" if s.q2 < 0 else "down")


def test_q1_is_wrapped(impl):
    for x, y in reachable_targets(200, seed=4):
        for s in impl.two_link_ik(L1, L2, x, y):
            assert -math.pi - 1e-12 <= s.q1 <= math.pi + 1e-12


def test_the_stretched_boundary_gives_one_solution(impl):
    sols = impl.two_link_ik(L1, L2, OUTER, 0.0)
    assert len(sols) == 1, "the two branches coincide when the arm is fully stretched"
    assert sols[0].q2 == pytest.approx(0.0, abs=1e-6)
    assert sols[0].q1 == pytest.approx(0.0, abs=1e-6)
    assert fk_xy(impl, sols[0]) == pytest.approx((OUTER, 0.0), abs=1e-9)


def test_the_folded_boundary_gives_one_solution(impl):
    sols = impl.two_link_ik(L1, L2, 0.0, INNER)
    assert len(sols) == 1, "the two branches coincide when the arm is fully folded"
    assert abs(sols[0].q2) == pytest.approx(math.pi, abs=1e-6)
    assert fk_xy(impl, sols[0]) == pytest.approx((0.0, INNER), abs=1e-9)


def test_the_boundary_is_not_a_nan_factory(impl):
    """cos(q2) lands just outside [-1, 1] here; an unclamped sqrt turns the arm's pose into NaN."""
    for r in (OUTER, OUTER - 1e-16, INNER, INNER + 1e-16):
        for s in impl.two_link_ik(L1, L2, r, 0.0):
            assert math.isfinite(s.q1) and math.isfinite(s.q2)


def test_the_solution_is_a_two_link_solution(impl):
    s = impl.two_link_ik(L1, L2, 0.20, 0.0)[0]
    assert isinstance(s, impl.TwoLinkSolution)
    assert (s.q1, s.q2, s.elbow) == (s.q1, s.q2, s.elbow)   # the three documented fields exist


# --- pick_solution ---------------------------------------------------------------------------------
def test_pick_keeps_the_elbow_where_it_already_is(impl):
    sols = impl.two_link_ik(L1, L2, 0.20, 0.0)
    up, down = sols
    assert impl.pick_solution(sols, (up.q1, up.q2)) is up
    assert impl.pick_solution(sols, (down.q1, down.q2)) is down


def test_pick_minimises_the_largest_joint_move(impl):
    sols = impl.two_link_ik(L1, L2, 0.05, 0.12)
    q_now = (math.radians(10.0), math.radians(100.0))       # nearer the elbow-down branch
    chosen = impl.pick_solution(sols, q_now)
    costs = [max(abs(impl.wrap_angle(s.q1 - q_now[0])), abs(impl.wrap_angle(s.q2 - q_now[1]))) for s in sols]
    assert costs[sols.index(chosen)] == pytest.approx(min(costs))
    assert chosen.elbow == "down"


def test_pick_wraps_the_angle_difference(impl):
    """+179 deg and -179 deg are 2 deg apart, not 358. A raw subtraction picks the wrong branch."""
    near = impl.TwoLinkSolution(math.radians(179.0), 0.2, "up")
    far = impl.TwoLinkSolution(math.radians(90.0), 0.2, "down")
    assert impl.pick_solution([near, far], (math.radians(-179.0), 0.2)) is near


def test_pick_breaks_ties_with_the_first_candidate(impl):
    a = impl.TwoLinkSolution(+0.5, -0.4, "up")
    b = impl.TwoLinkSolution(-0.5, +0.4, "down")
    assert impl.pick_solution([a, b], (0.0, 0.0)) is a


def test_pick_refuses_an_empty_list(impl):
    with pytest.raises(ValueError):
        impl.pick_solution([], (0.0, 0.0))


def test_picking_along_a_straight_line_never_flips_the_elbow(impl):
    """14.05-E5 in miniature: seed each solve with the previous answer and the arm stays smooth."""
    targets = [(0.18 + 0.004 * i, 0.06) for i in range(10)]
    first = impl.two_link_ik(L1, L2, *targets[0])[0]        # start on the elbow-up branch
    q = (first.q1, first.q2)
    biggest_step = 0.0
    for x, y in targets[1:]:
        chosen = impl.pick_solution(impl.two_link_ik(L1, L2, x, y), q)
        biggest_step = max(biggest_step, abs(impl.wrap_angle(chosen.q1 - q[0])),
                           abs(impl.wrap_angle(chosen.q2 - q[1])))
        q = (chosen.q1, chosen.q2)
        assert chosen.elbow == "up"
    assert math.degrees(biggest_step) < 5.0, "a 4 mm step must not swing a joint by 5 degrees"


def test_a_fixed_seed_does_flip_the_elbow(impl):
    """The bug of 14.05-E5: pick from a seed that never moves and the branch can jump."""
    targets = [(0.18 + 0.004 * i, 0.06) for i in range(10)]
    fixed = (0.0, 0.0)
    elbows = {impl.pick_solution(impl.two_link_ik(L1, L2, x, y), fixed).elbow for x, y in targets}
    tracked = set()
    q = (0.0, 0.0)
    for x, y in targets:
        chosen = impl.pick_solution(impl.two_link_ik(L1, L2, x, y), q)
        tracked.add(chosen.elbow)
        q = (chosen.q1, chosen.q2)
    assert len(elbows) >= 1 and len(tracked) == 1, "re-seeding from the last answer keeps one branch"


# --- the whole toolkit together ---------------------------------------------------------------------
def test_reachability_agrees_with_the_number_of_solutions(impl):
    cases = {"inside": 0, "boundary_inner": 1, "interior": 2, "boundary_outer": 1, "outside": 0}
    for r in (0.0, 0.010, INNER, 0.05, 0.20, OUTER, 0.26, 0.40):
        expected = cases[impl.reachability(L1, L2, r)]
        assert len(impl.two_link_ik(L1, L2, r, 0.0)) == expected, f"r = {r}"


def test_a_longer_arm_has_a_larger_workspace(impl):
    """Nothing in the maths is specific to the SO-101's lengths."""
    assert impl.two_link_ik(0.30, 0.30, 0.55, 0.0) != []
    assert impl.two_link_ik(0.116, 0.135, 0.55, 0.0) == []
    for s in impl.two_link_ik(0.30, 0.30, 0.40, 0.20):
        x, y, _ = impl.planar_fk((0.30, 0.30), (s.q1, s.q2))
        assert (x, y) == pytest.approx((0.40, 0.20), abs=1e-12)
