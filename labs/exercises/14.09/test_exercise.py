"""Checker for 14.09 — collision checking, path validity and the resolution that decides it.

Run: ``python course.py check 14.09`` (or ``--solution`` to see the reference pass).
No ROS, no MoveIt, no hardware.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

LINK_RADIUS = 0.02
JOINT_LIMITS = np.array([[-1.91986, 1.91986], [-1.74533, 1.74533], [-1.69, 1.69]])
EXTENT = float((JOINT_LIMITS[:, 1] - JOINT_LIMITS[:, 0]).sum())      # 10.7104 rad


# --- closest_point_on_segment ----------------------------------------------------------------
def test_projects_onto_the_middle(impl):
    got = impl.closest_point_on_segment([0, 0, 0], [1, 0, 0], [0.4, 0.7, 0.0])
    np.testing.assert_allclose(np.asarray(got), [0.4, 0.0, 0.0], atol=1e-12)


def test_clamps_beyond_the_ends(impl):
    """Without the clamp you get a point on the infinite LINE and the arm hits things behind it."""
    np.testing.assert_allclose(np.asarray(impl.closest_point_on_segment([0, 0, 0], [1, 0, 0],
                                                                        [5.0, 1.0, 0.0])),
                               [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(np.asarray(impl.closest_point_on_segment([0, 0, 0], [1, 0, 0],
                                                                        [-3.0, 1.0, 0.0])),
                               [0.0, 0.0, 0.0], atol=1e-12)


def test_handles_a_degenerate_segment(impl):
    got = impl.closest_point_on_segment([0.2, 0.3, 0.4], [0.2, 0.3, 0.4], [9.0, 9.0, 9.0])
    np.testing.assert_allclose(np.asarray(got), [0.2, 0.3, 0.4], atol=1e-12)


def test_a_point_on_the_segment_maps_to_itself(impl):
    rng = np.random.default_rng(9)
    a, b = np.array([0.1, -0.2, 0.3]), np.array([0.4, 0.5, -0.1])
    for t in rng.uniform(0, 1, size=20):
        p = a + t * (b - a)
        np.testing.assert_allclose(np.asarray(impl.closest_point_on_segment(a, b, p)), p, atol=1e-12)


# --- capsule_sphere_clearance ------------------------------------------------------------------
def test_clearance_subtracts_both_radii(impl):
    # axis distance 0.5, radii 0.02 and 0.03 -> 0.45 of gap
    got = impl.capsule_sphere_clearance([0, 0, 0], [1, 0, 0], 0.02, [0.5, 0.5, 0.0], 0.03)
    assert got == pytest.approx(0.45)


def test_clearance_is_negative_when_they_overlap(impl):
    got = impl.capsule_sphere_clearance([0, 0, 0], [1, 0, 0], 0.05, [0.5, 0.04, 0.0], 0.03)
    assert got == pytest.approx(0.04 - 0.05 - 0.03)


def test_clearance_is_zero_when_exactly_touching(impl):
    got = impl.capsule_sphere_clearance([0, 0, 0], [1, 0, 0], 0.02, [0.5, 0.05, 0.0], 0.03)
    assert got == pytest.approx(0.0, abs=1e-12)


def test_clearance_uses_the_end_cap_beyond_the_segment(impl):
    """A sphere off the end is measured from the endpoint, not from the infinite line."""
    got = impl.capsule_sphere_clearance([0, 0, 0], [1, 0, 0], 0.0, [2.0, 0.0, 0.0], 0.0)
    assert got == pytest.approx(1.0)


# --- state_clearance ----------------------------------------------------------------------------
def test_no_obstacles_means_infinite_clearance(impl):
    assert math.isinf(impl.state_clearance([0.0, 0.0, 0.0], []))


def test_a_sphere_on_the_tip_collides(impl):
    q = [0.0, 0.0, 0.0]
    tip = np.asarray(impl.arm_points(q))[-1]
    assert impl.state_clearance(q, [(tip, 0.01)]) == pytest.approx(-0.03)   # -(0.02 + 0.01)


def test_a_distant_sphere_does_not(impl):
    q = [0.0, 0.0, 0.0]
    assert impl.state_clearance(q, [([0.0, 1.0, 0.0], 0.05)]) > 0.5


def test_it_reports_the_WORST_of_several_obstacles(impl):
    q = [0.0, 0.3, -0.4]
    tip = np.asarray(impl.arm_points(q))[-1]
    far = ([0.0, 1.0, 0.0], 0.05)
    near = (tip + np.array([0.0, 0.05, 0.0]), 0.01)
    worst = impl.state_clearance(q, [far, near])
    assert worst == pytest.approx(impl.state_clearance(q, [near]))


def test_it_checks_every_link_not_just_the_last(impl):
    """A sphere on the UPPER ARM must be found even though the tip is far from it."""
    q = [0.0, 0.4, -0.8]
    points = np.asarray(impl.arm_points(q))
    midpoint = (points[1] + points[2]) / 2.0
    assert impl.state_clearance(q, [(midpoint, 0.005)]) == pytest.approx(-0.025)


# --- interpolate_states ---------------------------------------------------------------------------
def test_endpoints_are_always_included(impl):
    q0, q1 = np.array([0.0, 0.0, 0.0]), np.array([1.0, -0.5, 0.25])
    states = np.asarray(impl.interpolate_states(q0, q1, 0.3))
    np.testing.assert_allclose(states[0], q0, atol=1e-12)
    np.testing.assert_allclose(states[-1], q1, atol=1e-12)


def test_the_step_is_never_exceeded(impl):
    q0, q1 = np.array([-0.8, 0.2, 0.0]), np.array([0.9, -0.7, 1.1])
    for step in (0.5, 0.1, 0.03):
        states = np.asarray(impl.interpolate_states(q0, q1, step))
        assert np.abs(np.diff(states, axis=0)).max() <= step + 1e-12


def test_the_count_is_segments_plus_one(impl):
    """1.0 rad at 0.25 rad/step is FOUR segments and FIVE states."""
    states = np.asarray(impl.interpolate_states([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.25))
    assert states.shape == (5, 3)


def test_a_step_that_does_not_divide_evenly_rounds_up(impl):
    states = np.asarray(impl.interpolate_states([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.3))
    assert states.shape == (5, 3)              # ceil(1.0 / 0.3) = 4 segments
    assert np.abs(np.diff(states[:, 0])).max() <= 0.3 + 1e-12


def test_a_zero_length_move_returns_one_state(impl):
    states = np.asarray(impl.interpolate_states([0.2, 0.2, 0.2], [0.2, 0.2, 0.2], 0.1))
    assert states.shape == (1, 3)


def test_rejects_a_nonpositive_step(impl):
    with pytest.raises(ValueError):
        impl.interpolate_states([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.0)


# --- path_clearance --------------------------------------------------------------------------------
def test_a_clear_path_stays_clear(impl):
    q0, q1 = np.array([-0.8, -0.3, 0.9]), np.array([0.8, -0.3, 0.9])
    clearance, _ = impl.path_clearance(q0, q1, [([0.0, 0.0, 0.60], 0.03)], max_step=0.05)
    assert clearance > 0.0


def test_an_obstacle_in_the_middle_is_found_and_located(impl):
    q0, q1 = np.array([-0.8, -0.3, 0.9]), np.array([0.8, -0.3, 0.9])
    middle = (q0 + q1) / 2.0
    tip = np.asarray(impl.arm_points(middle))[-1]
    clearance, index = impl.path_clearance(q0, q1, [(tip, 0.01)], max_step=0.02)
    states = np.asarray(impl.interpolate_states(q0, q1, 0.02))
    assert clearance < 0.0
    assert index == pytest.approx(len(states) // 2, abs=3)


def test_a_coarse_step_can_miss_what_a_fine_step_finds(impl):
    """Tunnelling, in one assertion. This is the whole point of the lesson."""
    q0, q1 = np.array([-0.8, -0.3, 0.9]), np.array([0.8, -0.3, 0.9])
    middle = (q0 + q1) / 2.0
    centre = np.asarray(impl.arm_points(middle))[-1] + np.array([0.0, 0.0, 0.03])
    obstacles = [(centre, 0.015)]
    span = float(np.max(np.abs(q1 - q0)))
    coarse, _ = impl.path_clearance(q0, q1, obstacles, max_step=span / 5)
    fine, _ = impl.path_clearance(q0, q1, obstacles, max_step=span / 200)
    assert coarse > 0.0, "a 5-step check should miss this obstacle"
    assert fine < 0.0, "a 200-step check must find it"


def test_a_coarse_check_is_never_more_pessimistic_than_the_truth(impl):
    """A dense check approximates the true minimum, so it bounds every coarse answer from below.

    (Note what this does NOT say: refining the step is not monotone, because a finer sample set
    does not contain the coarser one. Only the dense reference is meaningful.)
    """
    q0, q1 = np.array([-0.6, -0.2, 0.8]), np.array([0.7, -0.4, 1.0])
    obstacles = [([0.20, 0.02, 0.14], 0.02)]
    truth, _ = impl.path_clearance(q0, q1, obstacles, max_step=0.0005)
    for step in (0.4, 0.2, 0.1, 0.05, 0.01):
        clearance, _ = impl.path_clearance(q0, q1, obstacles, max_step=step)
        assert truth <= clearance + 1e-6


# --- required_segment_fraction -----------------------------------------------------------------------
def test_the_fraction_formula(impl):
    got = impl.required_segment_fraction(JOINT_LIMITS, 0.4399, 0.010)
    assert got == pytest.approx(0.010 / (EXTENT * 0.4399), rel=1e-9)


def test_the_so101_numbers_from_the_lesson(impl):
    """The real arm: extent 19.6116 rad, worst Jacobian column 0.4399 m/rad."""
    so101 = np.array([[-1.91986, 1.91986], [-1.74533, 1.74533], [-1.69, 1.69],
                      [-1.65806, 1.65806], [-2.74385, 2.84121]])
    assert impl.required_segment_fraction(so101, 0.4399, 0.010) == pytest.approx(0.00116, abs=1e-5)
    # and the inverse: what the generated default of 0.005 actually permits
    extent = float((so101[:, 1] - so101[:, 0]).sum())
    assert extent == pytest.approx(19.6116, abs=1e-3)
    assert 0.005 * extent * 0.4399 == pytest.approx(0.0431, abs=1e-3)


def test_a_smaller_tool_step_needs_a_smaller_fraction(impl):
    coarse = impl.required_segment_fraction(JOINT_LIMITS, 0.25, 0.02)
    fine = impl.required_segment_fraction(JOINT_LIMITS, 0.25, 0.005)
    assert fine == pytest.approx(coarse / 4.0, rel=1e-9)


def test_rejects_a_nonpositive_tool_step(impl):
    with pytest.raises(ValueError):
        impl.required_segment_fraction(JOINT_LIMITS, 0.25, 0.0)


def test_the_fraction_actually_guarantees_the_resolution(impl):
    """Use the fraction to pick a step, then check no state-to-state tip motion exceeds it."""
    q0, q1 = np.array([-0.8, -0.3, 0.9]), np.array([0.8, -0.3, 0.9])
    worst_column = max(impl.max_jacobian_column(q)
                       for q in np.asarray(impl.interpolate_states(q0, q1, 0.01)))
    tool_step = 0.015
    fraction = impl.required_segment_fraction(JOINT_LIMITS, worst_column, tool_step)
    summed_step = fraction * EXTENT
    states = np.asarray(impl.interpolate_states(q0, q1, summed_step / len(q0)))
    tips = np.array([np.asarray(impl.arm_points(q))[-1] for q in states])
    assert np.linalg.norm(np.diff(tips, axis=0), axis=1).max() <= tool_step + 1e-9
