"""Tests for 15.10 — support polygon, centre of mass, tipping margin, frames and the decision."""

from __future__ import annotations

import math

import numpy as np
import pytest

TRACK = 0.200          # labs/config/karmel.yaml: drive.wheel_separation_m
CASTER_X = 0.100       # labs/config/karmel.yaml: chassis.caster_offset_x_m — IN FRONT of the axle


# ---------------------------------------------------------------- convex hull
def test_hull_of_a_square_drops_the_interior_point(impl):
    pts = [(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5)]
    hull = impl.convex_hull_2d(pts)
    assert len(hull) == 4
    assert (0.5, 0.5) not in hull
    assert set(hull) == {(0, 0), (1, 0), (1, 1), (0, 1)}


def test_hull_is_counter_clockwise(impl):
    hull = impl.convex_hull_2d([(0, 0), (1, 0), (1, 1), (0, 1)])
    area = 0.0
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        area += a[0] * b[1] - b[0] * a[1]
    assert area > 0, "the shoelace area is positive only for a counter-clockwise ring"


def test_hull_handles_duplicates_and_tiny_sets(impl):
    assert len(impl.convex_hull_2d([(0, 0), (0, 0), (1, 1)])) == 2
    assert impl.convex_hull_2d([(0, 0)]) == [(0, 0)]


# ---------------------------------------------------------------- support polygon
def test_karmel_support_is_a_triangle_ending_at_the_axle(impl):
    poly = impl.support_polygon(TRACK, CASTER_X)
    assert len(poly) == 3
    assert min(x for x, _ in poly) == pytest.approx(0.0), \
        "nothing supports this base behind the wheel axle — that is the lesson"
    assert max(x for x, _ in poly) == pytest.approx(0.100), \
        "the caster is the single forward-most contact, on the centreline"


def test_a_rear_caster_pair_adds_two_contacts(impl):
    poly = impl.support_polygon(TRACK, CASTER_X, -0.120)
    assert min(x for x, _ in poly) == pytest.approx(-0.120)
    assert len(poly) >= 4


def test_a_duplicate_contact_pair_at_the_axle_changes_nothing(impl):
    """A contact that coincides with one already there adds load, not stability."""
    poly = impl.support_polygon(TRACK, CASTER_X, 0.0)
    assert len(poly) == 3
    assert min(x for x, _ in poly) == pytest.approx(0.0)


# ---------------------------------------------------------------- combined CoM
def test_two_equal_masses_average(impl):
    total, com = impl.combined_com([(1.0, (0.0, 0.0, 0.0)), (1.0, (0.2, 0.0, 0.0))])
    assert total == pytest.approx(2.0)
    np.testing.assert_allclose(com, [0.1, 0.0, 0.0], atol=1e-12)


def test_the_heavier_part_dominates(impl):
    total, com = impl.combined_com([(1.6, (0.0, 0.0, 0.035)), (0.632, (0.15, 0.0, 0.25))])
    assert total == pytest.approx(2.232)
    assert com[0] == pytest.approx(0.632 * 0.15 / 2.232)


def test_a_payload_moves_the_com_forward(impl):
    base = [(1.6, (0.0, 0.0, 0.035)), (0.632, (0.15, 0.0, 0.25))]
    _, without = impl.combined_com(base)
    _, with_load = impl.combined_com(base + [(0.3, (0.30, 0.0, 0.10))])
    assert with_load[0] > without[0]


def test_degenerate_inputs_raise(impl):
    with pytest.raises(ValueError):
        impl.combined_com([])
    with pytest.raises(ValueError):
        impl.combined_com([(0.0, (0.0, 0.0, 0.0))])


# ---------------------------------------------------------------- stability margin
def test_the_centre_of_a_square_is_the_most_stable_point(impl):
    square = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    assert impl.stability_margin(square, (0.0, 0.0)) == pytest.approx(1.0)
    assert impl.stability_margin(square, (0.5, 0.0)) == pytest.approx(0.5)


def test_outside_is_negative(impl):
    square = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    assert impl.stability_margin(square, (1.5, 0.0)) < 0
    assert impl.stability_margin(square, (1.5, 0.0)) == pytest.approx(-0.5)


def test_a_com_on_the_edge_has_zero_margin(impl):
    square = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    assert impl.stability_margin(square, (1.0, 0.0)) == pytest.approx(0.0, abs=1e-12)


def test_karmel_with_the_com_at_the_origin_is_exactly_on_the_limit(impl):
    """base_link is the wheel midpoint, and the REAR edge IS the axle."""
    poly = impl.support_polygon(TRACK, CASTER_X)
    assert impl.stability_margin(poly, (0.0, 0.0)) == pytest.approx(0.0, abs=1e-12)
    assert impl.stability_margin(poly, (-0.020, 0.0)) < 0
    assert impl.stability_margin(poly, (0.030, 0.0)) > 0


def test_the_rear_caster_rescues_a_rearward_com(impl):
    poly = impl.support_polygon(TRACK, CASTER_X, -0.120)
    assert impl.stability_margin(poly, (-0.020, 0.0)) > 0


def test_the_polygon_narrows_towards_the_caster(impl):
    """A lateral offset costs more margin the further forward the CoM already is."""
    poly = impl.support_polygon(TRACK, CASTER_X)
    assert impl.stability_margin(poly, (0.070, 0.0)) > impl.stability_margin(poly, (0.070, 0.030))
    assert impl.stability_margin(poly, (0.070, 0.030)) == pytest.approx(0.0, abs=1e-12), \
        "x + y = 0.100 is exactly the edge from the left wheel to the caster"
    assert impl.stability_margin(poly, (0.020, 0.030)) == pytest.approx(0.020)


def test_margin_uses_the_z_free_projection(impl):
    """Only the ground projection matters for static tipping; a 3-vector must work too."""
    square = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    assert impl.stability_margin(square, (0.0, 0.0, 5.0)) == pytest.approx(1.0)


# ---------------------------------------------------------------- frames
def test_no_rotation_is_a_subtraction(impl):
    p = impl.object_in_arm_frame((1.20, 0.40, 0.0), (-0.040, 0.0, 0.070), (1.50, 0.40, 0.030))
    np.testing.assert_allclose(p, [0.340, 0.0, -0.040], atol=1e-12)


def test_yaw_rotates_the_offset(impl):
    """Base at the origin facing +y; an object 0.3 m along map +y is 0.3 m ahead of the base."""
    p = impl.object_in_arm_frame((0.0, 0.0, math.pi / 2), (0.0, 0.0, 0.0), (0.0, 0.30, 0.0))
    np.testing.assert_allclose(p, [0.30, 0.0, 0.0], atol=1e-12)


def test_yaw_of_minus_ninety_degrees(impl):
    p = impl.object_in_arm_frame((0.0, 0.0, -math.pi / 2), (0.0, 0.0, 0.0), (0.0, 0.30, 0.0))
    np.testing.assert_allclose(p, [-0.30, 0.0, 0.0], atol=1e-12)


def test_the_mount_offset_is_subtracted_in_the_base_frame_not_the_map(impl):
    """The catch: rotate first, then subtract the mount — it is bolted to the robot."""
    p = impl.object_in_arm_frame((0.0, 0.0, math.pi / 2), (0.050, 0.0, 0.0), (0.0, 0.30, 0.0))
    np.testing.assert_allclose(p, [0.25, 0.0, 0.0], atol=1e-12)


def test_a_docking_error_shows_up_as_a_position_error(impl):
    nominal = impl.object_in_arm_frame((1.20, 0.40, 0.0), (-0.04, 0, 0.07), (1.50, 0.40, 0.03))
    off = impl.object_in_arm_frame((1.25, 0.42, math.radians(8)), (-0.04, 0, 0.07),
                                   (1.50, 0.40, 0.03))
    err_mm = float(np.linalg.norm(off - nominal)) * 1000
    assert 30 < err_mm < 90, "a 'good' Nav2 dock is still an order of magnitude over 15.07"


# ---------------------------------------------------------------- the decision
def test_both_conditions_must_hold(impl):
    assert impl.should_use_arm(0.015, 0.060)
    assert not impl.should_use_arm(0.005, 0.060)
    assert not impl.should_use_arm(0.015, 0.010)
    assert not impl.should_use_arm(0.005, 0.010)


def test_a_negative_margin_is_never_acceptable(impl):
    assert not impl.should_use_arm(0.100, -0.001)


def test_the_thresholds_are_tunable(impl):
    assert impl.should_use_arm(0.006, 0.020, min_manipulability=0.005, min_margin_m=0.015)


def test_the_boundary_is_inclusive(impl):
    assert impl.should_use_arm(0.010, 0.030)
