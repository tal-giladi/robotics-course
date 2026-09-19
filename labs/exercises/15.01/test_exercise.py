"""Checker for 15.01 — the contact model: friction cones, grip force, torsion, force closure.

Run: ``python course.py check 15.01`` (or ``--solution`` to see the reference pass).
Every expected number in this file is computed by hand in the comment above it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

G = 9.81


def box_pinch(impl, half_width: float = 0.030, offset: float = 0.0, tilt: float = 0.0,
              mu: float = 0.6):
    """Two contacts on the vertical faces of a box, optionally slid along the face or tilted."""
    left = impl.Contact((-half_width, +offset), (math.cos(tilt), -math.sin(tilt)), mu)
    right = impl.Contact((+half_width, -offset), (-math.cos(tilt), math.sin(tilt)), mu)
    return [left, right]


# --- friction_cone_half_angle -----------------------------------------------------------------
def test_cone_half_angle_is_atan_mu(impl):
    # atan(1.0) = 45 deg exactly; atan(0.6) = 30.96375653 deg
    assert impl.friction_cone_half_angle(1.0) == pytest.approx(math.pi / 4)
    assert math.degrees(impl.friction_cone_half_angle(0.6)) == pytest.approx(30.96375653, abs=1e-6)


def test_a_frictionless_contact_has_a_zero_cone(impl):
    assert impl.friction_cone_half_angle(0.0) == pytest.approx(0.0)


def test_a_negative_mu_is_rejected(impl):
    with pytest.raises(ValueError):
        impl.friction_cone_half_angle(-0.1)


# --- required_normal_force / max_payload_kg ----------------------------------------------------
def test_grip_force_for_the_full_can(impl):
    # 2 * 0.345 * (9.81 + 2.0) / (0.6 * 2) = 8.1489 / 1.2 = 6.79075 N per pad
    f = impl.required_normal_force(0.345, 0.6, accel_m_s2=2.0, safety=2.0)
    assert f == pytest.approx(6.79075, abs=1e-5), "see the worked example in lesson 15.01, Level 2"


def test_grip_force_with_no_safety_factor_and_no_acceleration(impl):
    # 1 * 0.5 * 9.81 / (0.5 * 2) = 4.905 N
    assert impl.required_normal_force(0.5, 0.5) == pytest.approx(4.905, abs=1e-9)


def test_halving_mu_doubles_the_force(impl):
    a = impl.required_normal_force(0.2, 0.6, safety=2.0)
    b = impl.required_normal_force(0.2, 0.3, safety=2.0)
    assert b == pytest.approx(2.0 * a, rel=1e-12)


def test_three_fingers_need_two_thirds_of_the_force_of_two(impl):
    two = impl.required_normal_force(0.2, 0.6, n_contacts=2)
    three = impl.required_normal_force(0.2, 0.6, n_contacts=3)
    assert three == pytest.approx(two * 2 / 3, rel=1e-12)


def test_a_frictionless_pinch_is_rejected(impl):
    with pytest.raises(ValueError):
        impl.required_normal_force(0.1, 0.0)


def test_zero_contacts_is_rejected(impl):
    with pytest.raises(ValueError):
        impl.required_normal_force(0.1, 0.5, n_contacts=0)


def test_max_payload_inverts_required_force(impl):
    for mass, mu, a, s in ((0.345, 0.6, 2.0, 2.0), (0.12, 0.25, 0.0, 1.5), (1.0, 0.9, 1.0, 3.0)):
        f = impl.required_normal_force(mass, mu, accel_m_s2=a, safety=s)
        back = impl.max_payload_kg(f, mu, accel_m_s2=a, safety=s)
        assert back == pytest.approx(mass, rel=1e-12), "max_payload_kg must invert required_normal_force"


def test_max_payload_of_the_so101_jaw(impl):
    # 2 * 0.45 * 4.7 / (2 * 10.81) = 4.23 / 21.62 = 0.195652 kg  (lesson 15.01-E1d)
    assert impl.max_payload_kg(4.7, 0.45, accel_m_s2=1.0, safety=2.0) == pytest.approx(0.1956522, abs=1e-6)


# --- torsional_friction_moment -----------------------------------------------------------------
def test_a_point_contact_resists_no_torsion(impl):
    assert impl.torsional_friction_moment(100.0, 1.0, 0.0) == pytest.approx(0.0)


def test_torsional_moment_of_a_soft_pad(impl):
    # (2/3) * 0.8 * 6.0 * 0.008 = 0.0256 N*m
    assert impl.torsional_friction_moment(6.0, 0.8, 0.008) == pytest.approx(0.0256, abs=1e-9)


def test_torsional_moment_is_linear_in_radius(impl):
    a = impl.torsional_friction_moment(5.0, 0.7, 0.004)
    b = impl.torsional_friction_moment(5.0, 0.7, 0.012)
    assert b == pytest.approx(3.0 * a, rel=1e-12)


# --- Contact.force_is_feasible -----------------------------------------------------------------
def test_a_pure_normal_push_is_always_feasible(impl):
    c = impl.Contact((0.0, 0.0), (1.0, 0.0), 0.3)
    assert c.force_is_feasible((5.0, 0.0))


def test_a_pull_is_never_feasible(impl):
    c = impl.Contact((0.0, 0.0), (1.0, 0.0), 10.0)
    assert not c.force_is_feasible((-1.0, 0.0)), "a finger pushes; it cannot pull on the surface"


def test_the_cone_boundary_is_feasible_and_just_outside_is_not(impl):
    # mu = 0.5: f = (1, 0.5) is exactly on the boundary, (1, 0.51) is outside
    c = impl.Contact((0.0, 0.0), (1.0, 0.0), 0.5)
    assert c.force_is_feasible((1.0, 0.5))
    assert not c.force_is_feasible((1.0, 0.51))


def test_an_unnormalised_normal_is_handled(impl):
    c = impl.Contact((0.0, 0.0), (7.0, 0.0), 0.5)
    assert c.force_is_feasible((1.0, 0.5))
    assert not c.force_is_feasible((1.0, 0.6))


# --- is_antipodal ------------------------------------------------------------------------------
def test_aligned_flat_faces_are_antipodal(impl):
    assert impl.is_antipodal(*box_pinch(impl, mu=0.6))


def test_a_slippery_pinch_on_parallel_faces_is_still_antipodal(impl):
    # the grasp line is normal to both faces, so it is inside any cone however small
    assert impl.is_antipodal(*box_pinch(impl, mu=0.05))


def test_contacts_slid_far_along_the_face_are_not_antipodal(impl):
    # 15 mm offset on a 30 mm half-width: the line tilts 26.6 deg, inside the 31.0 deg cone
    assert impl.is_antipodal(*box_pinch(impl, offset=0.015, mu=0.6))
    # 30 mm offset: the line tilts 45 deg, outside it
    assert not impl.is_antipodal(*box_pinch(impl, offset=0.030, mu=0.6))


def test_a_taper_needs_mu_above_tan_of_the_taper(impl):
    # tan(40 deg) = 0.8391, so mu = 0.6 fails and mu = 1.0 passes
    assert not impl.is_antipodal(*box_pinch(impl, tilt=math.radians(40), mu=0.6))
    assert impl.is_antipodal(*box_pinch(impl, tilt=math.radians(40), mu=1.0))


def test_coincident_contacts_are_not_antipodal(impl):
    c = impl.Contact((0.0, 0.0), (1.0, 0.0), 0.6)
    assert not impl.is_antipodal(c, c)


# --- primitive_wrenches ------------------------------------------------------------------------
def test_primitive_wrenches_shape_and_unit_forces(impl):
    contacts = box_pinch(impl, mu=0.6)
    w = impl.primitive_wrenches(contacts)
    assert w.shape == (4, 3), "2 cone edges per contact, 3 wrench components"
    assert np.allclose(np.linalg.norm(w[:, :2], axis=1), 1.0), "the force part must be a unit vector"


def test_a_force_through_the_com_makes_no_torque(impl):
    # a single frictionless contact at (-0.03, 0) pushing along +x, com at the origin
    c = impl.Contact((-0.030, 0.0), (1.0, 0.0), 0.0)
    w = impl.primitive_wrenches([c], com=(0.0, 0.0))
    assert np.allclose(w[:, 2], 0.0)


def test_the_torque_row_is_scaled_by_the_length_scale(impl):
    # contact at (0, 0.02) pushing along +x: tau = cross2((0, 0.02), (1, 0)) = -0.02 N*m
    # with L = 0.05 the stored value is -0.4
    c = impl.Contact((0.0, 0.020), (1.0, 0.0), 0.0)
    w = impl.primitive_wrenches([c], com=(0.0, 0.0), length_scale_m=0.05)
    assert w[0, 2] == pytest.approx(-0.4, abs=1e-12)


# --- epsilon_quality ---------------------------------------------------------------------------
def test_epsilon_of_the_reference_pinch(impl):
    # the lesson's headline number: 2 contacts, 60 mm box, mu 0.6
    assert impl.epsilon_quality(box_pinch(impl, mu=0.6)) == pytest.approx(0.2529, abs=5e-4)


def test_epsilon_is_zero_without_force_closure(impl):
    assert impl.epsilon_quality(box_pinch(impl, offset=0.030, mu=0.6)) == 0.0
    assert impl.epsilon_quality(box_pinch(impl, tilt=math.radians(40), mu=0.6)) == 0.0


def test_epsilon_is_exactly_zero_not_merely_small(impl):
    eps = impl.epsilon_quality(box_pinch(impl, tilt=math.radians(40), mu=0.6))
    assert isinstance(eps, float) and eps == 0.0, "return exactly 0.0, never a small negative number"


def test_a_single_contact_has_no_force_closure(impl):
    assert impl.epsilon_quality([impl.Contact((0.0, 0.0), (1.0, 0.0), 0.6)]) == 0.0


def test_epsilon_is_monotone_in_friction(impl):
    values = [impl.epsilon_quality(impl.evenly_spaced_contacts(2, 0.030, mu))
              for mu in (0.1, 0.25, 0.4, 0.6, 0.8, 1.0)]
    assert values == sorted(values), "more friction can never make a grasp worse"
    assert values[0] == pytest.approx(0.0511, abs=5e-4)
    assert values[-1] == pytest.approx(0.3235, abs=5e-4)


def test_adding_a_finger_never_lowers_epsilon(impl):
    values = [impl.epsilon_quality(impl.evenly_spaced_contacts(n, 0.030, 0.6)) for n in (2, 3, 4, 5)]
    assert all(b >= a - 1e-12 for a, b in zip(values, values[1:])), "extra fingers cannot hurt"
    assert values[0] == pytest.approx(0.2529, abs=5e-4)
    # three cones already positively span the plane at mu = 0.6, so 4 and 5 add nothing
    assert values[1] == pytest.approx(values[2], abs=1e-6)
    assert values[2] == pytest.approx(values[3], abs=1e-6)


def test_epsilon_is_scale_free_in_force(impl):
    """Doubling the grip force cannot change epsilon — it is built from UNIT wrenches."""
    a = impl.epsilon_quality(box_pinch(impl, mu=0.6))
    b = impl.epsilon_quality(box_pinch(impl, mu=0.6))
    assert a == pytest.approx(b)


def test_misalignment_costs_most_of_the_quality(impl):
    aligned = impl.epsilon_quality(box_pinch(impl, mu=0.6))
    off = impl.epsilon_quality(box_pinch(impl, offset=0.015, mu=0.6))
    assert off == pytest.approx(0.0422, abs=5e-4)
    assert off < 0.2 * aligned, "15 mm of misalignment costs over 80% of the quality"


# --- has_force_closure -------------------------------------------------------------------------
def test_force_closure_agrees_with_epsilon(impl):
    cases = [box_pinch(impl, mu=0.6), box_pinch(impl, offset=0.030, mu=0.6),
             box_pinch(impl, tilt=math.radians(40), mu=0.6),
             box_pinch(impl, tilt=math.radians(40), mu=1.0),
             impl.evenly_spaced_contacts(3, 0.030, 0.6)]
    for contacts in cases:
        assert impl.has_force_closure(contacts) == (impl.epsilon_quality(contacts) > 1e-9)


def test_force_closure_implies_antipodal_for_two_contacts(impl):
    for offset in (0.0, 0.005, 0.015, 0.030, 0.045):
        contacts = box_pinch(impl, offset=offset, mu=0.6)
        if impl.has_force_closure(contacts):
            assert impl.is_antipodal(*contacts), (
                f"offset {offset} m: a planar 2-contact grasp with force closure must be antipodal")
