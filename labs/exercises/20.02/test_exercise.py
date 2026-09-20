"""Checker for 20.02 — wire, fuse, centre of gravity and tipping.

Run: ``python course.py check 20.02`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "20-final-robot" / "code",):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import power_budget_v2 as pb  # noqa: E402


# --- wire_drop_v ------------------------------------------------------------------------------
def test_drop_counts_the_return_conductor(impl):
    one_way = 0.02095
    assert impl.wire_drop_v(18, 1.0, 5.0) == pytest.approx(2 * one_way * 5.0)


def test_drop_is_linear_in_length_and_current(impl):
    assert impl.wire_drop_v(16, 0.5, 4.0) == pytest.approx(impl.wire_drop_v(16, 1.0, 2.0))
    assert impl.wire_drop_v(16, 0.0, 10.0) == 0.0


def test_thicker_wire_drops_less(impl):
    drops = [impl.wire_drop_v(awg, 0.5, 6.0) for awg in (22, 20, 18, 16, 14, 12)]
    assert drops == sorted(drops, reverse=True)


def test_the_course_branches_all_stay_under_one_percent(impl):
    for branch in pb.karmel_v2_branches():
        drop = impl.wire_drop_v(branch.awg, branch.length_m, branch.peak_a)
        assert drop == pytest.approx(branch.drop_v)
        assert drop / branch.source_v < 0.01


# --- fuse_rating_a ----------------------------------------------------------------------------
def test_fuse_is_above_the_working_current(impl):
    assert impl.fuse_rating_a(5.9, 8.0) == 7.5       # 1.25 * 5.9 = 7.375
    assert impl.fuse_rating_a(1.71, 8.0) == 3.0
    assert impl.fuse_rating_a(7.61, 13.0) == 10.0


def test_a_fuse_exactly_at_1_25_is_accepted(impl):
    assert impl.fuse_rating_a(4.0, 13.0) == 5.0      # 1.25 * 4.0 = 5.0 exactly


def test_a_fuse_the_wire_cannot_support_is_refused(impl):
    with pytest.raises(ValueError):
        impl.fuse_rating_a(12.0, 8.0)                # needs 15 A, 18 AWG carries 8 A
    with pytest.raises(ValueError):
        impl.fuse_rating_a(40.0, 100.0)              # above the biggest blade fuse


def test_it_agrees_with_the_course_sizing(impl):
    for branch in pb.karmel_v2_branches():
        ampacity = pb.AWG_ampacity(branch.awg)
        assert impl.fuse_rating_a(branch.continuous_a, ampacity) == pb.fuse_for(
            branch.continuous_a, branch.awg)


# --- centre_of_gravity ------------------------------------------------------------------------
def test_two_equal_masses_average(impl):
    assert impl.centre_of_gravity([(1.0, 0.0, 0.0, 0.0), (1.0, 0.2, 0.0, 0.1)]) == pytest.approx(
        (2.0, 0.1, 0.0, 0.05))


def test_a_heavy_part_dominates(impl):
    total, x, _, _ = impl.centre_of_gravity([(9.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)])
    assert (total, x) == pytest.approx((10.0, 0.1))


def test_empty_is_an_error_not_a_zero_division(impl):
    with pytest.raises(ValueError):
        impl.centre_of_gravity([])


def test_it_reproduces_karmels_centre_of_gravity(impl):
    parts = [(p.mass_kg, p.x_m, p.y_m, p.z_m) for p in pb.karmel_v2_parts()]
    total, x, y, z = impl.centre_of_gravity(parts)
    reference = pb.centre_of_gravity(pb.karmel_v2_parts())
    assert (total, x, y, z) == pytest.approx(
        (reference.mass_kg, reference.x_m, reference.y_m, reference.z_m))
    assert total == pytest.approx(2.66, abs=0.01)
    assert x == pytest.approx(0.021, abs=0.002), "the CG must sit AHEAD of the wheel axle"


# --- tipping_accel ----------------------------------------------------------------------------
def test_the_formula(impl):
    assert impl.tipping_accel(0.10, 0.10) == pytest.approx(9.81)
    assert impl.tipping_accel(0.021, 0.070) == pytest.approx(9.81 * 0.021 / 0.070)


def test_a_higher_centre_of_gravity_tips_sooner(impl):
    assert impl.tipping_accel(0.03, 0.20) < impl.tipping_accel(0.03, 0.05)


def test_a_centre_of_gravity_on_or_behind_the_axle_is_already_over(impl):
    assert impl.tipping_accel(0.0, 0.07) == 0.0
    assert impl.tipping_accel(-0.02, 0.07) == 0.0
    assert impl.tipping_accel(0.03, 0.0) == 0.0


def test_karmel_as_built_survives_its_own_acceleration_limit(impl):
    cg = pb.centre_of_gravity(pb.karmel_v2_parts())
    a_tip = impl.tipping_accel(cg.x_m, cg.z_m)
    assert a_tip == pytest.approx(pb.tipping_accel_m_s2(cg))
    assert a_tip > 2.0, "less than 2x the configured 1.0 m/s^2 acceleration limit is not enough margin"


def test_the_arm_on_the_rear_deck_tips_it(impl):
    _, _, parts = pb.whatif("arm-back")
    _, x, _, z = impl.centre_of_gravity([(p.mass_kg, p.x_m, p.y_m, p.z_m) for p in parts])
    assert x < 0.0 and impl.tipping_accel(x, z) == 0.0
