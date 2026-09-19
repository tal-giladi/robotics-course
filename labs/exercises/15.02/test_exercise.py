"""Checker for 15.02 — gripper models and the selection logic.

Run: ``python course.py check 15.02`` (or ``--solution`` to see the reference pass).
Every expected number is computed by hand in the comment above it.
"""

from __future__ import annotations

import math

import pytest

G = 9.81


def so101_jaw(impl):
    """The course's conservative SO-101 jaw: 45 mm stroke, 4.7 N per pad."""
    f = impl.servo_jaw_force(1.0, 0.045, torque_limit=0.30, efficiency=0.7)
    return impl.JawGripper("SO-101 jaw", stroke_m=0.045, max_force_n=f, pad_mu=0.6,
                           pad_radius_m=0.006, finger_length_m=0.045)


def small_suction(impl):
    """A 20 mm cup on a small 12 V diaphragm pump (about 60 kPa of vacuum)."""
    return impl.SuctionGripper("20 mm cup, 60 kPa", cup_diameter_m=0.020, n_cups=1,
                               gauge_pressure_pa=60_000.0, efficiency=0.7)


def verdict_kind(text: str) -> str:
    """The tests pin the decision, not the wording: everything before the first colon, lowercased."""
    head = text.split(":", 1)[0].strip().lower()
    for kind in ("yes", "no", "risky", "works"):
        if head.startswith(kind):
            return kind
    raise AssertionError(f"verdict must start with yes/no/risky/works, got {text!r}")


# --- kgcm_to_nm / servo_jaw_force ---------------------------------------------------------------
def test_one_kgcm_is_0_0981_nm(impl):
    assert impl.kgcm_to_nm(1.0) == pytest.approx(0.0981, abs=1e-12)


def test_fifteen_kgcm_at_the_course_torque_limit(impl):
    # 15 kg*cm = 1.4715 N*m; 0.30 * 1.4715 * 0.7 / 0.045 = 0.309015 / 0.045 = 6.867 N
    f = impl.servo_jaw_force(impl.kgcm_to_nm(15.0), 0.045)
    assert f == pytest.approx(6.867, abs=1e-3), "see the worked example in lesson 15.02, Level 2"


def test_the_full_stall_column_is_three_times_the_course_limit(impl):
    nm = impl.kgcm_to_nm(20.0)
    limited = impl.servo_jaw_force(nm, 0.045, torque_limit=0.30)
    full = impl.servo_jaw_force(nm, 0.045, torque_limit=1.00)
    assert full == pytest.approx(limited / 0.30, rel=1e-12)


def test_a_longer_lever_gives_less_force(impl):
    nm = impl.kgcm_to_nm(20.0)
    assert impl.servo_jaw_force(nm, 0.090) == pytest.approx(impl.servo_jaw_force(nm, 0.045) / 2)


def test_a_zero_lever_is_rejected(impl):
    with pytest.raises(ValueError):
        impl.servo_jaw_force(1.0, 0.0)


# --- suction ------------------------------------------------------------------------------------
def test_suction_force_of_the_20mm_cup(impl):
    # A = pi * 0.010^2 = 3.14159e-4 m^2; 60000 * 3.14159e-4 * 0.7 = 13.1947 N
    assert impl.suction_force_n(60_000.0, 0.020, 0.7) == pytest.approx(13.1947, abs=1e-3)


def test_suction_force_scales_with_the_square_of_the_diameter(impl):
    small = impl.suction_force_n(60_000.0, 0.020, 0.7)
    big = impl.suction_force_n(60_000.0, 0.040, 0.7)
    assert big == pytest.approx(4.0 * small, rel=1e-12), "force goes as area, so as d^2"


def test_atmospheric_ceiling_on_a_20mm_cup(impl):
    # 101 kPa is the theoretical maximum: 101000 * 3.14159e-4 = 31.73 N
    assert impl.suction_force_n(101_000.0, 0.020, 1.0) == pytest.approx(31.730, abs=1e-2)


def test_cup_payload_and_the_shear_penalty(impl):
    cup = small_suction(impl)
    # 13.1947 / (2 * 10.81) = 0.61030 kg; sideways it is lip_mu = 0.5 of that
    assert cup.max_payload_kg() == pytest.approx(0.61030, abs=1e-4)
    assert cup.max_payload_kg(vertical_pull=False) == pytest.approx(0.30515, abs=1e-4)
    assert cup.shear_force_n() == pytest.approx(0.5 * cup.normal_force_n(), rel=1e-12)


def test_four_cups_lift_four_times_as_much(impl):
    one = impl.SuctionGripper("1", 0.020, 1, 60_000.0, 0.7)
    four = impl.SuctionGripper("4", 0.020, 4, 60_000.0, 0.7)
    assert four.normal_force_n() == pytest.approx(4.0 * one.normal_force_n(), rel=1e-12)


# --- JawGripper ---------------------------------------------------------------------------------
def test_fits_includes_the_clearance(impl):
    jaw = so101_jaw(impl)  # 45 mm stroke
    assert jaw.fits(0.030), "30 + 10 = 40 mm fits in a 45 mm stroke"
    assert jaw.fits(0.034), "34 + 10 = 44 mm still fits"
    assert not jaw.fits(0.036), "36 + 10 = 46 mm does not fit — the clearance is not optional"
    assert not jaw.fits(0.044), "a 44 mm object in a 45 mm jaw leaves no room to approach"


def test_fits_honours_a_custom_clearance(impl):
    jaw = so101_jaw(impl)
    assert jaw.fits(0.043, clearance_m=0.002)
    assert not jaw.fits(0.043, clearance_m=0.010)


def test_so101_payload_against_pad_friction(impl):
    jaw = so101_jaw(impl)
    # F = 0.30 * 1.0 * 0.7 / 0.045 = 4.6667 N per pad
    assert jaw.max_force_n == pytest.approx(4.6667, abs=1e-3)
    # 2 * 0.6 * 4.6667 / (2 * 10.81) = 5.6 / 21.62 = 0.25902 kg
    assert jaw.max_payload_kg(mu=0.6) == pytest.approx(0.25902, abs=1e-4)
    assert jaw.max_payload_kg(mu=0.3) == pytest.approx(0.12951, abs=1e-4)


def test_tripling_pad_friction_triples_the_payload(impl):
    jaw = so101_jaw(impl)
    assert jaw.max_payload_kg(mu=0.9) == pytest.approx(3.0 * jaw.max_payload_kg(mu=0.3), rel=1e-12)


def test_pad_mu_is_the_default(impl):
    jaw = so101_jaw(impl)
    assert jaw.max_payload_kg() == pytest.approx(jaw.max_payload_kg(mu=jaw.pad_mu), rel=1e-12)


# --- evaluate -----------------------------------------------------------------------------------
def test_evaluate_returns_exactly_three_verdicts(impl):
    obj = impl.TargetObject("wooden cube 30 mm", 0.030, 0.020, "textured", flat_face=True)
    v = impl.evaluate(obj, so101_jaw(impl), small_suction(impl))
    assert set(v) == {"parallel jaw", "suction", "compliant jaw"}


def test_a_small_textured_cube_is_easy_for_the_jaw(impl):
    obj = impl.TargetObject("wooden cube 30 mm", 0.030, 0.020, "textured", flat_face=True)
    v = impl.evaluate(obj, so101_jaw(impl), small_suction(impl))
    assert verdict_kind(v["parallel jaw"]) == "yes"
    assert verdict_kind(v["suction"]) == "yes"
    assert verdict_kind(v["compliant jaw"]) == "works"


def test_a_wide_object_is_ruled_out_by_stroke_for_both_jaw_types(impl):
    can = impl.TargetObject("full 330 ml can", 0.066, 0.345, "smooth-rigid", flat_face=True)
    v = impl.evaluate(can, so101_jaw(impl), small_suction(impl))
    assert verdict_kind(v["parallel jaw"]) == "no"
    assert verdict_kind(v["compliant jaw"]) == "no", "compliance does not extend the stroke"
    assert verdict_kind(v["suction"]) == "yes", "610 g of capacity against a 345 g can"


def test_porosity_is_checked_before_flatness(impl):
    """A porous object WITH a flat face still cannot hold a vacuum."""
    box = impl.TargetObject("cardboard box", 0.030, 0.060, "porous", flat_face=True)
    v = impl.evaluate(box, so101_jaw(impl), small_suction(impl))
    assert verdict_kind(v["suction"]) == "no"
    assert "porous" in v["suction"].lower(), "say WHY: the reason is porosity, not flatness"


def test_a_curved_object_defeats_suction(impl):
    pen = impl.TargetObject("marker pen", 0.017, 0.012, "smooth-rigid", flat_face=False)
    v = impl.evaluate(pen, so101_jaw(impl), small_suction(impl))
    assert verdict_kind(v["suction"]) == "no"
    assert verdict_kind(v["parallel jaw"]) == "yes"
    assert verdict_kind(v["compliant jaw"]) == "yes", "no flat face -> a compliant finger conforms"


def test_an_object_that_fits_but_is_too_heavy(impl):
    # 30 mm wide, smooth-rigid (mu 0.45), 1 kg: needs 2*1*10.81/(0.45*2) = 24.0 N per pad
    brick = impl.TargetObject("small steel block", 0.030, 1.000, "smooth-rigid", flat_face=True)
    v = impl.evaluate(brick, so101_jaw(impl), small_suction(impl))
    assert verdict_kind(v["parallel jaw"]) == "no"
    assert "fit" not in v["parallel jaw"].lower(), "it fits — the reason must be force, not stroke"


def test_a_fragile_object_on_a_hard_small_pad_is_risky(impl):
    hard = impl.JawGripper("hard pads", stroke_m=0.080, max_force_n=30.0, pad_mu=0.6,
                           pad_radius_m=0.001)
    egg = impl.TargetObject("egg", 0.045, 0.060, "smooth-rigid", flat_face=True, fragile=True)
    assert verdict_kind(impl.evaluate(egg, hard, small_suction(impl))["parallel jaw"]) == "risky"


def test_a_fragile_object_on_a_soft_wide_pad_is_fine(impl):
    soft = impl.JawGripper("soft pads", stroke_m=0.080, max_force_n=30.0, pad_mu=0.6,
                           pad_radius_m=0.008)
    egg = impl.TargetObject("egg", 0.045, 0.060, "smooth-rigid", flat_face=True, fragile=True)
    assert verdict_kind(impl.evaluate(egg, soft, small_suction(impl))["parallel jaw"]) == "yes"


def test_a_soft_object_prefers_a_compliant_finger(impl):
    sock = impl.TargetObject("sock", 0.030, 0.050, "soft", flat_face=False)
    v = impl.evaluate(sock, so101_jaw(impl), small_suction(impl))
    assert verdict_kind(v["compliant jaw"]) == "yes"


def test_a_heavy_flat_object_beyond_the_cup(impl):
    # 20 mm cup at 60 kPa holds 610 g; a 2 kg flat plate is beyond it
    plate = impl.TargetObject("steel plate", 0.200, 2.000, "smooth-rigid", flat_face=True)
    v = impl.evaluate(plate, so101_jaw(impl), small_suction(impl))
    assert verdict_kind(v["suction"]) == "no"
    assert "porous" not in v["suction"].lower(), "the reason is capacity, not porosity"
