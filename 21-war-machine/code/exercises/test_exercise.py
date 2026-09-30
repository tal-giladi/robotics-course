"""Exercise checkers for module 21.

Run:

    py -m pytest 21-war-machine/code/exercises

Unfilled stubs (NotImplementedError) are skipped, so a fresh clone is green. Each test
cross-checks the student's function against the shipped reference implementation or against
the numbers in the lesson text.
"""

from __future__ import annotations

import math

import pytest

import student
import trigger_pipeline as tp
import projectile_range as pr
import releaser as rl
import war_power_budget as wpb


def _filled(fn, *args) -> bool:
    try:
        fn(*args)
        return True
    except NotImplementedError:
        return False
    except Exception:  # noqa: BLE001 — a filled function raising something else is filled
        return True


# ==============================================================================================
#  21.01-E2
# ==============================================================================================
def test_2101_e2_cg_shift_matches_reference() -> None:
    if not _filled(student.cg_shift, 1.0, 0.0, 0.0, []):
        pytest.skip("21.01-E2: fill in cg_shift")
    cases = [
        ((2.66, 0.021, 0.070), [(1.0, 0.090, 0.055)]),
        ((2.66, 0.021, 0.070), [(0.5, 0.05, 0.05), (0.3, 0.10, 0.04)]),
        ((4.16, 0.038, 0.064), [(2.85, 0.075, 0.058)]),
    ]
    for (bm, bx, bz), added in cases:
        ref = wpb.cg_shift(bm, bx, bz, added)
        got = student.cg_shift(bm, bx, bz, added)
        assert got[0] == pytest.approx(ref[0], abs=1e-6)
        assert got[1] == pytest.approx(ref[1], abs=1e-6)
        assert got[2] == pytest.approx(ref[2], abs=1e-6)


def test_2101_e2_spec_delta_shape() -> None:
    if not _filled(student.spec_delta):
        pytest.skip("21.01-E2: fill in spec_delta")
    rows = student.spec_delta("assault")
    assert len(rows) == 10, "the delta has exactly ten rows"
    expected_labels = ["mass", "centre of gravity", "tipping (rear / nose)",
                       "switched rail peak", "runtime", "LiDAR plane", "camera cone",
                       "e-stop removes", "software states", "power hardware"]
    assert [r[0] for r in rows] == expected_labels
    assert str(rows[0][1]).startswith("2.66")


# ==============================================================================================
#  21.03-E2
# ==============================================================================================
@pytest.mark.parametrize("stroke_mm", [2.0, 15.0, 40.0, 100.0])
def test_2103_e2_pulse_matches_reference(stroke_mm: float) -> None:
    if not _filled(student.solenoid_pulse_ms, 15.0):
        pytest.skip("21.03-E2: fill in solenoid_pulse_ms")
    ref = rl.solenoid_pulse_ms(stroke_m=stroke_mm / 1000.0)
    assert student.solenoid_pulse_ms(stroke_mm) == pytest.approx(ref)


# ==============================================================================================
#  21.04-E2
# ==============================================================================================
def test_2104_e2_cannon_muzzle_matches_reference() -> None:
    if not _filled(student.cannon_muzzle_velocity_m_s, 120.0, 0.10, 0.05):
        pytest.skip("21.04-E2: fill in cannon_muzzle_velocity_m_s")
    got = student.cannon_muzzle_velocity_m_s(120.0, 0.10, 0.05, 0.5)
    assert got == pytest.approx(pr.muzzle_velocity_m_s(pr.cannon()[1], 0.05), rel=1e-9)
    # the lesson number: ~15.5 m/s for the 4 cm bouncy ball
    assert 15.0 < got < 16.0


def test_2104_e2_range_matches_reference() -> None:
    if not _filled(student.projectile_range_m, 15.5, 0.45):
        pytest.skip("21.04-E2: fill in projectile_range_m")
    got = student.projectile_range_m(15.5, 0.45)
    assert got == pytest.approx(pr.range_m(15.5, 0.45), rel=1e-9)
    assert 10.0 < got < 12.0


# ==============================================================================================
#  21.05-E2
# ==============================================================================================
def _reference_fire_times(frames, threshold=1500, deadman_ms=300.0) -> list[float]:
    r = tp.RcTrigger(fire_threshold_us=threshold, deadman_ms=deadman_ms)
    return [t for t, p in frames if r.update(p, t)]


@pytest.mark.parametrize(
    "frames,threshold,deadman",
    [
        # normal press
        ([(0.0, 1000), (0.03, 1900), (0.06, 1900), (0.09, 1000), (0.12, 1900)], 1500, 300.0),
        # dead-man: held switch across the gap must not fire on the relink
        ([(0.0, 1900), (0.03, 1000), (0.53, 1900), (0.56, 1000), (0.59, 1900)], 1500, 300.0),
        # unused channels (None) do not reset the link state
        ([(0.0, None), (0.03, 1900), (0.06, None), (0.09, 1000), (0.12, 1900)], 1500, 300.0),
    ],
)
def test_2105_e2_ppm_decode_matches_reference(frames, threshold, deadman) -> None:
    if not _filled(student.decode_ppm_fire_edges, frames):
        pytest.skip("21.05-E2: fill in decode_ppm_fire_edges")
    got = student.decode_ppm_fire_edges(frames, threshold, deadman)
    assert got == pytest.approx(_reference_fire_times(frames, threshold, deadman))


# ==============================================================================================
#  21.06-E2
# ==============================================================================================
def test_2106_e2_fire_gate_matches_reference() -> None:
    if not _filled(student.vision_fire_frames, [[]], 3):
        pytest.skip("21.06-E2: fill in vision_fire_frames")
    # 12 good frames, 1 bad, 16 good: a fire at the end of every completed dwell streak
    good = [("aruco", 0.8, 2.5)]
    frames = (good * 12) + [[]] + (good * 16)
    ref = tp.VisionTrigger(dwell_frames=5, confidence_min=0.5, d_min_m=1.0, d_max_m=5.0,
                           cooldown_s=0.0)
    ref_fires = [i for i in range(len(frames)) if ref.update(frames[i], i * 0.033)]
    got = student.vision_fire_frames(frames, dwell=5, confidence_min=0.5, d_min_m=1.0, d_max_m=5.0)
    assert got == ref_fires
    assert got == [4, 8, 17, 22]


def test_2106_e2_confidence_and_distance() -> None:
    if not _filled(student.vision_fire_frames, [[]], 3):
        pytest.skip("21.06-E2: fill in vision_fire_frames")
    low_conf = [[("aruco", 0.4, 2.0)]] * 10
    too_far = [[("aruco", 0.9, 6.0)]] * 10
    wrong_label = [[("ball", 0.9, 2.0)]] * 10
    assert student.vision_fire_frames(low_conf, dwell=3) == []
    assert student.vision_fire_frames(too_far, dwell=3) == []
    assert student.vision_fire_frames(wrong_label, dwell=3) == []


# ==============================================================================================
#  21.07-E2
# ==============================================================================================
def test_2107_e2_single_burst_is_one_clap() -> None:
    if not _filled(student.clap_times, [(0.0, 3.0)]):
        pytest.skip("21.07-E2: fill in clap_times")
    env = [(0.00, 3.0), (0.02, 18.0), (0.04, 2.0), (1.0, 2.0)]
    assert student.clap_times(env) == pytest.approx([0.04])


def test_2107_e2_double_clap_both_registered() -> None:
    if not _filled(student.clap_times, [(0.0, 3.0)]):
        pytest.skip("21.07-E2: fill in clap_times")
    env = [(0.00, 3.0), (0.02, 18.0), (0.04, 2.0), (0.45, 3.0), (0.47, 19.0), (0.49, 2.0)]
    assert student.clap_times(env) == pytest.approx([0.04, 0.49])


def test_2107_e2_oscillation_is_not_a_clap() -> None:
    if not _filled(student.clap_times, [(0.0, 3.0)]):
        pytest.skip("21.07-E2: fill in clap_times")
    env = [((t), 15.0 + (6.0 if i % 2 else -6.0)) for i, t in enumerate(i * 0.002 for i in range(200))]
    assert student.clap_times(env) == []


def test_2107_e2_rapid_repeats_merge() -> None:
    if not _filled(student.clap_times, [(0.0, 3.0)]):
        pytest.skip("21.07-E2: fill in clap_times")
    env = [(0.00, 3.0), (0.01, 18.0), (0.02, 2.0),
           (0.05, 3.0), (0.06, 18.0), (0.07, 2.0)]   # 2nd burst 50 ms after the first clap
    claps = student.clap_times(env)
    assert len(claps) == 1 and claps[0] == pytest.approx(0.02)


# ==============================================================================================
#  21.08-E2
# ==============================================================================================
def test_2108_e2_thevenin_matches_reference() -> None:
    if not _filled(student.thevenin_load_v, 12.6, 0.06, 15.0):
        pytest.skip("21.08-E2: fill in thevenin_load_v")
    assert student.thevenin_load_v(12.6, 0.06, 15.0) == pytest.approx(wpb.voltage_at_load(12.6, 15.0, 0.06))
    assert student.thevenin_load_v(9.9, 0.06, 15.0) < 9.9


def test_2108_e2_wire_resistance_matches_reference() -> None:
    if not _filled(student.wire_resistance_ohm, 18, 1.0):
        pytest.skip("21.08-E2: fill in wire_resistance_ohm")
    for awg in (18, 16, 14):
        assert student.wire_resistance_ohm(awg, 0.6) == pytest.approx(
            wpb.wire_resistance_ohm(awg, 0.6), rel=1e-3)
    assert student.wire_resistance_ohm(18, 0.6) < student.wire_resistance_ohm(14, 0.6)
