"""Module 21 shipped code: state machine, triggers, launch mechanics, budgets.

Runs without hardware (fake detectors, explicit time).

    py -m pytest 21-war-machine/code
"""

from __future__ import annotations

import math

import pytest

import projectile_range as pr
import releaser as rl
import trigger_pipeline as tp
import war_power_budget as wpb


# ==============================================================================================
#  Weapon state machine
# ==============================================================================================
def test_happy_path_load_arm_fire_confirm() -> None:
    w = tp.Weapon("cannon")
    assert w.state is tp.State.SAFE
    w.load(0.0)
    assert w.state is tp.State.LOADED
    w.arm(0.1, estop_out=True)
    assert w.can_fire
    assert w.fire(0.2)
    assert w.state is tp.State.FIRED
    assert w.confirm(0.3)
    assert w.state is tp.State.SAFE


def test_double_fire_guard() -> None:
    """FIRED -> SAFE, never back to ARMED: the second shot needs a fresh LOADED (21.01 rule 2)."""
    w = tp.Weapon("cannon")
    w.load(0.0)
    w.arm(0.1, True)
    assert w.fire(0.2)
    assert not w.fire(0.25)          # still FIRED: no second shot
    w.confirm(0.3)
    assert not w.fire(0.4)           # SAFE: no payload, no shot


def test_arm_requires_loaded_and_estop() -> None:
    w = tp.Weapon("cannon")
    with pytest.raises(ValueError):
        w.arm(0.0, True)             # not loaded
    w.load(0.1)
    with pytest.raises(ValueError):
        w.arm(0.2, estop_out=False)  # e-stop in: the bus has no power
    assert w.state is tp.State.LOADED


def test_estop_is_a_hard_reset_from_any_state() -> None:
    for state in (tp.State.SAFE, tp.State.LOADED, tp.State.ARMED, tp.State.FIRED, tp.State.JAMMED):
        w = tp.Weapon("cannon")
        w.state = state              # force every state, then...
        w.estop(1.0)
        assert w.state is tp.State.SAFE


def test_jam_path_and_reset() -> None:
    w = tp.Weapon("cannon", release_confirm_ms=2000.0)
    w.load(0.0)
    w.arm(0.1, True)
    w.fire(0.2)
    assert not w.timeout(1.5)        # 1.3 s: still waiting
    assert w.timeout(2.3)            # 2.1 s: the payload is somewhere
    assert w.state is tp.State.JAMMED
    w.reset_jam(3.0, payload_present=True)
    assert w.state is tp.State.LOADED   # payload still in the slot: it is loaded again


def test_reset_jam_empty_slot() -> None:
    w = tp.Weapon("cannon")
    w.load(0.0)
    w.arm(0.1, True)
    w.fire(0.2)
    w.timeout(3.0)
    w.reset_jam(4.0, payload_present=False)
    assert w.state is tp.State.SAFE


# ==============================================================================================
#  Button trigger
# ==============================================================================================
def test_button_edge_and_debounce() -> None:
    b = tp.ButtonTrigger(debounce_ms=20.0)
    assert not b.update(False, 0.0)
    assert b.update(True, 0.01)      # rising edge: fire
    assert not b.update(True, 0.015)  # still held: no machine gun
    assert not b.update(False, 0.02)
    assert not b.update(True, 0.029)  # 19 ms after the last edge: bounced out by the debounce
    assert not b.update(False, 0.035)
    assert b.update(True, 0.06)       # 50 ms after the last edge: clean press


# ==============================================================================================
#  RC trigger
# ==============================================================================================
def test_rc_relink_baseline_and_edge() -> None:
    r = tp.RcTrigger(deadman_ms=300.0)
    assert not r.update(1900, 0.0)   # first frame: relink baseline, no fire
    assert not r.update(1000, 0.03)
    assert r.update(1900, 0.06)      # rising edge on a live link: fire
    assert not r.update(1900, 0.09)  # held: one fire per press
    assert not r.update(1000, 0.12)  # released


def test_rc_deadman_eats_the_held_switch() -> None:
    """Transmitter died with the switch held; it restarts: no fire on the relink frame."""
    r = tp.RcTrigger(deadman_ms=300.0)
    r.update(1900, 0.0)              # baseline (switch happens to be on)
    r.update(1000, 0.03)
    # ... 0.5 s of silence (dead) ...
    assert not r.update(1900, 0.53)  # relink: state resets, no fire
    assert not r.update(1000, 0.56)  # operator releases
    assert r.update(1900, 0.59)      # fresh press: fire


def test_rc_link_alive() -> None:
    r = tp.RcTrigger(deadman_ms=300.0)
    r.update(1000, 0.0)
    assert r.link_alive(0.2)
    assert not r.link_alive(0.4)


# ==============================================================================================
#  Vision trigger
# ==============================================================================================
def _det(conf: float, dist: float) -> tp.Detection:
    return tp.Detection("aruco", conf, dist)


def test_vision_dwell_requires_consecutive_frames() -> None:
    v = tp.VisionTrigger(dwell_frames=5, confidence_min=0.5, d_min_m=1.0, d_max_m=5.0)
    for i in range(4):
        assert not v.update([_det(0.9, 2.0)], i * 0.033)
    assert v.update([_det(0.9, 2.0)], 4 * 0.033)   # 5th consecutive good frame: fire
    # a miss resets the streak
    v2 = tp.VisionTrigger(dwell_frames=5)
    for i in range(3):
        v2.update([_det(0.9, 2.0)], i * 0.033)
    v2.update([], 3 * 0.033)                        # one bad frame
    for i in range(4):
        assert not v2.update([_det(0.9, 2.0)], (4 + i) * 0.033)
    assert v2.update([_det(0.9, 2.0)], 8 * 0.033)


def test_vision_confidence_and_distance_gate() -> None:
    v = tp.VisionTrigger(dwell_frames=1, confidence_min=0.5, d_min_m=1.0, d_max_m=5.0, cooldown_s=0.0)
    assert not v.update([_det(0.4, 2.0)], 0.0)      # low confidence
    assert not v.update([_det(0.9, 0.5)], 0.1)      # too close
    assert not v.update([_det(0.9, 6.0)], 0.2)      # too far
    assert not v.update([tp.Detection("ball", 0.9, 2.0)], 0.3)  # wrong label
    assert v.update([_det(0.9, 2.0)], 0.4)          # everything in order


def test_vision_cooldown() -> None:
    v = tp.VisionTrigger(dwell_frames=1, cooldown_s=2.0)
    assert v.update([_det(0.9, 2.0)], 0.0)
    assert not v.update([_det(0.9, 2.0)], 0.5)      # still cooling down
    assert v.update([_det(0.9, 2.0)], 2.5)


# ==============================================================================================
#  Sound trigger
# ==============================================================================================
def test_sound_single_clap_does_not_fire() -> None:
    s = tp.SoundTrigger()
    seq = [(0.00, 3), (0.02, 18), (0.04, 2), (1.0, 2)]
    assert not any(s.update(env, t) for t, env in seq)


def test_sound_double_clap_fires() -> None:
    s = tp.SoundTrigger()
    seq = [(0.00, 3), (0.02, 18), (0.04, 2), (0.45, 3), (0.47, 19), (0.49, 2)]
    fired = [t for t, env in seq if s.update(env, t)]
    assert fired == [0.49]


def test_sound_claps_too_far_apart_are_noise() -> None:
    s = tp.SoundTrigger(max_gap_ms=900.0)
    seq = [(0.00, 3), (0.02, 18), (0.04, 2), (2.0, 3), (2.02, 19), (2.04, 2)]
    assert not any(s.update(env, t) for t, env in seq)


def test_sound_hysteresis_ignores_threshold_oscillation() -> None:
    """Noise oscillating around ONE threshold must not register claps (21.07 Level 2)."""
    s = tp.SoundTrigger(high_db=15.0, low_db=8.0)
    t = 0.0
    fired = False
    for i in range(200):
        env = 15.0 + (6.0 if i % 2 else -6.0)       # 9..21 dB, never below low_db
        fired |= s.update(env, t)
        t += 0.002
    assert not fired


# ==============================================================================================
#  Fire pipeline
# ==============================================================================================
def test_pipeline_blocks_fire_when_estop_in() -> None:
    p = tp.FirePipeline(tp.Weapon("cannon"))
    p.weapon.load(0.0)
    p.weapon.arm(0.1, True)
    p.set_estop(False, 0.2)
    assert not p.on_trigger(True, 0.3)
    assert p.weapon.state is tp.State.SAFE
    assert p.shots == 0


def test_pipeline_counts_and_confirms() -> None:
    p = tp.FirePipeline(tp.Weapon("cannon"))
    p.weapon.load(0.0)
    p.weapon.arm(0.1, True)
    assert p.on_trigger(True, 0.2)
    assert p.step_release(0.4, True) is tp.State.SAFE
    assert p.shots == 1
    assert p.step_release(0.5) is tp.State.SAFE     # nothing left to confirm


# ==============================================================================================
#  Releaser sizing
# ==============================================================================================
def test_flap_sizing() -> None:
    f = rl.flap_edge_force_n(0.15)
    assert f == pytest.approx(0.15 * 9.81 * 3.0)
    t = rl.flap_torque_n_m(f)
    assert t == pytest.approx(f * 0.08)
    assert rl.MG996R_TORQUE_N_M / t > 2.0            # the MG996R has margin
    assert rl.SG90_TORQUE_N_M / t < 1.0              # the SG90 does not


def test_solenoid_v_squared_law() -> None:
    assert rl.solenoid_force_n(42.0, 12.0) == pytest.approx(42.0)
    assert rl.solenoid_force_n(42.0, 9.9) == pytest.approx(42.0 * (9.9 / 12.0) ** 2)
    assert rl.solenoid_force_n(42.0, 9.9) == pytest.approx(28.6, abs=0.05)


def test_pin_net_force_clears_the_requirement_at_empty_pack() -> None:
    for rated in (42.0, 85.0):
        assert rl.pin_net_force_n(rated, 9.0) > 0.0


def test_pulse_clamped() -> None:
    assert rl.solenoid_pulse_ms() == pytest.approx(125.0)       # 75 ms travel + 50 margin
    assert rl.solenoid_pulse_ms(stroke_m=0.002) == pytest.approx(100.0)   # clamped up
    assert rl.solenoid_pulse_ms(stroke_m=0.040) == pytest.approx(250.0)   # in range
    assert rl.solenoid_pulse_ms(stroke_m=0.100) == pytest.approx(300.0)   # clamped down


# ==============================================================================================
#  Launch mechanics
# ==============================================================================================
def test_elastic_energy_and_muzzle_velocity() -> None:
    e = pr.elastic_energy_j(120.0, 0.12)
    assert e == pytest.approx(0.864)
    v = pr.muzzle_velocity_m_s(0.864 * 0.7, 0.004)
    assert v == pytest.approx(math.sqrt(2 * 0.6048 / 0.004))
    assert 15.0 < v < 20.0


def test_cannon_work_energy() -> None:
    w, e = pr.cannon()
    assert w == pytest.approx(12.0)
    assert e == pytest.approx(6.0)
    v = pr.muzzle_velocity_m_s(e, 0.05)
    assert v == pytest.approx(15.49, abs=0.05)


def test_range_model() -> None:
    assert pr.ideal_range_m(10.0, 45.0) == pytest.approx(100.0 / 9.81)
    assert pr.range_m(10.0, 0.3) == pytest.approx(100.0 / 9.81 * 0.3)
    # 45 degrees maximises the ideal range
    assert pr.ideal_range_m(10.0, 45.0) > pr.ideal_range_m(10.0, 30.0)


def test_recoil() -> None:
    assert pr.recoil_m_s(15.5, 0.05, 5.51) == pytest.approx(0.141, abs=0.001)
    assert pr.recoil_m_s(2.1, 0.15, 6.46) < 0.1


def test_arm_throw_is_a_lober() -> None:
    i, e, v = pr.arm_throw(0.15, 0.30, 0.003, 0.15)
    assert i == pytest.approx(0.0165)
    assert e < 1.0                                   # the honest number (21.04 Level 4)
    assert v == pytest.approx(math.radians(60.0) / 0.15 * 0.30)


# ==============================================================================================
#  Budgets (war_power_budget)
# ==============================================================================================
def test_cg_shift_matches_the_model() -> None:
    m, x, z = wpb.cg_shift(2.66, 0.021, 0.070, [(1.0, 0.090, 0.055)])
    assert m == pytest.approx(3.66)
    assert x * 1000 == pytest.approx(39.85, abs=0.05)
    assert z * 1000 == pytest.approx(65.9, abs=0.05)


def test_packages_compose() -> None:
    base = wpb.centre_of_gravity(wpb.base_parts()).mass_kg
    for pkg in ("scout", "assault", "siege"):
        assert wpb.centre_of_gravity(wpb.parts_for(pkg)).mass_kg > base


def test_spec_delta_has_ten_rows() -> None:
    rows = wpb.spec_delta("assault")
    assert len(rows) == 10
    assert rows[0][0] == "mass"
    assert rows[0][1] == "2.66 kg"
    assert "5.51 kg" in rows[0][2]


def test_voltage_sag_model() -> None:
    r = wpb.branch_resistance_ohm(18, 0.6)
    assert r == pytest.approx(0.039, abs=0.001)
    r_total = wpb.PACK_R_INT_OHM + r
    v_full = wpb.voltage_at_load(12.6, 15.0, r_total)
    v_empty = wpb.voltage_at_load(9.9, 15.0, r_total)
    assert v_full == pytest.approx(12.6 - 15.0 * r_total)
    assert v_empty < v_full
    assert wpb.solenoid_force_fraction(v_empty) < wpb.solenoid_force_fraction(v_full)
    assert wpb.solenoid_force_fraction(v_empty) < 0.6          # the empty-pack case is the design case


def test_firing_cost_is_small() -> None:
    plan = wpb.firing_cost({"cannon shot (120 N x 0.10 m)": 100})
    assert plan["pack_fraction_pct"] < 2.0
