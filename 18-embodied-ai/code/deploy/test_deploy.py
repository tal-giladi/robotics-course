"""Tests for 18-embodied-ai/code/deploy (lessons 18.06-18.10). No robot, no GPU.

  py -m pytest 18-embodied-ai/code/deploy -q        (from the repository root)
"""
from __future__ import annotations

import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import latency_sim as ls  # noqa: E402
import safety_wrapper as sw  # noqa: E402
import success_stats as ss  # noqa: E402


# ---------------------------------------------------------------- success_stats
def test_wilson_matches_published_values():
    # 8/10 -> [0.490, 0.943] is the textbook Wilson 95% interval
    iv = ss.wilson_interval(8, 10)
    assert iv.low == pytest.approx(0.490, abs=0.001)
    assert iv.high == pytest.approx(0.943, abs=0.001)


def test_wilson_never_degenerate_at_extremes():
    assert ss.wilson_interval(10, 10).low == pytest.approx(0.722, abs=0.001)
    assert ss.wald_interval(10, 10).width == 0.0          # the bug Wilson fixes
    assert ss.wilson_interval(0, 10).high > 0.25


def test_clopper_pearson_is_wider_than_wilson():
    for k, n in [(8, 10), (18, 20), (45, 50)]:
        assert ss.clopper_pearson_interval(k, n).width > ss.wilson_interval(k, n).width
    assert ss.clopper_pearson_interval(8, 10).low == pytest.approx(0.444, abs=0.001)


def test_coverage_wilson_beats_wald_at_small_n():
    assert ss.coverage(ss.wald_interval, 10, 0.9) < 0.7
    assert ss.coverage(ss.wilson_interval, 10, 0.9) > 0.9


def test_trials_for_half_width():
    assert ss.trials_for_half_width(0.8, 0.10) == 60
    assert ss.trials_for_half_width(0.5, 0.10) == 93


def test_fisher_exact_and_newcombe():
    assert ss.fisher_exact_two_sided(17, 20, 11, 20) == pytest.approx(0.082, abs=0.001)
    d = ss.newcombe_difference(17, 20, 11, 20)
    assert d.low > 0 and d.high < 0.6
    assert ss.fisher_exact_two_sided(10, 20, 10, 20) == pytest.approx(1.0)


def test_fixed_design_error_rates():
    fd = ss.fixed_design(0.5, 0.8)
    assert fd.alpha <= 0.05 and fd.power >= 0.8
    assert (fd.n, fd.k_min) == (18, 13)


def test_sprt_decides_and_controls_errors():
    t = ss.SPRT(0.5, 0.8)
    decisions = [t.update(c == "1") for c in "1101111011111"]
    assert decisions[-1] is not None and decisions[-1].startswith("accept H1")
    asn_bad, acc_bad = ss.simulate_sprt(0.5, 0.5, 0.8, runs=4000)
    asn_good, acc_good = ss.simulate_sprt(0.8, 0.5, 0.8, runs=4000)
    assert acc_bad < 0.07 and acc_good > 0.75
    assert asn_bad < 18 and asn_good < 18     # on average fewer trials than the fixed design


# ---------------------------------------------------------------- latency_sim
def test_fast_local_inference_async_has_no_idle_ticks():
    r = ls.simulate(ls.Scenario(mode="async", infer_ms=20, rtt_ms=0.5, jitter_ms=0, obs_kb=0))
    assert r.idle_pct < 1.0
    assert r.rms_err_mm < 15


def test_sync_blocks_the_loop_async_does_not():
    sc = ls.Scenario(infer_ms=150, rtt_ms=40, jitter_ms=5, obs_kb=100, uplink_mbps=40)
    sync = ls.simulate(replace(sc, mode="sync"))
    asyn = ls.simulate(replace(sc, mode="async"))
    assert sync.idle_pct > 3 * max(asyn.idle_pct, 0.5)
    assert ls.mean_reaction_s(replace(sc, mode="async")) < ls.mean_reaction_s(replace(sc, mode="sync"))


def test_latency_longer_than_chunk_means_every_action_is_dropped():
    # 2.5 s latency, 50 actions at 30 Hz = 1.67 s: every returned action is already in the past
    r = ls.simulate(ls.Scenario(mode="async", infer_ms=2500, rtt_ms=0, jitter_ms=0, obs_kb=0))
    assert r.idle_pct == pytest.approx(100.0)


def test_raw_images_over_wifi_add_upload_latency():
    rng = __import__("random").Random(0)
    raw = ls.request_latency_s(ls.Scenario(infer_ms=0, rtt_ms=0, jitter_ms=0, obs_kb=1800, uplink_mbps=40), rng)
    assert raw == pytest.approx(0.36, abs=1e-6)


def test_higher_threshold_means_fresher_actions_but_more_requests():
    sc = ls.Scenario(mode="async", infer_ms=60, rtt_ms=20, jitter_ms=5)
    low = ls.simulate(replace(sc, threshold=0.3))
    high = ls.simulate(replace(sc, threshold=0.8))
    assert high.stale_p(0.5) < low.stale_p(0.5)
    assert high.requests > low.requests


# ---------------------------------------------------------------- safety_wrapper
def _q(*deg_and_grip: float) -> np.ndarray:
    *deg, grip = deg_and_grip
    return np.array([math.radians(d) for d in deg] + [grip])


@pytest.fixture()
def filt() -> sw.SafetyFilter:
    return sw.SafetyFilter(sw.so101_default_config())


HOME = _q(0, 60, -100, 40, 0, 0.2)


def test_legal_small_step_passes_unchanged(filt):
    target = HOME + np.array([0.02, 0, 0, 0, 0, 0])
    res = filt.filter(HOME, target, 1 / 30)
    assert res.events == []
    np.testing.assert_allclose(res.command, target)


def test_nan_holds_position(filt):
    target = HOME.copy()
    target[2] = float("nan")
    res = filt.filter(HOME, target, 1 / 30)
    assert "non_finite_action" in res.events
    np.testing.assert_allclose(res.command, HOME)


def test_velocity_is_clipped_to_limit(filt):
    target = HOME + np.array([math.radians(40), 0, 0, 0, 0, 0])
    res = filt.filter(HOME, target, 1 / 30)
    assert "velocity_limit" in res.events
    assert res.command[0] - HOME[0] == pytest.approx(math.radians(90) / 30)


def test_joint_limit_clip(filt):
    start = _q(0, 10, -60, 0, 0, 0.2)     # low pose so the workspace box does not interfere
    target = start.copy()
    target[2] = math.radians(30)          # elbow upper limit is 0 deg
    q = start.copy()
    for _ in range(200):
        q = filt.filter(q, target, 1 / 30).command
        filt.consecutive = 0              # isolate this rule from the trip budget
    assert q[2] == pytest.approx(0.0, abs=1e-9)


def test_workspace_floor_is_never_crossed(filt):
    cfg = filt.cfg
    goal = _q(0, 0, -90, 0, 0, 0.2)       # legal joints, fingertip ~0.1 m below the table
    q = HOME.copy()
    zs = []
    for _ in range(120):
        step = np.clip(goal - q, -math.radians(2), math.radians(2))
        res = filt.filter(q, q + step, 1 / 30)
        q = res.command
        zs.append(sw.approx_fk(q)[2])
    assert min(zs) >= cfg.box.min_xyz[2] - 1e-6
    assert min(zs) == pytest.approx(cfg.box.min_xyz[2], abs=0.003)   # it did reach the boundary
    assert filt.tripped and filt.trip_reason == "too_many_violations"


def test_trip_latches_until_reset(filt):
    bad = np.full(6, float("inf"))
    for _ in range(filt.cfg.max_consecutive_violations):
        filt.filter(HOME, bad, 1 / 30)
    assert filt.tripped
    res = filt.filter(HOME, HOME + 0.01, 1 / 30)       # a good action does not un-trip it
    assert res.tripped
    np.testing.assert_allclose(res.command, HOME)
    filt.reset()
    assert not filt.filter(HOME, HOME, 1 / 30).tripped


def test_overload_trips_immediately(filt):
    res = filt.filter(HOME, HOME, 1 / 30, load=np.array([0.1, 0.95, 0.1, 0.1, 0.1, 0.1]))
    assert res.tripped and filt.trip_reason == "overload"


def test_stale_observation_repeats_last_command(filt):
    first = filt.filter(HOME, HOME + np.array([0.01, 0, 0, 0, 0, 0]), 1 / 30).command
    res = filt.filter(HOME, HOME + 0.3, 1 / 30, obs_age_s=0.5)
    assert "stale_observation" in res.events
    np.testing.assert_allclose(res.command, first)


def test_lerobot_roundtrip_and_safe_robot_wrapper():
    action = {"shoulder_pan.pos": 10.0, "shoulder_lift.pos": 60.0, "elbow_flex.pos": -100.0,
              "wrist_flex.pos": 40.0, "wrist_roll.pos": 0.0, "gripper.pos": 20.0}
    back = sw.to_lerobot_action(sw.from_lerobot_action(action))
    for k, v in action.items():
        assert back[k] == pytest.approx(v)
    arm = sw.FakeArm()
    robot = sw.SafeRobot(arm, sw.SafetyFilter(sw.so101_default_config()))
    obs = robot.get_observation()
    res = robot.send_action({**obs, "shoulder_pan.pos": obs["shoulder_pan.pos"] + 45.0}, obs)
    assert "velocity_limit" in res.events
    assert arm.sent[-1]["shoulder_pan.pos"] == pytest.approx(obs["shoulder_pan.pos"] + 3.0)


def test_demo_runs(capsys):
    sw.demo()
    out = capsys.readouterr().out
    assert "workspace=" in out and "too_many_violations" in out


# ---------------------------------------------------------------- action_representations
def test_action_representations_numbers(capsys):
    import action_representations as ar
    a = ar.demo_chunk()
    assert np.abs(ar.from_bins(ar.to_bins(a)) - a).max() <= (ar.HIGH - ar.LOW) / ar.BINS / 2 + 1e-12
    np.testing.assert_allclose(ar.idct_ii(ar.dct_ii(a)), a, atol=1e-12)
    ar.main()
    out = capsys.readouterr().out
    assert "1 Euler steps:   0.0% of samples" in out
    assert "10 Euler steps: 100.0% of samples" in out


# ---------------------------------------------------------------- cross_embodiment
def test_cross_embodiment_padding_and_weights(capsys):
    import cross_embodiment as ce
    padded, mask = ce.pad_with_mask(np.ones((5, 6)))
    assert padded.shape == (5, ce.MAX_ACTION_DIM) and mask.sum() == 6
    with pytest.raises(ValueError):
        ce.pad_with_mask(np.ones((2, 40)))
    w = ce.sampling_weights({"a": 100, "b": 1}, 0.0)
    assert w["a"] == pytest.approx(0.5)
    w1 = ce.sampling_weights({"a": 100, "b": 1}, 1.0)
    assert w1["b"] == pytest.approx(1 / 101)
    ce.main()
    out = capsys.readouterr().out
    assert "SO-101 is 98% of the summed loss" in out and "normalized: 34%" in out


def test_peeking_inflates_false_claims():
    once, peek = ss.peeking_false_claim_rate(0.5, 0.5, runs=2000)
    assert once < 0.06
    assert peek > 2 * once


def test_run_episode_with_fake_clock():
    t = [0.0]
    def clock() -> float:
        return t[0]
    def sleep(dt: float) -> None:
        t[0] += max(dt, 1 / 30)
    arm = sw.FakeArm()
    counts = sw.run_episode(arm, lambda o: {**o, "shoulder_pan.pos": o["shoulder_pan.pos"] + 30.0},
                            sw.SafetyFilter(sw.so101_default_config()), duration_s=2.0, clock=clock, sleep=sleep)
    assert counts.get("velocity_limit", 0) >= 10
    assert len(arm.sent) <= 61
