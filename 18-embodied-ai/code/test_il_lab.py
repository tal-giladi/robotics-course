"""Tests for il_lab.py, inspect_demos.py and compute_budget.py: small, CPU-only versions of every
claim the lessons 18.01-18.05 make. Run:  py -m pytest 18-embodied-ai/code/test_il_lab.py -q
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import torch

import compute_budget as cb
import il_lab
import inspect_demos

torch.set_num_threads(1)


def test_behavior_cloning_compounds_and_dagger_fixes_it() -> None:
    r = il_lab.bc_vs_dagger(dagger_iters=2, n_eval=60, epochs=300, verbose=False)
    assert r["expert_success"] > 0.95
    assert r["bc_success"] < 0.5
    assert r["bc_err"][80] > 3 * r["bc_err"][20]            # error grows over the episode
    assert r["dagger_success"][-1] > 0.85
    assert r["dataset_sizes"] == [800, 1600, 2400]


def test_mse_averages_modes_and_diffusion_samples_one() -> None:
    r = il_lab.multimodal(n_eval=40, verbose=False)            # default 3000 training iterations
    first = r["mse_first_chunk"]
    assert abs(first[2, 1]) < 0.1                           # third MSE waypoint: straight ahead, not +/-0.4 m
    assert r["mse_collision_rate"] > 0.4
    assert r["ddpm_success"] > 0.75
    assert r["ddpm_collision_rate"] < r["mse_collision_rate"]
    assert 0.15 < r["ddpm_left_fraction"] < 0.85             # both modes are sampled


def test_chunking_and_temporal_ensembling() -> None:
    r = il_lab.chunking(n_eval=40, epochs=400, verbose=False)
    assert r["ensemble"]["jitter"] < r["single"]["jitter"]
    assert r["ensemble"]["max_jump"] < r["open-loop"]["max_jump"]
    assert r["ensemble"]["rms_error"] < r["open-loop"]["rms_error"]


@pytest.mark.parametrize("m", [0.0, 0.1])
def test_temporal_ensemble_weights_oldest_first(m: float) -> None:
    k = 10
    chunk = np.repeat(np.arange(k, dtype=float)[:, None], 2, axis=1)       # action j has value j
    predict = lambda obs: np.repeat(chunk[None], len(obs), axis=0)
    _, acts = il_lab.chunk_rollout(il_lab.LaneTask(), predict, k, "ensemble", 1,
                                   np.random.default_rng(0), obs_noise_m=0.0, m=m)
    candidates = np.arange(k - 1, -1, -1, dtype=float)                     # oldest prediction used index 9
    w = np.exp(-m * np.arange(k))
    assert acts[0, k - 1, 0] == pytest.approx(float((w * candidates).sum() / w.sum()))


def test_ddim_is_deterministic_given_the_noise() -> None:
    obs = np.random.default_rng(0).normal(size=(50, 2))
    chunks = np.random.default_rng(1).normal(size=(50, 4))
    dp = il_lab.DiffusionPolicy(obs, chunks, iters=20)
    a = dp.sample(obs[:3], torch.Generator().manual_seed(5), ddim_steps=5)
    b = dp.sample(obs[:3], torch.Generator().manual_seed(5), ddim_steps=5)
    assert a.shape == (3, 4)
    np.testing.assert_allclose(a, b)
    assert float(dp.abar[-1]) < 1e-3 < float(dp.abar[0])                 # ends at (almost) pure noise


def test_inspect_demos_flags_exactly_the_bad_synthetic_episodes() -> None:
    flags = inspect_demos.flag([inspect_demos.episode_stats(e) for e in inspect_demos.synthetic_episodes()])
    assert set(flags) == {3, 7, 12}
    assert "pause" in flags[3][0] and "gripper" in flags[7][0] and "jerky" in flags[12][0]


def test_compute_budget_numbers() -> None:
    assert cb.full_finetune_gb(80e6) == pytest.approx(1.28)
    assert cb.lora_gb(1.2e9) == pytest.approx(2.568)
    b = cb.ControlBudget(control_hz=30, chunk_size=50, execute_steps=50, inference_s=1.0)
    assert b.open_loop_s == pytest.approx(50 / 30)
    assert b.synchronous_duty == pytest.approx((50 / 30) / (50 / 30 + 1.0 - 1 / 30))
    assert b.stale_steps == pytest.approx(30.0)
    assert math.isclose(cb.ControlBudget(30, 100, 59, 0.25).synchronous_duty, 0.9008, abs_tol=1e-3)
