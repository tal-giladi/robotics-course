"""Checker for 17.05 — the REINFORCE estimator.

Run: ``python course.py check 17.05`` (or ``--solution`` to see the reference pass).
Gradients are checked against central finite differences and against the exact bandit gradient,
so a sign error or a missing factor fails. The whole file runs in about a second.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

approx = pytest.approx


def numeric_gradient(f, x: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Central finite differences of a scalar function of a vector."""
    x = np.asarray(x, dtype=float)
    out = np.zeros_like(x)
    for k in range(x.size):
        step = np.zeros_like(x)
        step[k] = eps
        out[k] = (f(x + step) - f(x - step)) / (2 * eps)
    return out


# --- returns_to_go --------------------------------------------------------------------------------
def test_returns_to_go_hand_computed(impl):
    g = impl.returns_to_go(np.array([-1.0, -1.0, -1.0, 10.0]), 0.9)
    assert np.asarray(g).shape == (4,)
    assert g == approx([4.58, 6.2, 8.0, 10.0])


def test_returns_to_go_without_discount_is_a_reverse_cumulative_sum(impl):
    g = impl.returns_to_go(np.array([1.0, 2.0, 3.0]), 1.0)
    assert g == approx([6.0, 5.0, 3.0])


def test_returns_to_go_last_step_is_just_its_reward(impl):
    g = impl.returns_to_go(np.array([0.0, 0.0, 7.0]), 0.5)
    assert g[-1] == approx(7.0) and g[0] == approx(0.25 * 7.0)


# --- softmax_score --------------------------------------------------------------------------------
LOGITS = np.array([0.5, 0.0, -0.5])


def test_softmax_score_matches_finite_differences(impl):
    for action in range(3):
        analytic = impl.softmax_score(LOGITS, action)
        numeric = numeric_gradient(lambda z, a=action: math.log(impl.softmax(z)[a]), LOGITS)
        assert np.asarray(analytic).shape == (3,)
        assert analytic == approx(numeric, abs=1e-6)


def test_softmax_score_is_one_hot_minus_pi(impl):
    pi = impl.softmax(LOGITS)
    assert impl.softmax_score(LOGITS, 1) == approx([-pi[0], 1 - pi[1], -pi[2]])
    assert float(np.sum(impl.softmax_score(LOGITS, 2))) == approx(0.0, abs=1e-12)


# --- gaussian_score -------------------------------------------------------------------------------
def gauss_logpdf(a: float, mu: float, sigma: float) -> float:
    return -0.5 * ((a - mu) / sigma) ** 2 - math.log(sigma * math.sqrt(2 * math.pi))


def test_gaussian_score_matches_finite_differences(impl):
    a, mu, sigma, eps = 0.42, 0.30, 0.1, 1e-6
    d_mu, d_log_sigma = impl.gaussian_score(a, mu, sigma)
    assert d_mu == approx((gauss_logpdf(a, mu + eps, sigma) - gauss_logpdf(a, mu - eps, sigma)) / (2 * eps), rel=1e-5)
    hi, lo = math.exp(math.log(sigma) + eps), math.exp(math.log(sigma) - eps)
    assert d_log_sigma == approx((gauss_logpdf(a, mu, hi) - gauss_logpdf(a, mu, lo)) / (2 * eps), rel=1e-5)


def test_gaussian_score_hand_computed(impl):
    assert impl.gaussian_score(0.42, 0.30, 0.10) == approx((12.0, 0.44))
    # an action exactly at the mean: no push on mu, and sigma is told to shrink
    assert impl.gaussian_score(0.3, 0.3, 0.2) == approx((0.0, -1.0))


# --- mean_baseline --------------------------------------------------------------------------------
def test_mean_baseline_averages_over_episodes_not_time(impl):
    returns = np.array([[10.0, 0.0], [0.0, 4.0], [2.0, 2.0]])
    b = impl.mean_baseline(returns)
    assert np.asarray(b).shape == (2,)
    assert b == approx([4.0, 2.0])


# --- policy_gradient_estimate ---------------------------------------------------------------------
def test_estimate_hand_computed(impl):
    scores = np.array([[[1.0, 0.0], [0.0, 2.0]],     # episode 0, two steps, two parameters
                       [[-1.0, 1.0], [0.5, 0.0]]])   # episode 1
    returns = np.array([[3.0, 1.0], [-2.0, 4.0]])
    # episode 0: [1,0]*3 + [0,2]*1 = [3, 2] ; episode 1: [-1,1]*-2 + [0.5,0]*4 = [4, -2]
    assert impl.policy_gradient_estimate(scores, returns) == approx([3.5, 0.0])


def test_estimate_subtracts_the_baseline(impl):
    scores = np.array([[[1.0, 0.0], [0.0, 1.0]]])
    returns = np.array([[5.0, 5.0]])
    assert impl.policy_gradient_estimate(scores, returns, np.array([2.0, 6.0])) == approx([3.0, -1.0])
    assert impl.policy_gradient_estimate(scores, returns, np.zeros(2)) == approx(
        impl.policy_gradient_estimate(scores, returns)), "baseline=None must equal a zero baseline"


def test_estimate_sums_over_time_and_averages_over_episodes(impl):
    scores = np.array([[[1.0], [1.0]]])
    returns = np.array([[1.0, 1.0]])
    single = impl.policy_gradient_estimate(scores, returns)
    assert single == approx([2.0]), "the steps of one episode are summed"
    doubled = impl.policy_gradient_estimate(np.concatenate([scores, scores]), np.concatenate([returns, returns]))
    assert doubled == approx(single), "duplicating an episode must not change the estimate"


# --- the estimator is unbiased, and the baseline cuts its variance --------------------------------
def bandit_batch(impl, rng, n_episodes: int, logits: np.ndarray, R: np.ndarray):
    """One-step 'episodes': sample an action, observe a noisy reward. Returns (scores, returns)."""
    pi = impl.softmax(logits)
    actions = rng.choice(len(R), size=n_episodes, p=pi)
    rewards = R[actions] + rng.normal(0.0, 1.0, n_episodes)
    scores = np.array([[impl.softmax_score(logits, a)] for a in actions])  # (n, 1, 3)
    return scores, rewards.reshape(-1, 1)


def test_estimator_converges_to_the_exact_gradient(impl):
    R = np.array([1.0, 3.0, 0.0])
    pi = impl.softmax(LOGITS)
    exact = pi * (R - pi @ R)  # [-0.2168, 0.4829, -0.2661]
    scores, returns = bandit_batch(impl, np.random.default_rng(0), 20_000, LOGITS, R)
    assert impl.policy_gradient_estimate(scores, returns) == approx(exact, abs=0.03)


def test_the_baseline_keeps_the_mean_and_shrinks_the_spread(impl):
    R = np.array([1.0, 3.0, 0.0])
    rng = np.random.default_rng(1)
    plain, based = [], []
    for _ in range(200):
        scores, returns = bandit_batch(impl, rng, 50, LOGITS, R)
        b = impl.mean_baseline(returns)
        plain.append(impl.policy_gradient_estimate(scores, returns))
        based.append(impl.policy_gradient_estimate(scores, returns, b))
    plain, based = np.array(plain), np.array(based)
    assert plain.mean(axis=0) == approx(based.mean(axis=0), abs=0.06), "a baseline must not bias the estimate"
    assert based.std(axis=0).mean() < plain.std(axis=0).mean(), "a baseline must reduce the variance"
