"""17.05 — reference solution. Read it after you have tried ``student.py`` yourself."""

from __future__ import annotations

import numpy as np


def returns_to_go(rewards: np.ndarray, gamma: float) -> np.ndarray:
    rewards = np.asarray(rewards, dtype=float)
    out = np.empty_like(rewards)
    acc = 0.0
    for t in range(len(rewards) - 1, -1, -1):
        acc = rewards[t] + gamma * acc
        out[t] = acc
    return out


def softmax(logits: np.ndarray) -> np.ndarray:
    e = np.exp(logits - np.max(logits, axis=-1, keepdims=True))
    return e / np.sum(e, axis=-1, keepdims=True)


def softmax_score(logits: np.ndarray, action: int) -> np.ndarray:
    pi = softmax(np.asarray(logits, dtype=float))
    one_hot = np.zeros_like(pi)
    one_hot[int(action)] = 1.0
    return one_hot - pi


def gaussian_score(action: float, mu: float, sigma: float) -> tuple[float, float]:
    d = float(action) - float(mu)
    return float(d / sigma**2), float(d**2 / sigma**2 - 1.0)


def mean_baseline(returns: np.ndarray) -> np.ndarray:
    return np.asarray(returns, dtype=float).mean(axis=0)


def policy_gradient_estimate(
    scores: np.ndarray, returns: np.ndarray, baseline: np.ndarray | None = None
) -> np.ndarray:
    scores = np.asarray(scores, dtype=float)
    advantages = np.asarray(returns, dtype=float)
    if baseline is not None:
        advantages = advantages - np.asarray(baseline, dtype=float)[None, :]
    return np.einsum("itp,it->p", scores, advantages) / scores.shape[0]
