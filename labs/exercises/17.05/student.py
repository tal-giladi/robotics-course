"""17.05 — The REINFORCE estimator, in numpy.

Fill in every ``TODO(student)``. Check with ``python course.py check 17.05``.

The tests compare your analytic gradient with central finite differences of the true expected
return, so a sign error, a missing 1/sigma**2 or a confused axis cannot slip through.

Notation used throughout:

    theta        policy parameters
    pi(a|s)      the policy's probability (softmax over logits) or density (Gaussian)
    score        grad_theta log pi(a|s)              "how do I make THIS action more likely?"
    G_t          return-to-go from step t, discounted
    A_t          advantage estimate G_t - b(s_t)
    estimate     (1 / n_episodes) * sum over episodes sum over t of score_t * A_t
"""

from __future__ import annotations

import numpy as np


def returns_to_go(rewards: np.ndarray, gamma: float) -> np.ndarray:
    """Discounted return-to-go for every step of ONE episode.

        G_t = sum_{k >= t} gamma**(k - t) * rewards[k]

    ``rewards`` has shape (T,); the result has shape (T,). Example: rewards [-1, -1, -1, 10]
    with gamma 0.9 gives [4.58, 6.2, 8.0, 10.0].
    """
    # TODO(student): accumulate backwards; do not build a (T, T) matrix.
    raise NotImplementedError("returns_to_go")


def softmax(logits: np.ndarray) -> np.ndarray:
    """Numerically stable softmax over the last axis (provided, don't change)."""
    e = np.exp(logits - np.max(logits, axis=-1, keepdims=True))
    return e / np.sum(e, axis=-1, keepdims=True)


def softmax_score(logits: np.ndarray, action: int) -> np.ndarray:
    """grad_logits log pi(action) for a softmax policy: one-hot(action) - pi.

    ``logits`` has shape (n_actions,); the result has shape (n_actions,).
    """
    # TODO(student): two lines.
    raise NotImplementedError("softmax_score")


def gaussian_score(action: float, mu: float, sigma: float) -> tuple[float, float]:
    """Score of a Gaussian policy a ~ N(mu, sigma**2), as (d/dmu, d/d(log sigma)).

        d log pi / d mu        = (a - mu) / sigma**2
        d log pi / d(log sigma) = (a - mu)**2 / sigma**2 - 1

    Return Python floats.
    """
    # TODO(student): the two formulas above.
    raise NotImplementedError("gaussian_score")


def mean_baseline(returns: np.ndarray) -> np.ndarray:
    """The per-timestep baseline: the mean return-to-go over the episodes of a batch.

    ``returns`` has shape (n_episodes, T) — every episode in the batch has the same length here.
    The result has shape (T,): baseline[t] is the average of returns[:, t].
    """
    # TODO(student): one line.
    raise NotImplementedError("mean_baseline")


def policy_gradient_estimate(
    scores: np.ndarray, returns: np.ndarray, baseline: np.ndarray | None = None
) -> np.ndarray:
    """The REINFORCE gradient estimate of a batch of episodes.

        estimate = mean over episodes of  sum_t scores[i, t] * (returns[i, t] - baseline[t])

    ``scores`` has shape (n_episodes, T, n_params), ``returns`` (n_episodes, T), ``baseline``
    either ``None`` (no baseline) or shape (T,). The result has shape (n_params,).

    Note the asymmetry, and it matters: SUM over time (an episode's steps all contribute),
    MEAN over episodes (more episodes must not mean a bigger step).
    """
    # TODO(student): build the advantages, then contract. np.einsum or broadcasting + sum.
    raise NotImplementedError("policy_gradient_estimate")
