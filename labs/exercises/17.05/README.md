# 17.05 — The REINFORCE estimator

Lesson: [17.05 Policy gradients — REINFORCE intuitively and mathematically](../../../17-reinforcement-learning/17.05-policy-gradients.md)

The gradient estimator on its own, in numpy: no environment, no training loop, no PyTorch. The
tests check your analytic gradients against central finite differences and against the exact
gradient of a 3-action bandit, so a sign error or a missing `1/sigma**2` cannot pass.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `returns_to_go(rewards, gamma)` | `G_t = Σ_{k≥t} γ^(k−t) r_k` for one episode |
| `softmax_score(logits, action)` | `∇_logits log π(action)` = one-hot − π |
| `gaussian_score(action, mu, sigma)` | `(∂log π/∂μ, ∂log π/∂log σ)` for `a ~ N(μ, σ²)` |
| `mean_baseline(returns)` | per-timestep mean return-to-go over the episodes of a batch |
| `policy_gradient_estimate(scores, returns, baseline)` | Σ over time, mean over episodes, of `score · advantage` |

`softmax` is provided. Each stub raises `NotImplementedError` — replace the body, keep the signature.

## Check

```bash
python course.py check 17.05              # your code
python course.py check 17.05 --solution   # the reference
```

Then watch the same estimator train a controller:

```bash
python 17-reinforcement-learning/code/reinforce.py --part dock
```

## Hints

* Build the returns-to-go backwards in one pass: `acc = r[t] + gamma * acc`.
* `softmax_score` sums to zero over actions — that is the reason a constant baseline cannot bias
  the estimate, and a cheap self-check.
* `gaussian_score` returns the derivative with respect to **log σ**, not σ. An action exactly at
  the mean gives `(0, -1)`: no push on the mean, shrink the spread.
* The contraction is `sum` over time and `mean` over episodes. Getting that backwards makes your
  step size depend on the batch size and on the episode length.
* `np.einsum("itp,it->p", scores, advantages)` does the whole contraction in one call.
