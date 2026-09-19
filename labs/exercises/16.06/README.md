# 16.06 — The six numbers that tell you whether a learned component is working

Lesson: [16.06 Evaluating ML components inside a robot](../../../16-machine-learning/16.06-evaluating-ml-in-robots.md)

mAP tells you about the model. These tell you about the robot: whether its confidence means
anything, whether its success rate is a measurement or a rumour, and whether today's world still
looks like the one it was trained in.

## What to implement (`student.py`)

| Function | Answers |
|---|---|
| `reliability_table(scores, correct, bins)` | in each score bucket, how often is the detector actually right? |
| `expected_calibration_error(...)` | one number for "how much does 0.9 lie?" |
| `fit_temperature(scores, correct)` | the single scalar that makes the scores honest |
| `apply_temperature(scores, T)` | apply it (monotone — it cannot change AP) |
| `wilson_interval(successes, trials)` | is 18/20 really "90 % success"? |
| `psi(reference, current)` | has the world moved since the acceptance test? |
| `summarise_episodes(episodes)` | per-condition success rate, interval, outcome classes, median time |

`Bin`, `Episode`, `logit` and `sigmoid` are given.

## Check

```bash
python course.py check 16.06              # your code
python course.py check 16.06 --solution   # the reference, to see what passing looks like
```

## Hints

- **Binning:** equal-width bins over [0, 1]; the last bin includes 1.0 (`scores <= hi`), all others
  are half-open. Skip empty bins — do not emit a `Bin` with `n = 0`, or ECE will divide by zero.
- **ECE weights by bin population**, not by bin: a bin with three detections must not count as much
  as one with three thousand.
- **`fit_temperature`** is a one-dimensional grid search over negative log-likelihood. Work in logit
  space (`logit` is given), and remember the sign: you are *minimising* NLL, which means *maximising*
  `y·log p + (1-y)·log(1-p)`. `T > 1` softens, `T < 1` sharpens.
- **Wilson, not the textbook interval.** `p ± z√(p(1−p)/n)` gives nonsense near 0 and 1 (it happily
  returns an upper bound above 1, and a zero-width interval for 0/20). The formula in the docstring
  is in the lesson, and the tests pin 18/20 to [0.699, 0.972].
- **PSI bins are the *reference's* quantiles**, so the reference is uniform across bins by
  construction and only the current sample's shape matters. `np.quantile` + `np.searchsorted` +
  `np.bincount(..., minlength=bins)` is the whole implementation. Add the 1e-3 or an empty bin gives
  you `log(0)`.
- **`summarise_episodes`** must be robust to a condition with no successes (`median_time` is `nan`,
  not a crash) and must preserve the order conditions first appear (`dict.fromkeys` does this).
