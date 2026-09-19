# 10.03 — The Bayes filter: predict and update on a 1D grid

Lesson: [10.03 The Bayes filter — predict and update on a 1D grid](../../../10-localization/10.03-bayes-filter.md)

Build a histogram filter: a robot in a corridor with doors keeps a probability for every cell,
blurs it when it moves (prediction) and sharpens it when its door detector reads something (update).

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `normalize(belief)` | a new array that sums to 1; `ValueError` if the sum is not positive |
| `predict(belief, move, kernel, wrap=False)` | motion update: shift by `move` cells, blur with the noise `kernel`; circular corridor (`wrap=True`) or end walls (`wrap=False`) |
| `door_likelihood(doors, z_door, p_hit, p_false)` | measurement model `P(z | cell)` for every cell |
| `update(belief, likelihood)` | Bayes' rule: `normalize(likelihood * belief)` |
| `run_filter(prior, doors, moves, readings, kernel, p_hit, p_false, wrap=False)` | the loop: predict, then update unless the reading is `None`; returns every belief |

`corridor_sim.py` is **provided** (not part of the exercise): it drives karmel down a simulated
10 m corridor and produces the `moves` and door `readings` from encoders and LiDAR.

## Check

```bash
python course.py check 10.03              # your code
python course.py check 10.03 --solution   # the reference, to see what passing looks like
```

The tests start with hand-computed beliefs (a 5-cell corridor, step by step), then check global
localization on a synthetic circular corridor (the peak must be within one cell of the truth in at
least 13 of 20 runs), and finally run your filter on the simulated robot: after 20 cells, starting
from "no idea" (a uniform prior), the belief peak must be within one cell of the true cell and hold
more than 80% of the probability within ±1 cell.

## Hints

* `np.roll(b, 1)` moves everything one cell to the right, and the last cell wraps to cell 0.
* `np.add.at(out, targets, values)` adds correctly even when several cells land in the same target.
* A likelihood does not sum to 1; a belief does.
