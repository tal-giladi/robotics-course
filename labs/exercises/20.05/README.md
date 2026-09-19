# 20.05 — Verdicts and the regression report

Lesson: [20.05 Testing the whole robot](../../../20-final-robot/20.05-testing-strategy.md)

A scenario run produces numbers; a test suite produces decisions. These two functions are the
step in between, and they are what makes a change to the robot reviewable.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `verdict(run, thresholds)` | every threshold one run violated, as sorted keys |
| `regressions(before, after, thresholds)` | what got worse between two suite runs |

## Check

```bash
python course.py check 20.05
python course.py check 20.05 --solution
```

The last tests run the real simulator suite from
[`20-final-robot/code/scenario_suite.py`](../../../20-final-robot/code/scenario_suite.py) twice —
once with the collision monitor and once without — and require your report to name the two
regressions it produces.

## Hints

* Report **every** violated threshold. Stopping at the first one costs a whole suite run per bug,
  and on hardware a suite run costs an afternoon.
* `"stop-safely"` inverts the reach rule and adds one of its own: a robot that stopped without
  the safety layer firing stopped by luck, and luck is not a test result.
* Comparisons are strict — a run exactly at the limit passes.
* In `regressions`, `REGRESSION` and `FIXED` are mutually exclusive, but a `WORSE` line can
  accompany either. Watch the order the tests expect: the pass/fail line first, then clearance,
  then time.
* The time rule needs **both** conditions (25 % and 1 s), or every two-second scenario reports a
  regression whenever the scheduler sneezes.
