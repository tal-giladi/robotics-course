# 20.04 — Stopping distance, safe speed, and which hazards are still open

Lesson: [20.04 The safety case](../../../20-final-robot/20.04-safety-case.md)

Every speed limit on the robot is the output of the first two functions. The third is the review
that decides whether the safety case is an argument or a list of good intentions.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `stopping_distance_m(speed, dead_time_s, decel)` | dead-time travel plus braking distance |
| `max_safe_speed_m_s(margin_m, dead_time_s, decel)` | the exact inverse |
| `open_hazards(hazards)` | the hazards whose controls do not hold |

## Check

```bash
python course.py check 20.04
python course.py check 20.04 --solution
```

The tests reproduce the stop-chain table of
[`20-final-robot/code/safety_case.py`](../../../20-final-robot/code/safety_case.py), including
the result that the e-stop acts fastest and stops the robot slowest.

## Hints

* Two terms: `v * t_dead` at full speed, then `v^2 / (2a)` braking. Only the second is quadratic,
  which is why doubling the speed more than doubles the distance.
* Invert by solving `v^2/(2a) + v*t - m = 0` for the positive root:
  `v = -a*t + sqrt(a^2*t^2 + 2*a*m)`. Check your algebra by feeding the answer back into
  `stopping_distance_m` — the tests do exactly that.
* An e-stop removes power, so the deceleration is coasting friction, not braking. Do not assume
  the fastest-acting mechanism gives the shortest stop.
* `open_hazards` accumulates reasons; one hazard can be open for several of them at once, and the
  ids come back sorted and unique.
