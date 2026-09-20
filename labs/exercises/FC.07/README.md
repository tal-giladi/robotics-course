# FC.07 — Is this loop real-time?

Lesson: [FC.07 Real-time concepts: determinism, latency, jitter and watchdogs](../../../optional-foundations/cpp-and-embedded/FC.07-real-time-concepts.md)

Three questions you ask about every control loop, turned into code: *did* it meet its deadline,
*can* it meet its deadline, and what happens when it doesn't.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `percentile(values, fraction)` | nearest-rank percentile — the tail is what matters, not the mean |
| `analyse_loop(timestamps, nominal_hz, tolerance)` | periods, p99, worst jitter, deadline misses |
| `total_utilisation(tasks)` | Σ WCET / period |
| `rate_monotonic_bound(n)` | Liu & Layland's $n(2^{1/n} - 1)$ |
| `is_schedulable(tasks)` | verdict + one-line reason |
| `ticks_diff`, `CommandWatchdog` | the rule from `labs/firmware/pico/watchdog.py`, from its specification |
| `stopping_distance_m(...)` | what a 300 ms watchdog costs in metres |

## Check

```bash
python course.py check FC.07              # your code
python course.py check FC.07 --solution   # the reference
```

The tests use karmel's real task set (100 Hz control, 50 Hz telemetry, 10 Hz sensors with a
blocking 30 ms ultrasonic read), one loop with a single 5 ms late step, and a watchdog fed just
before the 2³⁰ ms tick counter wraps.

## Hints

* A period that is *shorter* than nominal is the loop catching up. Only long periods are misses.
* `is_schedulable` has three branches and the strings are compared exactly — read the docstring's
  examples, including the three-decimal formatting and the `-` (a plain hyphen) in the third one.
* The watchdog must return `True` exactly once per trip, and it starts tripped.
* `stopping_distance_m` adds two things: the coasting distance while nobody has noticed
  (`timeout + one control step`) and the braking distance $v^2 / 2a$.
