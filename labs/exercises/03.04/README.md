# 03.04 — A drift-free fixed-rate scheduler

Lesson: [03.04 Loops, timing and concurrency — threads, asyncio, processes](../../../03-robot-software/03.04-timing-and-concurrency.md)

`while True: work(); time.sleep(0.02)` does not run at 50 Hz: it runs at
`1 / (0.02 + work time + oversleep)` Hz, and the error accumulates forever. This exercise builds
the scheduler every robot loop in the course can use: deadlines on a fixed grid, late ticks run
immediately, hopeless overruns are skipped and reported, and time comes from an injected clock
so the tests are exact and take microseconds.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `FixedRateScheduler(rate_hz, clock, sleep)` | deadlines at `t0 + k·period` of the injected monotonic clock |
| `FixedRateScheduler.wait()` | sleep until the next deadline; returns the number of skipped deadlines |
| `summarize_periods(timestamps, nominal)` | mean/std/min/max period, worst and p99 absolute error |

## Check

```bash
python course.py check 03.04              # your code
python course.py check 03.04 --solution   # the reference
```

The tests use a `FakeClock` whose `sleep` can oversleep by a fixed amount (like a real OS). They
check that 1000 ticks at 100 Hz take exactly 10 s, that a 1 ms oversleep on *every* tick
still ends at 10.001 s (a naive loop ends at 11 s), that loop-body time is absorbed, that a 25 ms
overrun skips one tick instead of bursting, that the phase survives a 1.2 s "debugger pause",
and that `sleep` never receives a negative duration. One smoke test runs 10 ticks on the real clock.

## Hints

* Store the *index* of the next deadline (or the deadline itself) and advance it by whole periods.
  Never compute the next deadline from `now`.
* `math.floor((now - deadline) / period)` is the number of extra deadlines that have passed.
