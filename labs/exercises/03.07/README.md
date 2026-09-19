# 03.07 — Turn a telemetry log into answers

Lesson: [03.07 Logging, telemetry and plotting what the robot did](../../../03-robot-software/03.07-logging-and-telemetry.md)

A robot run produces a few thousand rows of CSV. Before you plot anything, you want one line of
numbers that tells you whether the run is even worth looking at: how many samples, at what rate,
how many were lost, how far it drove, and when the watchdog tripped.

In this exercise you write that reduction — standard library only, because it has to run on the
Pi straight after a run, over SSH, with nothing installed.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `read_telemetry_csv(path, *, strict=False)` | parse the CSV `labs/robot/log_telemetry.py` writes; empty cell → `None`; reject a file that is not a telemetry log; survive a half-written last row |
| `summarize(samples, meters_per_tick)` | one `RunSummary`: rate, gaps, lost samples, path length, battery, range validity |
| `flag_episodes(samples, flag)` | every contiguous stretch where a flag bit was set, with start, end and count |

The exact definitions are in the docstrings. Follow them literally — the tests do.

## Check

```bash
python course.py check 03.07              # your code
python course.py check 03.07 --solution   # the reference
```

Small hand-computed cases come first. Then the tests run against `run_telemetry.csv`, a committed
7-second recording of the course simulator that contains, on purpose:

* two watchdog episodes — one 300 ms after boot before anything commanded the robot, one 300 ms
  after the last command;
* a 0.22 s telemetry gap that swallowed exactly 10 samples, so the run averages 48.6 Hz and not 50;
* a stretch at the start where the front ToF sensor had no reading at all, because the wall was
  beyond its 4 m range.

Regenerate it with `python labs/exercises/03.07/make_run.py` (deterministic, no hardware).

## Hints

* `csv.DictReader` gives you `dict[str, str]`; check `reader.fieldnames` before trusting a file.
* An empty cell is `""`, not `None`. Write one `_optional_float` helper and use it four times.
* `statistics.median` on the interval list, not the mean: one 0.22 s gap must not move the
  "normal" interval.
* `distance_m` is a **path length**: take `abs()` per step, so driving back does not cancel
  driving out — and a turn in place comes out as 0.
* For episodes, iterate with `enumerate` and track the start *index*. Two telemetry rows from a
  stalled robot can be byte-identical, so `list.index` would find the wrong one.
