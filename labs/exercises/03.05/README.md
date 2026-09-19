# 03.05 — A fake DifferentialBase that passes the conformance suite

Lesson: [03.05 Hardware abstraction — interfaces, drivers and fakes](../../../03-robot-software/03.05-hardware-abstraction.md)

Every robot program in the course talks to a `DifferentialBase`. The simulator (`SimBase`) and
the real robot (`SerialBase`) implement it. In this exercise you write the third implementation:
an in-memory fake that is exact and instant, for unit tests. Then you write a small piece of
business logic, `drive_distance`, against the interface only, and prove it runs unchanged on
your fake and on the simulator.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `FakeBase` | ideal wheels, integrated ticks, simulated 0.3 s watchdog, flags, command spy, close/context manager |
| `drive_distance(base, distance_m, …)` | drive straight by encoder ticks on *any* base, time out safely, always stop |

## Check

```bash
python course.py check 03.05              # your code
python course.py check 03.05 --solution   # the reference
```

The **conformance tests** are written against the `DifferentialBase` contract only and run twice:
once on your `FakeBase` and once on `SimBase` (with the course's ideal simulated robot). They
check increasing time, ticks consistent with the commanded speed (±3 %), the velocity-mode flag,
clamping, the watchdog, `stop()`, and `close()`. More tests pin down the fake's exact behavior
(1 rev/s for 1 s = exactly 2464 ticks). Then `drive_distance` must reach 0.5 m and −0.3 m on
both bases (±2 cm), stop within 0.2 s, and on a robot with blocked wheels raise `TimeoutError`
*and still call `stop()`*.

## Hints

* Store wheel angles as floats and round to ticks only in `read()`, or the fake loses precision.
* Feed the watchdog in one private `_command()` helper that both `set_wheel_*` methods call.
* `math.copysign(speed, distance_m)` gives the signed wheel speed.
