# 14.03 — A start-up that does not break the arm

Lesson: [14.03 Controlling servos — the STS3215 bus, registers and safe motion](../../../14-robotic-arm/14.03-controlling-servos.md)

Exercise **14.03-E1**. A bus servo holds its last `Goal_Position` in a register even when it is
limp. Enable torque without thinking about that and the joint drives there at full speed — the
40° snap that everyone with an SO-101 has seen once. The fix is five lines in the right order,
plus a rate limit and a load check around every move.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `speed_register(deg_per_s)` | `Goal_Velocity` in steps/s — and never 0, which means "no limit" |
| `acceleration_register(deg_per_s2)` | `Acceleration`, 1..254, 100 steps/s² per unit |
| `torque_register(percent)` | `Torque_Limit`, 0..1000 for 0..100 % of stall |
| `decode_sign_magnitude(raw, bit)` | Feetech's sign-magnitude encoding: 1036 at bit 10 is **−12**, not +103.6 |
| `clamp_goals(goals, limits)` | clamp into the software limits, report what was clamped, **raise** for an unlimited joint |
| `health_problems(telemetry, cfg)` | voltage, temperature and load, as strings a human can act on |
| `safe_enable_torque(bus, cfg)` | limp → read → check → write limits → write goal → **then** torque on |
| `move_smoothly(bus, goals, cfg)` | walk to the goal at `max_step_deg` per tick, abort on overload, time out |

`JointLimits`, `SafetyConfig`, `SafetyError`, `Telemetry`, `read_telemetry` and the six-servo
`FakeArmBus` simulator are given.

## Check

```bash
python course.py check 14.03              # your code
python course.py check 14.03 --solution   # the reference, to see what passing looks like
```

27 tests, well under a second, no hardware. Several of them check the **order** of bus
operations, not just the final pose: a solution that enables torque before writing the goal ends
up in the right place and still fails, because on a real arm it got there by throwing the joint.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Do the four conversions first — everything else uses them, and each is one line.
* `0` is a magic value in **two** registers: `Goal_Velocity = 0` means full speed and
  `Acceleration = 0` means maximum. Never let rounding produce one. `max(1, round(...))`.
* Sign-magnitude is not two's complement. Mask off the bits *below* the sign bit for the
  magnitude, then apply the sign. `1036 = 1024 + 12` → `-12`.
* The two tests people fail first: the operation-order one (`Goal_Position` must be written
  *before* `Torque_Enable`) and the one asserting that `clamp_goals` **raises** for a joint with
  no configured limits instead of passing it through.
* In `move_smoothly`, keep a separate *commanded* position that you step toward the target, and
  read the *present* position back from the bus. Using the present position as the thing you step
  from makes the loop stall whenever the arm lags behind, which it always does.
* On an overload, write the present positions as the goal **before** raising. Raising without that
  leaves the servo pushing against whatever blocked it, at its torque limit, for as long as your
  exception handler takes.
* The tests pass `bus.advance` as the `sleep` callback, so calling `sleep(dt)` once per tick is
  what makes simulated time move. Forget it and the loop spins to its timeout.
