# 14.11 — The safety supervisor

Lesson: [14.11 Arm safety — torque, speed, workspace limits and e-stop](../../../14-robotic-arm/14.11-arm-safety.md)

The layer that decides whether the arm is allowed to move, and the three numbers that justify the
limits it enforces. It sits above 14.03's servo safety and below whatever decided to move — your
code, a planner, or a learned policy.

What it cannot do, and what no software can: remove energy from the servos. That is the e-stop.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `tip_speed(joint_speed_deg_s)` | worst-case tool speed: the whole arm swinging at full reach |
| `kinetic_energy(joint_speed_deg_s)` | why halving the speed quarters the impact |
| `pinch_force(lever_m, torque_percent)` | why the arm is harmless at the tool and dangerous at a joint |
| `ArmState` | `DISABLED` / `ENABLED` / `STOPPED` |
| `SafetySupervisor.heartbeat` / `watchdog_expired` | the watchdog |
| `SafetySupervisor.enable` / `stop` / `clear_fault` | the latching state machine |
| `SafetySupervisor.check_move` | every reason to refuse, as strings |

`JointLimits`, `Box` and `SafetyError` are given.

## Check

```bash
python course.py check 14.11              # your code
python course.py check 14.11 --solution   # the reference, to see what passing looks like
```

26 tests, well under a second. A fake clock makes the watchdog tests instant and deterministic.

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* `tip_speed` takes **degrees** per second and the formula needs **radians**. Getting it wrong
  gives 163 m/s instead of 2.9, and there is a test whose only job is to catch that.
* `clear_fault()` goes to `DISABLED`, not `ENABLED`. Recovery is two deliberate acts, exactly like
  twisting an e-stop and *then* re-enabling. A single call that resumes motion is the bug the
  whole state machine exists to prevent.
* `enable()` from `STOPPED` must raise. From `DISABLED` or `ENABLED` it is fine.
* The watchdog expires at **strictly more** than `watchdog_s`. Exactly `watchdog_s` is still fine;
  there is a test on the boundary.
* A joint with **no configured limits** is a problem to report, never a joint that may move
  freely. Same rule as `clamp_goals` in 14.03, and for the same reason.
* `check_move` returns *every* problem, not the first. A caller who fixes one and retries should
  not discover the next one by moving the arm.
