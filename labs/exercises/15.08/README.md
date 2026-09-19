# 15.08 — The three functions a pick-and-place pipeline is made of

Lesson: [15.08 A pick-and-place pipeline with a state machine](../../../15-manipulation/15.08-pick-and-place-pipeline.md)

Strip the motion, the perception and the ROS plumbing away and a pick-and-place pipeline is three
decisions: where do I go next, is anything in the jaws, and what do I do about *this* failure. All
three fit in one file with no dependencies, and all three are where the bugs live.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `next_phase(phase)` | the nominal sequence, with `DONE`/`FAILED` absorbing |
| `verify_grasp(reading, expected_width)` | `ok` / `empty_grasp` / `partial_grasp`, from three signals |
| `RetryPolicy.decide(outcome, attempts, total)` | `CONTINUE` / `RETRY` / `REPERCEIVE` / `NUDGE` / `SKIP_OBJECT` / `ABORT` |

`Phase`, `NOMINAL`, `Outcome`, `Action` and `GripperReading` are given.

## Check

```bash
python course.py check 15.08              # your code
python course.py check 15.08 --solution   # the reference, to see what passing looks like
```

26 tests, well under a second. They pin the sequence (something always checks the jaws after
`CLOSE`, and again after `LIFT`), the verification (a jaw closed on its own stop reads a *high*
load, so width must be checked first; two objects at once is as wrong as a corner grasp), and the
policy's ordering (the global budget outranks every tag, an unknown tag aborts rather than
retrying, and `place_blocked` can never be `SKIP_OBJECT` because you are holding the thing).

A skipped test means that part is not implemented yet; the check passes only when nothing is
skipped.

## Hints

* Check the conditions in `decide` **in the documented order**. Several tests exist only to pin
  that order, because it is where the real bugs are: a policy that checks the per-object budget
  before the global one runs forever on a robot with a stuck gripper.
* `verify_grasp` checks width *before* load on purpose. An empty jaw at its own mechanical stop
  draws plenty of current; the load reading is a second opinion, never the first.
* Every failure `Outcome` needs a useful `detail` string. There is a test for it, because you will
  be reading these in a log at 23:00 with the robot stopped.
* `ABORT` for an unknown tag is not defensive pessimism, it is the correct behaviour. A tag nobody
  wrote a rule for means your code has a path you did not think about, and the robot is the wrong
  place to find that out.
