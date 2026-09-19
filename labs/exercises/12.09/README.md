# 12.09 — The mission layer above Nav2

Lesson: [12.09 Waypoints and missions with the Nav2 Simple Commander API](../../../12-navigation/12.09-waypoints-and-missions.md)

Nav2 answers "drive to this pose". It does **not** answer "visit the kitchen, then the bedroom,
then come home; if a door is shut, try once more and then skip that room and tell me". That policy
is yours, and this exercise is it.

You write it against the `Navigator` protocol, which
`nav2_simple_commander.robot_navigator.BasicNavigator` satisfies method for method, so the same
state machine runs against the course simulator and against the real robot.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `load_places(path)` | a places YAML → `{name: Place}`, with `yaw` and `frame_id` defaults |
| `mission_from_names(names, places, **kwargs)` | build a mission, refusing unknown names loudly |
| `WaypointMission.run(navigator, max_iterations)` | the state machine: attempt, poll, read feedback, retry, skip or abort |

`Place`, `TaskResult`, `Feedback`, `StopState`, `Stop` and `MissionReport` are given. The docstring
of `run` spells out the algorithm step by step.

## Check

```bash
python course.py check 12.09              # your code
python course.py check 12.09 --solution   # the reference, to see what passing looks like
```

The tests drive your state machine with a fake navigator that returns scripted results: a clean
three-stop run, a stop that fails once and is retried, a stop that fails every time (skipped, or
aborting the mission with `stop_on_failure`), a cancelled goal, a rejected goal, per-stop
`max_attempts`, an arrival task that must only run on success, and a task that never finishes and
has to be cancelled after `max_iterations` polls.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* Read the feedback **once, after** the poll loop, not inside it: `BasicNavigator.getFeedback()`
  keeps returning the last feedback after the task finishes, and a retried stop must accumulate
  the time and recoveries of every attempt.
* A rejected goal (`go_to_pose` returns False) costs an attempt but must not poll.
* CANCELED is not FAILED. Somebody cancelled on purpose: do not retry, and do not run the
  remaining stops.
* `stop_on_failure` only matters when a stop has run out of attempts.
* Marking the remaining stops `SKIPPED` is what makes the report readable afterwards — do it in
  `run`, not in the per-stop logic.
