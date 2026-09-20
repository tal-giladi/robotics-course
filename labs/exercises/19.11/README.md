# 19.11 — Endpointing, keyword routing and the latency budget

Lesson: [19.11 Talking to the robot — speech in, speech out](../../../19-llm-robot-agents/19.11-voice-interface.md)

The three pieces of a voice interface that are engineering rather than model choice: deciding when
the person stopped talking, deciding what goes to the model and what must never wait for it, and
knowing which of the five stages owns your four seconds of delay.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `rms_db`, `frames`, `speech_flags` | frame levels on a real numpy signal |
| `segments` | utterances, with the hangover rule |
| `phrase_score`, `is_stop` | the wake word and the stop word |
| `route` | stop first, then wake, then command, then ignore |
| `LatencyBudget`, `asr_latency_s`, `travel_m` | where the seconds go, and what they cost in metres |

`VADConfig` is given. The tests synthesise their own audio, so no microphone and no models.

## Check

```bash
python course.py check 19.11              # your code
python course.py check 19.11 --solution   # the reference
```

## Hints

* A segment ends where the **speech** ended, `hang` frames before the silence proved it: close at
  `i - hang + 1`. Closing at `i` makes every utterance 400 ms longer than it was, which the ASR
  then charges you for.
* The hangover is the delay the user feels and it is *not* in the returned times. It belongs in
  the latency budget, where you can see it next to the ASR and decide which to cut.
* `route` checks the stop word before anything else, including whether the robot is awake.
  Ordering it after the wake check means "stop" only works if you say "hey karmel" first, which
  is not what a person does when a robot is about to hit something.
* Whole-word matching for the stop word. `"stopwatch" in text` is a substring test and it will fire
  on a podcast.
* `travel_m(0.02)` versus `travel_m(4.28)` is the argument for the whole design: 6 mm against
  1.28 m at the course robot's 0.3 m/s.
