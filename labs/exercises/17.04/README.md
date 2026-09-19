# 17.04 — The three pieces of DQN

Lesson: [17.04 From tables to networks — DQN](../../../17-reinforcement-learning/17.04-deep-q-networks.md)

`17-reinforcement-learning/code/dqn.py` writes the replay buffer, the exploration schedule and the
TD target inline inside its training loop. Here they are pulled out and tested on their own, in
numpy, so a mistake costs you a second instead of a three-minute training run.

No PyTorch is needed: a network is represented by the Q-values it already produced.

## What to implement (`student.py`)

| Part | Does |
|---|---|
| `ReplayBuffer.add` | append; overwrite the **oldest** transition once the buffer is full |
| `ReplayBuffer.all` | every stored transition, oldest first (mind the wrap-around) |
| `ReplayBuffer.sample(batch_size, rng)` | a uniform batch without replacement, as stacked arrays with fixed dtypes |
| `epsilon_at(step, total_steps, …)` | linear decay over the first `fraction` of the run, then flat |
| `td_target(reward, terminated, next_q, gamma)` | `r + γ (1 − terminated) · max_a' next_q` |
| `double_dqn_target(…)` | online network picks the action, target network scores it |

Each stub raises `NotImplementedError` — replace the body, keep the signature.

## Check

```bash
python course.py check 17.04              # your code
python course.py check 17.04 --solution   # the reference
```

Then run the real thing, which uses the same logic in PyTorch:

```bash
python 17-reinforcement-learning/code/dqn.py
```

## Hints

* `terminated` comes back from `sample` as **floats**, because the target multiplies by `1 - terminated`.
* The mask is about termination only. A transition cut off by the time limit (truncation) still
  bootstraps: `terminated = 0.0`.
* `all()` must return the transitions oldest-first even after the buffer has wrapped several times;
  the oldest one lives at the write cursor, not at index 0.
* `sample` draws exactly one `rng.choice(len(self), size=batch_size, replace=False)`, so the same
  seed gives the same batch.
* Vectorize the targets: `np.max(..., axis=1)`, `np.argmax(..., axis=1)`, `np.take_along_axis`.
