# 17.03 — Q-learning on a gridworld robot

Lesson: [17.03 Q-learning on a gridworld robot](../../../17-reinforcement-learning/17.03-q-learning-gridworld.md)

karmel lives in a 4 × 6 floor plan: start `S`, charger `G` (+10), stairs `X` (−10), walls `#`,
−1 per move. The `GridWorld` class in `student.py` is provided. You implement the learning rule.

```text
    col:  0 1 2 3 4 5
    row 0 S . . # . G
    row 1 . # . # . .
    row 2 . # . . . X
    row 3 . . . # . .
```

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `q_learning_update(Q, s, a, r, s_next, terminated, alpha, gamma)` | one TD update of `Q[s, a]` in place, no bootstrap from a terminal state; returns the TD error |
| `greedy_action(Q, s)` | `argmax_a Q[s, a]`, ties → lowest action index |
| `greedy_policy(Q)` | the greedy action of every state, shape `(n_states,)` |
| `epsilon_greedy(Q, s, epsilon, rng)` | random action with probability ε, else greedy; exactly one `rng.random()` per call plus one `rng.integers(4)` when exploring |

Each stub raises `NotImplementedError` — replace the body, keep the signature.

## Check

```bash
python course.py check 17.03              # your code
python course.py check 17.03 --solution   # the reference
```

The tests are hand-computed updates (worked arithmetic in comments), statistical checks of
ε-greedy, one hand-driven episode, and a deterministic sweep: applying your update with α = 1 to
every (state, action) pair 60 times must reproduce the optimal Q-values from value iteration, and
your greedy policy must take the shortest safe path from `S`. No training loops: under a second.

Then train for real with your code:

```bash
python 17-reinforcement-learning/code/gridworld_q.py
python 17-reinforcement-learning/code/gridworld_q.py --slip 0.2 --episodes 5000 --alpha 0.05
```

## Hints

* The target for a terminal transition is just `r`. Bootstrapping from `Q[G]` is the most common bug.
* `np.argmax` already breaks ties toward the lowest index.
* Keep `Q[s, a]` a float array; integer arrays silently truncate updates.
