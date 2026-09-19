# 17.07 — Wrap the course simulator as a Gymnasium environment

Lesson: [17.07 Wrap the course simulator as a Gymnasium environment and train go-to-goal](../../../17-reinforcement-learning/17.07-robot-gym-environment.md)

Turn `robotlab.sim.DiffDriveSim` into a `gymnasium.Env` that any RL library can train on:
karmel starts at a random pose in an empty 4 m × 4 m room and must drive to a random goal.
The full contract (observation, action, step length, reward, termination) is in the docstring
at the top of `student.py`.

Needs `pip install gymnasium` (the tests skip without it).

## What to implement (`student.py`)

| Part | Does |
|---|---|
| `twist_to_wheels(v, w, r, b)` | body velocity → left/right wheel speeds (rad/s) |
| `make_observation(pose, goal, v, w, d_max, v_max, w_max)` | the 5 normalized numbers, clipped, `float32` |
| `step_reward(d_prev, d, success, collided)` | 10 × progress − 0.05, ±10 on success / collision |
| `GoToGoalEnv.__init__` | `observation_space` and `action_space` (both `Box`, `float32`) |
| `GoToGoalEnv.reset(seed, options)` | seed via `super().reset(seed=seed)`, sample or accept start/goal, build the simulator |
| `GoToGoalEnv.step(action)` | clip, convert, 5 × 0.02 s physics steps, reward, `terminated` / `truncated` |

## Check

```bash
python course.py check 17.07              # your code
python course.py check 17.07 --solution   # the reference
```

The tests (a few seconds, no training) run Gymnasium's own `check_env` with warnings treated as
failures, verify the spaces, seeding (same seed → identical episode), the start/goal sampling
rules, and scripted episodes: driving straight at a goal 0.6 m ahead must terminate with success,
driving into a wall must terminate with a collision, standing still must be truncated exactly at
`max_episode_steps`, and an action of 7.0 must behave like 1.0.

Then train PPO on YOUR environment:

```bash
python 17-reinforcement-learning/code/train_go_to_goal.py --exercise student
```

## Hints

* `terminated` and `truncated` must be Python `bool`s and never both `True`.
* Only `self.np_random` may produce random numbers; `np.random.default_rng()` without a seed breaks reproducibility and `check_env` notices.
* A collision can happen in any of the 5 substeps; remember it even if a later substep clears `sim.collided`.
