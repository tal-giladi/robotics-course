"""corridor_mdp.py — a robot task written down as an MDP, and the agent–environment loop (17.01).

karmel is in a 6-cell corridor. Cell 5 is the charging dock (+10, episode ends). Cell 0 is the top
of a staircase (-10, episode ends). Every move costs -1 (time, battery). The floor is slippery:
with probability 0.2 a command does nothing.

    cell:   0        1   2   3   4   5
          [STAIRS]   .   S   .   .  [DOCK]

    py 17-reinforcement-learning/code/corridor_mdp.py
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

LEFT, RIGHT = 0, 1


@dataclass(frozen=True)
class CorridorMDP:
    """The five parts of an MDP: states S, actions A, transitions P, rewards R, discount gamma."""

    n_cells: int = 6
    start: int = 2
    slip: float = 0.2  # probability that a command has no effect
    step_reward: float = -1.0
    dock_reward: float = 10.0
    stairs_reward: float = -10.0
    gamma: float = 0.9

    def terminal(self, s: int) -> bool:
        return s in (0, self.n_cells - 1)

    def transitions(self, s: int, a: int) -> list[tuple[float, int, float, bool]]:
        """P(s', r | s, a) as a list of (probability, next_state, reward, terminated)."""
        target = s - 1 if a == LEFT else s + 1
        outcomes = []
        for prob, s2 in ((1.0 - self.slip, target), (self.slip, s)):
            if s2 == 0:
                outcomes.append((prob, s2, self.stairs_reward, True))
            elif s2 == self.n_cells - 1:
                outcomes.append((prob, s2, self.dock_reward, True))
            else:
                outcomes.append((prob, s2, self.step_reward, False))
        return outcomes


class CorridorEnv:
    """The same MDP as a black box with the Gymnasium-style reset/step API. The agent never sees P."""

    def __init__(self, mdp: CorridorMDP, max_steps: int = 50) -> None:
        self.mdp, self.max_steps = mdp, max_steps
        self.rng = np.random.default_rng()

    def reset(self, seed: int | None = None) -> tuple[int, dict]:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.s, self.t = self.mdp.start, 0
        return self.s, {}

    def step(self, a: int) -> tuple[int, float, bool, bool, dict]:
        outcomes = self.mdp.transitions(self.s, a)
        i = self.rng.choice(len(outcomes), p=[o[0] for o in outcomes])
        _, self.s, reward, terminated = outcomes[i]
        self.t += 1
        truncated = not terminated and self.t >= self.max_steps
        return self.s, reward, terminated, truncated, {}


def run_episode(env: CorridorEnv, policy, seed: int) -> tuple[float, float, int]:
    """One episode: returns (undiscounted return, discounted return, steps)."""
    s, _ = env.reset(seed=seed)
    total, discounted, k = 0.0, 0.0, 0
    while True:
        a = policy(s, env.rng)
        s, r, terminated, truncated, _ = env.step(a)
        total += r
        discounted += env.mdp.gamma**k * r
        k += 1
        if terminated or truncated:
            return total, discounted, k


def main() -> None:
    mdp = CorridorMDP()
    print("P(s', r | s=2, a) in the corridor:")
    for a, name in ((LEFT, "LEFT "), (RIGHT, "RIGHT")):
        print(f"  a={name}:", ", ".join(f"p={p:.1f} -> s'={s2} r={r:+.0f}{' (end)' if d else ''}"
                                        for p, s2, r, d in mdp.transitions(2, a)))
    policies = {
        "always RIGHT": lambda s, rng: RIGHT,
        "always LEFT": lambda s, rng: LEFT,
        "uniform random": lambda s, rng: int(rng.integers(2)),
    }
    env = CorridorEnv(mdp)
    print(f"\n{'policy':<15} {'mean return':>12} {'mean disc. return':>18} {'mean steps':>11}   (5000 episodes, seeds 0..4999)")
    for name, policy in policies.items():
        results = np.array([run_episode(env, policy, seed) for seed in range(5000)])
        print(f"{name:<15} {results[:, 0].mean():>12.2f} {results[:, 1].mean():>18.2f} {results[:, 2].mean():>11.2f}")


if __name__ == "__main__":
    main()
