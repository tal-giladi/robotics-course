"""bandits.py — returns, discounting, value functions and exploration (lesson 17.02).

    py 17-reinforcement-learning/code/bandits.py            # all three parts
    py 17-reinforcement-learning/code/bandits.py --part 3   # just the bandit experiment

Part 1: the discounted return of a reward sequence.
Part 2: the value of a policy in the corridor MDP (17.01), two ways: averaging sampled returns
        (Monte Carlo) and solving the Bellman expectation equations exactly (a linear system).
Part 3: a 4-armed bandit — karmel choosing between 4 docking approaches with unknown success
        probabilities — epsilon-greedy vs greedy vs optimistic initial values vs UCB.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from corridor_mdp import LEFT, RIGHT, CorridorEnv, CorridorMDP, run_episode  # noqa: E402

DOCKING_SUCCESS = np.array([0.55, 0.70, 0.62, 0.40])  # true, hidden from the agent


def discounted_return(rewards: list[float], gamma: float) -> float:
    """G_0 = r_1 + gamma r_2 + gamma^2 r_3 + ..."""
    return float(sum(gamma**k * r for k, r in enumerate(rewards)))


def policy_value_exact(mdp: CorridorMDP, policy_probs: np.ndarray) -> np.ndarray:
    """Solve V = r_pi + gamma P_pi V for every cell. ``policy_probs[s] = [p(LEFT), p(RIGHT)]``."""
    n = mdp.n_cells
    P = np.zeros((n, n))
    r = np.zeros(n)
    for s in range(n):
        if mdp.terminal(s):
            continue  # V(terminal) = 0
        for a in (LEFT, RIGHT):
            for prob, s2, reward, done in mdp.transitions(s, a):
                p = policy_probs[s, a] * prob
                r[s] += p * reward
                if not done:
                    P[s, s2] += p
    return np.linalg.solve(np.eye(n) - mdp.gamma * P, r)


def bandit_experiment(
    probs: np.ndarray, epsilon: float, steps: int = 1000, runs: int = 2000, q0: float = 0.0,
    ucb_c: float | None = None, seed: int = 0,
) -> dict[str, float]:
    """Vectorized over ``runs`` independent robots. Rewards are 1 (docked) or 0 (failed)."""
    rng = np.random.default_rng(seed)
    k = len(probs)
    Q = np.full((runs, k), q0, dtype=float)
    N = np.zeros((runs, k))
    rows = np.arange(runs)
    best = int(np.argmax(probs))
    rewards = np.zeros((runs, steps))
    optimal = np.zeros((runs, steps), dtype=bool)
    for t in range(steps):
        if ucb_c is not None:
            bonus = ucb_c * np.sqrt(np.log(t + 1) / np.maximum(N, 1e-9))
            scores = np.where(N == 0, np.inf, Q + bonus)
            greedy = np.argmax(scores, axis=1)
        else:
            noise = rng.random((runs, k)) * 1e-6  # random tie-breaking
            greedy = np.argmax(Q + noise, axis=1)
        explore = rng.random(runs) < epsilon
        actions = np.where(explore, rng.integers(k, size=runs), greedy)
        r = (rng.random(runs) < probs[actions]).astype(float)
        N[rows, actions] += 1
        Q[rows, actions] += (r - Q[rows, actions]) / N[rows, actions]  # incremental mean
        rewards[:, t] = r
        optimal[:, t] = actions == best
    return {
        "reward_last100": float(rewards[:, -100:].mean()),
        "optimal_last100": float(optimal[:, -100:].mean()),
        "total_reward": float(rewards.sum(axis=1).mean()),
        "regret": float(steps * probs[best] - rewards.sum(axis=1).mean()),
    }


def part1() -> None:
    rewards = [-1.0, -1.0, -1.0, 10.0]
    print("Part 1 - discounted return of rewards", rewards)
    for gamma in (1.0, 0.9, 0.5, 0.0):
        horizon = "inf" if gamma == 1.0 else f"{1 / (1 - gamma):.0f}"
        print(f"  gamma={gamma:<4}  G = {discounted_return(rewards, gamma):7.3f}   effective horizon 1/(1-gamma) = {horizon}")


def part2() -> None:
    mdp = CorridorMDP()
    print("\nPart 2 - V(s) in the corridor, gamma = 0.9")
    right = np.tile([0.0, 1.0], (mdp.n_cells, 1))
    uniform = np.full((mdp.n_cells, 2), 0.5)
    env = CorridorEnv(mdp)
    for name, probs, policy in (
        ("always RIGHT", right, lambda s, rng: RIGHT),
        ("uniform random", uniform, lambda s, rng: int(rng.integers(2))),
    ):
        V = policy_value_exact(mdp, probs)
        mc = np.mean([run_episode(env, policy, seed)[1] for seed in range(5000)])
        print(f"  {name:<15} exact V = {np.round(V, 3)}   Monte Carlo V(start=2) = {mc:.3f} (5000 episodes)")


def part3() -> None:
    print(f"\nPart 3 - 4 docking approaches, true success rates {DOCKING_SUCCESS} (best = approach 1)")
    print("  2000 simulated robots x 1000 docking attempts each, seed 0")
    print(f"  {'strategy':<32} {'success, last 100':>18} {'picks best, last 100':>21} {'regret':>8}")
    configs = [
        ("greedy (epsilon = 0)", dict(epsilon=0.0)),
        ("epsilon-greedy, epsilon = 0.01", dict(epsilon=0.01)),
        ("epsilon-greedy, epsilon = 0.1", dict(epsilon=0.1)),
        ("epsilon-greedy, epsilon = 0.3", dict(epsilon=0.3)),
        ("greedy, optimistic Q0 = 1.0", dict(epsilon=0.0, q0=1.0)),
        ("UCB, c = 1", dict(epsilon=0.0, ucb_c=1.0)),
    ]
    for name, kwargs in configs:
        res = bandit_experiment(DOCKING_SUCCESS, **kwargs)
        print(f"  {name:<32} {res['reward_last100']:>18.3f} {res['optimal_last100']:>21.3f} {res['regret']:>8.1f}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--part", type=int, choices=[1, 2, 3])
    args = ap.parse_args()
    for n, fn in ((1, part1), (2, part2), (3, part3)):
        if args.part in (None, n):
            fn()


if __name__ == "__main__":
    main()
