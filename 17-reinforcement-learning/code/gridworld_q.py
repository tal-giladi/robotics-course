"""gridworld_q.py — train tabular Q-learning on the 17.03 gridworld with YOUR exercise code.

    py 17-reinforcement-learning/code/gridworld_q.py                 # uses labs/exercises/17.03/student.py
    py 17-reinforcement-learning/code/gridworld_q.py --solution      # the reference implementation
    py 17-reinforcement-learning/code/gridworld_q.py --solution --slip 0.2 --episodes 3000

Prints the learning curve (mean return per 50 episodes), the greedy policy as arrows, the learned
V(s) = max_a Q(s, a), and the optimal values from value iteration for comparison.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load_exercise(solution: bool) -> ModuleType:
    kind = "solution" if solution else "student"
    path = ROOT / "labs" / "exercises" / "17.03" / f"{kind}.py"
    spec = importlib.util.spec_from_file_location(f"ex_17_03_{kind}", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def train(ex: ModuleType, env, episodes: int, alpha: float, gamma: float, epsilon: float, seed: int):
    """Plain Q-learning with a constant epsilon. Returns (Q, list of episode returns)."""
    rng = np.random.default_rng(seed)
    Q = np.zeros((env.n_states, env.n_actions))
    returns = []
    for _ in range(episodes):
        s = env.reset()
        total = 0.0
        while True:
            a = ex.epsilon_greedy(Q, s, epsilon, rng)
            s2, r, terminated, truncated = env.step(a, rng)
            ex.q_learning_update(Q, s, a, r, s2, terminated, alpha, gamma)
            total += r
            s = s2
            if terminated or truncated:
                break
        returns.append(total)
    return Q, returns


def value_iteration(env, gamma: float, sweeps: int = 200) -> np.ndarray:
    """Optimal Q* using the known model (handles slip by averaging the three possible moves)."""
    Q = np.zeros((env.n_states, env.n_actions))
    for _ in range(sweeps):
        for s in range(env.n_states):
            if env.is_terminal(s) or env.cell(s) == "#":
                continue
            for a in range(env.n_actions):
                outcomes = [(1.0 - env.slip, a), (env.slip / 2, (a + 1) % 4), (env.slip / 2, (a + 3) % 4)]
                q = 0.0
                for p, move in outcomes:
                    if p == 0:
                        continue
                    s2, r, done = env.transition(s, move)
                    q += p * (r + (0.0 if done else gamma * Q[s2].max()))
                Q[s, a] = q
    return Q


def greedy_rollout(ex: ModuleType, env, Q: np.ndarray, episodes: int, seed: int) -> tuple[float, float]:
    """Mean return and fraction of episodes that reached G, acting greedily."""
    rng = np.random.default_rng(seed)
    returns, reached = [], 0
    for _ in range(episodes):
        s, total = env.reset(), 0.0
        while True:
            s, r, terminated, truncated = env.step(ex.greedy_action(Q, s), rng)
            total += r
            if terminated or truncated:
                reached += env.cell(s) == "G"
                break
        returns.append(total)
    return float(np.mean(returns)), reached / episodes


def show_values(env, V: np.ndarray) -> str:
    lines = []
    for r in range(env.n_rows):
        cells = []
        for c in range(env.n_cols):
            s = r * env.n_cols + c
            ch = env.cell(s)
            cells.append(f"{ch:>6}" if ch in "#GX" else f"{V[s]:6.2f}")
        lines.append(" ".join(cells))
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--solution", action="store_true")
    ap.add_argument("--episodes", type=int, default=500)
    ap.add_argument("--alpha", type=float, default=0.5)
    ap.add_argument("--gamma", type=float, default=0.9)
    ap.add_argument("--epsilon", type=float, default=0.1)
    ap.add_argument("--slip", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    ex = load_exercise(args.solution)
    env = ex.GridWorld(slip=args.slip)
    Q, returns = train(ex, env, args.episodes, args.alpha, args.gamma, args.epsilon, args.seed)
    print(f"Q-learning: {args.episodes} episodes, alpha={args.alpha}, gamma={args.gamma}, "
          f"epsilon={args.epsilon}, slip={args.slip}, seed={args.seed}")
    block = max(1, args.episodes // 10)
    print("mean return per block of", block, "episodes:",
          " ".join(f"{np.mean(returns[i:i + block]):.1f}" for i in range(0, args.episodes, block)))
    print("\ngreedy policy:")
    print(ex.GridWorld.render_policy(env, ex.greedy_policy(Q)))
    print("\nlearned V(s) = max_a Q(s, a):")
    print(show_values(env, Q.max(axis=1)))
    Q_star = value_iteration(env, args.gamma)
    print("\noptimal V*(s) from value iteration (uses the model, which Q-learning never sees):")
    print(show_values(env, Q_star.max(axis=1)))
    print("optimal policy:")
    print(env.render_policy(np.argmax(Q_star, axis=1)))
    mean_return, reached = greedy_rollout(ex, env, Q, 1000, seed=123)
    print(f"\ngreedy policy over 1000 test episodes: mean return {mean_return:.2f}, reached the charger {reached:.1%}")
    opt_return, opt_reached = greedy_rollout(ex, env, Q_star, 1000, seed=123)
    print(f"optimal policy, same 1000 episodes:   mean return {opt_return:.2f}, reached the charger {opt_reached:.1%}")


if __name__ == "__main__":
    main()
