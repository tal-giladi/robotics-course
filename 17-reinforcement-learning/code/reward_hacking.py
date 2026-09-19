"""reward_hacking.py — find, measure and fix a reward hack in the karmel environment (lesson 17.08).

    py 17-reinforcement-learning/code/reward_hacking.py scripted     # no training: score 3 fixed behaviours
    py 17-reinforcement-learning/code/reward_hacking.py compare      # needs the two trained models below
    py 17-reinforcement-learning/code/reward_hacking.py compare --episodes 20

Train the models first (a few minutes each):
    py 17-reinforcement-learning/code/train_go_to_goal.py --reward naive_heading
    py 17-reinforcement-learning/code/train_go_to_goal.py --reward heading_fixed

"scripted" is the cheapest hack detector there is: before training anything, compute what a
degenerate behaviour (spin in place, stand still) would earn under your reward and compare it with
what the behaviour you actually want earns. If the degenerate one wins, the optimizer will find it.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import rl_tools  # noqa: E402
from karmel_env import KarmelGoToGoalEnv, heading_controller  # noqa: E402

BEHAVIOURS = {
    "go to goal (hand-written)": heading_controller,
    "spin in place": lambda obs: np.array([0.0, 1.0], dtype=np.float32),
    "stand still": lambda obs: np.zeros(2, dtype=np.float32),
}


def discounted(rewards: list[float], gamma: float = 0.99) -> float:
    return float(sum(gamma**k * r for k, r in enumerate(rewards)))


def scripted(episodes: int) -> None:
    print(f"mean over {episodes} episodes (seeds 10000..): undiscounted return / discounted (gamma 0.99) / success")
    print(f"{'behaviour':<28}" + "".join(f"{mode:>26}" for mode in ("progress", "naive_heading", "heading_fixed")))
    for name, policy in BEHAVIOURS.items():
        cells = []
        for mode in ("progress", "naive_heading", "heading_fixed"):
            env = KarmelGoToGoalEnv(reward_mode=mode)
            totals, discs, wins = [], [], []
            for i in range(episodes):
                obs, _ = env.reset(seed=10_000 + i)
                rewards = []
                while True:
                    obs, r, terminated, truncated, info = env.step(policy(obs))
                    rewards.append(r)
                    if terminated or truncated:
                        break
                totals.append(sum(rewards))
                discs.append(discounted(rewards))
                wins.append(info["is_success"])
            cells.append(f"{np.mean(totals):8.1f} / {np.mean(discs):6.1f} / {np.mean(wins):4.2f}")
        print(f"{name:<28}" + "".join(f"{c:>26}" for c in cells))


def compare(episodes: int) -> None:
    from stable_baselines3 import PPO

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = {
        "naive_heading": HERE / "runs" / "ppo_goal_naive_heading_s0" / "model.zip",
        "heading_fixed": HERE / "runs" / "ppo_goal_heading_fixed_s0" / "model.zip",
    }
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    for mode, path in runs.items():
        if not path.exists():
            sys.exit(f"missing {path}: train it first (see the module docstring)")
        model = PPO.load(path, device="cpu")

        def policy(obs, model=model):
            return model.predict(obs, deterministic=True)[0]

        res = rl_tools.evaluate(policy, lambda mode=mode: KarmelGoToGoalEnv(reward_mode=mode), episodes=episodes)
        print(f"{mode:<14} " + ", ".join(f"{k}={v:.2f}" for k, v in res.items() if k != "episodes"))

        env = KarmelGoToGoalEnv(reward_mode=mode)
        obs, _ = env.reset(seed=10_003)
        xs, ys, headings, cumulative, total = [env.sim.pose.x], [env.sim.pose.y], [env.sim.pose.theta], [0.0], 0.0
        while True:
            obs, r, terminated, truncated, info = env.step(policy(obs))
            total += r
            xs.append(env.sim.pose.x)
            ys.append(env.sim.pose.y)
            headings.append(env.sim.pose.theta)
            cumulative.append(total)
            if terminated or truncated:
                break
        t = np.arange(len(xs)) * env.control_dt
        axes[0].plot(xs, ys, label=f"{mode} ({'reached' if info['is_success'] else 'did not reach'} goal)")
        axes[0].plot(*env.goal, "k*", markersize=14)
        axes[1].plot(t, np.unwrap(headings), label=mode)
        axes[2].plot(t, cumulative, label=mode)
    axes[0].set(xlim=(0, 4), ylim=(0, 4), aspect="equal", title="path, episode seed 10003", xlabel="x [m]", ylabel="y [m]")
    axes[1].set(title="unwrapped heading", xlabel="time [s]", ylabel="rad")
    axes[2].set(title="cumulative reward", xlabel="time [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    out = HERE / "runs" / "reward_hacking.png"
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"plot: {out}  ({math.pi * 2:.2f} rad = one full turn)")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["scripted", "compare"])
    ap.add_argument("--episodes", type=int, default=None)
    args = ap.parse_args()
    if args.what == "scripted":
        scripted(args.episodes or 20)
    else:
        compare(args.episodes or 100)


if __name__ == "__main__":
    main()
