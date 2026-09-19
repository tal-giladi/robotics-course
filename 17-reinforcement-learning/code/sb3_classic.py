"""sb3_classic.py — A2C, PPO and SAC with Stable-Baselines3 on Gymnasium classics (lesson 17.06).

    py 17-reinforcement-learning/code/sb3_classic.py cartpole            # A2C vs PPO, 50k steps each
    py 17-reinforcement-learning/code/sb3_classic.py pendulum            # PPO vs SAC, 20k steps each
    py 17-reinforcement-learning/code/sb3_classic.py cartpole --seeds 0 1 2
    py 17-reinforcement-learning/code/sb3_classic.py pendulum --steps 100000   # the "longer run"

CartPole-v1: discrete actions (push left/right), reward +1 per step upright, max 500.
Pendulum-v1: continuous torque in [-2, 2], reward in about [-16, 0] per step, 200 steps;
             a good swing-up-and-hold policy scores about -150 per episode, random about -1200.

Every run writes Monitor CSVs to runs/sb3/<env>_<algo>_s<seed>/ and a combined learning-curve
PNG to runs/sb3/<env>_curves.png.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import rl_tools  # noqa: E402

SETUPS = {
    "cartpole": {"env_id": "CartPole-v1", "algos": ["a2c", "ppo"], "steps": 50_000, "n_envs": 4},
    "pendulum": {"env_id": "Pendulum-v1", "algos": ["ppo", "sac"], "steps": 20_000, "n_envs": 1},
}


def make_model(algo: str, venv, seed: int):
    from stable_baselines3 import A2C, PPO, SAC

    if algo == "a2c":
        return A2C("MlpPolicy", venv, seed=seed, device="cpu", verbose=0)
    if algo == "ppo":
        return PPO("MlpPolicy", venv, seed=seed, device="cpu", verbose=0, n_steps=512, batch_size=128)
    if algo == "sac":
        return SAC("MlpPolicy", venv, seed=seed, device="cpu", verbose=0, learning_starts=1000)
    raise ValueError(algo)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("task", choices=sorted(SETUPS))
    ap.add_argument("--steps", type=int)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0])
    args = ap.parse_args()

    import torch
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.evaluation import evaluate_policy

    torch.set_num_threads(2)
    setup = SETUPS[args.task]
    steps = args.steps or setup["steps"]
    runs = {}
    for algo in setup["algos"]:
        for seed in args.seeds:
            name = f"{algo}_s{seed}"
            out = HERE / "runs" / "sb3" / f"{args.task}_{name}"
            out.mkdir(parents=True, exist_ok=True)
            for old in out.glob("*monitor.csv"):
                old.unlink()
            n_envs = setup["n_envs"] if algo != "sac" else 1
            venv = make_vec_env(setup["env_id"], n_envs=n_envs, seed=seed, monitor_dir=str(out))
            model = make_model(algo, venv, seed)
            t0 = time.perf_counter()
            model.learn(total_timesteps=steps)
            seconds = time.perf_counter() - t0
            eval_env = make_vec_env(setup["env_id"], n_envs=1, seed=10_000 + seed)
            mean, std = evaluate_policy(model, eval_env, n_eval_episodes=20, deterministic=True)
            print(f"\n{setup['env_id']} {algo.upper()} seed {seed}: {steps} steps in {seconds:.0f} s; "
                  f"deterministic evaluation over 20 episodes: {mean:.1f} +/- {std:.1f}")
            table = rl_tools.curve_table(rl_tools.read_monitor_dir(out), [int(steps * f) for f in (0.1, 0.25, 0.5, 0.75, 1.0)], window=20)
            rl_tools.print_curve_table(table)
            model.save(out / "model.zip")
            venv.close()
            runs[name] = out
    png = HERE / "runs" / "sb3" / f"{args.task}_curves.png"
    rl_tools.plot_curves(runs, png, window=20, title=setup["env_id"])
    print(f"\nlearning curves: {png}")


if __name__ == "__main__":
    main()
