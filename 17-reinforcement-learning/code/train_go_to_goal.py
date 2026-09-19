"""train_go_to_goal.py — train karmel to drive to a goal with PPO or SAC (lessons 17.07–17.09).

    py 17-reinforcement-learning/code/train_go_to_goal.py                         # PPO, 60k steps, empty room
    py 17-reinforcement-learning/code/train_go_to_goal.py --obstacles 3 --steps 300000
    py 17-reinforcement-learning/code/train_go_to_goal.py --algo sac --steps 30000 --n-envs 4
    py 17-reinforcement-learning/code/train_go_to_goal.py --reward naive_heading   # lesson 17.08
    py 17-reinforcement-learning/code/train_go_to_goal.py --randomize             # lesson 17.09
    py 17-reinforcement-learning/code/train_go_to_goal.py --eval-only runs/ppo_goal/model.zip
    py 17-reinforcement-learning/code/train_go_to_goal.py --exercise student       # train YOUR 17.07 env

Writes to --out (default runs/<algo>_goal...): model.zip, monitor CSVs (one per env),
learning_curve.png, summary.json. Prints a learning-curve table and a 100-episode evaluation on
seeds 10000..10099, which training never sees.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import rl_tools  # noqa: E402
from karmel_env import KarmelGoToGoalEnv, heading_controller  # noqa: E402

ROOT = HERE.parent.parent


def exercise_env_class(kind: str):
    """``GoToGoalEnv`` from labs/exercises/17.07/<kind>.py."""
    path = ROOT / "labs" / "exercises" / "17.07" / f"{kind}.py"
    spec = importlib.util.spec_from_file_location(f"ex_17_07_{kind}", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.GoToGoalEnv


class EnvFactory:
    """A picklable ``() -> env`` (SubprocVecEnv on Windows/macOS starts fresh processes)."""

    def __init__(self, kwargs: dict[str, Any], exercise: str | None = None) -> None:
        self.kwargs, self.exercise = kwargs, exercise

    def __call__(self):
        if self.exercise:
            return exercise_env_class(self.exercise)()
        return KarmelGoToGoalEnv(**self.kwargs)


def build_model(algo: str, venv, seed: int, device: str = "cpu"):
    from stable_baselines3 import PPO, SAC

    if algo == "ppo":
        n_envs = venv.num_envs
        return PPO(
            "MlpPolicy", venv, seed=seed, device=device, verbose=0,
            n_steps=max(64, 2048 // n_envs),  # rollout = 2048 transitions in total
            batch_size=256, n_epochs=10, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
            clip_range=0.2, ent_coef=0.0, policy_kwargs={"net_arch": [64, 64]},
        )
    if algo == "sac":
        return SAC(
            "MlpPolicy", venv, seed=seed, device=device, verbose=0,
            buffer_size=200_000, learning_starts=2000, batch_size=256, learning_rate=3e-4,
            gamma=0.99, tau=0.005, train_freq=1, gradient_steps=1, policy_kwargs={"net_arch": [128, 128]},
        )
    raise ValueError(algo)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--algo", choices=["ppo", "sac"], default="ppo")
    ap.add_argument("--steps", type=int, default=60_000, help="total environment steps (all envs)")
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--vec", choices=["subproc", "dummy"], default="subproc")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--obstacles", type=int, default=0)
    ap.add_argument("--reward", default="progress", choices=["progress", "naive_heading", "heading_fixed", "sparse"])
    ap.add_argument("--randomize", action="store_true", help="domain randomization (17.09)")
    ap.add_argument("--max-episode-steps", type=int, default=200)
    ap.add_argument("--exercise", choices=["student", "solution"], help="train the 17.07 exercise env instead")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--eval-episodes", type=int, default=100)
    ap.add_argument("--eval-only", type=Path, help="load this model.zip and only evaluate")
    args = ap.parse_args()

    env_kwargs: dict[str, Any] = {
        "n_obstacles": args.obstacles, "reward_mode": args.reward, "randomize": args.randomize,
        "max_episode_steps": args.max_episode_steps,
    }
    factory = EnvFactory(env_kwargs, args.exercise)
    # Evaluate on the SAME task definition but always with the dense reward switched off from
    # the success metric's point of view: success/collision come from info, not from the reward.
    eval_factory = factory

    from stable_baselines3 import PPO, SAC

    if args.eval_only:
        algo_cls = SAC if "sac" in args.eval_only.as_posix().lower() else PPO
        model = algo_cls.load(args.eval_only, device="cpu")
    else:
        from stable_baselines3.common.env_util import make_vec_env
        from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

        name = f"{args.algo}_goal" + (f"_obs{args.obstacles}" if args.obstacles else "")
        name += "" if args.reward == "progress" else f"_{args.reward}"
        name += "_dr" if args.randomize else ""
        name += f"_{args.exercise}" if args.exercise else ""
        out = args.out or HERE / "runs" / f"{name}_s{args.seed}"
        out.mkdir(parents=True, exist_ok=True)
        for old in out.glob("*monitor.csv"):
            old.unlink()
        venv = make_vec_env(
            factory, n_envs=args.n_envs, seed=args.seed, monitor_dir=str(out),
            monitor_kwargs={"info_keywords": ("is_success",)},
            vec_env_cls=SubprocVecEnv if args.vec == "subproc" and args.n_envs > 1 else DummyVecEnv,
        )
        model = build_model(args.algo, venv, args.seed)
        print(f"training {args.algo.upper()} for {args.steps} steps on {args.n_envs} envs -> {out}")
        t0 = time.perf_counter()
        model.learn(total_timesteps=args.steps)
        train_s = time.perf_counter() - t0
        venv.close()
        model.save(out / "model.zip")
        log = rl_tools.read_monitor_dir(out)
        checkpoints = sorted({int(args.steps * f) for f in (0.1, 0.2, 0.3, 0.5, 0.7, 1.0)})
        table = rl_tools.curve_table(log, checkpoints)
        print(f"trained in {train_s:.0f} s wall clock ({args.steps / train_s:.0f} steps/s)")
        rl_tools.print_curve_table(table)
        rl_tools.plot_curves({name: out}, out / "learning_curve.png", title=f"{name}, seed {args.seed}")

    def policy(obs: np.ndarray) -> np.ndarray:
        action, _ = model.predict(obs, deterministic=True)
        return action

    t0 = time.perf_counter()
    result = rl_tools.evaluate(policy, eval_factory, episodes=args.eval_episodes)
    baseline = None if args.exercise else rl_tools.evaluate(heading_controller, eval_factory, episodes=args.eval_episodes)
    print(f"evaluation over {args.eval_episodes} unseen episodes ({time.perf_counter() - t0:.0f} s):")
    print("  learned policy :", json.dumps({k: round(v, 3) for k, v in result.items()}))
    if baseline:
        print("  hand-written   :", json.dumps({k: round(v, 3) for k, v in baseline.items()}))
    if not args.eval_only:
        rl_tools.save_json({"args": {k: str(v) for k, v in vars(args).items()}, "train_seconds": train_s,
                            "curve": table, "eval": result, "baseline": baseline}, out / "summary.json")


if __name__ == "__main__":
    main()
