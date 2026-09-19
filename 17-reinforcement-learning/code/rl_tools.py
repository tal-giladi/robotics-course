"""rl_tools.py — shared helpers for module 17: read Monitor logs, plot learning curves, evaluate.

SB3's ``Monitor`` wrapper writes one ``*.monitor.csv`` per environment: a JSON header line, then
``r,l,t[,extra keys]`` per finished episode (return, length, wall-clock seconds since start).
We read them with the standard library (pandas is optional in SB3 >= 2.9).
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np


def read_monitor_dir(folder: str | Path) -> dict[str, np.ndarray]:
    """All episodes of all ``*.monitor.csv`` files in ``folder``, sorted by finish time.

    Returns arrays ``r`` (return), ``l`` (length), ``t`` (seconds), ``steps`` (cumulative env steps
    over all envs at the end of each episode) and ``is_success`` if it was logged.
    """
    rows: list[dict[str, float]] = []
    for path in sorted(Path(folder).glob("*monitor.csv")):
        with path.open(encoding="utf-8") as f:
            f.readline()  # '#{"t_start": ..., "env_id": ...}'
            for row in csv.DictReader(f):
                rows.append({k: _to_float(v) for k, v in row.items()})
    rows.sort(key=lambda row: row["t"])
    if not rows:
        raise FileNotFoundError(f"no *.monitor.csv episodes in {folder}")
    out = {k: np.array([row[k] for row in rows]) for k in rows[0]}
    out["steps"] = np.cumsum(out["l"])
    return out


def _to_float(value: str) -> float:
    if value in ("True", "False"):
        return float(value == "True")
    return float(value)


def moving_average(x: np.ndarray, window: int) -> np.ndarray:
    """Trailing mean over ``window`` episodes (shorter at the start)."""
    c = np.cumsum(np.insert(np.asarray(x, dtype=float), 0, 0.0))
    idx = np.arange(1, len(x) + 1)
    lo = np.maximum(0, idx - window)
    return (c[idx] - c[lo]) / (idx - lo)


def curve_table(log: dict[str, np.ndarray], checkpoints: list[int], window: int = 100) -> list[dict[str, float]]:
    """Mean return / length / success over the last ``window`` episodes before each step checkpoint."""
    table = []
    for step in checkpoints:
        mask = log["steps"] <= step
        n = int(mask.sum())
        if n == 0:
            continue
        sl = slice(max(0, n - window), n)
        entry = {"steps": step, "episodes": n, "return": float(log["r"][sl].mean()), "length": float(log["l"][sl].mean())}
        if "is_success" in log:
            entry["success"] = float(log["is_success"][sl].mean())
        table.append(entry)
    return table


def print_curve_table(table: list[dict[str, float]]) -> None:
    has_success = any("success" in row for row in table)
    print(f"{'env steps':>10} {'episodes':>9} {'return':>8} {'length':>7}" + (f" {'success':>8}" if has_success else ""))
    for row in table:
        line = f"{row['steps']:>10d} {row['episodes']:>9d} {row['return']:>8.2f} {row['length']:>7.1f}"
        if has_success:
            line += f" {row.get('success', float('nan')):>8.2f}"
        print(line)


def plot_curves(runs: dict[str, str | Path], out_png: str | Path, window: int = 100, title: str = "") -> None:
    """Learning curves (return and, if logged, success rate vs environment steps) for several runs."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    logs = {name: read_monitor_dir(folder) for name, folder in runs.items()}
    with_success = any("is_success" in log for log in logs.values())
    fig, axes = plt.subplots(1, 2 if with_success else 1, figsize=(11 if with_success else 6, 4), squeeze=False)
    for name, log in logs.items():
        axes[0, 0].plot(log["steps"], moving_average(log["r"], window), label=name)
        if with_success and "is_success" in log:
            axes[0, 1].plot(log["steps"], moving_average(log["is_success"], window), label=name)
    axes[0, 0].set(xlabel="environment steps", ylabel=f"episode return (mean of last {window})", title=title)
    axes[0, 0].grid(alpha=0.3)
    axes[0, 0].legend()
    if with_success:
        axes[0, 1].set(xlabel="environment steps", ylabel="success rate", ylim=(-0.02, 1.02))
        axes[0, 1].grid(alpha=0.3)
    fig.tight_layout()
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def evaluate(
    policy: Callable[[np.ndarray], np.ndarray],
    make_env: Callable[[], Any],
    episodes: int = 100,
    seed: int = 10_000,
) -> dict[str, float]:
    """Run ``episodes`` episodes (episode i uses seed ``seed + i``) and summarize them.

    Returns success rate, collision rate, timeout rate, mean return, mean steps of the successful
    episodes, and mean spin (total |heading change|, radians) per episode.
    """
    env = make_env()
    returns, success, collided, steps_ok, spins = [], [], [], [], []
    for i in range(episodes):
        obs, info = env.reset(seed=seed + i)
        total, steps = 0.0, 0
        while True:
            obs, reward, terminated, truncated, info = env.step(policy(obs))
            total += reward
            steps += 1
            if terminated or truncated:
                break
        returns.append(total)
        success.append(bool(info.get("is_success", False)))
        collided.append(bool(info.get("collided", False)))
        spins.append(float(info.get("spin_rad", math.nan)))
        if success[-1]:
            steps_ok.append(steps)
    env.close()
    n = float(episodes)
    return {
        "episodes": episodes,
        "success_rate": sum(success) / n,
        "collision_rate": sum(collided) / n,
        "timeout_rate": 1.0 - (sum(success) + sum(collided)) / n,
        "mean_return": float(np.mean(returns)),
        "mean_steps_to_goal": float(np.mean(steps_ok)) if steps_ok else math.nan,
        "mean_spin_rad": float(np.mean(spins)),
    }


def save_json(data: Any, path: str | Path) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")
