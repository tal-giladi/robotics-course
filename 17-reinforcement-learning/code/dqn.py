"""dqn.py — Deep Q-Network from scratch in PyTorch (lesson 17.04).

    py 17-reinforcement-learning/code/dqn.py                       # random-goal gridworld, ~30k steps
    py 17-reinforcement-learning/code/dqn.py --no-target           # ablation: no target network
    py 17-reinforcement-learning/code/dqn.py --no-replay           # ablation: learn only from the newest transitions
    py 17-reinforcement-learning/code/dqn.py --env cartpole --steps 60000    # the "longer run"

The gridworld: a 7 x 7 room with an internal wall. Every episode the robot AND the charger are
placed at random. The observation is a 2-channel 7 x 7 "image" (robot plane, goal plane),
flattened to 98 numbers. A Q-table would need 49 x 48 = 2352 rows; the network shares what it
learns about "move toward the goal" across all of them.
"""

from __future__ import annotations

import argparse
import random
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import torch
from torch import nn

WALLS = {(3, 1), (3, 2), (3, 3), (3, 4)}  # (row, col): a wall with gaps at both ends
SIZE = 7
MOVES = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # up, right, down, left


class RandomGoalGrid:
    """Gymnasium-style API: reset(seed) -> (obs, info); step(a) -> (obs, r, terminated, truncated, info)."""

    n_actions = 4
    obs_size = 2 * SIZE * SIZE

    def __init__(self, max_steps: int = 50) -> None:
        self.max_steps = max_steps
        self.free = [(r, c) for r in range(SIZE) for c in range(SIZE) if (r, c) not in WALLS]
        self.rng = np.random.default_rng()

    def reset(self, seed: int | None = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        i, j = self.rng.choice(len(self.free), size=2, replace=False)
        self.pos, self.goal, self.t = self.free[i], self.free[j], 0
        return self._obs(), {}

    def _obs(self) -> np.ndarray:
        img = np.zeros((2, SIZE, SIZE), dtype=np.float32)
        img[0][self.pos] = 1.0
        img[1][self.goal] = 1.0
        return img.reshape(-1)

    def step(self, action: int):
        dr, dc = MOVES[action]
        r, c = self.pos[0] + dr, self.pos[1] + dc
        if 0 <= r < SIZE and 0 <= c < SIZE and (r, c) not in WALLS:
            self.pos = (r, c)
        self.t += 1
        reached = self.pos == self.goal
        reward = 10.0 if reached else -1.0
        truncated = not reached and self.t >= self.max_steps
        return self._obs(), reward, reached, truncated, {"is_success": reached}

    def shortest_path(self) -> int:
        """BFS distance from the current position to the goal (for judging the learned policy)."""
        frontier, seen, d = [self.pos], {self.pos}, 0
        while frontier:
            if self.goal in frontier:
                return d
            nxt = []
            for r, c in frontier:
                for dr, dc in MOVES:
                    p = (r + dr, c + dc)
                    if 0 <= p[0] < SIZE and 0 <= p[1] < SIZE and p not in WALLS and p not in seen:
                        seen.add(p)
                        nxt.append(p)
            frontier, d = nxt, d + 1
        raise RuntimeError("goal unreachable")


class GymCartPole:
    """CartPole-v1 through the same tiny interface."""

    n_actions = 2
    obs_size = 4

    def __init__(self) -> None:
        import gymnasium as gym

        self.env = gym.make("CartPole-v1")

    def reset(self, seed: int | None = None):
        return self.env.reset(seed=seed)

    def step(self, action: int):
        obs, r, term, trunc, info = self.env.step(action)
        return obs, float(r), term, trunc, info


@dataclass
class DQNConfig:
    steps: int = 30_000
    gamma: float = 0.95
    lr: float = 1e-3
    batch: int = 64
    buffer: int = 20_000
    warmup: int = 1_000
    target_every: int = 500  # hard-copy the online network into the target network
    eps_start: float = 1.0
    eps_end: float = 0.05
    eps_fraction: float = 0.4  # of the run spent decaying epsilon
    hidden: int = 128
    use_target: bool = True
    use_replay: bool = True


def make_net(n_in: int, n_out: int, hidden: int) -> nn.Module:
    return nn.Sequential(nn.Linear(n_in, hidden), nn.ReLU(), nn.Linear(hidden, hidden), nn.ReLU(), nn.Linear(hidden, n_out))


def train_dqn(env, cfg: DQNConfig, seed: int = 0, log_every: int = 3000):
    """Returns (online network, list of (step, mean return of last 50 episodes, success rate))."""
    torch.manual_seed(seed)
    random.seed(seed)
    rng = np.random.default_rng(seed)
    q = make_net(env.obs_size, env.n_actions, cfg.hidden)
    target = make_net(env.obs_size, env.n_actions, cfg.hidden)
    target.load_state_dict(q.state_dict())
    opt = torch.optim.Adam(q.parameters(), lr=cfg.lr)
    # Without replay we still need a batch: use only the most recent transitions (highly correlated).
    buffer: deque = deque(maxlen=cfg.buffer if cfg.use_replay else cfg.batch)
    obs, _ = env.reset(seed=seed)
    ep_return, returns, successes, curve = 0.0, deque(maxlen=50), deque(maxlen=50), []
    for step in range(1, cfg.steps + 1):
        eps = max(cfg.eps_end, cfg.eps_start - (cfg.eps_start - cfg.eps_end) * step / (cfg.eps_fraction * cfg.steps))
        if rng.random() < eps:
            action = int(rng.integers(env.n_actions))
        else:
            with torch.no_grad():
                action = int(q(torch.as_tensor(obs, dtype=torch.float32)).argmax())
        obs2, reward, terminated, truncated, info = env.step(action)
        buffer.append((obs, action, reward, obs2, float(terminated)))
        obs, ep_return = obs2, ep_return + reward
        if terminated or truncated:
            returns.append(ep_return)
            successes.append(float(info.get("is_success", 0.0)))
            obs, _ = env.reset()
            ep_return = 0.0

        if step >= cfg.warmup and len(buffer) >= cfg.batch:
            batch = random.sample(buffer, cfg.batch) if cfg.use_replay else list(buffer)
            o, a, r, o2, done = (torch.as_tensor(np.array(x), dtype=torch.float32) for x in zip(*batch, strict=True))
            with torch.no_grad():
                bootstrap_net = target if cfg.use_target else q
                y = r + cfg.gamma * (1.0 - done) * bootstrap_net(o2).max(dim=1).values  # TD target
            q_sa = q(o).gather(1, a.long().unsqueeze(1)).squeeze(1)
            loss = nn.functional.smooth_l1_loss(q_sa, y)  # Huber loss
            opt.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(q.parameters(), 10.0)
            opt.step()
            if cfg.use_target and step % cfg.target_every == 0:
                target.load_state_dict(q.state_dict())
        if step % log_every == 0 and returns:
            curve.append((step, float(np.mean(returns)), float(np.mean(successes))))
    return q, curve


def evaluate_grid(q: nn.Module, episodes: int = 300, seed: int = 10_000) -> dict[str, float]:
    env = RandomGoalGrid()
    success, ratio = 0, []
    for i in range(episodes):
        obs, _ = env.reset(seed=seed + i)
        optimal = env.shortest_path()
        for t in range(1, env.max_steps + 1):
            with torch.no_grad():
                action = int(q(torch.as_tensor(obs)).argmax())
            obs, _, terminated, truncated, _ = env.step(action)
            if terminated:
                success += 1
                ratio.append(t / optimal)
                break
            if truncated:
                break
    return {"success_rate": success / episodes, "steps_vs_shortest": float(np.mean(ratio)) if ratio else float("nan")}


def evaluate_cartpole(q: nn.Module, episodes: int = 20, seed: int = 10_000) -> dict[str, float]:
    env = GymCartPole()
    lengths = []
    for i in range(episodes):
        obs, _ = env.reset(seed=seed + i)
        for t in range(1, 501):
            with torch.no_grad():
                obs, _, term, trunc, _ = env.step(int(q(torch.as_tensor(obs, dtype=torch.float32)).argmax()))
            if term or trunc:
                break
        lengths.append(t)
    return {"mean_episode_length": float(np.mean(lengths)), "max_is_500": 500.0}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", choices=["grid", "cartpole"], default="grid")
    ap.add_argument("--steps", type=int)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-target", action="store_true")
    ap.add_argument("--no-replay", action="store_true")
    args = ap.parse_args()
    torch.set_num_threads(1)  # tiny networks: threading overhead costs more than it saves

    if args.env == "grid":
        env, cfg = RandomGoalGrid(), DQNConfig()
    else:
        env = GymCartPole()
        cfg = DQNConfig(steps=60_000, gamma=0.99, lr=5e-4, buffer=50_000, target_every=1000, eps_fraction=0.2)
    if args.steps:
        cfg.steps = args.steps
    cfg.use_target, cfg.use_replay = not args.no_target, not args.no_replay
    print(f"DQN on {args.env}: {cfg.steps} steps, target network={cfg.use_target}, replay={cfg.use_replay}, seed={args.seed}")
    t0 = time.perf_counter()
    q, curve = train_dqn(env, cfg, seed=args.seed, log_every=max(1000, cfg.steps // 10))
    print(f"trained in {time.perf_counter() - t0:.0f} s")
    print(f"{'step':>7} {'return (last 50 ep)':>20} {'success (last 50 ep)':>21}")
    for step, ret, succ in curve:
        print(f"{step:>7} {ret:>20.2f} {succ:>21.2f}")
    result = evaluate_grid(q) if args.env == "grid" else evaluate_cartpole(q)
    print("greedy evaluation on unseen seeds:", {k: round(v, 3) for k, v in result.items()})


if __name__ == "__main__":
    main()
