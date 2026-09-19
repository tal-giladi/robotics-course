"""17.04 — reference solution. Read it after you have tried ``student.py`` yourself."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Transition:
    """One environment step, as DQN stores it."""

    obs: np.ndarray
    action: int
    reward: float
    next_obs: np.ndarray
    terminated: bool


@dataclass
class ReplayBuffer:
    """A fixed-capacity circular buffer of transitions."""

    capacity: int
    _items: list[Transition] = field(default_factory=list)
    _next: int = 0

    def __len__(self) -> int:
        return len(self._items)

    def add(self, transition: Transition) -> None:
        if len(self._items) < self.capacity:
            self._items.append(transition)
        else:
            self._items[self._next] = transition
        self._next = (self._next + 1) % self.capacity

    def all(self) -> list[Transition]:
        if len(self._items) < self.capacity:
            return list(self._items)
        start = self._next % self.capacity  # the oldest slot
        return self._items[start:] + self._items[:start]

    def sample(self, batch_size: int, rng: np.random.Generator) -> dict[str, np.ndarray]:
        if batch_size > len(self._items):
            raise ValueError(f"batch_size {batch_size} > {len(self._items)} stored transitions")
        idx = rng.choice(len(self._items), size=batch_size, replace=False)
        batch = [self._items[i] for i in idx]
        return {
            "obs": np.asarray([t.obs for t in batch], dtype=np.float32),
            "action": np.asarray([t.action for t in batch], dtype=np.int64),
            "reward": np.asarray([t.reward for t in batch], dtype=np.float32),
            "next_obs": np.asarray([t.next_obs for t in batch], dtype=np.float32),
            "terminated": np.asarray([t.terminated for t in batch], dtype=np.float32),
        }


def epsilon_at(step: int, total_steps: int, eps_start: float = 1.0, eps_end: float = 0.05,
               fraction: float = 0.4) -> float:
    return float(max(eps_end, eps_start - (eps_start - eps_end) * step / (fraction * total_steps)))


def td_target(reward: np.ndarray, terminated: np.ndarray, next_q: np.ndarray, gamma: float) -> np.ndarray:
    reward = np.asarray(reward, dtype=float)
    mask = 1.0 - np.asarray(terminated, dtype=float)
    return reward + gamma * mask * np.max(np.asarray(next_q, dtype=float), axis=1)


def double_dqn_target(reward: np.ndarray, terminated: np.ndarray, next_q_online: np.ndarray,
                      next_q_target: np.ndarray, gamma: float) -> np.ndarray:
    reward = np.asarray(reward, dtype=float)
    mask = 1.0 - np.asarray(terminated, dtype=float)
    best = np.argmax(np.asarray(next_q_online, dtype=float), axis=1)
    chosen = np.take_along_axis(np.asarray(next_q_target, dtype=float), best[:, None], axis=1)[:, 0]
    return reward + gamma * mask * chosen
