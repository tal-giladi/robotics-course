"""17.04 — The three pieces of DQN, in numpy.

Fill in every ``TODO(student)``. Check with ``python course.py check 17.04``.

These are exactly the parts that `17-reinforcement-learning/code/dqn.py` writes inline inside its
training loop, pulled out so you can test them in milliseconds instead of waiting 3 minutes for a
training run:

* ``ReplayBuffer``      the circular store of transitions, sampled uniformly at random
* ``epsilon_at``        the linear exploration schedule
* ``td_target``         the batched TD target, with the terminal mask
* ``double_dqn_target`` the same target with action selection and evaluation split

No PyTorch here: the networks are represented by the arrays of Q-values they already produced,
which is all the target computation needs.
"""

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
    terminated: bool  # True only for a real terminal state, never for a time-limit truncation


@dataclass
class ReplayBuffer:
    """A fixed-capacity circular buffer of transitions.

    * ``add`` appends; when the buffer is full the OLDEST transition is overwritten.
    * ``len(buffer)`` is the number of stored transitions (never more than ``capacity``).
    * ``all()`` returns them oldest-first.
    * ``sample(batch_size, rng)`` returns a batch of stacked arrays, sampled uniformly WITHOUT
      replacement, drawing exactly one ``rng.choice(len(self), size=batch_size, replace=False)``.
    """

    capacity: int
    _items: list[Transition] = field(default_factory=list)
    _next: int = 0  # where the next transition goes once the buffer is full

    def __len__(self) -> int:
        return len(self._items)

    def add(self, transition: Transition) -> None:
        """Store one transition, overwriting the oldest one when full."""
        # TODO(student): append while there is room, otherwise overwrite position self._next
        #   and advance it (modulo capacity).
        raise NotImplementedError("ReplayBuffer.add")

    def all(self) -> list[Transition]:
        """Every stored transition, oldest first."""
        # TODO(student): mind the wrap-around: once the buffer is full, the oldest transition is
        #   the one at self._next, not the one at index 0.
        raise NotImplementedError("ReplayBuffer.all")

    def sample(self, batch_size: int, rng: np.random.Generator) -> dict[str, np.ndarray]:
        """A uniformly random batch, as stacked arrays.

        Returns a dict with keys ``obs`` (float32, shape (batch, obs_size)), ``action`` (int64,
        shape (batch,)), ``reward`` (float32), ``next_obs`` (float32), ``terminated`` (float32,
        1.0 or 0.0 — the mask the TD target multiplies by).

        Raise ``ValueError`` if ``batch_size`` is larger than the number of stored transitions.
        """
        # TODO(student): pick indices with rng.choice(len(self), size=batch_size, replace=False),
        #   then stack the fields with the dtypes above.
        raise NotImplementedError("ReplayBuffer.sample")


def epsilon_at(step: int, total_steps: int, eps_start: float = 1.0, eps_end: float = 0.05,
               fraction: float = 0.4) -> float:
    """The exploration rate at ``step`` of a ``total_steps`` run.

    Linear from ``eps_start`` down to ``eps_end`` over the first ``fraction`` of the run, then
    constant at ``eps_end``:

        epsilon(step) = max(eps_end, eps_start - (eps_start - eps_end) * step / (fraction * total_steps))

    ``step`` counts from 1 (the first environment step), as in ``dqn.py``.
    """
    # TODO(student): one line.
    raise NotImplementedError("epsilon_at")


def td_target(reward: np.ndarray, terminated: np.ndarray, next_q: np.ndarray, gamma: float) -> np.ndarray:
    """The DQN regression target for a batch.

        y_i = reward_i + gamma * (1 - terminated_i) * max_a' next_q[i, a']

    ``reward`` and ``terminated`` have shape (batch,), ``next_q`` has shape (batch, n_actions)
    and holds the TARGET network's Q-values for the next observations. ``terminated`` may arrive
    as floats (1.0/0.0) or booleans. Returns a float array of shape (batch,).
    """
    # TODO(student): vectorized, no Python loop. Remember the mask.
    raise NotImplementedError("td_target")


def double_dqn_target(reward: np.ndarray, terminated: np.ndarray, next_q_online: np.ndarray,
                      next_q_target: np.ndarray, gamma: float) -> np.ndarray:
    """The Double DQN target: the ONLINE network picks the action, the TARGET network scores it.

        a*_i = argmax_a' next_q_online[i, a']
        y_i  = reward_i + gamma * (1 - terminated_i) * next_q_target[i, a*_i]

    Ties in the argmax go to the lowest action index (numpy's default).
    """
    # TODO(student): np.argmax on axis 1, then take_along_axis (or fancy indexing).
    raise NotImplementedError("double_dqn_target")
