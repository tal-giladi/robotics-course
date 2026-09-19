"""17.03 — Q-learning on a gridworld robot.

Fill in every ``TODO(student)`` (the four functions at the bottom). The GridWorld class is
provided: read it, don't change it. Check with ``python course.py check 17.03``.

The gridworld (provided) is a tiny floor plan: karmel starts at S,
must reach the charger G, and must not roll down the stairs X. Walls (#) block moves.

    col:  0 1 2 3 4 5
    row 0 S . . # . G
    row 1 . # . # . .
    row 2 . # . . . X
    row 3 . . . # . .

States are cell indices ``row * n_cols + col`` (0..23). Actions: 0 up, 1 right, 2 down, 3 left.
Rewards: -1 per move (time and battery), +10 on reaching G, -10 on falling down X. G and X end
the episode (``terminated``). Moving into a wall or off the map leaves the robot where it is
(and still costs -1). With ``slip > 0`` the robot moves in a random perpendicular direction with
that probability — like wheels slipping on a rug.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

import numpy as np

UP, RIGHT, DOWN, LEFT = 0, 1, 2, 3
ARROWS = "^>v<"
MOVES = {UP: (-1, 0), RIGHT: (0, 1), DOWN: (1, 0), LEFT: (0, -1)}

LAYOUT = (
    "S..#.G",
    ".#.#..",
    ".#...X",
    "...#..",
)


@dataclass
class GridWorld:
    """A deterministic (``slip=0``) or slippery gridworld with the Gymnasium-style API."""

    layout: tuple[str, ...] = LAYOUT
    step_reward: float = -1.0
    goal_reward: float = 10.0
    stairs_reward: float = -10.0
    slip: float = 0.0
    max_steps: int = 100
    n_actions: ClassVar[int] = 4
    n_rows: int = field(init=False)
    n_cols: int = field(init=False)
    state: int = field(init=False, default=0)
    steps: int = field(init=False, default=0)

    def __post_init__(self) -> None:
        self.n_rows, self.n_cols = len(self.layout), len(self.layout[0])
        self.start = self._find("S")
        self.state = self.start

    @property
    def n_states(self) -> int:
        return self.n_rows * self.n_cols

    def _find(self, char: str) -> int:
        for r, line in enumerate(self.layout):
            if char in line:
                return r * self.n_cols + line.index(char)
        raise ValueError(f"{char!r} not in layout")

    def cell(self, state: int) -> str:
        r, c = divmod(state, self.n_cols)
        return self.layout[r][c]

    def is_terminal(self, state: int) -> bool:
        return self.cell(state) in "GX"

    def transition(self, state: int, action: int) -> tuple[int, float, bool]:
        """The deterministic result of ``action`` in ``state``: (next_state, reward, terminated)."""
        r, c = divmod(state, self.n_cols)
        dr, dc = MOVES[action]
        nr, nc = r + dr, c + dc
        if not (0 <= nr < self.n_rows and 0 <= nc < self.n_cols) or self.layout[nr][nc] == "#":
            nr, nc = r, c  # bumped into a wall: stay
        nxt = nr * self.n_cols + nc
        kind = self.layout[nr][nc]
        if kind == "G":
            return nxt, self.goal_reward, True
        if kind == "X":
            return nxt, self.stairs_reward, True
        return nxt, self.step_reward, False

    def reset(self, rng: np.random.Generator | None = None) -> int:
        self.state, self.steps = self.start, 0
        return self.state

    def step(self, action: int, rng: np.random.Generator | None = None) -> tuple[int, float, bool, bool]:
        """Apply ``action``; return (next_state, reward, terminated, truncated)."""
        if self.slip > 0 and rng is not None and rng.random() < self.slip:
            action = (action + (1 if rng.random() < 0.5 else 3)) % 4  # perpendicular
        self.state, reward, terminated = self.transition(self.state, int(action))
        self.steps += 1
        truncated = not terminated and self.steps >= self.max_steps
        return self.state, reward, terminated, truncated

    def render_policy(self, policy: np.ndarray) -> str:
        """The layout with an arrow for the chosen action in every free cell."""
        rows = []
        for r, line in enumerate(self.layout):
            cells = []
            for c, ch in enumerate(line):
                s = r * self.n_cols + c
                cells.append(ch if ch in "#GX" else ARROWS[int(policy[s])])
            rows.append(" ".join(cells))
        return "\n".join(rows)


# --------------------------------------------------------------------------------------------
# The parts you implement
# --------------------------------------------------------------------------------------------
def q_learning_update(
    Q: np.ndarray, state: int, action: int, reward: float, next_state: int, terminated: bool,
    alpha: float, gamma: float,
) -> float:
    """One Q-learning (off-policy TD) update of ``Q[state, action]`` in place; return the TD error.

        target   = reward                                if terminated
                 = reward + gamma * max_a' Q[next_state, a']   otherwise
        td_error = target - Q[state, action]
        Q[state, action] += alpha * td_error

    ``Q`` is a float array of shape ``(n_states, n_actions)``. Modify it in place.
    """
    # TODO(student): compute the target (no bootstrapping from a terminal state!), the TD
    #   error, update Q in place and return the TD error as a Python float.
    raise NotImplementedError("q_learning_update")


def greedy_action(Q: np.ndarray, state: int) -> int:
    """The action with the highest Q-value in ``state``; ties go to the lowest action index."""
    # TODO(student): one line with numpy. Return a Python int.
    raise NotImplementedError("greedy_action")


def greedy_policy(Q: np.ndarray) -> np.ndarray:
    """``greedy_action`` for every state, as an int array of shape ``(n_states,)``."""
    # TODO(student): vectorize it (hint: the axis argument).
    raise NotImplementedError("greedy_policy")


def epsilon_greedy(Q: np.ndarray, state: int, epsilon: float, rng: np.random.Generator) -> int:
    """With probability ``epsilon`` a uniformly random action, otherwise the greedy action.

    Draw exactly one ``rng.random()`` to decide, and (only when exploring) one
    ``rng.integers(n_actions)`` to pick the action.
    """
    # TODO(student): explore with probability epsilon, otherwise exploit.
    raise NotImplementedError("epsilon_greedy")
