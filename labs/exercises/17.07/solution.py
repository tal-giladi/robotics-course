"""17.07 — Wrap the course simulator as a Gymnasium environment. REFERENCE SOLUTION.

Task: karmel starts somewhere in an empty 4 m x 4 m room and must drive to a goal point.

    observation  float32, shape (5,)
        [0] distance to goal / d_max            in [0, 1]   (d_max = room diagonal)
        [1] cos(heading error)                  in [-1, 1]  (heading error = wrap(bearing - theta))
        [2] sin(heading error)                  in [-1, 1]
        [3] measured forward speed / v_max      in [-1, 1]  (from the encoder speed estimate)
        [4] measured yaw rate / w_max           in [-1, 1]
    action       float32, shape (2,), in [-1, 1]: [v / v_max, w / w_max]
    one step     0.1 s = 5 simulator steps of 0.02 s with wheel-velocity commands
    reward       10 * (metres closer) - 0.05, +10 on reaching the goal, -10 on a collision
    terminated   reached the goal (distance < goal_tolerance_m) or collided
    truncated    max_episode_steps reached without terminating

v_max, w_max, wheel radius and separation come from labs/config/karmel.yaml.
"""

from __future__ import annotations

import math
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from robotlab.config import KarmelConfig, load_config
from robotlab.geometry import wrap_angle
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World

PHYSICS_DT = 0.02
SUBSTEPS = 5


def twist_to_wheels(v: float, w: float, wheel_radius_m: float, wheel_separation_m: float) -> tuple[float, float]:
    """Body velocity (v m/s forward, w rad/s CCW) -> wheel angular velocities (left, right) in rad/s."""
    left = (v - 0.5 * w * wheel_separation_m) / wheel_radius_m
    right = (v + 0.5 * w * wheel_separation_m) / wheel_radius_m
    return left, right


def make_observation(
    pose: tuple[float, float, float], goal: tuple[float, float], v: float, w: float,
    d_max: float, v_max: float, w_max: float,
) -> np.ndarray:
    """The 5-number observation described in the module docstring, clipped to its bounds, float32."""
    x, y, theta = pose
    dx, dy = goal[0] - x, goal[1] - y
    error = wrap_angle(math.atan2(dy, dx) - theta)
    obs = np.array([math.hypot(dx, dy) / d_max, math.cos(error), math.sin(error), v / v_max, w / w_max])
    low = np.array([0.0, -1.0, -1.0, -1.0, -1.0])
    return np.clip(obs, low, 1.0).astype(np.float32)


def step_reward(d_prev: float, d: float, success: bool, collided: bool) -> float:
    """10 per metre of progress, -0.05 per step, +10 on success, -10 on collision."""
    reward = 10.0 * (d_prev - d) - 0.05
    if success:
        reward += 10.0
    if collided:
        reward -= 10.0
    return float(reward)


class GoToGoalEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self, max_episode_steps: int = 200, goal_tolerance_m: float = 0.15, room_size_m: float = 4.0,
        config: KarmelConfig | None = None,
    ) -> None:
        super().__init__()
        self.cfg = config or load_config()
        self.max_episode_steps = max_episode_steps
        self.goal_tolerance_m = goal_tolerance_m
        self.room = room_size_m
        d = self.cfg.drive
        self.v_max, self.w_max = d.max_linear_speed_m_s, d.max_angular_speed_rad_s
        self.r, self.b = d.wheel_radius_m, d.wheel_separation_m
        self.d_max = math.hypot(room_size_m, room_size_m)
        self.world = World.rectangle_room(room_size_m, room_size_m)
        self.params = DiffDriveParams.ideal(self.cfg)
        self.sensor_params = SensorParams.ideal(self.cfg)
        self.sim: DiffDriveSim | None = None
        self.goal = (0.0, 0.0)
        self.steps = 0

        self.observation_space = spaces.Box(
            low=np.array([0.0, -1.0, -1.0, -1.0, -1.0], dtype=np.float32),
            high=np.ones(5, dtype=np.float32),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        options = options or {}
        start, goal = options.get("start"), options.get("goal")
        if start is None or goal is None:
            margin = 0.4
            while True:
                s = self.np_random.uniform(margin, self.room - margin, 2)
                g = self.np_random.uniform(margin, self.room - margin, 2)
                if np.linalg.norm(g - s) > 1.0:
                    break
            theta = float(self.np_random.uniform(-math.pi, math.pi))
            start = (float(s[0]), float(s[1]), theta) if start is None else start
            goal = (float(g[0]), float(g[1])) if goal is None else goal
        self.goal = (float(goal[0]), float(goal[1]))
        self.sim = DiffDriveSim(
            self.world, self.params, self.sensor_params, pose=tuple(start),
            seed=int(self.np_random.integers(2**31 - 1)),
        )
        self.steps = 0
        self.d_prev = self._distance()
        return self._obs(), self._info(False, False)

    def step(self, action):
        assert self.sim is not None, "call reset() first"
        a = np.clip(np.asarray(action, dtype=float).reshape(2), -1.0, 1.0)
        left, right = twist_to_wheels(a[0] * self.v_max, a[1] * self.w_max, self.r, self.b)
        collided = False
        for _ in range(SUBSTEPS):
            self.sim.set_velocity(left, right)
            self.sim.step(PHYSICS_DT)
            collided = collided or self.sim.collided
        self.steps += 1
        d = self._distance()
        success = d < self.goal_tolerance_m
        terminated = bool(success or collided)
        truncated = bool(not terminated and self.steps >= self.max_episode_steps)
        reward = step_reward(self.d_prev, d, success, collided)
        self.d_prev = d
        return self._obs(), reward, terminated, truncated, self._info(success, collided)

    def _distance(self) -> float:
        assert self.sim is not None
        return math.hypot(self.goal[0] - self.sim.pose.x, self.goal[1] - self.sim.pose.y)

    def _obs(self) -> np.ndarray:
        assert self.sim is not None
        wl, wr = self.sim.wheel_velocity_estimate
        v = 0.5 * self.r * (wl + wr)
        w = self.r * (wr - wl) / self.b
        return make_observation(tuple(self.sim.pose), self.goal, v, w, self.d_max, self.v_max, self.w_max)

    def _info(self, success: bool, collided: bool) -> dict[str, Any]:
        return {"is_success": success, "collided": collided, "distance": self._distance()}
