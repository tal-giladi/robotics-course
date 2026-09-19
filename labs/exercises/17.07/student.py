"""17.07 — Wrap the course simulator as a Gymnasium environment.

Fill in every ``TODO(student)``. Check with ``python course.py check 17.07``.
The tests call Gymnasium's own ``check_env`` on your class, so follow the API exactly.

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
    # TODO(student): differential-drive inverse kinematics. Each wheel's rim speed is
    #   v -/+ w * separation / 2; divide by the radius to get rad/s.
    raise NotImplementedError("twist_to_wheels")


def make_observation(
    pose: tuple[float, float, float], goal: tuple[float, float], v: float, w: float,
    d_max: float, v_max: float, w_max: float,
) -> np.ndarray:
    """The 5-number observation described in the module docstring, clipped to its bounds, float32."""
    # TODO(student): heading error = wrap_angle(atan2(dy, dx) - theta). Build the 5 numbers, clip
    #   the distance to [0, 1] and the rest to [-1, 1], return np.float32 with shape (5,).
    raise NotImplementedError("make_observation")


def step_reward(d_prev: float, d: float, success: bool, collided: bool) -> float:
    """10 per metre of progress, -0.05 per step, +10 on success, -10 on collision."""
    # TODO(student)
    raise NotImplementedError("step_reward")


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

        # TODO(student): define self.observation_space and self.action_space: both spaces.Box with
        #   dtype=np.float32 and the bounds from the module docstring (low of obs[0] is 0, not -1).
        raise NotImplementedError("GoToGoalEnv.__init__ (spaces)")

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        """Start a new episode; return ``(observation, info)``.

        1. Call ``super().reset(seed=seed)`` FIRST: it seeds ``self.np_random``. Use only
           ``self.np_random`` for randomness, so the same seed gives the same episode.
        2. Start pose and goal: ``options["start"] = (x, y, theta)`` and ``options["goal"] = (x, y)``
           when given; otherwise sample both positions uniformly in [0.4, room - 0.4] (repeat until
           they are more than 1.0 m apart) and theta uniformly in [-pi, pi).
        3. ``self.sim = DiffDriveSim(self.world, self.params, self.sensor_params, pose=start,
           seed=int(self.np_random.integers(2**31 - 1)))``; store ``self.goal``; set
           ``self.steps = 0`` and ``self.d_prev = self._distance()``.
        4. Return ``self._obs(), self._info(False, False)``.
        """
        # TODO(student)
        raise NotImplementedError("GoToGoalEnv.reset")

    def step(self, action):
        """Apply one action for 0.1 s; return ``(obs, reward, terminated, truncated, info)``.

        1. Clip the action to [-1, 1]; v = a[0] * v_max, w = a[1] * w_max; convert with
           ``twist_to_wheels`` using the nominal ``self.r`` and ``self.b``.
        2. SUBSTEPS times: ``self.sim.set_velocity(left, right)`` then ``self.sim.step(PHYSICS_DT)``.
           The step collided if ``self.sim.collided`` was True after ANY substep.
        3. Count the step. success = distance < goal_tolerance_m; terminated = success or collided;
           truncated = not terminated and steps >= max_episode_steps. Return Python bools.
        4. reward = ``step_reward(self.d_prev, d, success, collided)`` (a Python float); then update
           ``self.d_prev``. Return ``self._obs(), reward, terminated, truncated, self._info(success, collided)``.
        """
        # TODO(student)
        raise NotImplementedError("GoToGoalEnv.step")

    # --- provided helpers ------------------------------------------------------------------------
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
