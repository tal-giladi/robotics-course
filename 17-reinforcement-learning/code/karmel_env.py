"""karmel_env.py — the course simulator as a Gymnasium environment (lessons 17.07–17.09).

Task: drive karmel from a random start pose to a random goal in a 4 m x 4 m room, optionally
with round obstacles, without hitting anything. One environment step = 0.1 s of simulated time
(5 physics steps of ``DiffDriveSim`` at 0.02 s, with the simulated firmware velocity PID).

    observation (float32, all in [-1, 1]):
        [0]  distance to goal / 5.66 m (room diagonal), clipped to [0, 1]
        [1]  cos(heading error)        heading error = wrap(bearing to goal - robot heading)
        [2]  sin(heading error)
        [3]  measured forward speed v / 0.5 m/s      (from the encoder speed estimate)
        [4]  measured yaw rate  w / 2.5 rad/s
        [5:] n_rays range readings / ray_max_m, evenly spaced around the robot (only if n_rays > 0)
    action (float32 in [-1, 1]^2): [v command / 0.5 m/s, w command / 2.5 rad/s]

Reward modes (lesson 17.08):
    "progress"       10 * (metres closer to the goal) - 0.05 per step, +10 on success, -10 on collision
    "naive_heading"  "progress" + 3 * (radians the heading error SHRANK), clipped at 0 when it grows.
                     Looks reasonable, but spinning in place earns ~0.4 per step. It gets hacked.
    "heading_fixed"  "progress" + 3 * (change of |heading error|), NOT clipped: a potential-based
                     term, so a full spin earns exactly 0.
    "sparse"         +1 on success, 0 otherwise.

Domain randomization (lesson 17.09): ``randomize=True`` samples a different robot every episode
(motor gains, wheel radii, wheelbase, slip, observation noise, action latency).

Run a scripted episode:  py 17-reinforcement-learning/code/karmel_env.py
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / "labs" / "python") not in sys.path:  # robotlab without `pip install -e labs/python`
    sys.path.insert(0, str(ROOT / "labs" / "python"))

from robotlab.config import KarmelConfig, load_config  # noqa: E402
from robotlab.geometry import wrap_angle  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World  # noqa: E402

REWARD_MODES = ("progress", "naive_heading", "heading_fixed", "sparse")


@dataclass(frozen=True)
class Randomization:
    """Ranges sampled uniformly at every reset when ``randomize=True`` (lesson 17.09)."""

    motor_gain: tuple[float, float] = (0.80, 1.05)  # per motor, independently
    wheel_radius_scale: tuple[float, float] = (0.97, 1.03)  # per wheel
    wheel_separation_scale: tuple[float, float] = (0.95, 1.10)
    slip_std: tuple[float, float] = (0.0, 0.05)
    obs_noise_std: tuple[float, float] = (0.0, 0.03)  # on every observation entry
    action_delay_steps: tuple[int, int] = (0, 4)  # inclusive: 0..4 control steps (0-400 ms)


@dataclass(frozen=True)
class RobotVariant:
    """Hidden robot/sensor imperfections for one episode (fixed ones are used for evaluation)."""

    motor_gain_left: float = 1.0
    motor_gain_right: float = 1.0
    wheel_radius_scale_left: float = 1.0
    wheel_radius_scale_right: float = 1.0
    wheel_separation_scale: float = 1.0
    slip_std: float = 0.0
    obs_noise_std: float = 0.0
    action_delay_steps: int = 0

    def apply(self, params: DiffDriveParams) -> DiffDriveParams:
        return replace(
            params,
            motor_gain_left=self.motor_gain_left,
            motor_gain_right=self.motor_gain_right,
            wheel_radius_scale_left=self.wheel_radius_scale_left,
            wheel_radius_scale_right=self.wheel_radius_scale_right,
            wheel_separation_scale=self.wheel_separation_scale,
            slip_std=self.slip_std,
        )


class KarmelGoToGoalEnv(gym.Env):
    """Go-to-goal (and avoid obstacles) with the course's differential-drive simulator."""

    metadata = {"render_modes": ["rgb_array"], "render_fps": 10}

    def __init__(
        self,
        n_obstacles: int = 0,
        n_rays: int | None = None,
        reward_mode: str = "progress",
        max_episode_steps: int = 200,
        goal_tolerance_m: float = 0.15,
        room_size_m: float = 4.0,
        control_dt: float = 0.1,
        physics_dt: float = 0.02,
        ray_max_m: float = 2.0,
        randomize: bool = False,
        randomization: Randomization = Randomization(),
        variant: RobotVariant | None = None,
        render_mode: str | None = None,
        config: KarmelConfig | None = None,
    ) -> None:
        super().__init__()
        if reward_mode not in REWARD_MODES:
            raise ValueError(f"reward_mode must be one of {REWARD_MODES}")
        self.cfg = config or load_config()
        self.n_obstacles = n_obstacles
        self.n_rays = (16 if n_obstacles > 0 else 0) if n_rays is None else n_rays
        self.reward_mode = reward_mode
        self.max_episode_steps = max_episode_steps
        self.goal_tolerance_m = goal_tolerance_m
        self.room = room_size_m
        self.control_dt, self.physics_dt = control_dt, physics_dt
        self.substeps = max(1, round(control_dt / physics_dt))
        self.ray_max_m = ray_max_m
        self.randomize, self.randomization = randomize, randomization
        self.fixed_variant = variant or RobotVariant()
        self.render_mode = render_mode

        d = self.cfg.drive
        self.v_max, self.w_max = d.max_linear_speed_m_s, d.max_angular_speed_rad_s
        self.r, self.b = d.wheel_radius_m, d.wheel_separation_m  # nominal: what the robot "knows"
        self.max_wheel = d.max_wheel_speed_rad_s
        self.robot_radius = self.cfg.chassis.footprint_radius_m
        self.d_max = math.hypot(room_size_m, room_size_m)
        self._base_params = DiffDriveParams.ideal(self.cfg)
        self._sensor_params = SensorParams.ideal(self.cfg)
        self._ray_angles = np.linspace(-math.pi, math.pi, self.n_rays, endpoint=False)

        n_obs = 5 + self.n_rays
        low = np.full(n_obs, -1.0, dtype=np.float32)
        low[0] = 0.0
        low[5:] = 0.0
        self.observation_space = spaces.Box(low=low, high=np.ones(n_obs, dtype=np.float32), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.sim: DiffDriveSim | None = None
        self.goal = np.zeros(2)
        self.variant = self.fixed_variant

    # ------------------------------------------------------------------------------------------
    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)  # seeds self.np_random
        options = options or {}
        rng = self.np_random
        self.variant = self._sample_variant(rng) if self.randomize else self.fixed_variant

        circles = self._sample_obstacles(rng)
        segments = World.rectangle_room(self.room, self.room).segments
        self.world = World.from_segments(segments, circles if len(circles) else None)
        start = options.get("start")
        goal = options.get("goal")
        if start is None or goal is None:
            s, g = self._sample_start_goal(rng, circles)
            start = s if start is None else start
            goal = g if goal is None else goal
        self.goal = np.asarray(goal, dtype=float)
        self.sim = DiffDriveSim(
            self.world, self.variant.apply(self._base_params), self._sensor_params,
            pose=tuple(start), seed=int(rng.integers(2**31 - 1)),
        )
        self.steps = 0
        self.spin_rad = 0.0  # total |heading change|: the reward-hacking detector
        self.path_m = 0.0
        self._pending = [np.zeros(2)] * self.variant.action_delay_steps  # latency queue
        self._d_prev = self._distance()
        self._e_prev = self._heading_error()
        return self._observation(), self._info(False, False)

    def step(self, action):
        assert self.sim is not None, "call reset() first"
        action = np.clip(np.asarray(action, dtype=np.float64).reshape(2), -1.0, 1.0)
        self._pending.append(action)
        applied = self._pending.pop(0)  # with latency the robot executes an older command
        v, w = applied[0] * self.v_max, applied[1] * self.w_max
        left = np.clip((v - 0.5 * w * self.b) / self.r, -self.max_wheel, self.max_wheel)
        right = np.clip((v + 0.5 * w * self.b) / self.r, -self.max_wheel, self.max_wheel)

        x0, y0, th0 = self.sim.pose
        collided = False
        for _ in range(self.substeps):
            self.sim.set_velocity(float(left), float(right))
            self.sim.step(self.physics_dt)
            collided = collided or self.sim.collided
        x1, y1, th1 = self.sim.pose
        self.spin_rad += abs(wrap_angle(th1 - th0))
        self.path_m += math.hypot(x1 - x0, y1 - y0)
        self.steps += 1

        d, e = self._distance(), self._heading_error()
        success = d < self.goal_tolerance_m
        terminated = bool(success or collided)
        truncated = bool(not terminated and self.steps >= self.max_episode_steps)
        reward = self._reward(d, e, success, collided)
        self._d_prev, self._e_prev = d, e
        return self._observation(), float(reward), terminated, truncated, self._info(success, collided)

    # ------------------------------------------------------------------------------------------
    def _reward(self, d: float, e: float, success: bool, collided: bool) -> float:
        if self.reward_mode == "sparse":
            return 1.0 if success else 0.0
        progress = self._d_prev - d  # metres closer this step
        reward = 10.0 * progress - 0.05
        if self.reward_mode == "naive_heading":
            reward += 3.0 * max(0.0, abs(self._e_prev) - abs(e))  # only ever rewards turning toward
        elif self.reward_mode == "heading_fixed":
            reward += 3.0 * (abs(self._e_prev) - abs(e))  # difference of a potential: cycles sum to 0
        if success:
            reward += 10.0
        if collided:
            reward -= 10.0
        return reward

    def _distance(self) -> float:
        assert self.sim is not None
        return float(math.hypot(self.goal[0] - self.sim.pose.x, self.goal[1] - self.sim.pose.y))

    def _heading_error(self) -> float:
        assert self.sim is not None
        x, y, th = self.sim.pose
        return float(wrap_angle(math.atan2(self.goal[1] - y, self.goal[0] - x) - th))

    def _observation(self) -> np.ndarray:
        assert self.sim is not None
        e = self._heading_error()
        wl, wr = self.sim.wheel_velocity_estimate
        v = 0.5 * self.r * (wl + wr)
        w = self.r * (wr - wl) / self.b
        obs = [self._distance() / self.d_max, math.cos(e), math.sin(e), v / self.v_max, w / self.w_max]
        if self.n_rays:
            x, y, th = self.sim.pose
            ranges = self.world.raycast((x, y), th + self._ray_angles, self.ray_max_m)
            obs.extend(np.minimum(ranges, self.ray_max_m) / self.ray_max_m)
        out = np.asarray(obs, dtype=np.float64)
        if self.variant.obs_noise_std > 0:
            out = out + self.np_random.normal(0.0, self.variant.obs_noise_std, out.shape)
        return np.clip(out, self.observation_space.low, self.observation_space.high).astype(np.float32)

    def _info(self, success: bool, collided: bool) -> dict[str, Any]:
        return {
            "is_success": bool(success), "collided": bool(collided), "distance": self._distance(),
            "spin_rad": self.spin_rad, "path_m": self.path_m,
        }

    # ------------------------------------------------------------------------------------------
    def _sample_variant(self, rng: np.random.Generator) -> RobotVariant:
        R = self.randomization
        u = lambda lo_hi: float(rng.uniform(*lo_hi))  # noqa: E731
        return RobotVariant(
            motor_gain_left=u(R.motor_gain), motor_gain_right=u(R.motor_gain),
            wheel_radius_scale_left=u(R.wheel_radius_scale), wheel_radius_scale_right=u(R.wheel_radius_scale),
            wheel_separation_scale=u(R.wheel_separation_scale), slip_std=u(R.slip_std),
            obs_noise_std=u(R.obs_noise_std),
            action_delay_steps=int(rng.integers(R.action_delay_steps[0], R.action_delay_steps[1] + 1)),
        )

    def _sample_obstacles(self, rng: np.random.Generator) -> np.ndarray:
        circles: list[list[float]] = []
        for _ in range(self.n_obstacles):
            r = float(rng.uniform(0.15, 0.30))
            c = rng.uniform(0.9, self.room - 0.9, 2)
            circles.append([float(c[0]), float(c[1]), r])
        return np.array(circles).reshape(-1, 3)

    def _sample_start_goal(self, rng: np.random.Generator, circles: np.ndarray):
        margin = 0.4
        clear = self.robot_radius + 0.15

        def free(p: np.ndarray) -> bool:
            return all(math.hypot(p[0] - cx, p[1] - cy) > cr + clear for cx, cy, cr in circles)

        for _ in range(1000):
            s = rng.uniform(margin, self.room - margin, 2)
            g = rng.uniform(margin, self.room - margin, 2)
            if free(s) and free(g) and np.linalg.norm(g - s) > 1.0:
                theta = float(rng.uniform(-math.pi, math.pi))
                return (float(s[0]), float(s[1]), theta), (float(g[0]), float(g[1]))
        raise RuntimeError("could not place start and goal; fewer obstacles?")

    def render(self):
        """An RGB image of the room, obstacles, goal and robot (matplotlib, headless)."""
        if self.render_mode != "rgb_array" or self.sim is None:
            return None
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        from robotlab.sim import viz

        fig, ax = viz.new_axes(self.world, title=f"step {self.steps}")
        ax.plot(*self.goal, "g*", markersize=15)
        viz.draw_robot(ax, self.sim.pose)
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
        plt.close(fig)
        return img


def make_env(**kwargs: Any) -> KarmelGoToGoalEnv:
    """Factory used by the training scripts (picklable for SubprocVecEnv)."""
    return KarmelGoToGoalEnv(**kwargs)


def heading_controller(obs: np.ndarray) -> np.ndarray:
    """A hand-written baseline policy: turn toward the goal, drive when roughly facing it."""
    e = math.atan2(float(obs[2]), float(obs[1]))
    w = float(np.clip(2.0 * e / 2.5, -1.0, 1.0))
    v = float(np.clip(1.0 - abs(e) / (math.pi / 3), 0.0, 1.0)) * min(1.0, float(obs[0]) * 5.66 / 0.5)
    return np.array([v, w], dtype=np.float32)


if __name__ == "__main__":
    env = KarmelGoToGoalEnv()
    obs, info = env.reset(seed=0)
    print("start", env.sim.pose, "goal", env.goal, "obs", np.round(obs, 3))
    total, steps = 0.0, 0
    while True:
        obs, r, terminated, truncated, info = env.step(heading_controller(obs))
        total += r
        steps += 1
        if terminated or truncated:
            break
    print(f"hand-written controller: {steps} steps, return {total:.2f}, success={info['is_success']}, "
          f"collided={info['collided']}, final distance {info['distance']:.3f} m, spin {info['spin_rad']:.2f} rad")
