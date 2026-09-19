"""domain_randomization.py — does a policy survive a robot that isn't the simulator? (lesson 17.09)

    py 17-reinforcement-learning/code/domain_randomization.py sysid              # identify a hidden robot, ~10 s
    py 17-reinforcement-learning/code/domain_randomization.py robustness         # evaluate trained policies
    py 17-reinforcement-learning/code/domain_randomization.py robustness --episodes 20

"robustness" loads (train them first, a few minutes each):
    runs/ppo_goal_s0/model.zip       py 17-reinforcement-learning/code/train_go_to_goal.py
    runs/ppo_goal_dr_s0/model.zip    py 17-reinforcement-learning/code/train_go_to_goal.py --randomize --steps 100000
and evaluates them, plus the hand-written controller, on a panel of "test robots": the nominal
simulator, the course's realistic() preset, and robots with latency, noise and weak motors,
including two that are OUTSIDE the randomization ranges used in training.

"sysid" shows the other half of sim-to-real: measure the real robot (here: a hidden simulated one)
with two short test drives, and centre the randomization ranges on what you measured.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import rl_tools  # noqa: E402
from karmel_env import KarmelGoToGoalEnv, RobotVariant, heading_controller  # noqa: E402

from robotlab.config import load_config  # noqa: E402
from robotlab.geometry import wrap_angle  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World  # noqa: E402

_real = DiffDriveParams.realistic()
TEST_ROBOTS: dict[str, RobotVariant] = {
    "nominal (training sim)": RobotVariant(),
    "realistic() preset": RobotVariant(
        motor_gain_left=_real.motor_gain_left, motor_gain_right=_real.motor_gain_right,
        wheel_radius_scale_left=_real.wheel_radius_scale_left, wheel_radius_scale_right=_real.wheel_radius_scale_right,
        wheel_separation_scale=_real.wheel_separation_scale, slip_std=_real.slip_std,
    ),
    "100 ms command latency": RobotVariant(action_delay_steps=1),
    "200 ms latency + noise 0.03": RobotVariant(action_delay_steps=2, obs_noise_std=0.03),
    "worn: right motor 0.8, b +10%, slip 5%": RobotVariant(
        motor_gain_right=0.8, wheel_separation_scale=1.10, slip_std=0.05, action_delay_steps=1, obs_noise_std=0.02),
    "400 ms latency": RobotVariant(action_delay_steps=4),
    "OUTSIDE ranges: 500 ms latency": RobotVariant(action_delay_steps=5),
    "OUTSIDE ranges: 600 ms latency": RobotVariant(action_delay_steps=6),
    "OUTSIDE ranges: noise 0.10": RobotVariant(obs_noise_std=0.10),
}


def robustness(episodes: int) -> None:
    from stable_baselines3 import PPO

    policies = {"hand-written": heading_controller}
    for label, path in (("PPO nominal", HERE / "runs" / "ppo_goal_s0" / "model.zip"),
                        ("PPO randomized", HERE / "runs" / "ppo_goal_dr_s0" / "model.zip")):
        if path.exists():
            model = PPO.load(path, device="cpu")
            policies[label] = lambda obs, m=model: m.predict(obs, deterministic=True)[0]
        else:
            print(f"(skipping {label}: {path} not found)")
    print(f"success rate / mean steps to goal over {episodes} episodes per robot (seeds 10000..)")
    print(f"{'test robot':<40}" + "".join(f"{name:>22}" for name in policies))
    for robot, variant in TEST_ROBOTS.items():
        cells = []
        for policy in policies.values():
            res = rl_tools.evaluate(policy, lambda v=variant: KarmelGoToGoalEnv(variant=v), episodes=episodes)
            cells.append(f"{res['success_rate']:.2f} / {res['mean_steps_to_goal']:5.1f}")
        print(f"{robot:<40}" + "".join(f"{c:>22}" for c in cells), flush=True)


def sysid(seed: int = 7) -> None:
    """Estimate the hidden wheel radius and wheelbase scales of a robot from two test drives."""
    cfg = load_config()
    r, b = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    hidden = replace(DiffDriveParams.realistic(cfg), wheel_separation_scale=1.07)  # "the real robot"
    print(f"hidden truth: mean wheel radius scale {(hidden.wheel_radius_scale_left + hidden.wheel_radius_scale_right) / 2:.4f}, "
          f"wheelbase scale {hidden.wheel_separation_scale:.4f}")

    # Test 1: drive straight at 0.3 m/s for 5 s; measure the distance (tape measure / overhead camera).
    base = SimBase(DiffDriveSim(World(), hidden, SensorParams.realistic(cfg), pose=(0, 0, 0), seed=seed), dt=0.02)
    w_wheel = 0.3 / r
    for _ in range(250):
        base.set_wheel_velocity(w_wheel, w_wheel)
        state = base.read()
    for _ in range(25):  # stop and let it settle
        base.set_wheel_velocity(0.0, 0.0)
        state = base.read()
    x, y, _ = base.true_pose
    measured = math.hypot(x, y)
    commanded_rad = (state.left_ticks + state.right_ticks) / 2 / cfg.drive.ticks_per_wheel_rev * 2 * math.pi
    radius_scale = measured / (commanded_rad * r)  # metres per wheel radian, relative to nominal
    print(f"test 1: encoders say {commanded_rad * r:.3f} m, tape measure says {measured:.3f} m -> radius scale {radius_scale:.4f}")

    # Test 2: spin in place for 4 s; integrate the gyro (bias measured while standing still first).
    base = SimBase(DiffDriveSim(World(), hidden, SensorParams.realistic(cfg), pose=(0, 0, 0), seed=seed), dt=0.02)
    bias = np.mean([base.gyro_z() for _ in range(200)])
    s0 = base.read()
    yaw, w_spin = 0.0, 1.5 * (b / 2) / r
    for _ in range(200):
        base.set_wheel_velocity(-w_spin, w_spin)
        base.read()
        yaw += (base.gyro_z() - bias) * base.dt
    for _ in range(25):
        base.set_wheel_velocity(0.0, 0.0)
        s1 = base.read()
        yaw += (base.gyro_z() - bias) * base.dt
    d_left = (s1.left_ticks - s0.left_ticks) / cfg.drive.ticks_per_wheel_rev * 2 * math.pi * r * radius_scale
    d_right = (s1.right_ticks - s0.right_ticks) / cfg.drive.ticks_per_wheel_rev * 2 * math.pi * r * radius_scale
    b_est = (d_right - d_left) / yaw  # yaw = (d_right - d_left) / b
    true_yaw = base.true_pose.theta
    print(f"test 2: gyro yaw {yaw:.3f} rad (wrapped {wrap_angle(yaw):.3f}; true final heading {true_yaw:.3f}), "
          f"wheel travel difference {d_right - d_left:.3f} m -> wheelbase {b_est:.4f} m, scale {b_est / b:.4f}")
    print(f"randomize around what you measured, e.g. wheel_separation_scale U({b_est / b - 0.03:.2f}, {b_est / b + 0.03:.2f}) "
          f"instead of a blind U(0.90, 1.20)")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["robustness", "sysid"])
    ap.add_argument("--episodes", type=int, default=50)
    args = ap.parse_args()
    if args.what == "sysid":
        sysid()
    else:
        robustness(args.episodes)


if __name__ == "__main__":
    main()
