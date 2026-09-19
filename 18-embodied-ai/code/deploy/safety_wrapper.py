"""safety_wrapper.py - a deterministic safety filter between a learned policy and the arm (18.06, 18.10).

The policy proposes joint targets; this filter decides what is actually sent to the servos.
It never trusts the policy and never needs a GPU. numpy only.

  py safety_wrapper.py demo        a misbehaving fake policy on a fake SO-101: watch every rule fire
  py safety_wrapper.py box         print the approximate reach of the arm and the workspace box

Rules, applied in this order on every control tick:
  0. latched trip       once tripped, hold position until a human calls reset()
  1. overload           measured servo load above max_load                -> trip immediately
  2. stale observation  the joint reading is older than max_obs_age_s     -> repeat the last safe command
  3. non-finite action  NaN / inf / wrong shape                           -> hold position
  4. joint limits       clip each joint into [lower, upper]
  5. velocity limit     clip the step from the MEASURED position to max_vel * dt
  6. workspace box      if forward kinematics of the target leaves the box, shorten the step
                        (bisection) so the fingertip stops on the boundary
  7. violation budget   any rule 2-6 firing on max_consecutive_violations ticks in a row -> trip

Units: radians, rad/s, metres, seconds (course convention). LeRobot's SO-101 driver uses degrees
(use_degrees=True) and a 0-100 gripper range: convert with from_lerobot_action / to_lerobot_action.

The forward kinematics here is an APPROXIMATE planar model of an SO-101 with course sign
conventions (all angles 0 = arm stretched horizontally forward). Replace it with the FK from your
arm's URDF (lessons 14.04 and 14.08) and check it against a tape measure before trusting the box.
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


# ----------------------------------------------------------------------------------------------
# Kinematics (approximate) and configuration
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class ArmGeometry:
    """Approximate SO-101 link lengths in metres. MEASURE YOUR ARM."""
    shoulder_height_m: float = 0.12   # table to shoulder_lift axis
    upper_arm_m: float = 0.116        # shoulder_lift axis to elbow axis
    forearm_m: float = 0.135          # elbow axis to wrist_flex axis
    hand_m: float = 0.10              # wrist_flex axis to fingertip centre


def approx_fk(q: np.ndarray, g: ArmGeometry = ArmGeometry()) -> np.ndarray:
    """Fingertip position (x forward, y left, z up, metres) in the arm base frame.

    q[0] pan (+ = turn left), q[1..3] pitch joints (+ = lift up), all 0 = arm horizontal forward.
    The wrist roll (q[4]) and gripper (q[5]) do not move the fingertip centre in this model."""
    a1 = q[1]
    a2 = q[1] + q[2]
    a3 = q[1] + q[2] + q[3]
    r = g.upper_arm_m * math.cos(a1) + g.forearm_m * math.cos(a2) + g.hand_m * math.cos(a3)
    z = g.shoulder_height_m + g.upper_arm_m * math.sin(a1) + g.forearm_m * math.sin(a2) + g.hand_m * math.sin(a3)
    return np.array([r * math.cos(q[0]), r * math.sin(q[0]), z])


@dataclass(frozen=True)
class WorkspaceBox:
    min_xyz: np.ndarray
    max_xyz: np.ndarray

    def contains(self, p: np.ndarray, tol: float = 1e-9) -> bool:
        return bool(np.all(p >= self.min_xyz - tol) and np.all(p <= self.max_xyz + tol))

    def distance_outside(self, p: np.ndarray) -> float:
        below = np.maximum(self.min_xyz - p, 0.0)
        above = np.maximum(p - self.max_xyz, 0.0)
        return float(np.linalg.norm(below + above))


@dataclass(frozen=True)
class SafetyConfig:
    lower: np.ndarray                       # rad (gripper: 0..1 = closed..open)
    upper: np.ndarray
    max_vel: np.ndarray                     # rad/s per joint (gripper: fraction per second)
    box: WorkspaceBox
    max_obs_age_s: float = 0.2
    max_load: float = 0.8                   # fraction of rated servo load; None-safe (see filter)
    max_consecutive_violations: int = 15    # 0.5 s at 30 Hz
    fk: Callable[[np.ndarray], np.ndarray] = approx_fk


def so101_default_config() -> SafetyConfig:
    """Conservative first-deployment limits for a table-top SO-101 (course conventions)."""
    d = math.radians
    return SafetyConfig(
        lower=np.array([d(-90), d(-10), d(-150), d(-100), d(-160), 0.0]),
        upper=np.array([d(90), d(100), d(0), d(100), d(160), 1.0]),
        max_vel=np.array([d(90), d(90), d(90), d(120), d(180), 2.0]),   # first runs: slow
        box=WorkspaceBox(min_xyz=np.array([0.05, -0.25, 0.015]),        # 1.5 cm above the table
                         max_xyz=np.array([0.40, 0.25, 0.35])),
    )


# ----------------------------------------------------------------------------------------------
# The filter
# ----------------------------------------------------------------------------------------------
@dataclass
class FilterResult:
    command: np.ndarray
    events: list[str]
    tripped: bool


@dataclass
class SafetyFilter:
    cfg: SafetyConfig
    tripped: bool = False
    trip_reason: str = ""
    consecutive: int = 0
    counts: dict[str, int] = field(default_factory=dict)
    last_command: np.ndarray | None = None

    def reset(self) -> None:
        """Called by a human after inspecting the scene - never automatically."""
        self.tripped, self.trip_reason, self.consecutive = False, "", 0

    def _trip(self, reason: str) -> None:
        self.tripped, self.trip_reason = True, reason

    def filter(self, q_meas: np.ndarray, q_policy: np.ndarray, dt: float,
               obs_age_s: float = 0.0, load: np.ndarray | None = None) -> FilterResult:
        cfg = self.cfg
        q_meas = np.asarray(q_meas, dtype=float)
        hold = q_meas.copy()
        events: list[str] = []

        if self.tripped:
            return FilterResult(hold, ["tripped:" + self.trip_reason], True)

        if load is not None and np.any(np.asarray(load) > cfg.max_load):
            self._trip("overload")
            return self._done(hold, events + ["overload"], True)

        if obs_age_s > cfg.max_obs_age_s:
            last = hold if self.last_command is None else self.last_command.copy()
            return self._done(last, events + ["stale_observation"], True)

        q = np.asarray(q_policy, dtype=float)
        if q.shape != q_meas.shape or not np.all(np.isfinite(q)):
            return self._done(hold, events + ["non_finite_action"], True)

        clipped = np.clip(q, cfg.lower, cfg.upper)
        if not np.allclose(clipped, q):
            events.append("joint_limit")
        q = clipped

        max_step = cfg.max_vel * dt
        step = np.clip(q - q_meas, -max_step, max_step)
        if not np.allclose(step, q - q_meas):
            events.append("velocity_limit")
        q = q_meas + step

        if not cfg.box.contains(cfg.fk(q)):
            events.append("workspace")
            q = self._shorten_to_box(q_meas, step)

        return self._done(q, events, bool(events))

    def _shorten_to_box(self, q_meas: np.ndarray, step: np.ndarray) -> np.ndarray:
        box, fk = self.cfg.box, self.cfg.fk
        if not box.contains(fk(q_meas)):
            # Already outside (e.g. started there): allow the step only if it moves back towards the box.
            if box.distance_outside(fk(q_meas + step)) < box.distance_outside(fk(q_meas)):
                return q_meas + step
            return q_meas.copy()
        lo, hi = 0.0, 1.0                    # fraction of the step that stays inside
        for _ in range(20):
            mid = (lo + hi) / 2
            if box.contains(fk(q_meas + mid * step)):
                lo = mid
            else:
                hi = mid
        return q_meas + lo * step

    def _done(self, q: np.ndarray, events: list[str], violation: bool) -> FilterResult:
        self.last_command = q.copy()
        for e in events:
            self.counts[e] = self.counts.get(e, 0) + 1
        self.consecutive = self.consecutive + 1 if violation else 0
        if not self.tripped and self.consecutive >= self.cfg.max_consecutive_violations:
            self._trip("too_many_violations")
            events = events + ["trip"]
        return FilterResult(q, events, self.tripped)


# ----------------------------------------------------------------------------------------------
# LeRobot adapter (duck-typed: anything with get_observation() and send_action(dict))
# ----------------------------------------------------------------------------------------------
def from_lerobot_action(action: dict[str, float]) -> np.ndarray:
    """LeRobot SO-101 dict (degrees, gripper 0-100) -> course vector (radians, gripper 0-1).
    Check the key names on your version with robot.action_features."""
    return np.array([math.radians(action[f"{j}.pos"]) for j in JOINTS[:5]] + [action["gripper.pos"] / 100.0])


def to_lerobot_action(q: np.ndarray) -> dict[str, float]:
    out = {f"{j}.pos": math.degrees(float(v)) for j, v in zip(JOINTS[:5], q[:5])}
    out["gripper.pos"] = float(q[5]) * 100.0
    return out


class RobotLike(Protocol):
    def get_observation(self) -> dict: ...
    def send_action(self, action: dict) -> dict: ...


@dataclass
class SafeRobot:
    """Wraps a LeRobot-style robot so every action passes through the SafetyFilter.
    NOTE: calibration offsets and signs between LeRobot's degrees and approx_fk's conventions
    must be checked on your arm before the workspace box means anything."""
    robot: RobotLike
    safety: SafetyFilter
    dt: float = 1 / 30

    def get_observation(self) -> dict:
        return self.robot.get_observation()

    def send_action(self, action: dict, obs: dict, obs_age_s: float = 0.0) -> FilterResult:
        res = self.safety.filter(from_lerobot_action(obs), from_lerobot_action(action), self.dt, obs_age_s)
        self.robot.send_action(to_lerobot_action(res.command))
        return res


def run_episode(robot: RobotLike, policy_fn: Callable[[dict], dict], safety: SafetyFilter,
                duration_s: float, fps: float = 30.0, clock: Callable[[], float] | None = None,
                sleep: Callable[[float], None] | None = None) -> dict[str, int]:
    """A minimal fixed-rate control loop: observe -> policy -> safety filter -> send.

    Works with a LeRobot robot object (get_observation / send_action with '<joint>.pos' keys) or FakeArm.
    Stops early on a trip. Returns the filter's event counts for the trial log."""
    import time
    clock = clock or time.monotonic
    sleep = sleep or time.sleep
    safe = SafeRobot(robot, safety, dt=1.0 / fps)
    t_end = clock() + duration_s
    while clock() < t_end and not safety.tripped:
        t0 = clock()
        obs = safe.get_observation()
        action = policy_fn(obs)
        safe.send_action(action, obs, obs_age_s=clock() - t0)
        sleep(max(0.0, 1.0 / fps - (clock() - t0)))
    return dict(safety.counts)


# ----------------------------------------------------------------------------------------------
# Demo
# ----------------------------------------------------------------------------------------------
@dataclass
class FakeArm:
    """Servos that reach 60% of the commanded step per tick. Positions in LeRobot degrees."""
    q: np.ndarray = field(default_factory=lambda: np.array([0.0, math.radians(60), math.radians(-100),
                                                             math.radians(40), 0.0, 0.2]))
    sent: list[dict] = field(default_factory=list)

    def get_observation(self) -> dict:
        return to_lerobot_action(self.q)

    def send_action(self, action: dict) -> dict:
        self.sent.append(action)
        target = from_lerobot_action(action)
        self.q = self.q + 0.6 * (target - self.q)
        return action


def run_scenario(name: str, policy: Callable[[int, dict], dict], ticks: int) -> SafeRobot:
    arm = FakeArm()
    safe = SafeRobot(arm, SafetyFilter(so101_default_config()))
    z_min = approx_fk(arm.q)[2]
    for tick in range(ticks):
        obs = safe.get_observation()
        res = safe.send_action(policy(tick, obs), obs)
        z_min = min(z_min, approx_fk(arm.q)[2])
    counts = ", ".join(f"{k}={v}" for k, v in safe.safety.counts.items()) or "none"
    print(f"{name:<34} events: {counts}")
    print(f"{'':<34} tripped: {safe.safety.tripped} ({safe.safety.trip_reason or '-'}); "
          f"lowest fingertip z = {z_min:.3f} m")
    return safe


def demo() -> None:
    """Five classic learned-policy failures, each against a fresh arm and filter (LeRobot degrees)."""
    def nan_once(t: int, o: dict) -> dict:
        return {**o, "shoulder_pan.pos": float("nan") if t == 3 else o["shoulder_pan.pos"] + 1.0}

    def jump(t: int, o: dict) -> dict:           # +40 deg per tick at 30 Hz = 1200 deg/s requested
        return {**o, "shoulder_pan.pos": o["shoulder_pan.pos"] + 40.0} if t < 5 else dict(o)

    def past_limit(t: int, o: dict) -> dict:     # elbow_flex upper limit is 0 deg
        return {**o, "elbow_flex.pos": 30.0} if t < 8 else dict(o)

    def dive(t: int, o: dict) -> dict:           # legal joint angles and speeds, but aims below the table
        goal = {"shoulder_lift.pos": 0.0, "elbow_flex.pos": -90.0, "wrist_flex.pos": 0.0}
        return {**o, **{k: o[k] + max(-2.0, min(2.0, v - o[k])) for k, v in goal.items()}}

    def stuck_bad(t: int, o: dict) -> dict:      # keeps commanding NaN: must latch a trip
        return {**o, "wrist_flex.pos": float("inf")}

    print("velocity limit 90 deg/s at 30 Hz = 3 deg per tick; box floor 0.015 m above the table\n")
    run_scenario("1. NaN once", nan_once, 10)
    run_scenario("2. 1200 deg/s jump for 5 ticks", jump, 10)
    run_scenario("3. elbow past its limit", past_limit, 12)
    run_scenario("4. dive into the table (3 s)", dive, 90)
    safe = run_scenario("5. NaN forever (1 s)", stuck_bad, 30)
    safe.safety.reset()
    print("\nafter a human reset(): tripped =", safe.safety.tripped)


def show_box() -> None:
    cfg = so101_default_config()
    g = ArmGeometry()
    print(f"approx. max reach from shoulder axis: {g.upper_arm_m + g.forearm_m + g.hand_m:.3f} m")
    print(f"fingertip at all-zero joints: {np.round(approx_fk(np.zeros(6)), 3)} m")
    print(f"workspace box min {cfg.box.min_xyz} m, max {cfg.box.max_xyz} m")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["demo", "box"])
    a = ap.parse_args()
    demo() if a.cmd == "demo" else show_box()


if __name__ == "__main__":
    main()
