"""safety_layer.py — the deterministic shield between a learned policy and karmel's motors (17.09).

A learned policy is an untested function approximator: it can output NaN, full speed, or a
command that is fine in simulation and wrong on the floor. It never talks to the motors directly.
Every control cycle its command passes through ``SafetyShield.filter``, which applies these rules
in order and reports which ones fired:

  1. e-stop latched     a software stop request (teleop button, supervisor) latches until a human
                        calls reset(). The hardware e-stop (catalog item `estop`) cuts motor power
                        independently of ALL software, including this file.
  2. stale observation  the newest observation is older than max_obs_age_s  -> stop
  3. non-finite         NaN / inf in the command                              -> stop
  4. low battery        below the config's low_warning_v                      -> stop
  5. velocity limits    clamp |v| and |w| to deployment limits (lower than the robot's maximum)
  6. obstacle stop      front range below stop_distance_m while driving forward -> v = 0 (turning allowed)
  7. geofence           estimated position outside the allowed box and heading further out -> v = 0
  8. acceleration limit v and w change by at most accel * dt per cycle
  9. violation budget   rules 2, 3, 4, 6 or 7 firing on max_consecutive_violations cycles in a row
                        -> latch the e-stop (clamping by rules 5 and 8 is normal and not counted)

The motor controller's own watchdog (Pico firmware, 300 ms) is the last layer: if this process
hangs, the wheels stop anyway.

    py 17-reinforcement-learning/code/safety_layer.py        # a misbehaving policy in the simulator
"""

from __future__ import annotations

import math
import sys
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT / "labs" / "python") not in sys.path:
    sys.path.insert(0, str(ROOT / "labs" / "python"))

from robotlab.config import KarmelConfig, load_config  # noqa: E402


@dataclass(frozen=True)
class ShieldConfig:
    max_v_m_s: float = 0.25  # deployment limit: half of karmel's 0.5 m/s software maximum
    max_w_rad_s: float = 1.5
    max_linear_accel_m_s2: float = 0.5
    max_angular_accel_rad_s2: float = 3.0
    max_obs_age_s: float = 0.25
    stop_distance_m: float = 0.30
    low_battery_v: float = 10.5
    geofence: tuple[float, float, float, float] | None = None  # (x_min, y_min, x_max, y_max) in map
    max_consecutive_violations: int = 50  # 50 cycles at 10 Hz = 5 s

    @classmethod
    def from_config(cls, cfg: KarmelConfig | None = None, **overrides) -> ShieldConfig:
        cfg = cfg or load_config()
        base = cls(
            max_v_m_s=min(0.25, cfg.drive.max_linear_speed_m_s),
            max_w_rad_s=min(1.5, cfg.drive.max_angular_speed_rad_s),
            max_linear_accel_m_s2=min(0.5, cfg.drive.max_linear_accel_m_s2),
            low_battery_v=cfg.battery.low_warning_v,
        )
        return cls(**{**base.__dict__, **overrides})


@dataclass
class ShieldInput:
    """What the shield knows this cycle. Times in seconds on one monotonic clock."""

    now: float
    obs_stamp: float  # when the observation the policy used was measured
    v_cmd: float  # policy output, m/s
    w_cmd: float  # rad/s
    front_range_m: float | None = None
    battery_v: float | None = None
    x: float | None = None  # position estimate (map frame) for the geofence
    y: float | None = None
    theta: float | None = None


@dataclass
class SafetyShield:
    cfg: ShieldConfig
    estop_latched: bool = False
    _v: float = 0.0
    _w: float = 0.0
    _last_now: float | None = None
    _violations: int = 0
    history: list[tuple[float, tuple[str, ...]]] = field(default_factory=list)

    def request_stop(self) -> None:
        """Software e-stop (teleop button, supervisor). Latches."""
        self.estop_latched = True

    def reset(self) -> None:
        """Only a human calls this, after looking at the robot."""
        self.estop_latched, self._violations, self._v, self._w = False, 0, 0.0, 0.0

    def filter(self, inp: ShieldInput) -> tuple[float, float, tuple[str, ...]]:
        """Return the safe (v, w) to send and the names of the rules that fired."""
        c, fired = self.cfg, []
        dt = 0.0 if self._last_now is None else max(0.0, inp.now - self._last_now)
        self._last_now = inp.now
        v, w = inp.v_cmd, inp.w_cmd

        if self.estop_latched:
            return self._output(0.0, 0.0, ("estop_latched",), inp.now, hard=True)
        if inp.now - inp.obs_stamp > c.max_obs_age_s:
            fired.append("stale_observation")
            v, w = 0.0, 0.0
        if not (math.isfinite(v) and math.isfinite(w)):
            fired.append("non_finite")
            v, w = 0.0, 0.0
        if inp.battery_v is not None and inp.battery_v < c.low_battery_v:
            fired.append("low_battery")
            v, w = 0.0, 0.0
        if abs(v) > c.max_v_m_s or abs(w) > c.max_w_rad_s:
            fired.append("velocity_limit")
            v = max(-c.max_v_m_s, min(c.max_v_m_s, v))
            w = max(-c.max_w_rad_s, min(c.max_w_rad_s, w))
        if v > 0 and inp.front_range_m is not None and inp.front_range_m < c.stop_distance_m:
            fired.append("obstacle_stop")
            v = 0.0
        if c.geofence and inp.x is not None and inp.y is not None and inp.theta is not None and v != 0:
            x_min, y_min, x_max, y_max = c.geofence
            vx, vy = v * math.cos(inp.theta), v * math.sin(inp.theta)
            leaving = (inp.x < x_min and vx < 0) or (inp.x > x_max and vx > 0) or \
                      (inp.y < y_min and vy < 0) or (inp.y > y_max and vy > 0)
            if leaving:
                fired.append("geofence")
                v = 0.0

        violations = [r for r in fired if r != "velocity_limit"]
        self._violations = self._violations + 1 if violations else 0
        if self._violations >= c.max_consecutive_violations:
            self.estop_latched = True
            return self._output(0.0, 0.0, (*fired, "violation_budget"), inp.now, hard=True)

        # acceleration limit (a stop request from rules 2-4 is also ramped, but at 4x the rate)
        hard_stop = any(r in fired for r in ("stale_observation", "non_finite", "low_battery"))
        k = 4.0 if hard_stop else 1.0
        dv, dw = k * c.max_linear_accel_m_s2 * dt, k * c.max_angular_accel_rad_s2 * dt
        v_new = self._v + max(-dv, min(dv, v - self._v))
        w_new = self._w + max(-dw, min(dw, w - self._w))
        if (abs(v_new - v) > 1e-9 or abs(w_new - w) > 1e-9) and dt > 0:
            fired.append("accel_limit")
        return self._output(v_new, w_new, tuple(fired), inp.now)

    def _output(self, v: float, w: float, fired: tuple[str, ...], now: float, hard: bool = False):
        self._v, self._w = (0.0, 0.0) if hard else (v, w)
        if fired:
            self.history.append((now, fired))
        return self._v, self._w, fired


def run_with_shield(
    policy: Callable[[int], tuple[float, float]], steps: int = 120, dt: float = 0.1, seed: int = 0,
    stale_after: int | None = None, estop_at: int | None = None,
) -> list[tuple[int, float, float, tuple[str, ...]]]:
    """Drive the simulated karmel with ``policy(step) -> (v, w)`` through the shield at 1/dt Hz."""
    from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

    cfg = load_config()
    world = World.rectangle_room(3.0, 3.0)
    base = SimBase(DiffDriveSim(world, DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg),
                                pose=(1.0, 1.5, 0.0), seed=seed), dt=0.02)
    shield = SafetyShield(ShieldConfig.from_config(cfg, geofence=(0.3, 0.3, 2.7, 2.7)))
    r, b = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    log, obs_stamp = [], 0.0
    state = base.read()
    for k in range(steps):
        if stale_after is None or k < stale_after:
            obs_stamp = state.t  # the perception pipeline delivered a fresh observation
        v_cmd, w_cmd = policy(k)
        if estop_at is not None and k == estop_at:
            shield.request_stop()  # a human pressed the teleop stop button
        x, y, theta = base.true_pose  # stands in for the localization estimate
        v, w, fired = shield.filter(ShieldInput(
            now=state.t, obs_stamp=obs_stamp, v_cmd=v_cmd, w_cmd=w_cmd,
            front_range_m=state.range_m, battery_v=state.battery_v, x=x, y=y, theta=theta))
        for _ in range(round(dt / base.dt)):  # send a wheel command EVERY 20 ms cycle (firmware watchdog)
            base.set_wheel_velocity((v - 0.5 * w * b) / r, (v + 0.5 * w * b) / r)
            state = base.read()
        log.append((k, v, w, fired))
    base.close()
    return log


def misbehaving_policy(k: int) -> tuple[float, float]:
    """Full speed ahead, a NaN burst, into the wall, then a hard spin: what a bad checkpoint can do."""
    if 30 <= k < 33:
        return float("nan"), 0.0
    if k >= 100:
        return 0.0, 6.0
    return 1.0, 0.0


if __name__ == "__main__":
    log = run_with_shield(misbehaving_policy, steps=150, estop_at=130)
    print(f"{'step':>4} {'v sent':>7} {'w sent':>7}  rules fired")
    previous: tuple[str, ...] = ("-",)
    for k, v, w, fired in log:
        if fired != previous or k % 20 == 0:
            print(f"{k:>4} {v:>7.3f} {w:>7.3f}  {', '.join(fired) or '-'}")
            previous = fired
    print("\nstale observations from step 40 on (the camera pipeline froze):")
    for k, v, w, fired in run_with_shield(lambda k: (0.2, 0.0), steps=60, stale_after=40)[36:48]:
        print(f"{k:>4} {v:>7.3f} {w:>7.3f}  {', '.join(fired) or '-'}")
