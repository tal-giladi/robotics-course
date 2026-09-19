"""reach — put the gripper at a commanded (x, y, z), safely and verifiably (lesson 14.10).

The whole of module 14 in one script, in the order the checks must happen:

    target (x, y, z[, pitch])
      -> WORKSPACE BOX     is this point somewhere I am willing to send the arm?   (14.11)
      -> IK                is there a pose that reaches it?                        (14.05)
      -> JOINT LIMITS      is that pose one the arm can hold?                      (14.04)
      -> CONDITIONING      is it far enough from a singularity to be controllable?  (14.06)
      -> TRAJECTORY        a synchronized, limit-respecting path to it              (14.07)
      -> EXECUTION         rate-limited, load-aborting servo writes                 (14.03)
      -> VERIFICATION      where did the arm actually end up, and how far is that?  (14.10)

Every stage can refuse, and a refusal names the stage. Nothing is sent to a servo until all of
the checks above it have passed.

    py reach.py --dry-run 0.25 0.0 0.12                 # no hardware: plan, check, print
    py reach.py --dry-run 0.25 0.0 0.12 --simulate       # + run it against the fake servo bus
    py reach.py --dry-run --repeat 10 0.25 0.0 0.12      # repeatability of the whole pipeline
    py reach.py --port COM5 --robot-id karmel_follower 0.25 0.0 0.12       # MOVES THE ARM

> SAFETY (14.11): the last form energises the arm. Clamp the base, clear a 0.5 m radius, keep the
> PSU switch within reach, and start with the default low torque and speed limits. The arm goes
> LIMP when the script ends — support it with a hand.

Units: metres and radians in, metres and radians out. The servo layer converts.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray

import arm_kinematics as ak
import trajectories as tr
from servo_tools import (
    SO101_JOINTS,
    FakeArmBus,
    JointLimits,
    SafetyConfig,
    SafetyError,
    move_smoothly,
    safe_enable_torque,
)

Array = NDArray[np.float64]


# ============================================================================================
# The workspace box
# ============================================================================================
@dataclass(frozen=True)
class WorkspaceBox:
    """The region of space you are willing to send the gripper into.

    Defaults for an SO-101 clamped at the front edge of a table, with the table at z = 0:
    never below ``z_min`` (the table plus clearance), never behind the base, and never further
    out than ``x_max``, which is deliberately well inside the 0.45 m geometric reach — the last
    centimetres of reach are where the Jacobian falls apart (14.06).
    """

    x: tuple[float, float] = (0.10, 0.34)
    y: tuple[float, float] = (-0.22, 0.22)
    z: tuple[float, float] = (0.02, 0.32)

    def violations(self, point: ArrayLike) -> list[str]:
        p = np.asarray(point, dtype=float)
        out = []
        for name, value, (lo, hi) in zip("xyz", p, (self.x, self.y, self.z)):
            if value < lo:
                out.append(f"{name} = {value:.3f} m is below the box minimum {lo:.3f} m")
            elif value > hi:
                out.append(f"{name} = {value:.3f} m is above the box maximum {hi:.3f} m")
        return out

    def contains(self, point: ArrayLike) -> bool:
        return not self.violations(point)


# ============================================================================================
# The plan
# ============================================================================================
@dataclass(frozen=True)
class ReachPlan:
    ok: bool
    stage: str                      # which stage decided the outcome
    reason: str
    target: Array
    q: Array | None = None
    t: Array | None = None
    q_path: Array | None = None
    achieved: Array | None = None
    position_error_m: float = float("nan")
    sigma_min: float = float("nan")
    notes: list[str] = field(default_factory=list)

    def describe(self) -> str:
        lines = [f"target      {np.round(self.target, 4).tolist()} m",
                 f"result      {'OK' if self.ok else 'REFUSED'} at stage '{self.stage}': {self.reason}"]
        if self.q is not None:
            lines.append(f"joints      {np.degrees(self.q).round(2).tolist()} deg")
        if self.achieved is not None:
            lines.append(f"FK check    {np.round(self.achieved, 5).tolist()} m "
                         f"(residual {self.position_error_m * 1000:.3f} mm)")
        if not math.isnan(self.sigma_min):
            lines.append(f"sigma_min   {self.sigma_min:.4f} m/rad "
                         f"(worst-case {0.05 / self.sigma_min:.2f} rad/s for a 5 cm/s move)")
        if self.t is not None:
            lines.append(f"trajectory  {len(self.t)} points, {self.t[-1]:.3f} s")
        lines.extend(f"note        {n}" for n in self.notes)
        return "\n".join(lines)


def plan_reach(
    chain: ak.SerialChain,
    target_xyz: ArrayLike,
    q_start: ArrayLike,
    *,
    pitch: float | None = None,
    box: WorkspaceBox | None = None,
    min_sigma: float = 0.02,
    v_max: float = math.radians(30.0),
    a_max: float = math.radians(60.0),
    dt: float = 0.02,
    position_tol_m: float = 1e-4,
) -> ReachPlan:
    """Run every check, in order, and stop at the first refusal.

    ``min_sigma`` is the smallest singular value of the linear Jacobian you will accept at the
    goal pose, in m/rad. 0.02 means "one radian of the worst joint combination must buy at least
    2 cm of tool motion in the worst direction" (14.06).
    """
    box = box or WorkspaceBox()
    target = np.asarray(target_xyz, dtype=float)

    problems = box.violations(target)
    if problems:
        return ReachPlan(False, "workspace-box", "; ".join(problems), target)

    result = ak.ik_dls(chain, target, q_start, target_pitch=pitch, position_tol_m=position_tol_m)
    if not result.converged:
        return ReachPlan(False, "ik", f"{result.reason} (best error "
                         f"{result.position_error_m * 1000:.1f} mm after {result.iterations} iterations)",
                         target, q=result.q, position_error_m=result.position_error_m)

    if not chain.within_limits(result.q):
        outside = [f"{name} {math.degrees(v):.1f} deg" for name, v, lo, hi
                   in zip(chain.joint_names, result.q, chain.lower, chain.upper)
                   if not lo <= v <= hi]
        return ReachPlan(False, "joint-limits", "outside limits: " + ", ".join(outside), target, q=result.q)

    J = chain.geometric_jacobian(result.q)[:3]
    sigma_min = float(np.linalg.svd(J, compute_uv=False)[-1])
    if sigma_min < min_sigma:
        return ReachPlan(False, "conditioning",
                         f"sigma_min {sigma_min:.4f} < {min_sigma:.4f} m/rad — too close to a "
                         f"singularity to control", target, q=result.q, sigma_min=sigma_min)

    t, q_path, qd, _ = tr.joint_space_trajectory(np.asarray(q_start, dtype=float), result.q,
                                                 v_max, a_max, dt=dt)
    notes = []
    biggest = float(np.max(np.abs(result.q - np.asarray(q_start, dtype=float))))
    if biggest > math.radians(60.0):
        notes.append(f"large move: {math.degrees(biggest):.0f} deg on one joint — check the path is clear")
    achieved = chain.fk(result.q)[:3, 3]
    return ReachPlan(True, "planned", "all checks passed", target, q=result.q, t=t, q_path=q_path,
                     achieved=achieved, position_error_m=float(np.linalg.norm(achieved - target)),
                     sigma_min=sigma_min, notes=notes)


# ============================================================================================
# Execution
# ============================================================================================
def joint_vector_to_goals(chain: ak.SerialChain, q: ArrayLike, gripper: float = 30.0) -> dict[str, float]:
    """Radians -> the degrees-per-joint dict servo_tools expects.

    NOTE: this is the IDENTITY mapping (sign +1, offset 0). On a real arm you must use the
    measured map of 14.08 instead; this default is here so the no-hardware path runs.
    """
    goals = {name: math.degrees(value) for name, value in zip(chain.joint_names, np.asarray(q, dtype=float))}
    goals["gripper"] = gripper
    return goals


def execute(bus, plan: ReachPlan, chain: ak.SerialChain, cfg: SafetyConfig, sleep=None) -> dict[str, float]:
    """Walk the trajectory's waypoints through servo_tools' rate-limited, load-aborting mover."""
    if not plan.ok or plan.q_path is None:
        raise SafetyError(f"refusing to execute a plan that failed at stage '{plan.stage}'")
    safe_enable_torque(bus, cfg)
    present = bus.read_positions()
    for q in plan.q_path[::5]:                      # every 5th waypoint: 10 Hz, not 50
        present = move_smoothly(bus, joint_vector_to_goals(chain, q), cfg,
                                sleep=sleep or getattr(bus, "advance", None))
    return present


def measured_tool_position(chain: ak.SerialChain, present: dict[str, float]) -> Array:
    """Where FK says the gripper is, from the servos' reported positions."""
    q = np.radians([present[name] for name in chain.joint_names])
    return chain.fk(q)[:3, 3]


# ============================================================================================
# CLI
# ============================================================================================
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Move the SO-101's gripper to a commanded position.")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("--pitch", type=float, default=45.0, help="approach pitch below horizontal [deg]")
    parser.add_argument("--dry-run", action="store_true", help="plan and check only; touch no hardware")
    parser.add_argument("--simulate", action="store_true", help="with --dry-run: execute on FakeArmBus")
    parser.add_argument("--repeat", type=int, default=1, help="repeat the whole pipeline N times")
    parser.add_argument("--min-sigma", type=float, default=0.02, help="conditioning floor [m/rad]")
    parser.add_argument("--port", default="", help="serial port of the real arm")
    parser.add_argument("--robot-id", default="karmel_follower")
    parser.add_argument("--speed", type=float, default=30.0, help="servo speed limit [deg/s]")
    parser.add_argument("--torque", type=float, default=30.0,
                        help="servo torque limit, percent of stall")
    args = parser.parse_args(argv)

    chain = ak.load_so101()
    target = np.array([args.x, args.y, args.z])
    pitch = math.radians(args.pitch)
    q_home = np.radians([0.0, -30.0, 40.0, 50.0, 0.0])

    reached = []
    for attempt in range(args.repeat):
        plan = plan_reach(chain, target, q_home, pitch=pitch, min_sigma=args.min_sigma)
        if args.repeat > 1:
            print(f"--- attempt {attempt + 1}/{args.repeat} ---")
        print(plan.describe())
        if not plan.ok:
            return 1
        if args.dry_run and not args.simulate:
            continue
        cfg = SafetyConfig(torque_percent=args.torque, speed_deg_s=args.speed,
                           limits={j: JointLimits(-120.0, 120.0) for j in SO101_JOINTS})
        if args.dry_run:
            bus = FakeArmBus(positions=joint_vector_to_goals(chain, q_home))
        else:
            if not args.port:
                parser.error("give --port, or use --dry-run")
            from so101_bus import default_calibration_path, make_bus
            bus = make_bus(args.port, default_calibration_path(args.robot_id))
        try:
            present = execute(bus, plan, chain, cfg)
            measured = measured_tool_position(chain, present)
            reached.append(measured)
            print(f"measured    {np.round(measured, 5).tolist()} m "
                  f"(commanded-to-measured {np.linalg.norm(measured - target) * 1000:.3f} mm)")
        except SafetyError as exc:
            print("STOPPED:", exc)
            return 2
        finally:
            bus.disable_torque()

    if len(reached) > 1:
        points = np.array(reached)
        spread = np.linalg.norm(points - points.mean(axis=0), axis=1)
        print(f"\nrepeatability over {len(points)} runs: mean offset from the commanded point "
              f"{np.linalg.norm(points.mean(axis=0) - target) * 1000:.3f} mm, "
              f"spread (max |p - mean|) {spread.max() * 1000:.3f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
