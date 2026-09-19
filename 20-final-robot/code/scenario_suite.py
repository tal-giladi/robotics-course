"""The whole-robot test suite: scenarios, metrics, and a regression check between two builds.

Unit tests tell you a function is right. They never tell you the robot got to the kitchen. This
file is the rung above: each ``Scenario`` is a mission in the apartment world with a pass/fail
threshold on *behaviour* — did it arrive, how close did it get to the furniture, how often did
the safety layer have to intervene — and a ``Build`` is the software configuration under test.

    py 20-final-robot/code/scenario_suite.py list
    py 20-final-robot/code/scenario_suite.py run                       # the shipping build
    py 20-final-robot/code/scenario_suite.py run --build fast --seeds 3
    py 20-final-robot/code/scenario_suite.py compare shipping fast     # the regression report
    py 20-final-robot/code/scenario_suite.py builds

Everything runs in the course's 2D simulator (``robotlab.sim``): no ROS, no Gazebo, no hardware,
a few seconds per scenario, and a fixed seed so the same build gives the same numbers twice.
Lesson 20.05 explains what this rung can and cannot catch, and what to do in Gazebo and on the
floor instead.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for _path in (HERE, HERE.parents[1] / "labs" / "python"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from robotlab.config import load_config                                    # noqa: E402
from robotlab.geometry import angle_diff                                   # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World  # noqa: E402


# ==============================================================================================
#  The build under test
# ==============================================================================================
@dataclass(frozen=True)
class Build:
    """The software configuration being tested — the knobs a real change would move."""

    name: str
    max_speed_m_s: float = 0.30
    stop_distance_m: float = 0.35     # collision-monitor stop zone, measured from the robot's edge
    control_hz: float = 10.0
    lookahead_m: float = 0.40
    heading_gain: float = 2.0
    collision_monitor: bool = True
    fail_closed: bool = True          # no scan data == an obstacle, not a clear road
    note: str = ""


def builds() -> dict[str, Build]:
    """The builds the suite knows about. Add yours; that is how you use this file."""
    return {
        "shipping": Build("shipping", note="the configuration the course arrives at"),
        "fast": Build("fast", max_speed_m_s=0.50, note="the speed limit raised, nothing else changed"),
        "no-monitor": Build("no-monitor", collision_monitor=False,
                            note="the safety layer disabled — what it was buying you"),
        "fail-open": Build("fail-open", fail_closed=False,
                           note="the monitor treats 'no scan' as 'road is clear'"),
        "slow-loop": Build("slow-loop", control_hz=2.0, note="a control loop starved by a heavy node"),
        "short-lookahead": Build("short-lookahead", lookahead_m=0.15,
                                 note="a plausible 'tighter tracking' tuning change"),
    }


# ==============================================================================================
#  Scenarios
# ==============================================================================================
@dataclass(frozen=True)
class Scenario:
    """One mission, with the thresholds that make it a test instead of a demo."""

    id: str
    description: str
    start: tuple[float, float, float]
    waypoints: tuple[tuple[float, float], ...]
    max_time_s: float
    outcome: str = "reach"            # "reach" the goal, or "stop-safely" short of it
    min_clearance_m: float = 0.04     # the tightest the robot may get to any obstacle
    max_interventions: int = 4        # how often the safety layer may fire before it is a failure
    max_blind_distance_m: float = 0.05   # how far it may drive with no scan data
    extra_obstacle: tuple[float, float, float] | None = None   # (x, y, radius)
    lidar_blackout_s: tuple[float, float] | None = None
    realistic: bool = False


KITCHEN_RUN = ((2.6, 1.5), (3.5, 1.65), (4.3, 2.3), (5.0, 2.3))


def suite() -> list[Scenario]:
    """karmel's acceptance suite: the missions the final robot claims it can do."""
    return [
        Scenario("S1-kitchen", "Living room to the kitchen, through the lower doorway",
                 (1.0, 1.3, 0.0), KITCHEN_RUN, 60.0),
        Scenario("S2-study", "Living room to the study desk, through the upper-left doorway",
                 (1.0, 1.3, 0.0), ((1.6, 2.4), (2.05, 3.2), (1.8, 3.8), (0.9, 3.9)), 60.0),
        Scenario("S3-bedroom", "Living room to the bedroom, the long way through two doorways",
                 (1.0, 1.3, 0.0), ((2.05, 3.2), (3.0, 4.0), (3.9, 4.0), (4.2, 3.2)), 90.0),
        Scenario("S4-return", "Kitchen back to the dock in the living room",
                 (5.0, 2.3, 3.14), ((4.3, 2.3), (3.5, 1.65), (2.6, 1.5), (1.0, 1.3)), 60.0),
        Scenario("S5-chair", "The kitchen run with a chair narrowing the doorway — still passable",
                 (1.0, 1.3, 0.0), ((2.6, 1.5), (3.5, 1.80), (4.3, 2.3), (5.0, 2.3)), 75.0,
                 extra_obstacle=(3.4, 1.30, 0.20)),
        Scenario("S6-blocked", "The doorway is blocked: the robot must STOP, not arrive",
                 (1.0, 1.3, 0.0), KITCHEN_RUN, 40.0, outcome="stop-safely",
                 extra_obstacle=(3.45, 1.65, 0.40)),
        Scenario("S7-blind", "Kitchen run with the LiDAR dead from t=6 s to t=9.5 s, chair present",
                 (1.0, 1.3, 0.0), ((2.6, 1.5), (3.5, 1.80), (4.3, 2.3), (5.0, 2.3)), 75.0,
                 extra_obstacle=(3.4, 1.30, 0.20), lidar_blackout_s=(6.0, 9.5)),
        Scenario("S8-worn", "Kitchen run on a worn robot: mismatched motors, slip, battery sag",
                 (1.0, 1.3, 0.0), KITCHEN_RUN, 75.0, realistic=True),
    ]


def scenario_world(scenario: Scenario) -> World:
    """The apartment, plus whatever this scenario leaves lying in the way."""
    world = World.apartment()
    if scenario.extra_obstacle is None:
        return world
    circles = np.vstack([world.circles, np.array([scenario.extra_obstacle])])
    return World.from_segments(world.segments, circles, world.landmarks,
                               list(range(len(world.landmarks))))


# ==============================================================================================
#  Metrics
# ==============================================================================================
@dataclass
class Metrics:
    """What a run produced. Everything here is a number you can put a threshold on."""

    scenario: str
    build: str
    seed: int
    reached: bool = False
    collided: bool = False
    gave_up: bool = False
    duration_s: float = 0.0
    distance_m: float = 0.0
    min_clearance_m: float = float("inf")
    interventions: int = 0          # rising edges: how many times the stop zone had to fire
    blind_distance_m: float = 0.0   # distance driven with no usable scan — an invariant, not an outcome
    final_error_m: float = float("inf")

    def failures(self, scenario: Scenario) -> list[str]:
        """Every threshold this run violated. Empty list = the scenario passed."""
        out: list[str] = []
        if scenario.outcome == "reach" and not self.reached:
            out.append(f"did not reach the goal (stopped {self.final_error_m:.2f} m away)")
        if scenario.outcome == "stop-safely":
            if self.reached:
                out.append("reached a goal it should not have been able to reach")
            elif self.interventions == 0:
                out.append("stopped without the safety layer ever firing — it was luck, not design")
        if self.collided:
            out.append("collided")
        if self.duration_s > scenario.max_time_s:
            out.append(f"took {self.duration_s:.1f} s (limit {scenario.max_time_s:.0f} s)")
        if self.min_clearance_m < scenario.min_clearance_m:
            out.append(f"clearance {self.min_clearance_m * 100:.1f} cm "
                       f"(limit {scenario.min_clearance_m * 100:.0f} cm)")
        if self.interventions > scenario.max_interventions:
            out.append(f"{self.interventions} safety interventions (limit {scenario.max_interventions})")
        if self.blind_distance_m > scenario.max_blind_distance_m:
            out.append(f"drove {self.blind_distance_m:.2f} m with no scan data "
                       f"(limit {scenario.max_blind_distance_m:.2f} m)")
        return out


# ==============================================================================================
#  The controller under test: pure pursuit + a collision-monitor stop zone
# ==============================================================================================
def front_obstacle_m(base: SimBase, half_angle_rad: float = 0.5) -> float:  # noqa: D401
    """Closest LiDAR return within +/- ``half_angle_rad`` of straight ahead, in metres."""
    scan = base.scan()
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = np.asarray(scan.angles, dtype=float)
    ahead = np.abs(np.arctan2(np.sin(angles), np.cos(angles))) <= half_angle_rad
    valid = ahead & np.isfinite(ranges)
    return float(ranges[valid].min()) if valid.any() else float("inf")


def pursue(pose: tuple[float, float, float], target: tuple[float, float], build: Build) -> tuple[float, float]:
    """Pure pursuit, reduced to the two lines that matter: turn towards it, slow down while turning."""
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    heading_error = angle_diff(math.atan2(dy, dx), pose[2])
    w = max(-2.0, min(2.0, build.heading_gain * heading_error))
    v = 0.0 if abs(heading_error) > 0.8 else build.max_speed_m_s * max(0.0, math.cos(heading_error))
    return v, w


def run_scenario(scenario: Scenario, build: Build, seed: int = 0, dt: float = 0.02) -> Metrics:
    """Drive the mission in the simulator and measure it. Deterministic for a given seed."""
    cfg = load_config()
    world = scenario_world(scenario)
    params = DiffDriveParams.realistic(cfg) if scenario.realistic else DiffDriveParams.ideal(cfg)
    sensors = SensorParams.realistic(cfg) if scenario.realistic else SensorParams.ideal(cfg)
    base = SimBase(DiffDriveSim(world, params, sensors, pose=scenario.start, seed=seed), dt=dt)

    radius = cfg.chassis.footprint_radius_m
    wheel_r, wheel_b = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    control_every = max(1, round(1.0 / (build.control_hz * dt)))
    metrics = Metrics(scenario.id, build.name, seed)
    waypoints = list(scenario.waypoints)
    index = 0
    wheels = (0.0, 0.0)
    previous = (scenario.start[0], scenario.start[1])
    was_blocked = False
    blind = False
    stalled_steps = 0
    give_up_steps = round(5.0 / dt)      # 5 s without progress = the mission has failed, stop the run

    for step in range(int(scenario.max_time_s / dt) + 1):
        pose = tuple(base.true_pose)
        if step % control_every == 0:
            blackout = (scenario.lidar_blackout_s is not None
                        and scenario.lidar_blackout_s[0] <= base.sim.t <= scenario.lidar_blackout_s[1])
            ahead = None if blackout else front_obstacle_m(base)
            blind = ahead is None
            while index < len(waypoints) - 1 and math.dist(pose[:2], waypoints[index]) < build.lookahead_m:
                index += 1
            v, w = pursue(pose, waypoints[index], build)
            blocked = False
            if build.collision_monitor:
                # No data is not "the road is clear". A fail-open monitor drives blind.
                blocked = build.fail_closed if ahead is None else ahead < build.stop_distance_m + radius
                if blocked:
                    v, w = 0.0, 0.0                  # the stop zone overrules the controller
            # Count rising EDGES: "the stop zone fired 3 times" is information, "it was active for
            # 349 control ticks" is the same event counted at the loop rate.
            metrics.interventions += int(blocked and not was_blocked)
            was_blocked = blocked
            wheels = ((v - w * wheel_b / 2) / wheel_r, (v + w * wheel_b / 2) / wheel_r)

        base.set_wheel_velocity(*wheels)
        base.read()
        pose = tuple(base.true_pose)
        moved = math.dist(pose[:2], previous)
        metrics.distance_m += moved
        if blind:
            metrics.blind_distance_m += moved
        stalled_steps = 0 if moved > 1e-4 else stalled_steps + 1
        previous = pose[:2]
        metrics.collided |= base.sim.collided
        clearance = float(world.distance_to_obstacles([[pose[0], pose[1]]])[0]) - radius
        metrics.min_clearance_m = min(metrics.min_clearance_m, clearance)
        metrics.final_error_m = math.dist(pose[:2], waypoints[-1])
        if metrics.final_error_m < 0.25:
            metrics.reached = True
            break
        if stalled_steps >= give_up_steps:
            metrics.gave_up = True
            break

    metrics.duration_s = base.sim.t
    base.close()
    return metrics


# ==============================================================================================
#  Running the suite and comparing two runs
# ==============================================================================================
@dataclass
class SuiteResult:
    build: str
    runs: list[Metrics] = field(default_factory=list)

    def by_scenario(self) -> dict[str, list[Metrics]]:
        out: dict[str, list[Metrics]] = {}
        for run in self.runs:
            out.setdefault(run.scenario, []).append(run)
        return out

    @property
    def passed(self) -> int:
        scenarios = {s.id: s for s in suite()}
        return sum(not m.failures(scenarios[m.scenario]) for m in self.runs)


def run_suite(build: Build, seeds: int = 1, only: str | None = None) -> SuiteResult:
    result = SuiteResult(build.name)
    for scenario in suite():
        if only and only not in scenario.id:
            continue
        for seed in range(seeds):
            result.runs.append(run_scenario(scenario, build, seed))
    return result


def compare(before: SuiteResult, after: SuiteResult) -> list[str]:
    """What got worse. A regression report names the metric and the size of the change."""
    scenarios = {s.id: s for s in suite()}
    lines: list[str] = []
    for sid, runs in after.by_scenario().items():
        old = before.by_scenario().get(sid)
        if not old:
            continue
        scenario = scenarios[sid]
        old_pass = all(not m.failures(scenario) for m in old)
        new_pass = all(not m.failures(scenario) for m in runs)
        old_time = sum(m.duration_s for m in old) / len(old)
        new_time = sum(m.duration_s for m in runs) / len(runs)
        old_clear = min(m.min_clearance_m for m in old)
        new_clear = min(m.min_clearance_m for m in runs)
        if old_pass and not new_pass:
            why = "; ".join(sorted({f for m in runs for f in m.failures(scenario)}))
            lines.append(f"REGRESSION {sid}: passed before, fails now — {why}")
        elif not old_pass and new_pass:
            lines.append(f"FIXED      {sid}: failed before, passes now")
        if new_clear < old_clear - 0.02:
            lines.append(f"WORSE      {sid}: clearance {old_clear * 100:.1f} -> {new_clear * 100:.1f} cm")
        if new_time > old_time * 1.25 + 1.0:
            lines.append(f"WORSE      {sid}: {old_time:.1f} -> {new_time:.1f} s")
    return lines


# ==============================================================================================
#  Output
# ==============================================================================================
def print_runs(result: SuiteResult) -> int:
    scenarios = {s.id: s for s in suite()}
    print(f"build: {result.build}")
    print(f"{'scenario':12} {'seed':>4} {'reach':>5} {'t s':>6} {'dist m':>7} {'clear cm':>8} "
          f"{"stops":>5} {"blind m":>7}  verdict")
    failed = 0
    for m in result.runs:
        problems = m.failures(scenarios[m.scenario])
        failed += bool(problems)
        verdict = "PASS" if not problems else "FAIL: " + "; ".join(problems)
        print(f"{m.scenario:12} {m.seed:>4} {str(m.reached):>5} {m.duration_s:>6.1f} "
              f"{m.distance_m:>7.2f} {m.min_clearance_m * 100:>8.1f} {m.interventions:>5} {m.blind_distance_m:>7.2f}  {verdict}")
    print(f"\n{len(result.runs) - failed}/{len(result.runs)} runs passed")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["list", "builds", "run", "compare"])
    ap.add_argument("names", nargs="*", help="two build names, for compare")
    ap.add_argument("--build", default="shipping")
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--only", default=None, help="run only scenarios whose id contains this")
    args = ap.parse_args(argv)

    known = builds()
    if args.command == "list":
        for s in suite():
            extra = []
            if s.extra_obstacle:
                extra.append("chair in the way")
            if s.lidar_blackout_s:
                extra.append(f"LiDAR dead {s.lidar_blackout_s[0]:.0f}-{s.lidar_blackout_s[1]:.0f} s")
            if s.realistic:
                extra.append("worn hardware")
            print(f"{s.id:12} {s.description}")
            print(f"{'':12} limits: {s.max_time_s:.0f} s, clearance >= {s.min_clearance_m * 100:.0f} cm, "
                  f"<= {s.max_interventions} interventions"
                  + (f"; faults: {', '.join(extra)}" if extra else ""))
        return 0
    if args.command == "builds":
        for name, b in known.items():
            print(f"{name:16} speed {b.max_speed_m_s:.2f} m/s  stop {b.stop_distance_m:.2f} m  "
                  f"loop {b.control_hz:.0f} Hz  lookahead {b.lookahead_m:.2f} m  "
                  f"monitor {'on' if b.collision_monitor else 'OFF'}   {b.note}")
        return 0
    if args.command == "run":
        if args.build not in known:
            raise SystemExit(f"unknown build '{args.build}' (try: {', '.join(known)})")
        return print_runs(run_suite(known[args.build], args.seeds, args.only))

    if len(args.names) != 2 or any(n not in known for n in args.names):
        raise SystemExit(f"compare needs two build names from: {', '.join(known)}")
    before = run_suite(known[args.names[0]], args.seeds, args.only)
    after = run_suite(known[args.names[1]], args.seeds, args.only)
    print(f"{args.names[0]}: {before.passed}/{len(before.runs)} passed")
    print(f"{args.names[1]}: {after.passed}/{len(after.runs)} passed\n")
    lines = compare(before, after)
    if not lines:
        print("no regressions")
        return 0
    for line in lines:
        print(line)
    return 1 if any(line.startswith("REGRESSION") for line in lines) else 0


if __name__ == "__main__":
    raise SystemExit(main())
