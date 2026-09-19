"""Bringing the whole robot up, and knowing whether it is actually up.

Two things that are the same problem seen from both ends:

* **bringup** — twelve nodes with dependencies, started in the right order, each with a
  "ready" condition, so that "the robot is on" is a state and not a hope;
* **health** — every node reports a ``diagnostic_msgs/DiagnosticStatus`` at a known rate, the
  aggregator takes the worst, staleness is a level of its own, and the robot's overall state
  (INIT / READY / DEGRADED / FAULT / ESTOP) decides which missions may start.

    py 20-final-robot/code/bringup_health.py order                 # dependency waves
    py 20-final-robot/code/bringup_health.py timeline              # who is ready when
    py 20-final-robot/code/bringup_health.py timeline --fail lidar # ...and what that costs
    py 20-final-robot/code/bringup_health.py health                # a 44 s run with injected faults
    py 20-final-robot/code/bringup_health.py graph                 # mermaid dependency graph
    py 20-final-robot/code/bringup_health.py systemd               # the unit file for the Pi

Levels are the ROS 2 ``diagnostic_msgs`` ones: OK=0, WARN=1, ERROR=2, STALE=3.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field, replace
from enum import IntEnum

# ==============================================================================================
#  Diagnostics
# ==============================================================================================


class Level(IntEnum):
    """``diagnostic_msgs/DiagnosticStatus`` levels. The numeric values are part of the wire format."""

    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3

    def __str__(self) -> str:
        return self.name


@dataclass(frozen=True)
class Status:
    """What one component says about itself, and when it last said it."""

    name: str
    level: Level
    message: str
    stamp_s: float


def staleness_level(last_stamp_s: float, now_s: float, period_s: float, tolerance: float = 3.0) -> Level | None:
    """STALE when a component has missed ``tolerance`` reporting periods, else None.

    A silent node is not an OK node. This is the single most common integration bug: a crashed
    driver leaves its last OK message in the aggregator forever unless somebody times it out.
    """
    return Level.STALE if now_s - last_stamp_s > tolerance * period_s else None


def aggregate(statuses: list[Status]) -> Level:
    """The whole-robot level is the worst component level — exactly what diagnostic_aggregator does."""
    return max((s.level for s in statuses), default=Level.STALE)


# ==============================================================================================
#  The units of the bringup
# ==============================================================================================
@dataclass(frozen=True)
class Unit:
    """One thing the bringup starts. ``ready_s`` is measured from when its dependencies are ready."""

    name: str
    requires: tuple[str, ...] = ()
    ready_s: float = 1.0
    report_hz: float = 1.0
    criticality: str = "required"   # required | degraded | optional
    ready_check: str = ""           # how the bringup KNOWS it is up
    note: str = ""


def karmel_units() -> list[Unit]:
    """karmel's full bringup: base, sensors, localization, navigation, arm, agent, ops."""
    return [
        Unit("pico_link", (), 2.0, 1.0, "required",
             "serial hello 'I <fw> <proto>' answered within 2 s",
             "the Pico is the only thing that can stop the motors in 300 ms"),
        Unit("ros2_control", ("pico_link",), 3.0, 10.0, "required",
             "controller_manager lists diff_drive_controller as active"),
        Unit("robot_state_publisher", (), 1.5, 1.0, "required",
             "/robot_description latched and static tf present"),
        Unit("lidar", (), 4.0, 1.0, "required",
             "/scan at >= 8 Hz for 2 s", "RPLIDAR C1 needs ~3 s to spin up"),
        Unit("imu", (), 2.0, 1.0, "degraded",
             "/imu/data at >= 40 Hz", "without it the EKF runs on wheels only"),
        Unit("camera", (), 3.0, 1.0, "degraded",
             "/camera/image_raw/compressed at >= 10 Hz", "no camera = no perception skills"),
        Unit("ekf", ("ros2_control", "imu", "robot_state_publisher"), 1.0, 1.0, "required",
             "odom->base_footprint tf newer than 0.2 s"),
        Unit("map_server", (), 1.0, 1.0, "required", "/map latched"),
        Unit("amcl", ("map_server", "lidar", "ekf"), 2.5, 1.0, "required",
             "map->odom tf published, covariance below threshold"),
        Unit("nav2", ("amcl", "ros2_control"), 5.0, 1.0, "required",
             "all lifecycle nodes ACTIVE, navigate_to_pose action server available"),
        Unit("collision_monitor", ("lidar", "ros2_control"), 1.0, 10.0, "required",
             "/cmd_vel published (zero counts) at 20 Hz",
             "safety layer: nothing autonomous starts before this"),
        Unit("arm", ("ros2_control",), 4.0, 1.0, "degraded",
             "all 6 servos answer a ping, joint_states at 50 Hz"),
        Unit("skill_server", ("nav2", "arm", "camera"), 1.5, 1.0, "degraded",
             "every skill action server advertised"),
        Unit("agent_bridge", ("skill_server",), 1.0, 0.2, "optional",
             "model API reachable, tool schema loaded", "no internet = no new missions, robot still safe"),
        Unit("health_monitor", (), 0.5, 1.0, "required",
             "/diagnostics_agg published", "started first, dies last"),
        Unit("rosbag2", ("lidar", "ros2_control"), 1.0, 0.2, "optional",
             "the .mcap file is growing"),
    ]


# ==============================================================================================
#  Start order
# ==============================================================================================
def start_waves(units: list[Unit]) -> list[list[str]]:
    """Group the units into waves that can start in parallel (a Kahn layering of the DAG).

    Wave 0 depends on nothing; wave n depends only on waves < n. Within a wave the order is
    alphabetical, so the answer is the same on every machine and diffs are readable.
    """
    by_name = {u.name: u for u in units}
    unknown = {r for u in units for r in u.requires if r not in by_name}
    if unknown:
        raise ValueError(f"unknown dependencies: {', '.join(sorted(unknown))}")
    waves: list[list[str]] = []
    done: set[str] = set()
    remaining = dict(by_name)
    while remaining:
        wave = sorted(n for n, u in remaining.items() if set(u.requires) <= done)
        if not wave:
            raise ValueError(f"dependency cycle among: {', '.join(sorted(remaining))}")
        waves.append(wave)
        done |= set(wave)
        remaining = {n: u for n, u in remaining.items() if n not in done}
    return waves


def ready_times(units: list[Unit], failed: frozenset[str] = frozenset()) -> dict[str, float | None]:
    """When each unit is ready, in seconds from launch. ``None`` = never (it or a dependency failed)."""
    by_name = {u.name: u for u in units}
    ready: dict[str, float | None] = {}
    for wave in start_waves(units):
        for name in wave:
            unit = by_name[name]
            if name in failed:
                ready[name] = None
                continue
            deps = [ready[r] for r in unit.requires]
            ready[name] = None if any(d is None for d in deps) else max([0.0, *[d for d in deps if d is not None]]) + unit.ready_s
    return ready


# ==============================================================================================
#  The robot's state
# ==============================================================================================
class RobotState(IntEnum):
    """The one state a human, a mission and the agent all read before doing anything."""

    INIT = 0        # still coming up
    READY = 1       # everything required is OK
    DEGRADED = 2    # a non-essential component is down: reduced missions, reduced speed
    FAULT = 3       # something required is ERROR or STALE: no motion
    ESTOP = 4       # the button is pressed: actuator power is off, software only mirrors it

    def __str__(self) -> str:
        return self.name


#: What each state allows. The agent reads this table, it does not invent policy.
MISSION_POLICY: dict[RobotState, tuple[str, float]] = {
    RobotState.INIT: ("no missions; bringup still in progress", 0.0),
    RobotState.READY: ("all missions", 0.50),
    RobotState.DEGRADED: ("navigation only, no manipulation; return-to-dock always allowed", 0.25),
    RobotState.FAULT: ("no missions; stop and report", 0.0),
    RobotState.ESTOP: ("no missions; explicit human re-enable required", 0.0),
}


def robot_state(units: list[Unit], levels: dict[str, Level], *, estop: bool = False,
                bringup_complete: bool = True, bringup_timed_out: bool = False) -> RobotState:
    """Turn per-component levels into the one state everything else keys off.

    The order of the tests *is* the policy:

    1. the e-stop wins over everything — it is a fact about the hardware, not an opinion;
    2. while the bringup is still within its deadline, missing components mean INIT, not FAULT
       (a driver that needs 4 s to spin up is not a fault for those 4 s);
    3. a *required* component at ERROR or STALE is a FAULT, however healthy the rest is;
    4. DEGRADED is a designed state with its own mission policy, not a grey area.
    """
    if estop:
        return RobotState.ESTOP
    if not (bringup_complete or bringup_timed_out):
        return RobotState.INIT
    by_name = {u.name: u for u in units}
    for name, level in levels.items():
        unit = by_name.get(name)
        if unit is not None and unit.criticality == "required" and level >= Level.ERROR:
            return RobotState.FAULT
    for name, level in levels.items():
        unit = by_name.get(name)
        if unit is None or unit.criticality == "optional":
            continue
        if level >= Level.WARN:
            return RobotState.DEGRADED
    return RobotState.READY


# ==============================================================================================
#  A simulated session with injected faults
# ==============================================================================================
@dataclass
class Fault:
    """Something that happens to a component at a given time."""

    at_s: float
    unit: str
    level: Level
    message: str


@dataclass
class HealthMonitor:
    """Keeps the last status per component and applies the staleness rule at every tick."""

    units: list[Unit]
    statuses: dict[str, Status] = field(default_factory=dict)
    estop: bool = False

    def report(self, status: Status) -> None:
        self.statuses[status.name] = status

    def levels(self, now_s: float) -> dict[str, Level]:
        out: dict[str, Level] = {}
        for unit in self.units:
            status = self.statuses.get(unit.name)
            if status is None:
                out[unit.name] = Level.STALE
                continue
            stale = staleness_level(status.stamp_s, now_s, 1.0 / unit.report_hz)
            out[unit.name] = stale or status.level
        return out

    def state(self, now_s: float, bringup_complete: bool = True,
              bringup_timed_out: bool = False) -> RobotState:
        return robot_state(self.units, self.levels(now_s), estop=self.estop,
                           bringup_complete=bringup_complete, bringup_timed_out=bringup_timed_out)


def default_faults() -> list[Fault]:
    """A realistic bad afternoon, in the order these things actually happen."""
    return [
        Fault(20.0, "camera", Level.ERROR, "USB device disconnected (/dev/video0 gone)"),
        Fault(24.0, "imu", Level.WARN, "i2c read retried 3 times"),
        Fault(30.0, "lidar", Level.STALE, "stops publishing — the USB hub browned out"),
        Fault(38.0, "lidar", Level.OK, "scanning again after a replug"),
    ]


#: A bringup that has not finished by this time is not slow, it is broken.
BRINGUP_DEADLINE_S = 30.0


def simulate(units: list[Unit], faults: list[Fault], duration_s: float = 44.0,
             tick_s: float = 1.0, estop_at: float | None = None) -> list[tuple[float, RobotState, str]]:
    """Run the bringup and the session, returning only the moments the robot's STATE changed."""
    ready = ready_times(units)
    monitor = HealthMonitor(units)
    pending = sorted(faults, key=lambda f: f.at_s)
    forced: dict[str, tuple[Level, str]] = {}
    transitions: list[tuple[float, RobotState, str]] = []
    previous: RobotState | None = None
    t = 0.0
    while t <= duration_s + 1e-9:
        while pending and pending[0].at_s <= t:
            fault = pending.pop(0)
            forced[fault.unit] = (fault.level, fault.message)
        if estop_at is not None and t >= estop_at:
            monitor.estop = True
        reason = "bringup"
        for unit in units:
            when = ready.get(unit.name)
            if when is None or t + 1e-9 < when:
                continue                                   # not up yet: stays STALE
            level, message = forced.get(unit.name, (Level.OK, "ok"))
            if level == Level.STALE:
                continue                                   # stop reporting: the monitor times it out
            monitor.report(Status(unit.name, level, message, t))
        complete = all(w is not None and t + 1e-9 >= w for w in ready.values())
        state = monitor.state(t, bringup_complete=complete, bringup_timed_out=t >= BRINGUP_DEADLINE_S)
        if state != previous:
            worst = sorted(monitor.levels(t).items(), key=lambda kv: -kv[1])[0]
            reason = f"{worst[0]} is {worst[1]}" if worst[1] > Level.OK else "everything OK"
            if monitor.estop:
                reason = "e-stop pressed"
            transitions.append((t, state, reason))
            previous = state
        t += tick_s
    return transitions


# ==============================================================================================
#  Output
# ==============================================================================================
SYSTEMD_UNIT = """\
# /etc/systemd/system/karmel.service    (sudo systemctl enable --now karmel)
[Unit]
Description=karmel bringup (ROS 2 Jazzy)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=karmel
# The environment a login shell would have given you; a systemd service has none of it.
Environment=ROS_DOMAIN_ID=42
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=ROS_LOCALHOST_ONLY=0
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && \\
    source /home/karmel/ros2_ws/install/setup.bash && \\
    ros2 launch karmel_bringup robot.launch.py sim:=false use_ekf:=true'
# Stop means stop: SIGINT lets the nodes shut down cleanly (zero the wheels, park the arm).
KillSignal=SIGINT
TimeoutStopSec=20
Restart=on-failure
RestartSec=5
# Do NOT restart forever in a loop: five failures in two minutes means a human is needed.
StartLimitBurst=5
StartLimitIntervalSec=120

[Install]
WantedBy=multi-user.target
"""


def print_order(units: list[Unit]) -> None:
    for i, wave in enumerate(start_waves(units)):
        print(f"wave {i}: {', '.join(wave)}")


def print_timeline(units: list[Unit], failed: frozenset[str]) -> int:
    ready = ready_times(units, failed)
    by_name = {u.name: u for u in units}
    print(f"{'unit':20} {'ready at':>9}  {'criticality':12} ready check")
    for name, when in sorted(ready.items(), key=lambda kv: (kv[1] is None, kv[1] or 0.0, kv[0])):
        unit = by_name[name]
        stamp = f"{when:6.1f} s" if when is not None else "  NEVER"
        print(f"{name:22} {stamp:>9}  {unit.criticality:12} {unit.ready_check}")
    blocked = [n for n, w in ready.items() if w is None]
    complete = max((w for w in ready.values() if w is not None), default=0.0)
    print(f"\nbringup complete at {complete:.1f} s" if not blocked else
          f"\nbringup never completes: {', '.join(sorted(blocked))} never become ready")
    if blocked:
        levels = {n: (Level.STALE if n in blocked else Level.OK) for n in ready}
        state = robot_state(units, levels)
        allowed, speed = MISSION_POLICY[state]
        print(f"robot state {state}: {allowed} (speed limit {speed:.2f} m/s)")
        return 1
    return 0


def print_health(units: list[Unit], estop_at: float | None) -> None:
    print(f"{'t':>6}  {'state':9} why")
    for t, state, why in simulate(units, default_faults(), estop_at=estop_at):
        allowed, speed = MISSION_POLICY[state]
        print(f"{t:6.1f}  {str(state):9} {why}")
        print(f"{'':6}  {'':9}   -> {allowed}; speed limit {speed:.2f} m/s")


def mermaid(units: list[Unit]) -> str:
    style = {"required": "((%s))", "degraded": "(%s)", "optional": "[%s]"}
    lines = ["flowchart LR"]
    for unit in units:
        shape = style[unit.criticality] % unit.name
        lines.append(f"    {unit.name}{shape}")
    for unit in units:
        for dep in unit.requires:
            lines.append(f"    {dep} --> {unit.name}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["order", "timeline", "health", "graph", "systemd"])
    ap.add_argument("--fail", nargs="*", default=[], help="units that never come up (timeline)")
    ap.add_argument("--estop-at", type=float, default=None, help="press the e-stop at this time (health)")
    ap.add_argument("--slow", nargs="*", default=[],
                    help="units that take 10x longer to become ready (timeline)")
    args = ap.parse_args(argv)

    units = karmel_units()
    if args.slow:
        units = [replace(u, ready_s=u.ready_s * 10) if u.name in args.slow else u for u in units]

    if args.command == "order":
        print_order(units)
        return 0
    if args.command == "timeline":
        return print_timeline(units, frozenset(args.fail))
    if args.command == "health":
        print_health(units, args.estop_at)
        return 0
    if args.command == "graph":
        print("```mermaid")
        print(mermaid(units))
        print("```")
        return 0
    print(SYSTEMD_UNIT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
