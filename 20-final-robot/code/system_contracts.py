"""karmel's final-robot interface contracts, as data you can check.

An architecture document that lives in a wiki rots. This one is a Python file: every interface
between two parts of the final robot is a ``Contract`` with a type, a frame, units, a rate, a QoS
profile, an owner and a documented timeout behavior — and a handful of checks that fail loudly when
two parts disagree.

    py 20-final-robot/code/system_contracts.py table          # the interface table (markdown)
    py 20-final-robot/code/system_contracts.py check          # run every check on the real system
    py 20-final-robot/code/system_contracts.py check --broken # the same checks on a broken variant
    py 20-final-robot/code/system_contracts.py bandwidth      # bytes/s per link and per bus
    py 20-final-robot/code/system_contracts.py chain          # end-to-end latency budgets
    py 20-final-robot/code/system_contracts.py graph          # mermaid diagram of the system

Frames follow REP-105, units SI (REP-103). Message sizes are computed from the field layout of the
ROS 2 Jazzy messages, not guessed: see ``payload_bytes`` on each contract.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field, replace

# REP-105 frames plus karmel's sensor and arm frames.
KNOWN_FRAMES = {
    "map", "odom", "base_footprint", "base_link",
    "imu_link", "laser", "camera_link", "camera_optical_frame",
    "arm_base_link", "gripper_link",
}

RELIABLE, BEST_EFFORT = "reliable", "best_effort"
VOLATILE, TRANSIENT_LOCAL = "volatile", "transient_local"


@dataclass(frozen=True)
class Qos:
    """The three QoS settings that actually decide whether two nodes talk (plus the deadline)."""

    reliability: str = RELIABLE
    durability: str = VOLATILE
    depth: int = 10
    deadline_s: float | None = None  # publisher: offered; subscriber: requested

    def __str__(self) -> str:
        short = {RELIABLE: "rel", BEST_EFFORT: "best", VOLATILE: "vol", TRANSIENT_LOCAL: "t-local"}
        out = f"{short[self.reliability]}/{short[self.durability]}/{self.depth}"
        return out + (f"/dl {self.deadline_s:g}s" if self.deadline_s else "")


SENSOR_DATA = Qos(BEST_EFFORT, VOLATILE, 5)      # rclpy's qos_profile_sensor_data
LATCHED = Qos(RELIABLE, TRANSIENT_LOCAL, 1)      # maps, robot_description, static tf


@dataclass(frozen=True)
class Endpoint:
    """One node's end of a link."""

    node: str
    qos: Qos = Qos()
    rate_hz: float | None = None       # publisher: what it publishes at; subscriber: what it needs
    compute: str = "pi"                # pi | pico | jetson | cloud | arm


@dataclass(frozen=True)
class Contract:
    """One interface of the final robot."""

    topic: str
    msg_type: str
    units: str
    frame_id: str | None
    payload_bytes: int
    publishers: tuple[Endpoint, ...]
    subscribers: tuple[Endpoint, ...] = ()
    on_timeout: str = "—"
    multi_writer_ok: bool = False
    kind: str = "topic"                # topic | action | service | serial
    layer: str = "sense"               # sense | think | act | safety | ops

    @property
    def rate_hz(self) -> float:
        return max((p.rate_hz or 0.0) for p in self.publishers)

    @property
    def bytes_per_s(self) -> float:
        return sum((p.rate_hz or 0.0) * self.payload_bytes for p in self.publishers)


# --------------------------------------------------------------------------------------------
#  Message sizes (ROS 2 Jazzy field layouts; header = 4+4 stamp + frame_id string)
# --------------------------------------------------------------------------------------------
HEADER = 8 + 24                                   # stamp + a short frame_id, CDR-padded
TWIST_STAMPED = HEADER + 6 * 8                    # geometry_msgs/TwistStamped
ODOMETRY = HEADER + 24 + 7 * 8 + 36 * 8 + 6 * 8 + 36 * 8
LASER_SCAN_360 = HEADER + 7 * 4 + 4 + 360 * 4 + 4 + 360 * 4
IMU = HEADER + 4 * 8 + 9 * 8 + 3 * 8 + 9 * 8 + 3 * 8 + 9 * 8
JOINT_STATE_2 = HEADER + 2 * 24 + 3 * 2 * 8
IMAGE_COMPRESSED_VGA = HEADER + 16 + 45_000       # VGA JPEG at quality 80, measured range 30–60 kB
IMAGE_RAW_VGA = HEADER + 24 + 640 * 480 * 3
TF_MESSAGE_1 = HEADER + 24 + 7 * 8
BATTERY_STATE = HEADER + 10 * 4 + 6 * 4 + 3 * 24
DIAGNOSTIC_ARRAY = HEADER + 12 * 200               # 12 statuses with a few key/values each
POSE_STAMPED = HEADER + 7 * 8
MAP_2000 = HEADER + 40 + 2000 * 2000               # 100 m x 100 m at 5 cm, latched


# --------------------------------------------------------------------------------------------
#  The final robot
# --------------------------------------------------------------------------------------------
def karmel_system() -> list[Contract]:
    """Every interface of the final robot: base + LiDAR + camera + IMU + arm + agent."""
    return [
        # ---- sense ---------------------------------------------------------------------------
        Contract("/scan", "sensor_msgs/LaserScan", "m, rad", "laser", LASER_SCAN_360,
                 (Endpoint("sllidar_node", SENSOR_DATA, 10.0),),
                 (Endpoint("slam_toolbox", SENSOR_DATA, 5.0), Endpoint("amcl", SENSOR_DATA, 5.0),
                  Endpoint("local_costmap", SENSOR_DATA, 5.0), Endpoint("collision_monitor", SENSOR_DATA, 10.0)),
                 on_timeout="collision_monitor stops the robot after 1 s of no scan", layer="sense"),
        Contract("/imu/data", "sensor_msgs/Imu", "rad/s, m/s^2", "imu_link", IMU,
                 (Endpoint("bno055_node", SENSOR_DATA, 50.0),),
                 (Endpoint("ekf_filter_node", SENSOR_DATA, 20.0),),
                 on_timeout="EKF keeps predicting from wheel odometry; /diagnostics goes WARN", layer="sense"),
        Contract("/camera/image_raw/compressed", "sensor_msgs/CompressedImage", "JPEG bytes",
                 "camera_optical_frame", IMAGE_COMPRESSED_VGA,
                 (Endpoint("v4l2_camera", SENSOR_DATA, 15.0),),
                 (Endpoint("detector", SENSOR_DATA, 5.0, compute="jetson"),),
                 on_timeout="perception skills return PRECONDITION_FAILED; navigation continues", layer="sense"),
        Contract("/joint_states", "sensor_msgs/JointState", "rad, rad/s", None, JOINT_STATE_2,
                 (Endpoint("joint_state_broadcaster", Qos(RELIABLE, VOLATILE, 10), 50.0),),
                 (Endpoint("robot_state_publisher", Qos(RELIABLE, VOLATILE, 10), 10.0),),
                 on_timeout="tf stops updating the wheels; RViz shows a frozen robot", layer="sense"),
        Contract("/battery_state", "sensor_msgs/BatteryState", "V, A, %", "base_link", BATTERY_STATE,
                 (Endpoint("karmel_base", Qos(RELIABLE, VOLATILE, 5), 1.0),),
                 (Endpoint("health_monitor", Qos(RELIABLE, VOLATILE, 5), 0.2),
                  Endpoint("agent_bridge", Qos(RELIABLE, VOLATILE, 5), 0.2)),
                 on_timeout="health_monitor reports STALE and refuses new missions", layer="ops"),

        # ---- think ---------------------------------------------------------------------------
        Contract("/odom", "nav_msgs/Odometry", "m, m/s, rad/s", "odom", ODOMETRY,
                 (Endpoint("ekf_filter_node", Qos(RELIABLE, VOLATILE, 10), 30.0),),
                 (Endpoint("controller_server", Qos(RELIABLE, VOLATILE, 10), 20.0),
                  Endpoint("amcl", Qos(RELIABLE, VOLATILE, 10), 10.0)),
                 on_timeout="controller_server aborts the goal", layer="think"),
        Contract("/tf", "tf2_msgs/TFMessage", "m, quaternion", None, TF_MESSAGE_1,
                 (Endpoint("ekf_filter_node", Qos(RELIABLE, VOLATILE, 100), 30.0),
                  Endpoint("amcl", Qos(RELIABLE, VOLATILE, 100), 10.0),
                  Endpoint("robot_state_publisher", Qos(RELIABLE, VOLATILE, 100), 10.0)),
                 (Endpoint("controller_server", Qos(RELIABLE, VOLATILE, 100), 20.0),
                  Endpoint("local_costmap", Qos(RELIABLE, VOLATILE, 100), 10.0)),
                 on_timeout="tf2 lookups raise ExtrapolationException; Nav2 aborts",
                 multi_writer_ok=True, layer="think"),
        Contract("/map", "nav_msgs/OccupancyGrid", "cost 0..100", "map", MAP_2000,
                 (Endpoint("map_server", LATCHED, 0.0),),
                 (Endpoint("amcl", LATCHED, 0.0), Endpoint("global_costmap", LATCHED, 0.0)),
                 on_timeout="latched: a late subscriber still gets the last map", layer="think"),
        Contract("/goal_pose", "geometry_msgs/PoseStamped", "m, quaternion", "map", POSE_STAMPED,
                 (Endpoint("agent_bridge", Qos(RELIABLE, VOLATILE, 1), 0.2),),
                 (Endpoint("bt_navigator", Qos(RELIABLE, VOLATILE, 1), 0.0),),
                 on_timeout="—", layer="think"),
        Contract("navigate_to_pose", "nav2_msgs/action/NavigateToPose", "m, quaternion", "map", POSE_STAMPED,
                 (Endpoint("agent_bridge", Qos(RELIABLE, VOLATILE, 1), 0.1),),
                 (Endpoint("bt_navigator", Qos(RELIABLE, VOLATILE, 1), 0.0),),
                 on_timeout="goal aborts after the behavior tree's timeout; agent replans",
                 kind="action", layer="think"),
        Contract("/skill/pick", "karmel_interfaces/action/Pick", "object id", "map", POSE_STAMPED,
                 (Endpoint("agent_bridge", Qos(RELIABLE, VOLATILE, 1), 0.05),),
                 (Endpoint("skill_server", Qos(RELIABLE, VOLATILE, 1), 0.0),),
                 on_timeout="skill server torque-holds the arm and returns TIMEOUT",
                 kind="action", layer="act"),

        # ---- act -----------------------------------------------------------------------------
        Contract("/cmd_vel_nav", "geometry_msgs/TwistStamped", "m/s, rad/s", "base_link", TWIST_STAMPED,
                 (Endpoint("controller_server", Qos(RELIABLE, VOLATILE, 1), 20.0),),
                 (Endpoint("twist_mux", Qos(RELIABLE, VOLATILE, 1), 20.0),),
                 on_timeout="twist_mux drops to the next priority after 0.5 s", layer="act"),
        Contract("/cmd_vel_teleop", "geometry_msgs/TwistStamped", "m/s, rad/s", "base_link", TWIST_STAMPED,
                 (Endpoint("teleop_twist_keyboard", Qos(RELIABLE, VOLATILE, 1), 10.0),),
                 (Endpoint("twist_mux", Qos(RELIABLE, VOLATILE, 1), 10.0),),
                 on_timeout="twist_mux releases the lock after 0.5 s", layer="act"),
        Contract("/cmd_vel_smoothed", "geometry_msgs/TwistStamped", "m/s, rad/s", "base_link", TWIST_STAMPED,
                 (Endpoint("twist_mux", Qos(RELIABLE, VOLATILE, 1), 20.0),),
                 (Endpoint("collision_monitor", Qos(RELIABLE, VOLATILE, 1), 20.0),),
                 on_timeout="collision_monitor publishes zero", layer="act"),
        Contract("/cmd_vel", "geometry_msgs/TwistStamped", "m/s, rad/s", "base_link", TWIST_STAMPED,
                 (Endpoint("collision_monitor", Qos(RELIABLE, VOLATILE, 1), 20.0),),
                 (Endpoint("diff_drive_controller", Qos(RELIABLE, VOLATILE, 1), 20.0),),
                 on_timeout="diff_drive_controller halts after cmd_vel_timeout (0.5 s)", layer="act"),
        Contract("V <seq> <l_mrad_s> <r_mrad_s>", "serial line (labs/README.md protocol v1)",
                 "mrad/s", None, 32,
                 (Endpoint("karmel_hardware", Qos(RELIABLE, VOLATILE, 1), 50.0),),
                 (Endpoint("pico_firmware", Qos(RELIABLE, VOLATILE, 1), 50.0, compute="pico"),),
                 on_timeout="Pico watchdog stops the motors after 300 ms", kind="serial", layer="safety"),
        Contract("T <ms> <ticks...>", "serial telemetry (protocol v1)", "ticks, mrad/s, mV, mm", None, 64,
                 (Endpoint("pico_firmware", Qos(RELIABLE, VOLATILE, 1), 50.0, compute="pico"),),
                 (Endpoint("karmel_hardware", Qos(RELIABLE, VOLATILE, 1), 50.0),),
                 on_timeout="karmel_hardware reports ERROR to ros2_control; controllers deactivate",
                 kind="serial", layer="sense"),

        # ---- safety / ops ----------------------------------------------------------------------
        Contract("/estop", "std_msgs/Bool", "latched state", None, HEADER,
                 (Endpoint("karmel_base", Qos(RELIABLE, TRANSIENT_LOCAL, 1), 5.0),),
                 (Endpoint("health_monitor", Qos(RELIABLE, TRANSIENT_LOCAL, 1), 1.0),
                  Endpoint("bt_navigator", Qos(RELIABLE, TRANSIENT_LOCAL, 1), 1.0),
                  Endpoint("skill_server", Qos(RELIABLE, TRANSIENT_LOCAL, 1), 1.0)),
                 on_timeout="software mirror only — the relay already removed actuator power",
                 layer="safety"),
        Contract("/diagnostics", "diagnostic_msgs/DiagnosticArray", "levels", None, DIAGNOSTIC_ARRAY,
                 (Endpoint("karmel_base", Qos(RELIABLE, VOLATILE, 10), 1.0),
                  Endpoint("sllidar_node", Qos(RELIABLE, VOLATILE, 10), 1.0),
                  Endpoint("ekf_filter_node", Qos(RELIABLE, VOLATILE, 10), 1.0)),
                 (Endpoint("diagnostic_aggregator", Qos(RELIABLE, VOLATILE, 10), 0.5),),
                 on_timeout="aggregator marks the missing analyzer STALE",
                 multi_writer_ok=True, layer="ops"),
    ]


def broken_system() -> list[Contract]:
    """The same system with four mistakes an integration week really produces."""
    system = []
    for c in karmel_system():
        if c.topic == "/cmd_vel":
            # (1) the agent also publishes /cmd_vel directly: two writers on the actuator topic
            c = replace(c, publishers=c.publishers + (Endpoint("agent_bridge", Qos(RELIABLE, VOLATILE, 1), 2.0),))
        if c.topic == "/scan":
            # (2) a subscriber asks for RELIABLE from a BEST_EFFORT sensor publisher: no data at all
            c = replace(c, subscribers=tuple(
                replace(s, qos=Qos(RELIABLE, VOLATILE, 10)) if s.node == "collision_monitor" else s
                for s in c.subscribers))
        if c.topic == "/odom":
            # (3) the EKF was slowed to 10 Hz but the controller still needs 20 Hz
            c = replace(c, publishers=(replace(c.publishers[0], rate_hz=10.0),))
        if c.topic == "/imu/data":
            # (4) frame_id typo: "imu" is not a frame in the URDF
            c = replace(c, frame_id="imu")
        system.append(c)
    return system


# --------------------------------------------------------------------------------------------
#  The checks
# --------------------------------------------------------------------------------------------
@dataclass
class Problem:
    topic: str
    rule: str
    detail: str

    def __str__(self) -> str:
        return f"[{self.rule}] {self.topic}: {self.detail}"


def qos_incompatibility(pub: Qos, sub: Qos) -> str | None:
    """Why this publisher and subscriber will not exchange data (DDS request/offer rules)."""
    if pub.reliability == BEST_EFFORT and sub.reliability == RELIABLE:
        return "publisher offers best_effort, subscriber requests reliable — no messages are delivered"
    if pub.durability == VOLATILE and sub.durability == TRANSIENT_LOCAL:
        return "publisher is volatile, subscriber requests transient_local — late joiners get nothing"
    if sub.deadline_s is not None and (pub.deadline_s is None or pub.deadline_s > sub.deadline_s):
        offered = "none" if pub.deadline_s is None else f"{pub.deadline_s:g}s"
        return f"offered deadline {offered} is longer than the requested {sub.deadline_s:g}s"
    return None


def check_system(system: list[Contract]) -> list[Problem]:
    """Every rule the final robot's interfaces must obey. Empty list = the architecture is coherent."""
    problems: list[Problem] = []
    for c in system:
        if len(c.publishers) > 1 and not c.multi_writer_ok:
            names = ", ".join(p.node for p in c.publishers)
            problems.append(Problem(c.topic, "single-writer",
                                    f"{len(c.publishers)} publishers ({names}) — who wins is a race"))
        if c.frame_id is not None and c.frame_id not in KNOWN_FRAMES:
            problems.append(Problem(c.topic, "frame",
                                    f"frame_id '{c.frame_id}' is not in the robot's tf tree"))
        if not c.subscribers:
            problems.append(Problem(c.topic, "orphan", "published but nobody subscribes"))
        for sub in c.subscribers:
            for pub in c.publishers:
                why = qos_incompatibility(pub.qos, sub.qos)
                if why:
                    problems.append(Problem(c.topic, "qos", f"{pub.node} -> {sub.node}: {why}"))
            if sub.rate_hz and c.rate_hz and sub.rate_hz > c.rate_hz + 1e-9:
                problems.append(Problem(c.topic, "rate",
                                        f"{sub.node} needs {sub.rate_hz:g} Hz, publishers give {c.rate_hz:g} Hz"))
        if c.on_timeout == "—" and c.layer in ("act", "safety"):
            problems.append(Problem(c.topic, "timeout", "an actuation interface with no documented timeout behavior"))
    return problems


# --------------------------------------------------------------------------------------------
#  Bandwidth and latency
# --------------------------------------------------------------------------------------------
def bandwidth(system: list[Contract]) -> list[tuple[str, float]]:
    """Bytes per second per interface, biggest first (one copy per subscriber is ignored: DDS
    multicasts on the loopback, but an inter-machine link pays per subscriber)."""
    return sorted(((c.topic, c.bytes_per_s) for c in system), key=lambda kv: -kv[1])


@dataclass
class Step:
    """One hop of a latency chain: a period (1/rate) plus the work the node does."""

    name: str
    rate_hz: float | None
    compute_s: float = 0.0

    @property
    def worst_case_s(self) -> float:
        return (1.0 / self.rate_hz if self.rate_hz else 0.0) + self.compute_s


@dataclass
class Chain:
    name: str
    budget_s: float
    steps: list[Step] = field(default_factory=list)

    @property
    def worst_case_s(self) -> float:
        return sum(s.worst_case_s for s in self.steps)

    @property
    def ok(self) -> bool:
        return self.worst_case_s <= self.budget_s


def obstacle_chain() -> Chain:
    """LiDAR sees a chair leg -> the wheels actually slow down."""
    return Chain("obstacle in the path -> wheels slow", budget_s=0.40, steps=[
        Step("LiDAR scan period", 10.0),
        Step("driver + DDS", None, 0.005),
        Step("collision_monitor tick", 20.0, 0.002),
        Step("diff_drive_controller update", 50.0, 0.001),
        Step("serial line to the Pico @115200", None, 32 * 10 / 115200),
        Step("Pico control tick", 50.0, 0.0005),
    ])


def agent_chain() -> Chain:
    """'Bring me the bottle' -> the base starts moving."""
    return Chain("spoken command -> base moves", budget_s=8.0, steps=[
        Step("speech to text", None, 0.8),
        Step("LLM plan (cloud)", None, 2.5),
        Step("plan validation + skill dispatch", None, 0.05),
        Step("Nav2 behavior tree tick", 10.0, 0.01),
        Step("planner_server global plan", None, 0.15),
        Step("controller_server first command", 20.0, 0.01),
    ])


def estop_chain() -> Chain:
    """Button pressed -> motor current is zero. No software is involved."""
    return Chain("e-stop pressed -> actuator power off", budget_s=0.05, steps=[
        Step("mushroom button contact opens", None, 0.002),
        Step("relay coil de-energizes and contacts open", None, 0.010),
    ])


def chains() -> list[Chain]:
    return [obstacle_chain(), agent_chain(), estop_chain()]


# --------------------------------------------------------------------------------------------
#  Output
# --------------------------------------------------------------------------------------------
def table(system: list[Contract]) -> str:
    rows = ["| Interface | Type | Frame | Units | Rate | QoS | Owner | On timeout |",
            "|---|---|---|---|---|---|---|---|"]
    for c in sorted(system, key=lambda c: (c.layer, c.topic)):
        rate = f"{c.rate_hz:g} Hz" if c.rate_hz else "latched"
        rows.append(f"| `{c.topic}` | {c.msg_type} | {c.frame_id or '—'} | {c.units} | {rate} | "
                    f"{c.publishers[0].qos} | {c.publishers[0].node} | {c.on_timeout} |")
    return "\n".join(rows)


def mermaid(system: list[Contract]) -> str:
    def node_id(name: str) -> str:
        return name.replace("/", "_").replace(" ", "_").replace("<", "").replace(">", "")

    lines = ["flowchart LR"]
    seen: set[str] = set()
    for c in system:
        for endpoint in c.publishers + c.subscribers:
            if endpoint.node not in seen:
                seen.add(endpoint.node)
                lines.append(f'    {node_id(endpoint.node)}["{endpoint.node}<br/>({endpoint.compute})"]')
    for c in system:
        for pub in c.publishers:
            for sub in c.subscribers:
                label = c.topic if len(c.topic) < 26 else c.topic[:23] + "..."
                lines.append(f"    {node_id(pub.node)} -->|{label}| {node_id(sub.node)}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["table", "check", "bandwidth", "chain", "graph"])
    ap.add_argument("--broken", action="store_true", help="use the deliberately broken variant")
    args = ap.parse_args(argv)
    system = broken_system() if args.broken else karmel_system()

    if args.command == "table":
        print(table(system))
    elif args.command == "graph":
        print("```mermaid")
        print(mermaid(system))
        print("```")
    elif args.command == "bandwidth":
        total = 0.0
        print(f"{'interface':<36} {'kB/s':>9}")
        for topic, rate in bandwidth(system):
            total += rate
            print(f"{topic:<36} {rate / 1000:>9.1f}")
        print(f"{'TOTAL':<36} {total / 1000:>9.1f} kB/s = {total * 8 / 1e6:.1f} Mbit/s")
    elif args.command == "chain":
        for chain in chains():
            print(f"\n{chain.name}  (budget {chain.budget_s * 1000:.0f} ms)")
            for step in chain.steps:
                rate = f"{step.rate_hz:g} Hz" if step.rate_hz else "-"
                print(f"   {step.name:<38} {rate:>8}  {step.worst_case_s * 1000:7.1f} ms")
            verdict = "OK" if chain.ok else "OVER BUDGET"
            print(f"   {'worst case':<38} {'':>8}  {chain.worst_case_s * 1000:7.1f} ms  {verdict}")
    else:
        problems = check_system(system)
        if not problems:
            print(f"{len(system)} interfaces, no problems.")
            return 0
        for problem in problems:
            print(problem)
        print(f"\n{len(problems)} problem(s) in {len(system)} interfaces.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
