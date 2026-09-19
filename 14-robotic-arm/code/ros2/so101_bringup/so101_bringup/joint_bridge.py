"""joint_bridge — the SO-101's ros2_control "hardware", written in Python (lesson 14.08).

``topic_based_ros2_control/TopicBasedSystem`` turns the hardware interface into two topics:

    /so101/joint_commands        sensor_msgs/JointState   <- controller_manager writes goals here
    /so101/driver_joint_states   sensor_msgs/JointState   -> this node publishes the truth here

so the actual driver is an ordinary rclpy node — this one. It reuses 14.03's ``servo_tools``
safety layer unchanged, which means the clamping, the rate limiting, the load abort and the safe
torque-enable sequence all still apply underneath ros2_control.

    ros2 run so101_bringup joint_bridge --ros-args \\
        -p port:=/dev/ttyACM0 -p robot_id:=karmel_follower -p calibration:=/path/to/joint_map.json

    ros2 run so101_bringup joint_bridge --ros-args -p fake:=true      # no arm, for the tutorial

> SAFETY (14.11): this node energises the arm. Clamp the arm down, clear a 0.5 m radius, keep the
> servo PSU switch within reach, and start with the low torque/speed limits in SafetyConfig.
> Ctrl-C disables torque, which makes the arm GO LIMP AND FALL — support it with a hand.

Units: this node speaks URDF radians to ROS and LeRobot degrees/percent to the servos. Every
conversion goes through ``joint_map.JointMap``; nothing else in the file does arithmetic on angles.
"""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState

from so101_bringup.joint_map import ALL_JOINTS, JointMap

# servo_tools and so101_bus live in 14-robotic-arm/code; add that directory to PYTHONPATH, or
# copy the two files into this package. They are deliberately ROS-free so they can be tested
# without one.
try:
    from servo_tools import FakeArmBus, SafetyConfig, SafetyError, clamp_goals, safe_enable_torque
except ImportError:  # pragma: no cover - a clearer message than a bare ImportError
    print("cannot import servo_tools: add 14-robotic-arm/code to PYTHONPATH", file=sys.stderr)
    raise


class JointBridge(Node):
    def __init__(self) -> None:
        super().__init__("so101_joint_bridge")
        self.declare_parameter("port", "")
        self.declare_parameter("robot_id", "karmel_follower")
        self.declare_parameter("calibration", "")
        self.declare_parameter("fake", False)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("commands_topic", "/so101/joint_commands")
        self.declare_parameter("states_topic", "/so101/driver_joint_states")

        calibration = self.get_parameter("calibration").value
        self.map = JointMap.load(Path(calibration)) if calibration else JointMap()
        if not calibration:
            self.get_logger().warn(
                "no calibration file: using sign=+1, offset=0 for every joint. RViz will NOT "
                "match the real arm until you measure them (lesson 14.08, exercise E1).")

        self.cfg = SafetyConfig()
        self.bus = self._open_bus()
        self.present = safe_enable_torque(self.bus, self.cfg)
        self.get_logger().info(f"torque on, holding {self.present}")

        # Commands: the controller_manager publishes at its update_rate, so keep the queue at 1 —
        # an old goal is worse than no goal.
        self.create_subscription(JointState, self.get_parameter("commands_topic").value,
                                 self.on_command, 1)
        self.states = self.create_publisher(JointState, self.get_parameter("states_topic").value,
                                            QoSPresetProfiles.SENSOR_DATA.value)
        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self.publish_state)
        self.last_positions: dict[str, float] = dict(self.present)

    # --- hardware -----------------------------------------------------------------------
    def _open_bus(self):
        if self.get_parameter("fake").value:
            self.get_logger().warn("fake:=true — simulating six servos, nothing will move")
            return FakeArmBus()
        from so101_bus import default_calibration_path, make_bus   # noqa: PLC0415 - optional dep

        port = self.get_parameter("port").value
        if not port:
            raise RuntimeError("set -p port:=COM5 (Windows) or /dev/ttyACM0 (Linux), or -p fake:=true")
        return make_bus(port, default_calibration_path(self.get_parameter("robot_id").value))

    # --- ROS -> servos ------------------------------------------------------------------
    def on_command(self, msg: JointState) -> None:
        if not msg.position:
            return                                        # a state-only message: ignore it
        urdf = {name: float(p) for name, p in zip(msg.name, msg.position) if name in ALL_JOINTS}
        urdf, notes = self.map.clamp(urdf)
        for note in notes:
            self.get_logger().warn(f"clamped to URDF limits: {note}")
        goals, notes = clamp_goals(self.map.to_lerobot(urdf), self.cfg.limits)
        for note in notes:
            self.get_logger().warn(f"clamped to SafetyConfig limits: {note}")
        try:
            # No rate limiting here: joint_trajectory_controller already sends a smooth,
            # limit-checked stream at 50 Hz. move_smoothly's ramp would fight it.
            self.bus.write_goal_positions(goals)
        except SafetyError as exc:
            self.get_logger().error(f"refusing command: {exc}")

    # --- servos -> ROS ------------------------------------------------------------------
    def publish_state(self) -> None:
        if hasattr(self.bus, "advance"):                  # FakeArmBus: run its little simulation
            self.bus.advance(1.0 / float(self.get_parameter("publish_rate").value))
        lerobot = self.bus.read_positions()
        urdf = self.map.to_urdf(lerobot)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = [urdf[name] for name in ALL_JOINTS]
        self.states.publish(msg)
        self.last_positions = lerobot

    def shutdown(self) -> None:
        self.get_logger().warn("disabling torque — SUPPORT THE ARM, it will fall")
        try:
            self.bus.disable_torque()
        finally:
            if hasattr(self.bus, "close"):
                self.bus.close()


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = JointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
