"""base_node — karmel's simple real-robot driver: ROS 2 <-> protocol v1 <-> Pico.

This is the course's first "robot in ROS 2" path (lesson 04.16). It does not use ros2_control;
lesson 08.12 replaces it with karmel_hardware/KarmelSystem + diff_drive_controller.

Subscribes
  cmd_vel            geometry_msgs/TwistStamped   (geometry_msgs/Twist if use_stamped_cmd_vel:=false)
Publishes
  odom               nav_msgs/Odometry            wheel odometry, frame odom -> base_footprint
  /tf                odom -> base_footprint            (publish_tf:=false when robot_localization's EKF does it)
  joint_states       sensor_msgs/JointState       wheel positions/velocities for robot_state_publisher
  battery_state      sensor_msgs/BatteryState
  range/front        sensor_msgs/Range
  /diagnostics       diagnostic_msgs/DiagnosticArray (via diagnostic_updater)

Safety layers, from the outside in:
  1. cmd_vel timeout: no command for cmd_vel_timeout s -> target speed 0 (decelerating)
  2. speed limits (max_linear_speed, max_angular_speed, max_wheel_speed, curvature preserved)
  3. acceleration limits (max_linear_accel, max_angular_accel)
  4. the Pico's own watchdog (watchdog_ms): if this node dies, the motors stop anyway

Serial loss: when a read/write fails or no telemetry arrives for telemetry_timeout s, the port
is closed and re-opened every reconnect_period s. `port` may be a device (/dev/ttyACM0) or any
pyserial URL, e.g. socket://localhost:5760 for a fake Pico over TCP.
"""
from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState, Range
from tf2_ros import TransformBroadcaster

from karmel_base import protocol as proto
from karmel_base import wire
from karmel_base.kinematics import (integrate_odometry, Pose2D, ramp, twist_to_wheels,
                                    wheels_to_twist, yaw_to_quaternion)

try:
    import serial
except ImportError:  # pragma: no cover - python3-serial is an exec_depend
    serial = None


class BaseNode(Node):

    def __init__(self):
        super().__init__('base_node')

        # ---------------- parameters (defaults = labs/config/karmel.yaml; base.launch.py loads it)
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.045).value
        self.wheel_separation = self.declare_parameter('wheel_separation', 0.200).value
        self.ticks_per_rev = self.declare_parameter('ticks_per_rev', 2464).value
        self.max_wheel_speed = self.declare_parameter('max_wheel_speed', 17.0).value
        self.max_linear_speed = self.declare_parameter('max_linear_speed', 0.5).value
        self.max_angular_speed = self.declare_parameter('max_angular_speed', 2.5).value
        self.max_linear_accel = self.declare_parameter('max_linear_accel', 1.0).value
        self.max_angular_accel = self.declare_parameter('max_angular_accel', 5.0).value
        self.cmd_vel_timeout = self.declare_parameter('cmd_vel_timeout', 0.5).value
        self.use_stamped = self.declare_parameter('use_stamped_cmd_vel', True).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.left_joint = self.declare_parameter('left_joint', 'left_wheel_joint').value
        self.right_joint = self.declare_parameter('right_joint', 'right_wheel_joint').value
        self.control_rate = self.declare_parameter('control_rate', 50.0).value
        self.watchdog_ms = self.declare_parameter('watchdog_ms', 300).value
        self.telemetry_hz = self.declare_parameter('telemetry_hz', 50).value
        self.telemetry_timeout = self.declare_parameter('telemetry_timeout', 1.0).value
        self.reconnect_period = self.declare_parameter('reconnect_period', 2.0).value
        self.battery_full_v = self.declare_parameter('battery_full_v', 12.6).value
        self.battery_empty_v = self.declare_parameter('battery_cutoff_v', 9.9).value
        self.battery_cells = self.declare_parameter('battery_cells', 3).value
        self.battery_capacity_ah = self.declare_parameter('battery_capacity_ah', 3.5).value
        self.range_frame = self.declare_parameter('range_frame', 'range_front_link').value
        self.range_type = self.declare_parameter('range_type', 'infrared').value
        self.range_fov = self.declare_parameter('range_fov', 0.47).value
        self.range_min = self.declare_parameter('range_min', 0.04).value
        self.range_max = self.declare_parameter('range_max', 4.0).value

        # ---------------- state
        self.ser = None
        self.buffer = wire.LineBuffer()
        self.seq = 0
        self.last_connect_attempt = None
        self.connected_since = None
        self.firmware_version = None
        self.protocol_version = None
        self.last_telemetry = None           # proto.Telemetry
        self.last_telemetry_time = None      # rclpy Time
        self.telemetry_count = 0
        self.checksum_errors = 0
        self.nacks = 0
        self.reconnects = 0

        self.target_v = self.target_w = 0.0  # from cmd_vel (after speed limits)
        self.cmd_v = self.cmd_w = 0.0        # after acceleration limiting
        self.last_cmd_time = None

        self.pose = Pose2D()
        self.prev_ticks = None               # (left, right, ms) of the previous telemetry
        self.joint_offset = [0.0, 0.0]       # wheel angle accumulated before the last encoder reset

        # ---------------- ROS interfaces
        if self.use_stamped:
            self.create_subscription(TwistStamped, 'cmd_vel', lambda m: self.on_cmd_vel(m.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.range_pub = self.create_publisher(Range, 'range/front', qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.diagnostics = Updater(self)
        self.diagnostics.setHardwareID(f'karmel-pico@{self.port}')
        self.diagnostics.add('serial connection', self.diag_connection)
        self.diagnostics.add('motors', self.diag_motors)
        self.diagnostics.add('battery', self.diag_battery)

        self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.create_timer(1.0, self.publish_battery)
        self.get_logger().info(
            f'base_node: port={self.port} cmd_vel={"TwistStamped" if self.use_stamped else "Twist"} '
            f'publish_tf={self.publish_tf}')

    # ============================================================================ serial port
    def next_seq(self) -> int:
        self.seq = wire.next_seq(self.seq)
        return self.seq

    def try_connect(self) -> None:
        now = self.get_clock().now()
        if self.last_connect_attempt and (now - self.last_connect_attempt).nanoseconds < self.reconnect_period * 1e9:
            return
        self.last_connect_attempt = now
        if serial is None:
            self.get_logger().error('pyserial is not installed (apt install python3-serial)', once=True)
            return
        try:
            # serial_for_url accepts plain device paths and URLs like socket://host:port
            self.ser = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0, write_timeout=0.1)
        except (serial.SerialException, OSError, ValueError) as exc:
            self.get_logger().warn(f'cannot open {self.port}: {exc} (retrying every {self.reconnect_period} s)',
                                   throttle_duration_sec=10.0)
            self.ser = None
            return
        self.get_logger().info(f'connected to {self.port}')
        self.connected_since = now
        self.buffer = wire.LineBuffer()
        self.prev_ticks = None
        self.last_telemetry_time = None
        self.firmware_version = None
        try:
            self.ser.reset_input_buffer()
        except (serial.SerialException, OSError, AttributeError):
            pass
        self.send(proto.Hello(self.next_seq()))
        self.send(proto.SetParam(self.next_seq(), 'watchdog_ms', int(self.watchdog_ms)))
        self.send(proto.SetParam(self.next_seq(), 'telemetry_hz', int(self.telemetry_hz)))

    def disconnect(self, reason: str) -> None:
        if self.ser is None:
            return
        self.get_logger().error(f'serial connection lost: {reason}')
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001 - closing a dead port may raise anything
            pass
        self.ser = None
        self.connected_since = None
        self.reconnects += 1
        # Drop any motion in progress: after reconnecting the robot must be commanded again.
        self.cmd_v = self.cmd_w = self.target_v = self.target_w = 0.0

    def send(self, msg) -> None:
        if self.ser is None:
            return
        try:
            self.ser.write(proto.encode_bytes(msg))
        except (serial.SerialException, OSError) as exc:
            self.disconnect(f'write failed: {exc}')

    def read_serial(self) -> None:
        # The port was opened with timeout=0, so read() returns at once with whatever is buffered.
        # Don't size the read with in_waiting: for socket:// URLs pyserial reports only 0 or 1
        # there, and reading one byte per cycle falls hopelessly behind 50 Hz telemetry.
        data = b''
        try:
            while True:
                chunk = self.ser.read(4096)
                data += chunk
                if len(chunk) < 4096:
                    break
        except (serial.SerialException, OSError) as exc:
            self.disconnect(f'read failed: {exc}')
            return
        for line in self.buffer.feed(data):
            try:
                msg = proto.decode(line)
            except proto.ProtocolError as exc:
                self.checksum_errors += 1
                self.get_logger().debug(f'rejected line: {exc}')
                continue
            self.handle_message(msg)

    def handle_message(self, msg) -> None:
        if isinstance(msg, proto.Telemetry):
            self.on_telemetry(msg)
        elif isinstance(msg, proto.HelloReply):
            self.firmware_version = msg.firmware_version
            self.protocol_version = msg.protocol_version
            ok = msg.protocol_version == proto.PROTOCOL_VERSION
            level = self.get_logger().info if ok else self.get_logger().error
            level(f'Pico firmware {msg.firmware_version}, protocol v{msg.protocol_version}')
        elif isinstance(msg, proto.Ack) and not msg.ok:
            self.nacks += 1
            self.get_logger().warn(f'Pico rejected command {msg.seq}: {msg.reason}', throttle_duration_sec=5.0)

    # ============================================================================ control loop
    def on_cmd_vel(self, twist: Twist) -> None:
        v, w = twist.linear.x, twist.angular.z
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().warn('ignoring non-finite cmd_vel', throttle_duration_sec=5.0)
            return
        self.target_v = max(-self.max_linear_speed, min(self.max_linear_speed, v))
        self.target_w = max(-self.max_angular_speed, min(self.max_angular_speed, w))
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self) -> None:
        if self.ser is None:
            self.try_connect()
            if self.ser is None:
                return
        self.read_serial()
        if self.ser is None:
            return

        now = self.get_clock().now()
        # telemetry watchdog: a Pico that stopped talking is as good as unplugged
        reference = self.last_telemetry_time or self.connected_since
        if reference and (now - reference).nanoseconds > self.telemetry_timeout * 1e9:
            self.disconnect(f'no telemetry for {self.telemetry_timeout} s')
            return

        # cmd_vel timeout -> decelerate to zero
        if self.last_cmd_time is None or (now - self.last_cmd_time).nanoseconds > self.cmd_vel_timeout * 1e9:
            self.target_v = self.target_w = 0.0

        dt = 1.0 / self.control_rate
        self.cmd_v = ramp(self.cmd_v, self.target_v, self.max_linear_accel, dt)
        self.cmd_w = ramp(self.cmd_w, self.target_w, self.max_angular_accel, dt)
        left, right = twist_to_wheels(self.cmd_v, self.cmd_w, self.wheel_radius, self.wheel_separation,
                                      self.max_wheel_speed)
        # Sending every cycle (even zeros) keeps the Pico's watchdog fed while we are healthy.
        self.send(proto.VelocityCommand(self.next_seq(), wire.rad_s_to_mrad_s(left), wire.rad_s_to_mrad_s(right)))

    # ============================================================================ telemetry
    def on_telemetry(self, t: proto.Telemetry) -> None:
        stamp = self.get_clock().now()
        rad_per_tick = 2.0 * math.pi / self.ticks_per_rev

        if self.prev_ticks is not None:
            prev_left, prev_right, prev_ms = self.prev_ticks
            if t.ms < prev_ms:
                # Pico rebooted (clock went backwards): encoder counts restarted from zero.
                self.get_logger().warn('Pico clock went backwards — firmware restarted, re-basing encoders')
                self.joint_offset[0] += prev_left * rad_per_tick
                self.joint_offset[1] += prev_right * rad_per_tick
                prev_left = prev_right = 0
            d_left = (t.left_ticks - prev_left) * rad_per_tick * self.wheel_radius
            d_right = (t.right_ticks - prev_right) * rad_per_tick * self.wheel_radius
            self.pose = integrate_odometry(self.pose, d_left, d_right, self.wheel_separation)
        self.prev_ticks = (t.left_ticks, t.right_ticks, t.ms)
        self.last_telemetry = t
        self.last_telemetry_time = stamp
        self.telemetry_count += 1

        v, w = wheels_to_twist(t.left_mrad_s / 1000.0, t.right_mrad_s / 1000.0,
                               self.wheel_radius, self.wheel_separation)
        self.publish_odometry(stamp, v, w)
        self.publish_joint_states(stamp, t, rad_per_tick)
        self.publish_range(stamp, t)

    def publish_odometry(self, stamp, v: float, w: float) -> None:
        qx, qy, qz, qw = yaw_to_quaternion(self.pose.theta)
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        # Diagonal covariances. z, roll, pitch are "known" (planar robot) -> tiny; x, y, yaw grow
        # with travel in reality — a fixed value is the usual simplification. Tune in lesson 10.x.
        for i, var in enumerate([1e-3, 1e-3, 1e-6, 1e-6, 1e-6, 1e-2]):
            odom.pose.covariance[i * 7] = var
        for i, var in enumerate([1e-3, 1e-6, 1e-6, 1e-6, 1e-6, 5e-3]):
            odom.twist.covariance[i * 7] = var
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.pose.x
            tf.transform.translation.y = self.pose.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def publish_joint_states(self, stamp, t: proto.Telemetry, rad_per_tick: float) -> None:
        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.joint_offset[0] + t.left_ticks * rad_per_tick,
                       self.joint_offset[1] + t.right_ticks * rad_per_tick]
        js.velocity = [t.left_mrad_s / 1000.0, t.right_mrad_s / 1000.0]
        self.joint_pub.publish(js)

    def publish_range(self, stamp, t: proto.Telemetry) -> None:
        msg = Range()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.range_frame
        msg.radiation_type = Range.ULTRASOUND if self.range_type == 'ultrasound' else Range.INFRARED
        msg.field_of_view = float(self.range_fov)
        msg.min_range = float(self.range_min)
        msg.max_range = float(self.range_max)
        if wire.range_m(t) is not None:
            msg.range = wire.range_m(t)
        else:
            # REP-117: +inf = nothing in range, NaN = sensor error
            msg.range = math.nan if wire.has_flag(t, proto.FLAG_RANGE_ERROR) else math.inf
        self.range_pub.publish(msg)

    def publish_battery(self) -> None:
        t = self.last_telemetry
        if t is None or self.ser is None:
            return
        volts = wire.battery_v(t)                # None: the firmware has no battery reading (-1)
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = math.nan if volts is None else volts
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = float(self.battery_capacity_ah)
        span = self.battery_full_v - self.battery_empty_v
        if volts is None or span <= 0:
            msg.percentage = math.nan
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        else:
            msg.percentage = max(0.0, min(1.0, (volts - self.battery_empty_v) / span))
            msg.power_supply_health = (BatteryState.POWER_SUPPLY_HEALTH_DEAD if volts < self.battery_empty_v
                                       else BatteryState.POWER_SUPPLY_HEALTH_GOOD)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        msg.cell_voltage = [msg.voltage / self.battery_cells] * int(self.battery_cells)
        msg.cell_temperature = [math.nan] * int(self.battery_cells)
        msg.location = 'base'
        self.battery_pub.publish(msg)

    # ============================================================================ diagnostics
    def diag_connection(self, stat):
        if self.ser is None:
            stat.summary(DiagnosticStatus.ERROR, f'not connected to {self.port}')
        elif self.last_telemetry_time is None:
            stat.summary(DiagnosticStatus.WARN, 'connected, waiting for telemetry')
        elif self.protocol_version not in (None, proto.PROTOCOL_VERSION):
            stat.summary(DiagnosticStatus.ERROR, f'protocol mismatch: Pico speaks v{self.protocol_version}')
        else:
            stat.summary(DiagnosticStatus.OK, 'connected')
        stat.add('port', str(self.port))
        stat.add('firmware', str(self.firmware_version))
        stat.add('telemetry messages', str(self.telemetry_count))
        stat.add('rejected lines (checksum/format)', str(self.checksum_errors))
        stat.add('rejected commands (A ERR)', str(self.nacks))
        stat.add('reconnects', str(self.reconnects))
        return stat

    def diag_motors(self, stat):
        t = self.last_telemetry
        if t is None:
            stat.summary(DiagnosticStatus.STALE, 'no telemetry')
            return stat
        if wire.has_flag(t, proto.FLAG_WATCHDOG) and (abs(self.cmd_v) > 1e-3 or abs(self.cmd_w) > 1e-3):
            stat.summary(DiagnosticStatus.WARN, 'Pico watchdog tripped while commanding motion')
        else:
            stat.summary(DiagnosticStatus.OK, 'ok')
        stat.add('commanded v [m/s]', f'{self.cmd_v:.3f}')
        stat.add('commanded w [rad/s]', f'{self.cmd_w:.3f}')
        stat.add('left wheel [rad/s]', f'{t.left_mrad_s / 1000.0:.3f}')
        stat.add('right wheel [rad/s]', f'{t.right_mrad_s / 1000.0:.3f}')
        stat.add('pico watchdog tripped', str(wire.has_flag(t, proto.FLAG_WATCHDOG)))
        stat.add('velocity mode', str(wire.has_flag(t, proto.FLAG_VELOCITY_MODE)))
        return stat

    def diag_battery(self, stat):
        t = self.last_telemetry
        if t is None:
            stat.summary(DiagnosticStatus.STALE, 'no telemetry')
            return stat
        volts = wire.battery_v(t)
        if volts is None:
            stat.summary(DiagnosticStatus.WARN, 'no battery reading from the Pico')
        elif volts < self.battery_empty_v:
            stat.summary(DiagnosticStatus.ERROR, f'battery below cutoff: {volts:.2f} V')
        elif wire.has_flag(t, proto.FLAG_LOW_BATTERY):
            stat.summary(DiagnosticStatus.WARN, f'battery low: {volts:.2f} V')
        else:
            stat.summary(DiagnosticStatus.OK, f'{volts:.2f} V')
        stat.add('voltage [V]', 'n/a' if volts is None else f'{volts:.2f}')
        return stat

    def stop_motors(self) -> None:
        self.send(proto.StopCommand(self.next_seq()))


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
