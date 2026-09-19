#!/usr/bin/env python3
"""Publish a synthetic sensor_msgs/Imu (+ MagneticField) so you can learn the message without hardware.

    ros2 run --prefix "python3" ...          # not a package: just run it
    python3 fake_imu.py                       # imu/data_raw at 100 Hz, frame imu_link
    python3 fake_imu.py --yaw-rate 0.5 --bias 0.009 --fault none

Faults you can inject (one at a time) -- each is a real bug seen on real robots:

    --fault no-covariance   all-zero covariance: "unknown", and every consumer has to guess
    --fault zero-stamp      header.stamp = 0: TF and message_filters both give up
    --fault wrong-frame     frame_id = "base_link" when the chip is on imu_link
    --fault gravity-in-g    linear_acceleration in g instead of m/s^2 (reads 1.0, not 9.81)
    --fault left-handed     y axis inverted: yaw turns the wrong way (REP-145 forbids it)
    --fault sigma-not-var   covariance filled with sigma instead of sigma^2
    --fault stale           publishes at the right rate but stamps every message with boot time

Lessons: 07.06, 07.10, 07.11. Needs ROS 2 Jazzy (use the course Docker image).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, MagneticField

sys.path.insert(0, str(Path(__file__).resolve().parent))
from imu_conventions import G, diag_covariance, quaternion_from_rpy  # noqa: E402

FAULTS = ("none", "no-covariance", "zero-stamp", "wrong-frame", "gravity-in-g",
          "left-handed", "sigma-not-var", "stale")


class FakeImu(Node):
    """A hobby-grade 9-DOF IMU on a robot that is turning at a constant rate."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("fake_imu")
        self.args = args
        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=(ReliabilityPolicy.BEST_EFFORT if args.best_effort
                         else ReliabilityPolicy.RELIABLE),
        )
        self.pub = self.create_publisher(Imu, args.topic, qos)
        self.pub_mag = self.create_publisher(MagneticField, args.mag_topic, qos)
        self.rng = __import__("random").Random(args.seed)
        self.yaw = 0.0
        self.k = 0
        self.boot = self.get_clock().now()
        self.create_timer(1.0 / args.rate, self.tick)
        self.get_logger().info(
            f"publishing {args.topic} at {args.rate} Hz in frame '{args.frame}', "
            f"gyro bias {args.bias} rad/s, fault={args.fault}")

    # --- the message -------------------------------------------------------------------------
    def tick(self) -> None:
        a = self.args
        dt = 1.0 / a.rate
        self.k += 1
        self.yaw += a.yaw_rate * dt

        msg = Imu()
        now = self.get_clock().now()
        msg.header.stamp = (self.boot if a.fault == "stale" else now).to_msg()
        if a.fault == "zero-stamp":
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
        msg.header.frame_id = "base_link" if a.fault == "wrong-frame" else a.frame

        # gyroscope: the true rate, plus this chip's bias, plus white noise
        gx = self.rng.gauss(0.0, a.gyro_sigma)
        gy = self.rng.gauss(0.0, a.gyro_sigma)
        gz = a.yaw_rate + a.bias + self.rng.gauss(0.0, a.gyro_sigma)

        # accelerometer: gravity on +z while level, plus noise. A real robot adds its own
        # acceleration; --accel-x fakes that so you can watch tilt estimates break.
        ax = a.accel_x + self.rng.gauss(0.0, a.accel_sigma)
        ay = self.rng.gauss(0.0, a.accel_sigma)
        az = G + self.rng.gauss(0.0, a.accel_sigma)

        if a.fault == "gravity-in-g":
            ax, ay, az = ax / G, ay / G, az / G
        if a.fault == "left-handed":
            gy, ay = -gy, -ay
            gz = -gz

        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        if a.orientation:
            qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, self.yaw)
            msg.orientation.x, msg.orientation.y = qx, qy
            msg.orientation.z, msg.orientation.w = qz, qw
            msg.orientation_covariance = diag_covariance(a.orientation_sigma)
        else:
            # REP-145: "no orientation estimate" is -1 in element 0, not a zero quaternion with
            # zero covariance (which reads as "perfectly certain the robot is level").
            msg.orientation.w = 1.0
            msg.orientation_covariance = [-1.0] + [0.0] * 8

        gyro_cov = diag_covariance(a.gyro_sigma)
        accel_cov = diag_covariance(a.accel_sigma)
        if a.fault == "no-covariance":
            gyro_cov = accel_cov = [0.0] * 9
            msg.orientation_covariance = [0.0] * 9
        elif a.fault == "sigma-not-var":
            gyro_cov = [a.gyro_sigma if i in (0, 4, 8) else 0.0 for i in range(9)]
            accel_cov = [a.accel_sigma if i in (0, 4, 8) else 0.0 for i in range(9)]
        msg.angular_velocity_covariance = gyro_cov
        msg.linear_acceleration_covariance = accel_cov
        self.pub.publish(msg)

        # magnetometer at a tenth of the rate, in tesla (REP-145), not microtesla
        if self.k % 10 == 0:
            m = MagneticField()
            m.header = msg.header
            horiz = a.field_ut * math.cos(math.radians(a.inclination_deg)) * 1e-6
            m.magnetic_field.x = horiz * math.cos(-self.yaw) + a.hard_iron_ut * 1e-6
            m.magnetic_field.y = horiz * math.sin(-self.yaw)
            m.magnetic_field.z = -a.field_ut * math.sin(math.radians(a.inclination_deg)) * 1e-6
            m.magnetic_field_covariance = diag_covariance(a.mag_sigma_ut * 1e-6)
            self.pub_mag.publish(m)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default="imu/data_raw", help="REP-145 name for un-fused IMU data")
    p.add_argument("--mag-topic", default="imu/mag")
    p.add_argument("--frame", default="imu_link")
    p.add_argument("--rate", type=float, default=100.0, help="Hz")
    p.add_argument("--yaw-rate", type=float, default=0.0, help="true turn rate, rad/s")
    p.add_argument("--bias", type=float, default=0.009, help="gyro z bias, rad/s (0.009 ~ 0.52 deg/s)")
    p.add_argument("--gyro-sigma", type=float, default=0.003, help="rad/s per sample")
    p.add_argument("--accel-sigma", type=float, default=0.02, help="m/s^2 per sample")
    p.add_argument("--accel-x", type=float, default=0.0, help="fake forward acceleration, m/s^2")
    p.add_argument("--orientation", action="store_true",
                   help="also publish a fused orientation (what a BNO055 in NDOF mode gives you)")
    p.add_argument("--orientation-sigma", type=float, default=0.05, help="rad")
    p.add_argument("--field-ut", type=float, default=44.0, help="total geomagnetic field, uT")
    p.add_argument("--inclination-deg", type=float, default=45.0)
    p.add_argument("--hard-iron-ut", type=float, default=0.0)
    p.add_argument("--mag-sigma-ut", type=float, default=0.4)
    p.add_argument("--best-effort", action="store_true", help="publish with BEST_EFFORT reliability")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--fault", choices=FAULTS, default="none")
    return p


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if "--ros-args" in argv:
        cut = argv.index("--ros-args")
        argv, ros_args = argv[:cut], argv[cut:]
    args = build_parser().parse_args(argv)
    rclpy.init(args=ros_args or None)
    node = FakeImu(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
