#!/usr/bin/env python3
"""10.07 — feed robot_localization's ekf_node synthetic /odom + /imu and grade /odometry/filtered.

Wheel odometry: the true body velocities with slip noise on the yaw rate (sigma 0.08 rad/s) and a
+4 % wheel-separation scale error. IMU: gyro z with sigma 0.005 rad/s and a 0.002 rad/s bias.
Both are published with honest covariances. ``--zero-cov imu`` / ``--zero-cov odom`` reproduce the
driver that leaves its covariance array at zeros, for each of the two sensors in turn.

    python3 replay_odom_imu.py --data localization_out/replay
    python3 replay_odom_imu.py --data localization_out/replay --zero-cov odom --label zerocov_odom

Run it inside a ROS 2 Jazzy environment next to ``ekf_node`` — see ``README.md`` in this folder.
Make the data first with ``python 10-localization/code/export_replay.py``.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster

ODOM_VYAW_STD = 0.08     # rad/s of wheel-slip noise on the odometry yaw rate
ODOM_VYAW_SCALE = 1.04   # wheel separation 4 % off -> every turn over-reported by 4 %
ODOM_VX_STD = 0.01       # m/s
GYRO_STD = 0.005         # rad/s  (SensorParams.realistic())
GYRO_BIAS = 0.002        # rad/s


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Feeder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("replay_odom_imu")
        d = np.load(Path(args.data) / "replay.npz")
        t, truth, dt = d["t_all"], d["truth_all"], float(d["dt"])
        yaw = np.unwrap(truth[:, 2])
        vyaw = np.gradient(yaw, dt)
        vx = (np.gradient(truth[:, 0], dt) * np.cos(truth[:, 2])
              + np.gradient(truth[:, 1], dt) * np.sin(truth[:, 2]))
        rng = np.random.default_rng(args.seed)
        # The EKF's odom frame starts at the origin, so grade everything in the START frame.
        x0, y0, th0 = truth[0]
        c, s = math.cos(-th0), math.sin(-th0)
        dx, dy = truth[:, 0] - x0, truth[:, 1] - y0
        self.truth = np.column_stack([c * dx - s * dy, s * dx + c * dy, yaw - th0])
        self.t, self.dt, self.args = t, dt, args
        self.odom_vx = vx + rng.normal(0.0, ODOM_VX_STD, vx.size)
        self.odom_vyaw = vyaw * ODOM_VYAW_SCALE + rng.normal(0.0, ODOM_VYAW_STD, vyaw.size)
        self.imu_vyaw = vyaw + GYRO_BIAS + rng.normal(0.0, GYRO_STD, vyaw.size)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_filtered, 10)
        # Keep the broadcaster alive: a latched publisher that is garbage-collected takes its
        # transform with it, and robot_localization then silently drops every IMU message.
        self.static_tf = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = "base_footprint", "imu_link"
        tf.transform.translation.z = 0.05
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

        self.start_after = self.get_clock().now().nanoseconds * 1e-9 + 1.0  # let TF propagate
        self.i = 0
        self.rows: list[tuple] = []
        self.wheel_pose = np.zeros(3)   # dead reckoning from each source alone, for comparison
        self.gyro_yaw = 0.0
        self.create_timer(dt / args.speed, self.tick)

    def tick(self) -> None:
        if self.get_clock().now().nanoseconds * 1e-9 < self.start_after:
            return
        i = self.i
        if i >= len(self.t):
            rclpy.shutdown()
            return
        self.i += 1
        stamp = self.get_clock().now().to_msg()

        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_footprint"
        o.twist.twist.linear.x = float(self.odom_vx[i])
        o.twist.twist.angular.z = float(self.odom_vyaw[i])
        cov = [0.0] * 36
        cov[0] = ODOM_VX_STD**2        # vx
        cov[7] = 1e-4                  # vy: a differential drive cannot slide sideways
        cov[35] = 0.0 if self.args.zero_cov == "odom" else ODOM_VYAW_STD**2   # vyaw
        o.twist.covariance = cov
        self.odom_pub.publish(o)

        m = Imu()
        m.header.stamp, m.header.frame_id = stamp, "imu_link"
        m.angular_velocity.z = float(self.imu_vyaw[i])
        m.angular_velocity_covariance = [0.0] * 9
        m.angular_velocity_covariance[8] = 0.0 if self.args.zero_cov == "imu" else GYRO_STD**2
        m.orientation_covariance[0] = -1.0          # this message carries no orientation
        m.linear_acceleration_covariance[0] = -1.0  # ... and no acceleration
        self.imu_pub.publish(m)

        th = self.wheel_pose[2] + 0.5 * self.odom_vyaw[i] * self.dt
        self.wheel_pose[0] += self.odom_vx[i] * self.dt * math.cos(th)
        self.wheel_pose[1] += self.odom_vx[i] * self.dt * math.sin(th)
        self.wheel_pose[2] = wrap(self.wheel_pose[2] + self.odom_vyaw[i] * self.dt)
        self.gyro_yaw = wrap(self.gyro_yaw + self.imu_vyaw[i] * self.dt)

    def on_filtered(self, msg: Odometry) -> None:
        i = min(self.i, len(self.t) - 1)
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        tx, ty, tth = self.truth[i]
        self.rows.append((
            self.t[i], p.position.x, p.position.y, yaw, tx, ty, tth,
            math.hypot(p.position.x - tx, p.position.y - ty), abs(wrap(yaw - tth)),
            math.hypot(self.wheel_pose[0] - tx, self.wheel_pose[1] - ty),
            abs(wrap(self.wheel_pose[2] - tth)), abs(wrap(self.gyro_yaw - tth)),
        ))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data", default="localization_out/replay")
    ap.add_argument("--speed", type=float, default=1.0, help="MUST stay 1.0 unless you also scale dt")
    ap.add_argument("--seed", type=int, default=4)
    ap.add_argument("--label", default="fuse")
    ap.add_argument("--zero-cov", choices=["imu", "odom"], default=None,
                    help="publish that sensor's covariance as zeros")
    args = ap.parse_args()

    rclpy.init()
    node = Feeder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rows = np.array(node.rows) if node.rows else np.zeros((0, 12))
    out = Path(args.data) / f"rl_{args.label}.csv"
    np.savetxt(out, rows, delimiter=",", fmt="%.5f", comments="",
               header="t,x,y,yaw,tx,ty,tyaw,ekf_pos_err,ekf_yaw_err,wheel_pos_err,wheel_yaw_err,gyro_yaw_err")
    print(f"\n=== robot_localization run '{args.label}': {len(rows)} /odometry/filtered messages ===")
    if len(rows):
        rms = lambda a: math.sqrt(float((a**2).mean()))  # noqa: E731
        print("                        position RMSE   final pos err   heading RMSE   final heading err")
        for name, pe, ye in (("wheel odometry only", rows[:, 9], rows[:, 10]),
                             ("EKF (wheels + IMU) ", rows[:, 7], rows[:, 8])):
            print(f"  {name}    {rms(pe) * 100:6.1f} cm   {pe[-1] * 100:9.1f} cm   "
                  f"{math.degrees(rms(ye)):8.1f} deg   {math.degrees(ye[-1]):12.1f} deg")
        gy = rows[:, 11]
        print(f"  gyro-only heading           -               -         {math.degrees(rms(gy)):8.1f} deg   "
              f"{math.degrees(gy[-1]):12.1f} deg")
    print(f"wrote {out}")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
