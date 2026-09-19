#!/usr/bin/env python3
"""10.09 — replay a simulated apartment tour into AMCL and grade the result.

Publishes what AMCL needs and nothing else:
  /scan              sensor_msgs/LaserScan in the `laser` frame
  TF odom -> base_footprint   the DEAD-RECKONED odometry (it drifts 81 cm over the tour)
  TF base_footprint -> laser  static
  /initialpose       once, three seconds in

and records /amcl_pose against the ground truth.

    python3 replay_scan.py --data localization_out/replay --init 1.0 1.3 0.0 --label good

Run it inside a ROS 2 Jazzy environment, next to map_server + amcl + lifecycle_manager — see
``README.md`` in this folder. Make the data first with
``python 10-localization/code/export_replay.py``.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

SETTLE_S = 3.0        # the robot stands still this long before the tour starts
SCAN_BACKDATE_S = 0.06  # stamp scans slightly in the past so the TF buffer already brackets them
POSE_BACKDATE_S = 0.20  # same for /initialpose: AMCL looks base_footprint up AT that stamp


def yaw_quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Replay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("replay_scan")
        d = np.load(Path(args.data) / "replay.npz")
        self.args = args
        self.t_log = d["t"] / args.speed
        self.truth, self.odom = d["truth"], d["odom"]
        self.ranges, self.angles = d["ranges"], d["angles"]
        self.odom_yaw = np.unwrap(self.odom[:, 2])
        self.n = len(self.t_log)
        self.rows: list[tuple] = []
        self.t0: float | None = None
        self.i = 0

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.tf = TransformBroadcaster(self)
        # Keep the static broadcaster alive: a latched publisher that is garbage-collected takes
        # its transform with it, and the consumer then silently drops every message in that frame.
        self.static_tf = StaticTransformBroadcaster(self)
        s = TransformStamped()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id, s.child_frame_id = "base_footprint", "laser"
        s.transform.translation.z = 0.12          # karmel.yaml sensors.lidar.z_m
        s.transform.rotation.w = 1.0
        self.static_tf.sendTransform(s)

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", latched)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.init_sent = False
        self.deadline = self.get_clock().now().nanoseconds * 1e-9 + 15.0
        self.create_timer(0.02, self.tick_tf)                       # 50 Hz odometry TF
        self.create_timer(0.2, self.tick_initial_pose)              # send ONCE, when AMCL is listening
        self.create_timer(1.0 / (10.0 * args.speed), self.tick_scan)  # the log's scan rate

    def tick_initial_pose(self) -> None:
        """Publish /initialpose exactly once, and only once AMCL is actually subscribed.

        AMCL acts on an /initialpose only if its subscription has already matched — publishing
        before that is silently lost, and publishing *repeatedly* is worse: every message resets the
        particle cloud to a pose the robot has already driven away from. So wait for a subscriber
        (with a timeout, so the script still terminates if AMCL never comes up), send one message,
        and only then start the tour clock.
        """
        if self.init_sent:
            return
        if self.init_pub.get_subscription_count() == 0 and self.now() < self.deadline:
            return
        self.send_initial_pose()
        self.init_sent = True
        self.t0 = self.now() + SETTLE_S   # let AMCL settle on the new pose before the robot moves

    # --- publishing -----------------------------------------------------------------------------
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def send_initial_pose(self) -> None:
        m = PoseWithCovarianceStamped()
        # Backdated: AMCL transforms the initial pose through base_footprint -> odom AT this stamp,
        # and tf2 does not extrapolate into the future. A `now()` stamp loses the race and AMCL logs
        # "Failed to transform initial pose in time" — after which it has no pose at all.
        stamp = self.get_clock().now().nanoseconds - int(POSE_BACKDATE_S * 1e9)
        m.header.stamp.sec, m.header.stamp.nanosec = stamp // 1_000_000_000, stamp % 1_000_000_000
        m.header.frame_id = "map"
        x, y, yaw = self.args.init
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = yaw_quat(yaw)
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[35] = self.args.init_var
        m.pose.covariance = cov
        self.init_pub.publish(m)
        self.get_logger().info(f"initial pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg), "
                               f"variance {self.args.init_var}")

    def tick_tf(self) -> None:
        """Publish the dead-reckoned odom -> base_footprint transform (stationary until t0 is set)."""
        t = 0.0 if self.t0 is None else max(0.0, min(self.now() - self.t0, self.t_log[-1]))
        m = TransformStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = "odom", "base_footprint"
        m.transform.translation.x = float(np.interp(t, self.t_log, self.odom[:, 0]))
        m.transform.translation.y = float(np.interp(t, self.t_log, self.odom[:, 1]))
        m.transform.rotation.z, m.transform.rotation.w = yaw_quat(float(np.interp(t, self.t_log, self.odom_yaw)))
        self.tf.sendTransform(m)

    def tick_scan(self) -> None:
        if self.t0 is None:
            t = -1.0                       # the robot is still standing at the start pose
        else:
            t = self.now() - self.t0
            if t > self.t_log[-1]:
                rclpy.shutdown()
                return
        self.i = 0 if t < 0 else min(int(np.searchsorted(self.t_log, t)), self.n - 1)
        msg = LaserScan()
        stamp = self.get_clock().now().nanoseconds - int(SCAN_BACKDATE_S * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = stamp // 1_000_000_000, stamp % 1_000_000_000
        msg.header.frame_id = "laser"
        msg.angle_min, msg.angle_max = float(self.angles[0]), float(self.angles[-1])
        msg.angle_increment = float(self.angles[1] - self.angles[0])
        msg.range_min, msg.range_max = 0.15, 12.0
        msg.ranges = [float(r) for r in self.ranges[self.i]]
        self.scan_pub.publish(msg)

    # --- grading --------------------------------------------------------------------------------
    def on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        tx, ty, tth = self.truth[self.i]
        self.rows.append((self.t_log[self.i] * self.args.speed, p.position.x, p.position.y, yaw,
                          tx, ty, tth, math.hypot(p.position.x - tx, p.position.y - ty),
                          abs(wrap(yaw - tth)),
                          msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35]))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data", default="localization_out/replay", help="folder written by export_replay.py")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (1.0 = real time)")
    ap.add_argument("--init", type=float, nargs=3, default=[1.0, 1.3, 0.0], metavar=("X", "Y", "YAW"))
    ap.add_argument("--init-var", type=float, default=0.25)
    ap.add_argument("--label", default="run")
    args = ap.parse_args()

    rclpy.init()
    node = Replay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rows = np.array(node.rows) if node.rows else np.zeros((0, 12))
    out = Path(args.data) / f"amcl_{args.label}.csv"
    np.savetxt(out, rows, delimiter=",", fmt="%.5f", comments="",
               header="t,x,y,yaw,tx,ty,tyaw,pos_err,yaw_err,cov_xx,cov_yy,cov_yawyaw")
    print(f"\n=== AMCL run '{args.label}': {len(rows)} /amcl_pose messages ===")
    if len(rows):
        err, yerr, t = rows[:, 7], rows[:, 8], rows[:, 0]
        late = rows[t > 10.0]
        print(f"position error : mean {err.mean() * 100:5.1f} cm, max {err.max() * 100:6.1f} cm, "
              f"final {err[-1] * 100:5.1f} cm")
        print(f"heading error  : mean {math.degrees(yerr.mean()):5.1f} deg, final {math.degrees(yerr[-1]):5.1f} deg")
        if len(late):
            print(f"after t = 10 s : mean {late[:, 7].mean() * 100:5.1f} cm, max {late[:, 7].max() * 100:6.1f} cm")
        print(f"reported sigma : x   {math.sqrt(rows[0, 9]) * 100:5.1f} cm -> {math.sqrt(rows[-1, 9]) * 100:5.1f} cm")
        print(f"                 yaw {math.degrees(math.sqrt(rows[0, 11])):5.1f} deg -> "
              f"{math.degrees(math.sqrt(rows[-1, 11])):5.1f} deg")
    print(f"wrote {out}")
    sys.stdout.flush()


if __name__ == "__main__":
    main()
