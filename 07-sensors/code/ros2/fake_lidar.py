#!/usr/bin/env python3
"""Publish a synthetic sensor_msgs/LaserScan of a rectangular room, so you can learn the message.

    python3 fake_lidar.py                       # /scan, 360 beams, 10 Hz, frame laser
    python3 fake_lidar.py --samples 720 --rate 15 --box 1.5
    python3 fake_lidar.py --fault zeros-for-no-return

The robot sits at the centre of a `--room-x` by `--room-y` room (metres), optionally with a box
obstacle `--box` metres ahead, and turns at `--yaw-rate` rad/s. Ranges are ray-cast analytically,
so the numbers are exact up to the noise you ask for.

Faults (one at a time):

    --fault zeros-for-no-return   out-of-range beams are 0.0 instead of inf: phantom obstacles at
                                  the robot's own origin, and every costmap believes them
    --fault degrees               angle_min/max/increment in degrees: the scan becomes a tiny
                                  fan, or wraps many times, depending on the consumer
    --fault clockwise             angles run clockwise (left/right mirrored) -- REP-103 says CCW
    --fault wrong-frame           frame_id = "base_link", so everything is 12 cm too low
    --fault stale-stamp           stamps 0.5 s in the past: TF extrapolation errors downstream
    --fault half-count            len(ranges) does not match (angle_max-angle_min)/angle_increment

Lessons: 07.07, 07.10, 07.11. Needs ROS 2 Jazzy (use the course Docker image).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scan_geometry import ScanSpec, scan_angles  # noqa: E402

FAULTS = ("none", "zeros-for-no-return", "degrees", "clockwise", "wrong-frame",
          "stale-stamp", "half-count")


def raycast_room(angles: np.ndarray, x: float, y: float, yaw: float,
                 room_x: float, room_y: float, box_ahead: float | None,
                 box_size: float) -> np.ndarray:
    """Exact range to the walls of an axis-aligned room (and an optional box), from (x, y, yaw)."""
    world = angles + yaw
    cx, sy = np.cos(world), np.sin(world)
    big = 1e9
    with np.errstate(divide="ignore", invalid="ignore"):
        tx = np.where(cx > 0, (room_x / 2 - x) / cx, np.where(cx < 0, (-room_x / 2 - x) / cx, big))
        ty = np.where(sy > 0, (room_y / 2 - y) / sy, np.where(sy < 0, (-room_y / 2 - y) / sy, big))
    r = np.minimum(tx, ty)
    if box_ahead is not None:
        # axis-aligned square box centred box_ahead metres along +x of the world frame
        bx0, bx1 = box_ahead - box_size / 2, box_ahead + box_size / 2
        by0, by1 = -box_size / 2, box_size / 2
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (bx0 - x) / cx
            t2 = (bx1 - x) / cx
            t3 = (by0 - y) / sy
            t4 = (by1 - y) / sy
        tmin = np.maximum(np.minimum(t1, t2), np.minimum(t3, t4))
        tmax = np.minimum(np.maximum(t1, t2), np.maximum(t3, t4))
        hit = (tmax >= np.maximum(tmin, 0.0)) & np.isfinite(tmin)
        r = np.where(hit, np.minimum(r, np.maximum(tmin, 0.0)), r)
    return r


class FakeLidar(Node):
    """A 360-degree spinning 2D LiDAR in an empty rectangular room."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("fake_lidar")
        self.args = args
        self.spec = ScanSpec(n=args.samples, range_min=args.range_min, range_max=args.range_max,
                             scan_time=1.0 / args.rate, frame_id=args.frame)
        qos = (QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE) if args.reliable
               else qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, args.topic, qos)
        self.rng = np.random.default_rng(args.seed)
        self.yaw = 0.0
        self.create_timer(1.0 / args.rate, self.tick)
        self.get_logger().info(
            f"publishing {args.topic}: {args.samples} beams at {args.rate} Hz, frame "
            f"'{args.frame}', room {args.room_x}x{args.room_y} m, fault={args.fault}")

    def tick(self) -> None:
        a = self.args
        self.yaw += a.yaw_rate / a.rate
        spec = self.spec
        angles = scan_angles(spec)
        if a.fault == "clockwise":
            angles = -angles
        r = raycast_room(angles, a.x, a.y, self.yaw, a.room_x, a.room_y,
                         a.box if a.box > 0 else None, a.box_size)
        r = r + self.rng.normal(0.0, a.sigma, r.shape)
        no_return = r > spec.range_max
        r[no_return] = 0.0 if a.fault == "zeros-for-no-return" else float("inf")
        # a fraction of beams miss entirely (dark, glossy or grazing surfaces)
        miss = self.rng.random(r.shape) < a.dropout
        r[miss] = 0.0 if a.fault == "zeros-for-no-return" else float("inf")

        msg = LaserScan()
        now = self.get_clock().now()
        if a.fault == "stale-stamp":
            now = rclpy.time.Time(nanoseconds=now.nanoseconds - 500_000_000)
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "base_link" if a.fault == "wrong-frame" else spec.frame_id
        scale = 180.0 / math.pi if a.fault == "degrees" else 1.0
        msg.angle_min = float(spec.angle_min * scale)
        msg.angle_max = float((spec.angle_max - spec.angle_increment) * scale)
        msg.angle_increment = float(spec.angle_increment * scale)
        msg.time_increment = float(spec.time_increment)
        msg.scan_time = float(spec.scan_time)
        msg.range_min = float(spec.range_min)
        msg.range_max = float(spec.range_max)
        if a.fault == "half-count":
            r = r[: spec.n // 2]
        msg.ranges = [float(v) for v in r]
        if a.intensities:
            # a crude but honest model: intensity falls off as 1/r^2 and with grazing incidence
            inten = np.where(np.isfinite(r), a.intensity_scale / np.maximum(r, 0.1) ** 2, 0.0)
            msg.intensities = [float(v) for v in inten]
        self.pub.publish(msg)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default="/scan")
    p.add_argument("--frame", default="laser")
    p.add_argument("--samples", type=int, default=360)
    p.add_argument("--rate", type=float, default=10.0, help="Hz (revolutions per second)")
    p.add_argument("--room-x", type=float, default=6.0, help="room length, m")
    p.add_argument("--room-y", type=float, default=5.0, help="room width, m")
    p.add_argument("--x", type=float, default=0.0, help="robot x in the room, m")
    p.add_argument("--y", type=float, default=0.0, help="robot y in the room, m")
    p.add_argument("--yaw-rate", type=float, default=0.0, help="rad/s")
    p.add_argument("--box", type=float, default=0.0, help="distance to a box obstacle, m (0 = none)")
    p.add_argument("--box-size", type=float, default=0.3, help="box side, m")
    p.add_argument("--sigma", type=float, default=0.01, help="range noise, m")
    p.add_argument("--dropout", type=float, default=0.01, help="fraction of beams with no return")
    p.add_argument("--range-min", type=float, default=0.05)
    p.add_argument("--range-max", type=float, default=12.0)
    p.add_argument("--intensities", action="store_true")
    p.add_argument("--intensity-scale", type=float, default=200.0)
    p.add_argument("--reliable", action="store_true",
                   help="publish RELIABLE instead of the sensor-data QoS a real driver uses")
    p.add_argument("--seed", type=int, default=3)
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
    node = FakeLidar(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
