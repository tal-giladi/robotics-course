#!/usr/bin/env python3
"""Publish a synthetic depth image and the matching PointCloud2, so you can learn both layouts.

    python3 fake_depth.py                      # /depth/image_raw (16UC1) + /depth/points + info
    python3 fake_depth.py --encoding 32FC1     # the float metres variant
    python3 fake_depth.py --fault zero-is-valid

The scene is a wall at `--wall` metres with a box in the middle at `--box`, plus a "glass panel"
region and a "black object" region that the sensor fails on -- the two failures every depth camera
has, made visible.

Faults (one at a time):

    --fault zero-is-valid     invalid pixels are 0 in 16UC1 (correct) but the cloud keeps them as
                              real points at the camera origin -- a wall of phantom obstacles
    --fault metres-in-16u     16UC1 filled with metres instead of millimetres: everything is 1 mm
    --fault dense-lie         is_dense = True while the cloud contains NaNs
    --fault body-frame        frame_id = camera_link (x forward) instead of the optical frame

Lessons: 07.09, 07.10, 07.11. Needs ROS 2 Jazzy (use the course Docker image).
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

FAULTS = ("none", "zero-is-valid", "metres-in-16u", "dense-lie", "body-frame")


def depth_scene(width: int, height: int, wall_m: float, box_m: float, box_px: int,
                glass: bool, black: bool, sigma_rel: float, rng: np.random.Generator) -> np.ndarray:
    """A float32 depth image in metres, NaN where the sensor returns nothing."""
    d = np.full((height, width), wall_m, dtype=np.float32)
    cy, cx = height // 2, width // 2
    h = box_px // 2
    d[cy - h: cy + h, cx - h: cx + h] = box_m
    if glass:                       # a pane the projector/stereo sees straight through
        d[height // 6: height // 3, width // 8: width // 3] = np.nan
    if black:                       # a matte black object: no light comes back
        d[2 * height // 3: 5 * height // 6, 2 * width // 3: 7 * width // 8] = np.nan
    # stereo/ToF error grows with the square of the distance -- the single most important fact
    noise = rng.normal(0.0, 1.0, d.shape).astype(np.float32) * sigma_rel * d**2
    return d + noise


class FakeDepth(Node):
    """A depth camera: one depth image, one CameraInfo, one organised point cloud."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("fake_depth")
        self.args = args
        self.rng = np.random.default_rng(args.seed)
        self.pub_depth = self.create_publisher(Image, f"{args.namespace}/image_raw",
                                               qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, f"{args.namespace}/camera_info",
                                              qos_profile_sensor_data)
        self.pub_cloud = self.create_publisher(PointCloud2, args.cloud_topic,
                                               qos_profile_sensor_data)
        self.fx = self.fy = (args.width / 2.0) / math.tan(args.hfov_rad / 2.0)
        self.cx, self.cy = args.width / 2.0 - 0.5, args.height / 2.0 - 0.5
        self.create_timer(1.0 / args.fps, self.tick)
        pts = args.width * args.height
        self.get_logger().info(
            f"publishing {args.namespace}/image_raw ({args.encoding}) and {args.cloud_topic} "
            f"({pts} points, {pts * 16 / 1e6:.2f} MB per cloud) at {args.fps} Hz, "
            f"frame '{self.frame_id()}', fault={args.fault}")

    def frame_id(self) -> str:
        return "camera_link" if self.args.fault == "body-frame" else self.args.frame

    def tick(self) -> None:
        a = self.args
        stamp = self.get_clock().now().to_msg()
        frame = self.frame_id()
        d = depth_scene(a.width, a.height, a.wall, a.box, a.box_px, not a.no_glass,
                        not a.no_black, a.sigma_rel, self.rng)
        d[d < a.min_range] = np.nan          # nothing inside the minimum range comes back

        # --- depth image -------------------------------------------------------------------
        img = Image()
        img.header.stamp, img.header.frame_id = stamp, frame
        img.height, img.width = a.height, a.width
        img.is_bigendian = 0
        if a.encoding == "16UC1":
            scale = 1.0 if a.fault == "metres-in-16u" else 1000.0
            raw = np.nan_to_num(d * scale, nan=0.0)      # 0 is the "no data" value for 16UC1
            img.encoding, img.step = "16UC1", a.width * 2
            img.data = raw.astype(np.uint16).tobytes()
        else:
            img.encoding, img.step = "32FC1", a.width * 4   # NaN is the "no data" value for 32FC1
            img.data = d.astype(np.float32).tobytes()
        self.pub_depth.publish(img)

        # --- camera info -------------------------------------------------------------------
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, frame
        info.width, info.height = a.width, a.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

        # --- point cloud (organised: height x width, NaN where there is no point) -----------
        u = np.arange(a.width, dtype=np.float32)[None, :]
        v = np.arange(a.height, dtype=np.float32)[:, None]
        z = d.astype(np.float32)
        if a.fault == "zero-is-valid":
            z = np.nan_to_num(z, nan=0.0)
        x = (u - self.cx) * z / self.fx       # optical frame: x right, y down, z forward
        y = (v - self.cy) * z / self.fy
        xyz = np.stack((x, y, np.broadcast_to(z, x.shape)), axis=-1).astype(np.float32)
        padded = np.zeros((a.height, a.width, 4), dtype=np.float32)
        padded[..., :3] = xyz                  # 4th float = padding, as every real driver does

        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, frame
        cloud.height, cloud.width = a.height, a.width
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * a.width
        cloud.data = padded.tobytes()
        cloud.is_dense = bool(a.fault == "dense-lie" or not np.isnan(z).any())
        self.pub_cloud.publish(cloud)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--namespace", default="/depth")
    p.add_argument("--cloud-topic", default="/depth/points")
    p.add_argument("--frame", default="camera_depth_optical_frame")
    p.add_argument("--width", type=int, default=424)
    p.add_argument("--height", type=int, default=240)
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--hfov-rad", type=float, default=1.51, help="~87 deg, a D435i's depth FOV")
    p.add_argument("--wall", type=float, default=3.0, help="wall distance, m")
    p.add_argument("--box", type=float, default=1.2, help="box distance, m")
    p.add_argument("--box-px", type=int, default=60)
    p.add_argument("--min-range", type=float, default=0.28, help="m; nothing closer is reported")
    p.add_argument("--sigma-rel", type=float, default=0.002,
                   help="depth noise coefficient: sigma = coeff * range^2")
    p.add_argument("--encoding", choices=("16UC1", "32FC1"), default="16UC1")
    p.add_argument("--no-glass", action="store_true")
    p.add_argument("--no-black", action="store_true")
    p.add_argument("--seed", type=int, default=11)
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
    node = FakeDepth(args)
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
