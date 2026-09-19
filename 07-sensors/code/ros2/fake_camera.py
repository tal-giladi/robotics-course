#!/usr/bin/env python3
"""Publish a synthetic sensor_msgs/Image + CameraInfo pair, so you can learn the messages.

    python3 fake_camera.py                        # /camera/image_raw + /camera/camera_info
    python3 fake_camera.py --width 1280 --height 720 --fps 30   # watch the bandwidth
    python3 fake_camera.py --fault no-camera-info

The image is a moving checkerboard with a frame counter drawn as a bar, so you can see dropped
frames by eye in `rqt_image_view`. No OpenCV needed: the pixels are built with numpy.

Faults (one at a time):

    --fault no-camera-info     image only: every calibration-dependent node silently waits forever
    --fault uncalibrated       CameraInfo with K[0] = 0 -- the documented "not calibrated" marker
    --fault stamp-on-publish   stamp taken after the (simulated) exposure and transfer, not at
                               the start of exposure: the stamp lies by one whole pipeline delay
    --fault mismatched-info    CameraInfo says 1280x720 while the image is 640x480
    --fault body-frame         frame_id = camera_link (x forward) instead of the optical frame
                               (z forward), so every projected point lands in the wrong place

Lessons: 07.08, 07.10, 07.11. Needs ROS 2 Jazzy (use the course Docker image).
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

FAULTS = ("none", "no-camera-info", "uncalibrated", "stamp-on-publish", "mismatched-info",
          "body-frame")


def checkerboard(width: int, height: int, square: int, phase: int, counter: int) -> np.ndarray:
    """An rgb8 image: a scrolling checkerboard plus a bar whose length is `counter % width`."""
    xs = ((np.arange(width) + phase) // square) % 2
    ys = (np.arange(height) // square) % 2
    pattern = (xs[None, :] ^ ys[:, None]).astype(np.uint8) * 200 + 25
    img = np.repeat(pattern[:, :, None], 3, axis=2)
    img[:, :, 1] = np.minimum(255, img[:, :, 1].astype(int) + 20).astype(np.uint8)
    bar = counter % width
    img[: max(4, height // 20), :bar, :] = 255
    return img


class FakeCamera(Node):
    """A 640x480 USB webcam, as ROS 2 sees it."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("fake_camera")
        self.args = args
        qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE) if args.reliable
               else qos_profile_sensor_data)
        self.pub_img = self.create_publisher(Image, f"{args.namespace}/image_raw", qos)
        # With the no-camera-info fault the publisher is never created, so the topic does not even
        # appear in `ros2 topic list` -- exactly what a driver that forgot CameraInfo looks like.
        self.pub_info = (None if args.fault == "no-camera-info"
                         else self.create_publisher(CameraInfo, f"{args.namespace}/camera_info",
                                                    qos))
        self.k = 0
        self.create_timer(1.0 / args.fps, self.tick)
        mbps = args.width * args.height * 3 * args.fps * 8 / 1e6
        self.get_logger().info(
            f"publishing {args.namespace}/image_raw {args.width}x{args.height} rgb8 at "
            f"{args.fps} fps = {mbps:.1f} Mbit/s raw, frame '{self.frame_id()}', "
            f"fault={args.fault}")

    def frame_id(self) -> str:
        return "camera_link" if self.args.fault == "body-frame" else self.args.frame

    def camera_info(self, stamp) -> CameraInfo:
        a = self.args
        w, h = (1280, 720) if a.fault == "mismatched-info" else (a.width, a.height)
        fx = fy = (w / 2.0) / math.tan(a.hfov_rad / 2.0)
        cx, cy = w / 2.0 - 0.5, h / 2.0 - 0.5
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id()
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0] if a.fault == "uncalibrated" else list(a.distortion)
        if a.fault == "uncalibrated":
            info.k = [0.0] * 9          # K[0] == 0.0 is the documented "uncalibrated" marker
            info.p = [0.0] * 12
        else:
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return info

    def tick(self) -> None:
        a = self.args
        self.k += 1
        start_of_exposure = self.get_clock().now()
        pixels = checkerboard(a.width, a.height, a.square, (self.k * a.scroll) % a.width, self.k)
        if a.work_ms:                     # pretend the driver spends this long converting
            time.sleep(a.work_ms / 1000.0)

        msg = Image()
        stamp = (self.get_clock().now() if a.fault == "stamp-on-publish"
                 else start_of_exposure).to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id()
        msg.height, msg.width = a.height, a.width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = a.width * 3            # bytes per row: width * channels * bytes per channel
        msg.data = pixels.tobytes()
        self.pub_img.publish(msg)
        if self.pub_info is not None:
            self.pub_info.publish(self.camera_info(stamp))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--namespace", default="/camera")
    p.add_argument("--frame", default="camera_optical_frame")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--hfov-rad", type=float, default=1.20, help="karmel.yaml default")
    p.add_argument("--distortion", type=float, nargs=5,
                   default=[-0.32, 0.11, 0.0005, -0.0009, 0.0], help="plumb_bob k1 k2 p1 p2 k3")
    p.add_argument("--square", type=int, default=40, help="checkerboard square, px")
    p.add_argument("--scroll", type=int, default=4, help="px per frame")
    p.add_argument("--work-ms", type=float, default=0.0, help="fake driver processing time")
    p.add_argument("--reliable", action="store_true",
                   help="RELIABLE + depth 1 instead of the sensor-data QoS a real driver uses")
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
    node = FakeCamera(args)
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
