"""13.15 — measure the vision pipeline from outside: rate, end-to-end latency, dropped frames.

    ros2 run karmel_vision latency_probe --ros-args -p seconds:=20.0

Subscribes to the camera and to the detections and answers the three questions that decide whether
a detector is usable on a robot:

* how many frames per second actually reach a *decision* (not how fast the camera runs);
* how old a detection is when it arrives (capture stamp -> now), p50 and p90;
* how many camera frames were never processed at all.

`ros2 topic hz` answers none of these: it measures arrival rate, not age, and it cannot see which
frames the detector skipped.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


def key(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class LatencyProbe(Node):
    def __init__(self) -> None:
        super().__init__("latency_probe")
        self.declare_parameter("seconds", 20.0)
        self.camera_stamps: set[int] = set()
        self.detection_stamps: set[int] = set()
        self.ages: list[float] = []
        self.counts: list[int] = []
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, "/detections", self.on_detections, 10)
        self.t0 = self.get_clock().now()
        self.create_timer(float(self.get_parameter("seconds").value), self.finish)
        self.get_logger().info(f"measuring for {self.get_parameter('seconds').value} s ...")

    def on_image(self, msg: Image) -> None:
        self.camera_stamps.add(key(msg.header.stamp))

    def on_detections(self, msg: Detection2DArray) -> None:
        self.detection_stamps.add(key(msg.header.stamp))
        age_ms = (self.get_clock().now().nanoseconds - key(msg.header.stamp)) / 1e6
        self.ages.append(age_ms)
        self.counts.append(len(msg.detections))

    def finish(self) -> None:
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        frames, dets = len(self.camera_stamps), len(self.detection_stamps)
        processed = len(self.camera_stamps & self.detection_stamps)
        ages = np.array(self.ages) if self.ages else np.array([float("nan")])
        self.get_logger().info(
            f"\n  camera            {frames:5d} frames  ({frames / elapsed:5.2f} Hz)"
            f"\n  detections        {dets:5d} messages ({dets / elapsed:5.2f} Hz)"
            f"\n  frames processed  {processed:5d} of {frames} "
            f"({100 * processed / max(frames, 1):4.1f} %, {frames - processed} never looked at)"
            f"\n  detection age     p50 {np.percentile(ages, 50):7.1f} ms   p90 {np.percentile(ages, 90):7.1f} ms"
            f"   max {np.max(ages):7.1f} ms"
            f"\n  objects per frame {np.mean(self.counts) if self.counts else 0:.2f}")
        raise SystemExit


def main() -> None:
    rclpy.init()
    node = LatencyProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
