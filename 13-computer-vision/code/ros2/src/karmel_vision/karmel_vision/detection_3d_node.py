"""13.16 — `Detection2DArray` + depth (or size, or the floor) -> `Detection3DArray`.

    ros2 run karmel_vision detection_3d_node --ros-args -p method:=depth
    ros2 run karmel_vision detection_3d_node --ros-args -p method:=size
    ros2 run karmel_vision detection_3d_node --ros-args -p method:=ground

The node stays in `camera_optical_frame`: it does geometry, not transforms. Turning the result
into `map` is `object_map_node`'s job, because only that step needs TF and a robot pose (05.10).
"""
from __future__ import annotations

from collections import OrderedDict

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import BoundingBox3D, Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .geometry import (OBJECT_HEIGHT_M, Box, Intrinsics, depth_from_depth_image, depth_from_ground_plane,
                       depth_from_known_size, known_size_depth_sigma, plausible_size, position_covariance,
                       position_from_box)
from .scene import CAMERA_HEIGHT_M


def stamp_key(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Detection3DNode(Node):
    def __init__(self) -> None:
        super().__init__("detection_3d_node")
        self.declare_parameter("method", "depth")
        self.declare_parameter("camera_height_m", CAMERA_HEIGHT_M)
        self.declare_parameter("camera_pitch_rad", 0.0)
        self.declare_parameter("depth_sigma_m", 0.02)
        self.declare_parameter("pixel_sigma", 2.0)
        self.declare_parameter("max_depth_age_ms", 100.0)
        self.declare_parameter("plausibility_gate", True)

        self.bridge = CvBridge()
        self.intr: Intrinsics | None = None
        self.depths: OrderedDict[int, np.ndarray] = OrderedDict()      # stamp -> depth image (metres)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_rect", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, "/detections", self.on_detections, 10)
        self.pub = self.create_publisher(Detection3DArray, "/detections_3d", 10)
        self.rejected = 0

    def on_info(self, msg: CameraInfo) -> None:
        # CameraInfo belongs to the image size it was published with. If the detector ran on a
        # downscaled image and reported boxes in ORIGINAL pixels (as ours does), this K is right.
        intr = Intrinsics.from_camera_info_k(msg.k, msg.width, msg.height)
        if self.intr is None:
            self.get_logger().info(f"intrinsics: fx {intr.fx:.1f} fy {intr.fy:.1f} "
                                   f"cx {intr.cx:.1f} cy {intr.cy:.1f} ({intr.width}x{intr.height})")
        self.intr = intr

    def on_depth(self, msg: Image) -> None:
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        metres = raw.astype(np.float32) / 1000.0 if msg.encoding == "16UC1" else raw.astype(np.float32)
        self.depths[stamp_key(msg.header.stamp)] = metres
        while len(self.depths) > 30:
            self.depths.popitem(last=False)

    def depth_for(self, stamp) -> np.ndarray | None:
        """The depth frame captured with this colour frame — matched by stamp, never by arrival."""
        key = stamp_key(stamp)
        if key in self.depths:
            return self.depths[key]
        tolerance = float(self.get_parameter("max_depth_age_ms").value) * 1e6
        near = [k for k in self.depths if abs(k - key) <= tolerance]
        return self.depths[min(near, key=lambda k: abs(k - key))] if near else None

    def on_detections(self, msg: Detection2DArray) -> None:
        if self.intr is None:
            self.get_logger().warn("no CameraInfo yet", throttle_duration_sec=5.0)
            return
        method = self.get_parameter("method").value
        depth_img = self.depth_for(msg.header.stamp) if method == "depth" else None
        if method == "depth" and depth_img is None:
            self.get_logger().warn("no depth frame for this stamp", throttle_duration_sec=5.0)
            return

        out = Detection3DArray()
        out.header = msg.header                                      # same capture time, same frame
        for det in msg.detections:
            if not det.results:
                continue
            name = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            if name not in OBJECT_HEIGHT_M:
                continue
            box = Box.from_center_size(det.bbox.center.position.x, det.bbox.center.position.y,
                                       det.bbox.size_x, det.bbox.size_y)
            z, sigma_z = self.depth_of(method, box, depth_img, name)
            if z is None:
                self.rejected += 1
                continue
            if bool(self.get_parameter("plausibility_gate").value):
                ok, implied = plausible_size(name, box, self.intr, z)
                if not ok:
                    self.rejected += 1
                    self.get_logger().info(f"rejected {name}: {box.height:.0f} px at {z:.2f} m "
                                           f"implies {implied:.2f} m tall")
                    continue
            p = position_from_box(box, self.intr, z)
            cov = position_covariance(box, self.intr, z, sigma_z,
                                      float(self.get_parameter("pixel_sigma").value))
            out.detections.append(self.make_detection3d(msg.header, name, score, p, cov, box, z))
        self.pub.publish(out)

    def depth_of(self, method: str, box: Box, depth_img, name: str) -> tuple[float | None, float]:
        """The depth of one box and its 1-sigma uncertainty, by the configured method."""
        if method == "depth":
            z = depth_from_depth_image(depth_img, box)
            return z, float(self.get_parameter("depth_sigma_m").value)
        if method == "size":
            z = depth_from_known_size(box, self.intr, OBJECT_HEIGHT_M[name][1])
            if box.touches_border(self.intr):                        # truncated object: height is a lie
                return None, 0.0
            return z, known_size_depth_sigma(z, box, name)
        if method == "ground":
            z = depth_from_ground_plane(box, self.intr, float(self.get_parameter("camera_height_m").value),
                                        float(self.get_parameter("camera_pitch_rad").value))
            if z is None:
                return None, 0.0
            # one pixel of error on the bottom edge, propagated: sigma_Z / Z = sigma_v / (v - cy)
            sigma_v = float(self.get_parameter("pixel_sigma").value)
            return z, z * sigma_v / max(box.y2 - self.intr.cy, 1e-6)
        raise ValueError(f"unknown method {method!r}")

    def make_detection3d(self, header, name: str, score: float, p: np.ndarray, cov: np.ndarray,
                         box: Box, z: float) -> Detection3D:
        d = Detection3D()
        d.header = header
        d.id = name
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = name
        hyp.hypothesis.score = score
        hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z = (float(v) for v in p)
        c = np.zeros((6, 6))
        c[:3, :3] = cov                                              # the rest stays 0: we know no orientation
        hyp.pose.covariance = c.ravel().tolist()
        d.results.append(hyp)
        bbox = BoundingBox3D()
        bbox.center = hyp.pose.pose
        bbox.size.x = float(box.width * z / self.intr.fx)
        bbox.size.y = float(box.width * z / self.intr.fx)             # assume as deep as it is wide
        bbox.size.z = float(box.height * z / self.intr.fy)
        d.bbox = bbox
        return d


def main() -> None:
    rclpy.init()
    node = Detection3DNode()
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
