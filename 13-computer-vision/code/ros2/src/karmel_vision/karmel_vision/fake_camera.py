"""13.15/13.16 — a camera, a depth camera and a moving robot, without any hardware.

Publishes exactly what karmel's real bring-up publishes ([07.08](../../../../07-sensors/07.08-rgb-cameras.md)):

    /camera/image_raw         sensor_msgs/Image      bgr8, 640x480 by default, SensorDataQoS
    /camera/camera_info       sensor_msgs/CameraInfo K from labs/config/karmel.yaml
    /camera/depth/image_rect  sensor_msgs/Image      16UC1, millimetres, aligned to the colour image
    /tf, /tf_static           map -> odom -> base_footprint -> base_link -> camera_link -> camera_optical_frame

and prints the ground truth once, so 13.16 can be graded against it.

    ros2 run karmel_vision fake_camera --ros-args -p rate_hz:=10.0 -p spin_rad_s:=0.4

Every frame is stamped with its CAPTURE time. Nothing here simulates inference latency — that
belongs to the detector.
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .geometry import Intrinsics
from .scene import CAMERA_HEIGHT_M, CAMERA_X_M, CAMERA_Z_M, WHEEL_RADIUS_M, RobotPose, SceneObject, depth_map, to_optical, visible_box

CLASS_BGR = {"bottle": (60, 170, 70), "cup": (200, 180, 90)}          # green bottle, blue-ish mug


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def transform(parent: str, child: str, xyz, rpy=(0.0, 0.0, 0.0)) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in xyz)
    q = quaternion_from_rpy(*rpy)
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = q
    return t


class FakeCamera(Node):
    def __init__(self) -> None:
        super().__init__("fake_camera")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("hfov_rad", 1.20)
        self.declare_parameter("spin_rad_s", 0.0)
        self.declare_parameter("drive_m_s", 0.0)
        self.declare_parameter("start_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("noise_sigma", 4.0)
        self.declare_parameter("depth_noise_m", 0.02)
        self.declare_parameter("depth_dropout", 0.05)
        self.declare_parameter("publish_depth", True)
        # "class x y [diameter]" per object, in the map frame. The defaults are the ground truth
        # 13.16 grades against; move them to make the exercises harder.
        self.declare_parameter("objects", ["bottle 2.05 1.70 0.07", "cup 1.20 0.35 0.09"])
        self.declare_parameter("image_qos", "sensor")     # "sensor" (best effort) or "reliable"

        w = self.get_parameter("width").value
        h = self.get_parameter("height").value
        self.intr = Intrinsics.from_hfov(w, h, self.get_parameter("hfov_rad").value)
        self.objects = [SceneObject(f[0], float(f[1]), float(f[2]),
                                    diameter_m=float(f[3]) if len(f) > 3 else 0.07)
                        for f in (spec.split() for spec in self.get_parameter("objects").value)]
        start = self.get_parameter("start_pose").value
        self.pose = RobotPose(float(start[0]), float(start[1]), float(start[2]))
        self.bridge = CvBridge()
        self.rng = np.random.default_rng(0)
        self.noise = np.empty((h, w, 3), np.float32)

        qos = (qos_profile_sensor_data if self.get_parameter("image_qos").value == "sensor"
               else QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_image = self.create_publisher(Image, "/camera/image_raw", qos)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/camera_info", qos)
        self.pub_depth = self.create_publisher(Image, "/camera/depth/image_rect", qos)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.tf_static.sendTransform([
            transform("base_footprint", "base_link", (0.0, 0.0, WHEEL_RADIUS_M)),
            transform("base_link", "camera_link", (CAMERA_X_M, 0.0, CAMERA_Z_M)),
            # the optical frame: z forward, x right, y down (REP-103, 05.06)
            transform("camera_link", "camera_optical_frame", (0.0, 0.0, 0.0), (-math.pi / 2, 0.0, -math.pi / 2)),
        ])
        for obj in self.objects:
            p = obj.centre_map
            self.get_logger().info(f"ground truth: {obj.class_name} centre in map "
                                   f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}), height {obj.height_m:.3f} m")
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.dt = period
        self.timer = self.create_timer(period, self.tick)

    # ------------------------------------------------------------------ rendering
    def render(self) -> np.ndarray:
        """A 3-channel BGR frame: a floor that gets darker towards the horizon, plus the objects."""
        h, w = self.intr.height, self.intr.width
        rows = np.linspace(0.0, 1.0, h)[:, None]
        img = np.zeros((h, w, 3), np.float32)
        img[:] = (60 + 120 * rows)[..., None]                       # wall above, bright floor below
        img[: int(self.intr.cy), :] = 45.0
        for obj in sorted(self.objects, key=lambda o: -to_optical(self.pose, o.centre_map)[2]):
            box = visible_box(self.pose, obj, self.intr)
            if box is None:
                continue
            p1 = (int(round(box.x1)), int(round(box.y1)))
            p2 = (int(round(box.x2)), int(round(box.y2)))
            cv2.rectangle(img, p1, p2, CLASS_BGR[obj.class_name], thickness=-1)
            cv2.rectangle(img, p1, p2, tuple(0.6 * c for c in CLASS_BGR[obj.class_name]), thickness=2)
        sigma = float(self.get_parameter("noise_sigma").value)
        if sigma > 0:
            cv2.randn(self.noise, 0.0, sigma)        # 8x faster than numpy here, and this node
            img += self.noise                        # has to keep up with a 10 Hz camera
        return np.clip(img, 0, 255).astype(np.uint8)

    def render_depth(self) -> np.ndarray:
        """16UC1 millimetres, the encoding every RGB-D driver uses. 0 means 'no reading'."""
        depth = depth_map(self.pose, self.objects, self.intr)
        noise = float(self.get_parameter("depth_noise_m").value)
        if noise > 0:
            jitter = np.empty(depth.shape, np.float32)
            cv2.randn(jitter, 0.0, noise)
            depth = np.where(depth > 0, depth + jitter, 0.0)
        dropout = float(self.get_parameter("depth_dropout").value)
        if dropout > 0:                              # the holes every RGB-D camera leaves
            depth = np.where(self.rng.random(depth.shape) < dropout, 0.0, depth)
        return np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)

    # ------------------------------------------------------------------ publishing
    def camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "camera_optical_frame"
        info.width, info.height = self.intr.width, self.intr.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [self.intr.fx, 0.0, self.intr.cx, 0.0, self.intr.fy, self.intr.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.intr.fx, 0.0, self.intr.cx, 0.0, 0.0, self.intr.fy, self.intr.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def tick(self) -> None:
        w = float(self.get_parameter("spin_rad_s").value)
        v = float(self.get_parameter("drive_m_s").value)
        yaw = self.pose.yaw + w * self.dt
        self.pose = RobotPose(self.pose.x + v * self.dt * math.cos(yaw),
                              self.pose.y + v * self.dt * math.sin(yaw), yaw)
        stamp = self.get_clock().now().to_msg()                     # the CAPTURE time

        odom = transform("odom", "base_footprint", (self.pose.x, self.pose.y, 0.0), (0.0, 0.0, self.pose.yaw))
        odom.header.stamp = stamp
        map_odom = transform("map", "odom", (0.0, 0.0, 0.0))
        map_odom.header.stamp = stamp
        self.tf.sendTransform([map_odom, odom])

        msg = self.bridge.cv2_to_imgmsg(self.render(), encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_optical_frame"
        self.pub_image.publish(msg)
        self.pub_info.publish(self.camera_info(stamp))
        if bool(self.get_parameter("publish_depth").value):
            depth = self.bridge.cv2_to_imgmsg(self.render_depth(), encoding="16UC1")
            depth.header = msg.header
            self.pub_depth.publish(depth)


def main() -> None:
    rclpy.init()
    node = FakeCamera()
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
