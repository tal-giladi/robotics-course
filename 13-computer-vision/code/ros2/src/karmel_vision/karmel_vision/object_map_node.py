"""13.16 — `Detection3DArray` in the camera frame -> a list of objects in `map`.

    ros2 run karmel_vision object_map_node --ros-args -p truth:="[2.05, 1.70, 0.12]"

Three jobs, in this order:

1. transform every detection into `map` **at its own stamp** (05.10), retrying the "into the
   future" failures instead of falling back to `Time()`;
2. rotate the 3x3 covariance with it, because a transformed uncertainty is still an uncertainty;
3. fuse detections into objects (`karmel_vision.objects`) and publish markers + a scoreboard.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from .objects import SemanticObjectMap

CLASS_RGB = {"bottle": (0.2, 0.8, 0.3), "cup": (0.3, 0.6, 0.95)}


@dataclass
class Pending:
    msg: Detection3DArray
    received: Time


def quaternion_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ObjectMapNode(Node):
    def __init__(self) -> None:
        super().__init__("object_map_node")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("max_wait_s", 0.5)
        self.declare_parameter("min_hits", 3)
        self.declare_parameter("forget_after_s", 60.0)
        self.declare_parameter("use_latest", False)          # the 05.10 bug, on a switch
        self.declare_parameter("truth", [float("nan")] * 3)
        self.declare_parameter("report_every_s", 5.0)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.world = SemanticObjectMap(min_hits=int(self.get_parameter("min_hits").value),
                                       forget_after_s=float(self.get_parameter("forget_after_s").value))
        self.pending: list[Pending] = []
        self.dropped = 0
        self.create_subscription(Detection3DArray, "/detections_3d", self.on_detections, 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/object_map/markers", 1)
        self.create_timer(0.02, self.retry_pending)          # 50 Hz: faster than TF arrives
        self.create_timer(float(self.get_parameter("report_every_s").value), self.report)

    # ------------------------------------------------------------------ TF plumbing (05.10)
    def on_detections(self, msg: Detection3DArray) -> None:
        if not self.try_transform(msg, final=False):
            self.pending.append(Pending(msg, self.get_clock().now()))

    def retry_pending(self) -> None:
        now = self.get_clock().now()
        wait = Duration(seconds=float(self.get_parameter("max_wait_s").value))
        still = []
        for item in self.pending:
            give_up = (now - item.received) > wait
            if not self.try_transform(item.msg, final=give_up) and not give_up:
                still.append(item)
        self.pending = still

    def try_transform(self, msg: Detection3DArray, final: bool) -> bool:
        target = self.get_parameter("target_frame").value
        stamp = Time() if bool(self.get_parameter("use_latest").value) else Time.from_msg(msg.header.stamp)
        try:
            tf = self.buffer.lookup_transform(target, msg.header.frame_id, stamp)
        except ExtrapolationException as e:
            if "future" in str(e) and not final:
                return False                                  # the odometry for this stamp is not here yet
            self.warn(str(e), final)
            return True
        except (LookupException, ConnectivityException, TransformException) as e:
            if not final:
                return False
            self.warn(f"{type(e).__name__}: {e}", final)
            return True
        self.ingest(msg, tf)
        return True

    def warn(self, text: str, final: bool) -> None:
        if final:
            self.dropped += 1
        self.get_logger().warn(text, throttle_duration_sec=5.0)

    # ------------------------------------------------------------------ the actual work
    def ingest(self, msg: Detection3DArray, tf) -> None:
        r = quaternion_to_matrix(tf.transform.rotation)
        t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for det in msg.detections:
            hyp = det.results[0]
            p_cam = np.array([hyp.pose.pose.position.x, hyp.pose.pose.position.y, hyp.pose.pose.position.z])
            cov_cam = np.array(hyp.pose.covariance).reshape(6, 6)[:3, :3]
            self.world.observe(hyp.hypothesis.class_id, r @ p_cam + t, r @ cov_cam @ r.T,
                               hyp.hypothesis.score, t_s)
        self.world.prune(t_s)
        self.publish_markers(msg.header.stamp)

    def publish_markers(self, stamp) -> None:
        markers = MarkerArray()
        for obj in self.world.confirmed():
            m = Marker()
            m.header.frame_id = self.get_parameter("target_frame").value
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = obj.class_name, obj.object_id, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in obj.position)
            m.pose.orientation.w = 1.0
            # the sphere is drawn at 2 sigma: an honest marker shows what the robot does not know
            m.scale.x = m.scale.y = m.scale.z = max(0.05, 4 * obj.sigma_m)
            r, g, b = CLASS_RGB.get(obj.class_name, (0.8, 0.8, 0.8))
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.6
            m.lifetime = Duration(seconds=5.0).to_msg()
            markers.markers.append(m)
        self.pub_markers.publish(markers)

    def report(self) -> None:
        truth = np.array(self.get_parameter("truth").value, dtype=float)
        for obj in self.world.objects:
            line = (f"#{obj.object_id} {obj.class_name:7s} hits {obj.hits:4d} "
                    f"map ({obj.position[0]:+.3f}, {obj.position[1]:+.3f}, {obj.position[2]:+.3f}) "
                    f"sigma {obj.sigma_m * 100:5.1f} cm score {obj.mean_score:.2f}")
            if np.all(np.isfinite(truth)) and obj.class_name == "bottle":
                line += f"  error {np.linalg.norm(obj.position - truth) * 100:5.1f} cm"
            self.get_logger().info(line + ("" if obj.hits >= self.world.min_hits else "  (unconfirmed)"))
        if self.dropped:
            self.get_logger().info(f"{self.dropped} messages dropped: no transform in time")


def main() -> None:
    rclpy.init()
    node = ObjectMapNode()
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
