"""Lesson 05.10: take a detection (PointStamped in camera_optical_frame) and express it in base_link and map.

    ros2 run course_tf_examples bottle_to_map
    ros2 run course_tf_examples bottle_to_map --ros-args -p use_latest:=true      # ignore the stamp (wrong)

Subscribes  bottle/camera       geometry_msgs/PointStamped   (any frame_id; bottle_detector uses the optical frame)
Publishes   bottle/base_link    geometry_msgs/PointStamped
            bottle/map          geometry_msgs/PointStamped
            bottle/marker       visualization_msgs/Marker    (a green sphere in map, for RViz)

Pattern: look the transform up AT THE MESSAGE STAMP, never block inside the callback, and if the
transform for that stamp has not arrived yet, keep the message for a short while and retry. That is
what tf2_ros::MessageFilter does in C++.
"""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import (ConnectivityException, ExtrapolationException, LookupException,
                     TransformException)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker


@dataclass
class Pending:
    msg: PointStamped
    received: Time


class BottleToMap(Node):
    def __init__(self) -> None:
        super().__init__('bottle_to_map')
        self.targets = list(self.declare_parameter('target_frames', ['base_link', 'map']).value)
        self.use_latest = self.declare_parameter('use_latest', False).value
        self.max_wait = Duration(seconds=self.declare_parameter('max_wait_s', 0.5).value)
        self.tf_buffer = Buffer()                       # 10 s of history
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pubs = {f: self.create_publisher(PointStamped, f'bottle/{f}', 10) for f in self.targets}
        self.marker_pub = self.create_publisher(Marker, 'bottle/marker', 10)
        self.pending: list[Pending] = []
        self.create_subscription(PointStamped, 'bottle/camera', self.on_detection, 10)
        self.create_timer(0.02, self.retry_pending)     # 50 Hz: faster than odometry arrives
        self.last_log = self.get_clock().now()

    # ------------------------------------------------------------------ callbacks
    def on_detection(self, msg: PointStamped) -> None:
        if not self.try_transform(msg, final=False):
            self.pending.append(Pending(msg, self.get_clock().now()))

    def retry_pending(self) -> None:
        now = self.get_clock().now()
        still = []
        for item in self.pending:
            give_up = now - item.received > self.max_wait
            if not self.try_transform(item.msg, final=give_up) and not give_up:
                still.append(item)
        self.pending = still

    # ------------------------------------------------------------------ the actual work
    def try_transform(self, msg: PointStamped, final: bool) -> bool:
        """True when done (transformed, or failed for good). False = retry later."""
        stamp = Time() if self.use_latest else Time.from_msg(msg.header.stamp)
        results = {}
        for target in self.targets:
            try:
                # T_target_source at the moment the image was taken. No timeout: we are in a callback.
                t = self.tf_buffer.lookup_transform(target, msg.header.frame_id, stamp)
            except ExtrapolationException as e:
                if 'future' in str(e) and not final:
                    return False                         # odometry for this stamp not here yet
                self.warn(f'{target} <- {msg.header.frame_id}: {e}')
                return True
            except (LookupException, ConnectivityException, TransformException) as e:
                if not final:
                    return False
                self.warn(f'{target} <- {msg.header.frame_id}: {type(e).__name__}: {e}')
                return True
            results[target] = do_transform_point(msg, t)   # header becomes the transform's header
        for target, p in results.items():
            self.pubs[target].publish(p)
        if 'map' in results:
            self.publish_marker(results['map'])
        now = self.get_clock().now()
        if now - self.last_log > Duration(seconds=1.0):
            self.last_log = now
            text = '   '.join(f'in {f}: ({p.point.x:.3f}, {p.point.y:.3f}, {p.point.z:.3f})'
                               for f, p in results.items())
            age_ms = (now - Time.from_msg(msg.header.stamp)).nanoseconds / 1e6
            self.get_logger().info(f'bottle {text}   (stamp age {age_ms:.0f} ms)')
        return True

    def publish_marker(self, p: PointStamped) -> None:
        m = Marker()
        m.header = p.header
        m.ns, m.id, m.type, m.action = 'bottle', 0, Marker.SPHERE, Marker.ADD
        m.pose.position = p.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.07
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 1.0
        m.lifetime = Duration(seconds=1.0).to_msg()     # disappears if detections stop
        self.marker_pub.publish(m)

    def warn(self, text: str) -> None:
        self.get_logger().warn(text, throttle_duration_sec=2.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BottleToMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
