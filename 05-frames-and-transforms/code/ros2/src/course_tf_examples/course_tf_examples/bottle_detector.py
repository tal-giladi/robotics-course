"""A fake bottle detector publishing geometry_msgs/PointStamped in camera_optical_frame (lessons 05.10, 05.11).

Two modes:

  fixed reading (default)  always reports `point` = (0.05, -0.02, 0.60) in the camera
      ros2 run course_tf_examples bottle_detector

  world bottle             a real bottle standing at `map_point` in the map; reports where the camera
                           sees it at the moment the image was taken, `latency_s` ago
      ros2 run course_tf_examples bottle_detector --ros-args -p map_point:="[2.05, 1.70, 0.165]" -p latency_s:=0.3

Fault injection for 05.11:
      -p stamp_offset_s:=-12.0      stamps 12 s in the past (a stale queue, a wrong clock)
      -p stamp_offset_s:=0.5        stamps 0.5 s in the future (a device clock running ahead)
      -p frame_id:=camera_link      the classic wrong frame_id

A real detector (13.15) publishes the same message: the header says WHEN the image was taken and
WHICH frame the numbers are in.
"""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from course_tf_examples.tf_math import apply, transform_msg_to_matrix


class BottleDetector(Node):
    def __init__(self) -> None:
        super().__init__('bottle_detector')
        self.point = [float(v) for v in self.declare_parameter('point', [0.05, -0.02, 0.60]).value]
        # [] = fixed-reading mode. rclpy needs a typed default for an array parameter, hence [0.0]
        # with "length 3" as the switch.
        world = [float(v) for v in self.declare_parameter('map_point', [0.0]).value]
        self.map_point = world if len(world) == 3 else None
        self.latency = Duration(seconds=self.declare_parameter('latency_s', 0.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'camera_optical_frame').value
        self.offset = Duration(seconds=self.declare_parameter('stamp_offset_s', 0.0).value)
        rate = self.declare_parameter('rate_hz', 5.0).value
        self.pub = self.create_publisher(PointStamped, 'bottle/camera', 10)
        if self.map_point is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / rate, self.tick)

    def tick(self) -> None:
        msg = PointStamped()
        msg.header.frame_id = self.frame_id
        if self.map_point is None:
            # The moment the image was captured. A real driver takes it from the camera, not from
            # "whenever the detector finished" (that can be 50-300 ms later).
            msg.header.stamp = (self.get_clock().now() + self.offset).to_msg()
            msg.point.x, msg.point.y, msg.point.z = self.point
        else:
            # When the "image" was taken: the newest TF time (latency 0) or latency_s before now.
            taken = Time() if self.latency.nanoseconds == 0 else self.get_clock().now() - self.latency
            try:
                # T_optical_map at that moment: where the map point appeared in the camera
                t = self.tf_buffer.lookup_transform('camera_optical_frame', 'map', taken)
            except TransformException as e:
                self.get_logger().warn(f'waiting for TF: {e}', throttle_duration_sec=5.0)
                return
            p = apply(transform_msg_to_matrix(t.transform), self.map_point)
            msg.header.stamp = (Time.from_msg(t.header.stamp) + self.offset).to_msg()
            msg.point.x, msg.point.y, msg.point.z = (float(v) for v in p)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BottleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
