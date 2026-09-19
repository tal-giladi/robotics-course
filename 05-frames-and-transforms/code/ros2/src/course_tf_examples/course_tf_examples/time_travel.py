"""Time travel: where was the robot 2 s ago, expressed in where it is NOW?

    ros2 run course_tf_examples time_travel
    ros2 run course_tf_examples time_travel --ros-args -p delay_s:=20.0     # older than the buffer

lookup_transform_full(target, target_time, source, source_time, fixed_frame) answers:
"take source at source_time, go through fixed_frame (which must not move between the two times),
and express it in target at target_time". odom is a good fixed frame: it is continuous.
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from course_tf_examples.tf_math import yaw_from_quaternion


class TimeTravel(Node):
    def __init__(self) -> None:
        super().__init__('time_travel')
        self.delay = Duration(seconds=self.declare_parameter('delay_s', 2.0).value)
        self.frame = self.declare_parameter('frame', 'base_footprint').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.on_timer)

    def on_timer(self) -> None:
        # "Now" is usually not in the buffer yet (the newest transform is a few ms old), so use
        # the time of the newest transform we have, then go back `delay` from it.
        try:
            latest = self.tf_buffer.lookup_transform('odom', self.frame, rclpy.time.Time())
            t_now = rclpy.time.Time.from_msg(latest.header.stamp)
            msg = self.tf_buffer.lookup_transform_full(
                target_frame=self.frame, target_time=t_now,
                source_frame=self.frame, source_time=t_now - self.delay,
                fixed_frame='odom')
        except TransformException as e:
            self.get_logger().warn(f'{type(e).__name__}: {e}', throttle_duration_sec=5.0)
            return
        tr, q = msg.transform.translation, msg.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f'{self.delay.nanoseconds * 1e-9:.1f} s ago I was at x={tr.x:+.3f} y={tr.y:+.3f} '
            f'yaw={math.degrees(yaw):+.1f} deg (in my current {self.frame})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimeTravel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
