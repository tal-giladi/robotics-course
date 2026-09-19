"""Static broadcaster: karmel's sensor mounts, published once on /tf_static.

    ros2 run course_tf_examples karmel_static_tf
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from course_tf_examples.tf_math import KARMEL_STATIC, quaternion_from_euler


class KarmelStaticTf(Node):
    def __init__(self) -> None:
        super().__init__('karmel_static_tf')
        self.broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        transforms = []
        for parent, child, x, y, z, roll, pitch, yaw in KARMEL_STATIC:
            t = TransformStamped()
            t.header.stamp = now            # static transforms are valid at all times anyway
            t.header.frame_id = parent      # the frame the numbers are expressed in
            t.child_frame_id = child        # the frame being placed
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            transforms.append(t)
        # One call with the whole list. /tf_static is latched (transient local, depth 1): the
        # Python broadcaster republishes everything it has sent so far on every call, and it
        # silently IGNORES a later transform for a child frame it has already sent.
        self.broadcaster.sendTransform(transforms)
        self.get_logger().info(f'published {len(transforms)} static transforms')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KarmelStaticTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
