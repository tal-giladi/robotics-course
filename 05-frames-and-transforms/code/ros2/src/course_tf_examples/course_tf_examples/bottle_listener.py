"""Listener: where is the bottle the camera sees, in base_link and in map?

    ros2 run course_tf_examples bottle_listener
    ros2 run course_tf_examples bottle_listener --ros-args -p bottle:="[0.05, -0.02, 0.60]"

The bottle is a fixed point in camera_optical_frame (a detector would publish it; 05.10 does
that with PointStamped and tf2_geometry_msgs). Here we look up the transforms and apply the
4x4 matrices ourselves, so nothing is hidden.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (ConnectivityException, ExtrapolationException, LookupException,
                     TransformException)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from course_tf_examples.tf_math import apply, transform_msg_to_matrix


class BottleListener(Node):
    def __init__(self) -> None:
        super().__init__('bottle_listener')
        self.bottle = [float(v) for v in self.declare_parameter('bottle', [0.05, -0.02, 0.60]).value]
        self.source = self.declare_parameter('source_frame', 'camera_optical_frame').value
        # Buffer = the in-memory cache of every transform received (10 s of history by default).
        # TransformListener = the subscriber to /tf and /tf_static that fills it.
        # Keep BOTH as attributes: if the listener is garbage-collected, the buffer stops filling.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.on_timer)

    def lookup(self, target: str):
        """T_target_source as a 4x4 matrix, or None if TF cannot answer yet."""
        try:
            # Time() == "the latest transform available", not "now".
            # No timeout inside a callback: this node's executor also delivers /tf, so waiting
            # here would block the very messages we are waiting for.
            msg = self.tf_buffer.lookup_transform(target, self.source, Time())
        except LookupException as e:            # a frame does not exist (yet)
            self.get_logger().warn(f'{target} <- {self.source}: unknown frame: {e}',
                                   throttle_duration_sec=5.0)
            return None
        except ConnectivityException as e:      # both frames exist, but in two separate trees
            self.get_logger().error(f'{target} <- {self.source}: not connected: {e}',
                                    throttle_duration_sec=5.0)
            return None
        except ExtrapolationException as e:     # the requested time is outside the buffer
            self.get_logger().warn(f'{target} <- {self.source}: extrapolation: {e}',
                                   throttle_duration_sec=5.0)
            return None
        except TransformException as e:         # base class: anything else
            self.get_logger().error(f'{target} <- {self.source}: {e}', throttle_duration_sec=5.0)
            return None
        return transform_msg_to_matrix(msg.transform)

    def on_timer(self) -> None:
        T_base_cam = self.lookup('base_link')
        T_map_cam = self.lookup('map')
        if T_base_cam is None or T_map_cam is None:
            return
        p_base = apply(T_base_cam, self.bottle)
        p_map = apply(T_map_cam, self.bottle)
        self.get_logger().info(
            'bottle in base_link: ({:.3f}, {:.3f}, {:.3f})   in map: ({:.3f}, {:.3f}, {:.3f})'
            .format(*p_base, *p_map))


def wait_for_transform_demo(args=None) -> None:
    """Blocking with a timeout is fine OUTSIDE callbacks, when another thread fills the buffer.

        ros2 run course_tf_examples wait_for_transform
    """
    rclpy.init(args=args)
    node = Node('wait_for_transform')
    buffer = Buffer()
    # spin_thread=True: the listener gets its own executor thread, so /tf keeps arriving
    # while this (main) thread sleeps inside lookup_transform(timeout=...).
    listener = TransformListener(buffer, node, spin_thread=True)   # noqa: F841 (keep it alive)
    try:
        msg = buffer.lookup_transform('map', 'camera_optical_frame', Time(),
                                      timeout=Duration(seconds=3.0))
        tr = msg.transform.translation
        node.get_logger().info(
            f'map <- camera_optical_frame: translation ({tr.x:.3f}, {tr.y:.3f}, {tr.z:.3f})')
    except TransformException as e:
        node.get_logger().error(f'gave up after 3 s: {type(e).__name__}: {e}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BottleListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
