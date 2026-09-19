"""Fake 2D LiDAR: a LaserScan in frame `laser`, ray-cast against a 4 m x 3 m room in map (lesson 05.09).

    ros2 run course_tf_examples fake_scan
    ros2 run course_tf_examples fake_scan --ros-args -p frame_id:=base_link     # a frame_id bug for RViz
    ros2 run course_tf_examples fake_scan --ros-args -p mount_error_x:=-0.05    # URDF says 5 cm further forward than reality

Publishes scan (sensor_msgs/LaserScan) at 10 Hz with the sensor-data QoS (best effort), like real
LiDAR drivers. Needs TF map -> laser (the URDF demo launch provides it).
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from course_tf_examples.fake_world import beam_angles, raycast_room
from course_tf_examples.tf_math import yaw_from_quaternion


class FakeScan(Node):
    def __init__(self) -> None:
        super().__init__('fake_scan')
        self.frame_id = self.declare_parameter('frame_id', 'laser').value
        self.samples = int(self.declare_parameter('samples', 360).value)
        self.max_range = float(self.declare_parameter('max_range', 12.0).value)
        # Where the physical LiDAR really is, relative to where TF (the URDF) says: + = further forward.
        self.mount_error_x = float(self.declare_parameter('mount_error_x', 0.0).value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.angles = beam_angles(self.samples)
        self.create_timer(0.1, self.tick)

    def tick(self) -> None:
        try:
            # Where TF says the laser is (always 'laser', whatever frame_id we stamp the scan with).
            t = self.tf_buffer.lookup_transform('map', 'laser', Time())
        except TransformException as e:
            self.get_logger().warn(f'waiting for map <- laser: {e}', throttle_duration_sec=5.0)
            return
        q = t.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        x = t.transform.translation.x + self.mount_error_x * math.cos(yaw)
        y = t.transform.translation.y + self.mount_error_x * math.sin(yaw)
        ranges = raycast_room(x, y, yaw,
                              self.angles, max_range=self.max_range)
        scan = LaserScan()
        scan.header.stamp = t.header.stamp          # the scan is true for the pose we used
        scan.header.frame_id = self.frame_id
        scan.angle_min = float(self.angles[0])
        scan.angle_increment = 2.0 * math.pi / self.samples
        scan.angle_max = scan.angle_min + scan.angle_increment * (self.samples - 1)
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = self.max_range
        scan.ranges = [float(r) for r in ranges]
        self.pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
