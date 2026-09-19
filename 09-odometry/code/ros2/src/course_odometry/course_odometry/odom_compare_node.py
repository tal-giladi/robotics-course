"""odom_compare — watch several nav_msgs/Odometry topics and print how far apart they are (09.07).

    ros2 run course_odometry odom_compare                                         # /odom vs /odom_mine
    ros2 run course_odometry odom_compare --ros-args -p topics:="['/odom', '/odom_mine', '/odometry/filtered']"
    ros2 run course_odometry odom_compare --ros-args -p csv:=/tmp/odom_compare.csv

Every period seconds it prints, for each topic: pose, path length, the square root of the x/y/yaw
variances from the message's covariance, and the difference to the FIRST topic (the reference).
"""
from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from course_odometry.odometry_core import pose_difference, quaternion_to_yaw


class Track:
    def __init__(self) -> None:
        self.pose: tuple[float, float, float] | None = None
        self.sigmas = (math.nan, math.nan, math.nan)
        self.path_length = 0.0
        self.count = 0

    def update(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = (p.x, p.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
        if self.pose is not None:
            self.path_length += math.hypot(pose[0] - self.pose[0], pose[1] - self.pose[1])
        self.pose = pose
        cov = msg.pose.covariance
        self.sigmas = tuple(math.sqrt(max(cov[i], 0.0)) for i in (0, 7, 35))
        self.count += 1


class OdomCompareNode(Node):

    def __init__(self) -> None:
        super().__init__('odom_compare')
        self.topics = list(self.declare_parameter('topics', ['/odom', '/odom_mine']).value)
        period = self.declare_parameter('period', 1.0).value
        csv_path = self.declare_parameter('csv', '').value
        self.tracks = {topic: Track() for topic in self.topics}
        for topic in self.topics:
            self.create_subscription(Odometry, topic, lambda msg, t=topic: self.tracks[t].update(msg), 20)
        self.csv = open(csv_path, 'w', encoding='utf-8') if csv_path else None
        if self.csv:
            self.csv.write('t,topic,x,y,yaw,path_m,sigma_x,sigma_y,sigma_yaw\n')
        self.create_timer(period, self.report)

    def report(self) -> None:
        reference = self.tracks[self.topics[0]].pose
        now = self.get_clock().now().nanoseconds * 1e-9
        lines = []
        for topic, track in self.tracks.items():
            if track.pose is None:
                lines.append(f'{topic:22s} (no messages yet)')
                continue
            x, y, yaw = track.pose
            line = (f'{topic:22s} x={x:+7.3f} y={y:+7.3f} yaw={math.degrees(yaw):+7.1f}deg '
                    f'path={track.path_length:6.2f}m sigma=({track.sigmas[0]:.3f} m, {track.sigmas[1]:.3f} m, '
                    f'{math.degrees(track.sigmas[2]):.1f} deg)')
            if reference is not None and topic != self.topics[0]:
                dist, dyaw = pose_difference(track.pose, reference)
                line += f' | vs {self.topics[0]}: {dist * 100:.1f} cm, {math.degrees(dyaw):.2f} deg'
            lines.append(line)
            if self.csv:
                self.csv.write(f'{now:.3f},{topic},{x:.4f},{y:.4f},{yaw:.5f},{track.path_length:.3f},'
                               f'{track.sigmas[0]:.5f},{track.sigmas[1]:.5f},{track.sigmas[2]:.5f}\n')
        self.get_logger().info('\n' + '\n'.join(lines))

    def destroy_node(self) -> None:
        if self.csv:
            self.csv.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomCompareNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
