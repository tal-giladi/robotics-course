#!/usr/bin/env python3
"""Drive the robot through a fixed pattern of TwistStamped commands (smoke tests, SLAM demos).

    python3 drive_pattern.py --ros-args -p use_sim_time:=true
    python3 drive_pattern.py --pattern "0.15,0,6;0,0.6,10.5;0.15,0,4" --ros-args -p use_sim_time:=true

pattern = "v,w,seconds;v,w,seconds;..." (m/s, rad/s, seconds of ROS time). Each message is stamped
with the node clock, which matters: diff_drive_controller uses the stamp for its cmd_vel timeout.
Prints the /odom pose at the end.
"""
import argparse
import sys

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

DEFAULT = '0,0.6,10.5;0.15,0,5;0,-0.6,5;0.15,0,4;0,0,1'


class Driver(Node):

    def __init__(self, segments, topic):
        super().__init__('drive_pattern')
        self.segments = segments
        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.odom = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.start = None
        self.done = False
        self.create_timer(0.05, self.tick)

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:          # sim time not received yet
            return
        if self.start is None:
            self.start = now
        elapsed = (now - self.start).nanoseconds / 1e9
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        end = 0.0
        for v, w, duration in self.segments:
            end += duration
            if elapsed < end:
                msg.twist.linear.x, msg.twist.angular.z = v, w
                break
        else:
            self.done = True              # publish one final zero command
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pattern', default=DEFAULT)
    parser.add_argument('--topic', default='/cmd_vel')
    args, ros_args = parser.parse_known_args()
    segments = [tuple(float(x) for x in seg.split(',')) for seg in args.pattern.split(';') if seg]
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = Driver(segments, args.topic)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.odom:
        p = node.odom.pose.pose.position
        node.get_logger().info(f'done; odom x={p.x:.3f} y={p.y:.3f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
