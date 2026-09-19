#!/usr/bin/env python3
"""Record the wheel-velocity step response of whatever is under ros2_control.

    python3 step_response.py --speed 0.3 --ros-args -p use_sim_time:=true   # simulation
    python3 step_response.py --speed 0.3                                    # real robot

Publishes a step on /cmd_vel and logs (t, commanded wheel rad/s, measured wheel rad/s) from
/joint_states to a CSV. Feed the CSV to sysid_fit.py to get the plant's time constant.
"""
import argparse
import csv
import sys

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEEL_RADIUS = 0.045            # labs/config/karmel.yaml: drive.wheel_radius_m


class Step(Node):

    def __init__(self, speed, seconds, out):
        super().__init__('step_response')
        self.speed, self.seconds, self.out = speed, seconds, out
        self.rows = []
        self.t0 = None
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self.on_state, 50)
        self.create_timer(0.02, self.tick)
        self.done = False

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def tick(self):
        t = self.now_s()
        if t == 0.0:                       # waiting for /clock
            return
        if self.t0 is None:
            self.t0 = t
        elapsed = t - self.t0
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # 0.5 s of zero, then the step, then 0.5 s of zero again
        msg.twist.linear.x = self.speed if 0.5 <= elapsed < 0.5 + self.seconds else 0.0
        self.pub.publish(msg)
        if elapsed > self.seconds + 1.0:
            self.done = True

    def on_state(self, msg):
        if self.t0 is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.t0
        elapsed = self.now_s() - self.t0
        cmd = self.speed / WHEEL_RADIUS if 0.5 <= elapsed < 0.5 + self.seconds else 0.0
        try:
            i = msg.name.index('left_wheel_joint')
        except ValueError:
            return
        self.rows.append((round(t, 4), round(cmd, 4), round(msg.velocity[i], 4),
                          round(msg.position[i], 4)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--speed', type=float, default=0.3, help='m/s step')
    p.add_argument('--seconds', type=float, default=3.0, help='how long to hold the step')
    p.add_argument('--out', default='step.csv')
    args, ros_args = p.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = Step(args.speed, args.seconds, args.out)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)
    with open(args.out, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'cmd_rad_s', 'meas_rad_s', 'pos_rad'])
        w.writerows(node.rows)
    print(f'{len(node.rows)} samples -> {args.out}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
