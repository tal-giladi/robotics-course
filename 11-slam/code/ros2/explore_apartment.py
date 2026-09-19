#!/usr/bin/env python3
"""Drive karmel through a scripted exploration of a room, for SLAM lessons 11.06-11.09.

Teleop is fine for learning, but a *repeatable* path is what you need when you compare two
slam_toolbox configurations, or when you want the same map twice. This node follows a list of
waypoints expressed in the **odom** frame (odom starts at the robot's start pose, so waypoint
(0, 0) is always "where I started"), with an optional in-place 360 deg spin at chosen stops.

    # simulation (lesson 11.06), from the ros2_ws or anywhere the workspace is sourced
    python3 explore_apartment.py --ros-args -p use_sim_time:=true

    # slower, for the real robot (lesson 11.07)
    python3 explore_apartment.py --plan room --speed 0.12 --yaw-speed 0.5

    # your own path, "x,y[,spin];x,y[,spin];..." in the odom frame
    python3 explore_apartment.py --waypoints "0,0,spin;1.0,-1.4;2.4,-1.4,spin;0,0,spin"

    python3 explore_apartment.py --plan apartment --dry-run     # print the plan, drive nothing

Why the waypoints are in odom and not in map: the map frame jumps whenever slam_toolbox closes a
loop (REP-105), so a controller that chases a map-frame goal would jerk. odom is smooth. The
drift that makes odom unusable over a whole house is exactly what SLAM is there to fix, and over
one room it is small enough to follow a path with.

Controller: turn in place until the heading error is small, then drive forward with a proportional
heading correction. No obstacle avoidance - this is a *scripted* path through free space, not a
planner (that is Nav2, module 12). Keep a hand on the power switch when you run it on hardware.
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


@dataclass
class Waypoint:
    """A goal in the odom frame; `spin` adds a full turn in place once it is reached."""

    x: float
    y: float
    spin: bool = False


# Waypoints for karmel_gazebo's `apartment` world when the robot is spawned at x:=-1.5 y:=-0.5.
# odom coordinates = world coordinates + (1.5, 0.5). The tour: living room -> kitchen -> bedroom
# -> kitchen -> living room -> start, so slam_toolbox gets three chances to close a loop.
PLANS: dict[str, list[Waypoint]] = {
    'apartment': [
        Waypoint(0.0, 0.0, spin=True),      # world (-1.5, -0.5): first look around
        Waypoint(-0.7, -1.4),               # world (-2.2, -1.9): living room, south-west
        Waypoint(-0.7, 0.9),                # world (-2.2, 0.4): living room, west, past the TV
        Waypoint(1.1, 1.4, spin=True),      # world (-0.4, 0.9): living room, centre-north
        Waypoint(1.1, -0.7),                # world (-0.4, -1.2): in front of the kitchen door
        Waypoint(2.4, -0.7),                # world (0.9, -1.2): through the door, into the kitchen
        Waypoint(3.1, -1.55),               # world (1.6, -2.05): under the dining table's south side
        Waypoint(3.1, 0.0),                 # world (1.6, -0.5): kitchen, between the table legs
        Waypoint(3.5, 0.2, spin=True),      # world (2.0, -0.3): kitchen, by the counter
        Waypoint(3.3, 1.0),                 # world (1.8, 0.5): through the bedroom door in the wall
        Waypoint(2.8, 1.8, spin=True),      # world (1.3, 1.3): bedroom, clear of the bed
        Waypoint(1.7, 2.1),                 # world (0.2, 1.6): bedroom door, into the living room
        Waypoint(1.1, 1.4),                 # world (-0.4, 0.9): back where we were  -> loop closure
        Waypoint(1.1, -0.7),                # world (-0.4, -1.2)
        Waypoint(0.0, 0.0, spin=True),      # back to the start                      -> loop closure
    ],
    # A generic "one room" plan for 11.07: a 2 m x 1.5 m rectangle driven twice, spinning at the
    # start and at each return. Small enough for a living room, long enough to build drift.
    'room': [
        Waypoint(0.0, 0.0, spin=True),
        Waypoint(1.5, 0.0),
        Waypoint(1.5, 1.0),
        Waypoint(0.0, 1.0),
        Waypoint(0.0, 0.0, spin=True),
        Waypoint(1.5, 0.0),
        Waypoint(1.5, 1.0),
        Waypoint(0.0, 1.0),
        Waypoint(0.0, 0.0, spin=True),
    ],
}


def yaw_from_quaternion(q) -> float:
    """Yaw of a geometry_msgs/Quaternion, for a robot that only rotates about z."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Explorer(Node):
    """Follow `waypoints` (odom frame) and publish TwistStamped on /cmd_vel."""

    def __init__(self, waypoints: list[Waypoint], speed: float, yaw_speed: float,
                 tolerance: float, topic: str) -> None:
        super().__init__('explore')
        self.waypoints = waypoints
        self.speed = speed
        self.yaw_speed = yaw_speed
        self.tolerance = tolerance
        self.pub = self.create_publisher(TwistStamped, topic, 10)
        # Odometry from diff_drive_controller is BEST_EFFORT-friendly; match the common default.
        self.create_subscription(Odometry, '/odom', self._on_odom,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.pose: tuple[float, float, float] | None = None
        self.index = 0
        self.spin_left = 0.0          # radians of in-place turn still owed at this waypoint
        self.last_yaw: float | None = None
        self.travelled = 0.0
        self.last_xy: tuple[float, float] | None = None
        self.done = False
        self.create_timer(0.05, self._tick)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))
        if self.last_xy is not None:
            self.travelled += math.dist((p.x, p.y), self.last_xy)
        self.last_xy = (p.x, p.y)

    def _publish(self, v: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.pub.publish(msg)

    def _tick(self) -> None:
        if self.pose is None or self.done:
            return
        x, y, yaw = self.pose

        # --- an in-place spin owed at the waypoint we just reached -------------------------
        if self.spin_left > 0.0:
            if self.last_yaw is not None:
                self.spin_left -= abs(wrap(yaw - self.last_yaw))
            self.last_yaw = yaw
            if self.spin_left <= 0.0:
                self.get_logger().info(f'  spin done at ({x:+.2f}, {y:+.2f})')
                self.last_yaw = None
                self._advance()
            else:
                self._publish(0.0, self.yaw_speed)
            return

        goal = self.waypoints[self.index]
        dx, dy = goal.x - x, goal.y - y
        distance = math.hypot(dx, dy)
        if distance < self.tolerance:
            self.get_logger().info(
                f'waypoint {self.index + 1}/{len(self.waypoints)} ({goal.x:+.2f}, {goal.y:+.2f}) '
                f'reached at ({x:+.2f}, {y:+.2f}), {self.travelled:.2f} m driven')
            if goal.spin:
                self.spin_left = 2.0 * math.pi
                self.last_yaw = None
                self._publish(0.0, 0.0)
            else:
                self._advance()
            return

        heading_error = wrap(math.atan2(dy, dx) - yaw)
        if abs(heading_error) > 0.30:                       # turn in place first
            self._publish(0.0, math.copysign(min(self.yaw_speed, 1.5 * abs(heading_error)),
                                             heading_error))
        else:                                               # then drive, correcting as you go
            v = min(self.speed, 0.8 * distance + 0.05)
            self._publish(v, max(-self.yaw_speed, min(self.yaw_speed, 1.2 * heading_error)))

    def _advance(self) -> None:
        self.index += 1
        if self.index >= len(self.waypoints):
            self.done = True
            self._publish(0.0, 0.0)


def parse_waypoints(text: str) -> list[Waypoint]:
    """Parse "x,y[,spin];x,y[,spin];..." into waypoints."""
    out = []
    for chunk in text.split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [p.strip() for p in chunk.split(',')]
        out.append(Waypoint(float(parts[0]), float(parts[1]),
                            spin=len(parts) > 2 and parts[2].lower() in ('spin', '1', 'true')))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--plan', default='apartment', choices=sorted(PLANS))
    parser.add_argument('--waypoints', help='"x,y[,spin];..." in the odom frame; overrides --plan')
    parser.add_argument('--speed', type=float, default=0.18, help='m/s, forward (default 0.18)')
    parser.add_argument('--yaw-speed', type=float, default=0.6, help='rad/s (default 0.6)')
    parser.add_argument('--tolerance', type=float, default=0.12, help='m, waypoint reached')
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--dry-run', action='store_true', help='print the plan and exit')
    args, ros_args = parser.parse_known_args()

    waypoints = parse_waypoints(args.waypoints) if args.waypoints else PLANS[args.plan]
    if args.dry_run:
        total = 0.0
        prev = (0.0, 0.0)
        for i, w in enumerate(waypoints, 1):
            total += math.dist(prev, (w.x, w.y))
            prev = (w.x, w.y)
            print(f'{i:2d}. ({w.x:+.2f}, {w.y:+.2f}){"  + 360 deg spin" if w.spin else ""}')
        spins = sum(w.spin for w in waypoints)
        print(f'{len(waypoints)} waypoints, {total:.1f} m of path, {spins} spins; '
              f'about {total / args.speed + spins * 2 * math.pi / args.yaw_speed:.0f} s at '
              f'{args.speed} m/s')
        return 0

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = Explorer(waypoints, args.speed, args.yaw_speed, args.tolerance, args.topic)
    node.get_logger().info(f'following {len(waypoints)} waypoints at {args.speed} m/s')
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    for _ in range(10):                       # make sure the robot really stops
        node._publish(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
    if node.pose:
        x, y, yaw = node.pose
        node.get_logger().info(f'finished at odom ({x:+.3f}, {y:+.3f}, {math.degrees(yaw):+.1f} deg) '
                               f'after {node.travelled:.2f} m')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
