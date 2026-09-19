#!/usr/bin/env python3
"""Audit everything slam_toolbox needs, before you blame slam_toolbox (lesson 11.09).

Most bad maps are bad *inputs*: a scan that is too slow for how fast you drive, a laser frame
mounted 180 degrees out, a node that forgot `use_sim_time`, a TF chain with a gap, an odometry
that jumps. This node watches the live system (or a `ros2 bag play`) for a few seconds and prints
a verdict per item, so you can fix the input instead of tuning parameters that were never wrong.

    # live, in simulation
    python3 check_slam_inputs.py --ros-args -p use_sim_time:=true

    # live, on the real robot
    python3 check_slam_inputs.py

    # against a recorded bag (terminal 1: ros2 bag play tour --clock)
    python3 check_slam_inputs.py --seconds 20 --ros-args -p use_sim_time:=true

Exit code 0 = every check passed, 1 = at least one FAIL. Everything it prints is something you
can measure yourself with `ros2 topic hz`, `ros2 topic echo` and `ros2 run tf2_ros tf2_echo`;
this just does all of them at once and applies the thresholds from lesson 11.09.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import tf2_ros

PASS, WARN, FAIL = 'PASS', 'WARN', 'FAIL'


def yaw_of(t: TransformStamped) -> float:
    """Yaw of a TransformStamped's rotation."""
    q = t.transform.rotation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def roll_of(t: TransformStamped) -> float:
    """Roll of a TransformStamped's rotation (pi means the LiDAR is mounted upside down)."""
    q = t.transform.rotation
    return math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))


class Auditor(Node):
    """Collect scans, odometry and TF for a few seconds, then report."""

    def __init__(self, scan_topic: str, odom_topic: str, base_frame: str,
                 odom_frame: str, map_frame: str) -> None:
        super().__init__('check_slam_inputs')
        self.base_frame = base_frame
        self.odom_frame = odom_frame
        self.map_frame = map_frame
        sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, sensor_qos)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        self.scans: list[LaserScan] = []
        self.scan_stamps: list[float] = []
        self.scan_wall: list[float] = []
        self.scan_age: list[float] = []
        self.odom: list[Odometry] = []
        self.odom_stamps: list[float] = []

    @staticmethod
    def _stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_scan(self, msg: LaserScan) -> None:
        self.scans.append(msg)
        self.scan_stamps.append(self._stamp(msg))
        self.scan_wall.append(time.time())
        self.scan_age.append(self.get_clock().now().nanoseconds * 1e-9 - self._stamp(msg))

    def _on_odom(self, msg: Odometry) -> None:
        self.odom.append(msg)
        self.odom_stamps.append(self._stamp(msg))


def rate_of(stamps: list[float]) -> float:
    """Messages per second from a list of timestamps (0.0 if fewer than two)."""
    if len(stamps) < 2:
        return 0.0
    span = stamps[-1] - stamps[0]
    return (len(stamps) - 1) / span if span > 1e-9 else 0.0


def report(results: list[tuple[str, str, str]]) -> int:
    """Print `(status, name, detail)` rows and return a process exit code."""
    width = max(len(name) for _, name, _ in results)
    for status, name, detail in results:
        print(f'  [{status}] {name:<{width}}  {detail}')
    failures = sum(1 for s, _, _ in results if s == FAIL)
    warnings = sum(1 for s, _, _ in results if s == WARN)
    print(f'\n{len(results) - failures - warnings} passed, {warnings} warnings, {failures} failures')
    return 1 if failures else 0


def audit(node: Auditor, seconds: float) -> int:
    """Run the checks of lesson 11.09 against what `node` collected."""
    out: list[tuple[str, str, str]] = []
    scans, odom = node.scans, node.odom

    # ---------------------------------------------------------------- 1. is anything arriving?
    if not scans:
        out.append((FAIL, 'scan topic', 'no LaserScan received — wrong topic, wrong QoS, or no driver'))
        return report(out)
    sim_rate = rate_of(node.scan_stamps)
    wall_rate = (len(node.scan_wall) - 1) / max(node.scan_wall[-1] - node.scan_wall[0], 1e-9)
    rtf = sim_rate and wall_rate / sim_rate
    out.append((PASS if sim_rate >= 4.0 else WARN, 'scan rate',
                f'{sim_rate:5.1f} Hz on the message clock, {wall_rate:5.1f} Hz on the wall clock '
                f'(real-time factor {rtf:.2f})'))

    if not odom:
        out.append((FAIL, 'odometry topic', 'no nav_msgs/Odometry received'))
    else:
        out.append((PASS, 'odometry rate', f'{rate_of(node.odom_stamps):5.1f} Hz'))

    # ---------------------------------------------------------------- 2. clock agreement
    age = sorted(node.scan_age)
    median_age = age[len(age) // 2]
    if abs(median_age) > 5.0:
        status, note = FAIL, 'huge — this node and the publisher are on DIFFERENT clocks'
    elif abs(median_age) > 0.5:
        status, note = WARN, 'large — check transport delay, or a wrong use_sim_time somewhere'
    else:
        status, note = PASS, 'this node and the scan publisher agree on the clock'
    out.append((status, 'scan timestamp age', f'{median_age:+.3f} s median — {note}'))

    # ---------------------------------------------------------------- 3. the scan itself
    s = scans[-1]
    span = s.angle_max - s.angle_min
    n = len(s.ranges)
    out.append((PASS, 'scan geometry',
                f'{n} beams over {math.degrees(span):.1f} deg, '
                f'range {s.range_min:.2f}..{s.range_max:.1f} m, frame_id "{s.header.frame_id}"'))

    vals = [r for r in s.ranges]
    zeros = sum(1 for r in vals if r == 0.0)
    infs = sum(1 for r in vals if math.isinf(r))
    nans = sum(1 for r in vals if math.isnan(r))
    good = n - zeros - infs - nans
    if zeros > 0:
        out.append((FAIL, 'no-return marker',
                    f'{zeros} beams are exactly 0.0 — a driver bug (07.07). Every SLAM system will '
                    f'read them as obstacles on the robot'))
    else:
        out.append((PASS, 'no-return marker', f'{infs} inf, {nans} nan, no zeros — correct'))
    frac = good / n
    out.append((PASS if frac > 0.3 else WARN, 'valid returns',
                f'{good}/{n} beams ({frac:.0%}) in range — '
                f'{"fine" if frac > 0.3 else "very few; is the LiDAR seeing anything?"}'))

    # ---------------------------------------------------------------- 4. the TF chain
    for parent, child in ((node.odom_frame, node.base_frame),
                          (node.base_frame, s.header.frame_id),
                          (node.map_frame, node.odom_frame)):
        try:
            tf = node.buffer.lookup_transform(parent, child, rclpy.time.Time())
        except Exception as exc:                                   # noqa: BLE001 - report any TF error
            status = WARN if parent == node.map_frame else FAIL
            note = ' (only SLAM/AMCL publishes this; fine if none is running)' \
                if parent == node.map_frame else ''
            out.append((status, f'TF {parent} -> {child}', f'missing: {type(exc).__name__}{note}'))
            continue
        t = tf.transform.translation
        detail = (f'({t.x:+.3f}, {t.y:+.3f}, {t.z:+.3f}) m, yaw {math.degrees(yaw_of(tf)):+.1f} deg')
        if parent == node.base_frame:
            roll = abs(roll_of(tf))
            flipped = abs(roll - math.pi) < 0.2
            yaw = math.degrees(yaw_of(tf))
            extra = []
            if flipped:
                extra.append('roll ~180 deg: the LiDAR is UPSIDE DOWN, so the scan is mirrored')
            if abs(yaw) > 5.0:
                extra.append(f'mounted {yaw:+.0f} deg from forward — correct only if it really is')
            status = WARN if extra else PASS
            detail += ('  <- ' + '; '.join(extra)) if extra else ''
            out.append((status, f'TF {parent} -> {child}', detail))
        else:
            out.append((PASS, f'TF {parent} -> {child}', detail))

    # ---------------------------------------------------------------- 5. speed vs scan rate
    if odom and sim_rate > 0:
        speeds = [abs(m.twist.twist.linear.x) for m in odom]
        yaws = [abs(m.twist.twist.angular.z) for m in odom]
        v_max, w_max = max(speeds), max(yaws)
        dx = v_max / sim_rate
        dth = math.degrees(w_max / sim_rate)
        # slam_toolbox's correlative matcher searches +-coarse_search_angle_offset (0.349 rad).
        status = PASS if dth < 10.0 else (WARN if dth < 20.0 else FAIL)
        out.append((status, 'motion per scan',
                    f'at the fastest moment seen ({v_max:.2f} m/s, {math.degrees(w_max):.0f} deg/s) '
                    f'consecutive scans are {dx * 100:.1f} cm and {dth:.1f} deg apart '
                    f'(slam_toolbox searches +-20 deg)'))
        # ... and per *keyframe*, which is what actually gets matched.
        out.append((PASS, 'note',
                    'slam_toolbox only matches scans that pass minimum_travel_distance / '
                    'minimum_travel_heading / minimum_time_interval — see lesson 11.09'))

        jumps = 0
        for a, b in zip(odom, odom[1:]):
            d = math.dist((a.pose.pose.position.x, a.pose.pose.position.y),
                          (b.pose.pose.position.x, b.pose.pose.position.y))
            dt = max(Auditor._stamp(b) - Auditor._stamp(a), 1e-6)
            if d / dt > 3.0:                    # karmel's max is 0.5 m/s (karmel.yaml)
                jumps += 1
        out.append((PASS if jumps == 0 else FAIL, 'odometry continuity',
                    'smooth' if jumps == 0 else
                    f'{jumps} jumps faster than 3 m/s — odom must never teleport (REP-105)'))

    out.append((PASS, 'observation window', f'{seconds:.0f} s, {len(scans)} scans, {len(odom)} odom'))
    return report(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seconds', type=float, default=15.0, help='how long to watch')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--map-frame', default='map')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = Auditor(args.scan_topic, args.odom_topic, args.base_frame,
                   args.odom_frame, args.map_frame)
    print(f'watching for {args.seconds:.0f} s ...\n')
    start = time.time()
    while rclpy.ok() and time.time() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    code = audit(node, args.seconds)
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
