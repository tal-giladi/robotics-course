#!/usr/bin/env python3
"""Synchronise two sensor streams with message_filters, and show what the slop actually buys.

    # terminal 1
    python3 fake_lidar.py --rate 10
    # terminal 2
    python3 fake_imu.py --rate 100
    # terminal 3
    python3 sync_demo.py --seconds 6
    python3 sync_demo.py --seconds 6 --slop 0.002      # too tight: almost nothing pairs
    python3 sync_demo.py --seconds 6 --exact           # ExactTime: nothing ever pairs

Prints, per second, how many scans arrived, how many IMU samples arrived, how many synchronised
pairs came out, and the distribution of the time difference inside each pair.

Why you need this: a node that consumes two sensors must consume them *as a pair taken at the same
instant*. Two callbacks storing into member variables is the wrong answer -- it silently pairs
whatever happened to arrive last, and the error it introduces grows with the robot's speed.

Lessons: 07.10, 07.11. Needs ROS 2 Jazzy (course Docker image).
"""

from __future__ import annotations

import argparse
import statistics
import sys

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def stamp_s(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class SyncDemo(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("sync_demo")
        self.args = args
        self.n_scan = 0
        self.n_imu = 0
        self.dts: list[float] = []

        # message_filters Subscriber takes the same QoS as a normal subscription. Sensor drivers
        # publish with the sensor-data profile (BEST_EFFORT), so ask for that or nothing arrives.
        scan_sub = message_filters.Subscriber(self, LaserScan, args.scan_topic,
                                              qos_profile=qos_profile_sensor_data)
        imu_sub = message_filters.Subscriber(self, Imu, args.imu_topic,
                                             qos_profile=qos_profile_sensor_data)
        scan_sub.registerCallback(lambda _m: self.count("scan"))
        imu_sub.registerCallback(lambda _m: self.count("imu"))

        if args.exact:
            self.sync = message_filters.TimeSynchronizer([scan_sub, imu_sub], args.queue)
        else:
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [scan_sub, imu_sub], args.queue, args.slop)
        self.sync.registerCallback(self.on_pair)

        self.create_timer(1.0, self.report)
        self.create_timer(args.seconds, self.finish)
        policy = "ExactTime" if args.exact else f"ApproximateTime(slop={args.slop * 1e3:.0f} ms)"
        print(f"{policy}, queue {args.queue}: {args.scan_topic} + {args.imu_topic}")

    def count(self, which: str) -> None:
        if which == "scan":
            self.n_scan += 1
        else:
            self.n_imu += 1

    def on_pair(self, scan: LaserScan, imu: Imu) -> None:
        self.dts.append((stamp_s(scan) - stamp_s(imu)) * 1e3)

    def report(self) -> None:
        print(f"  scans {self.n_scan:4d}   imu {self.n_imu:5d}   pairs {len(self.dts):4d}")

    def finish(self) -> None:
        print(f"\nscans {self.n_scan}, imu {self.n_imu}, pairs {len(self.dts)} "
              f"({100 * len(self.dts) / max(1, self.n_scan):.0f}% of scans paired)")
        if self.dts:
            print(f"pair |dt|: mean {statistics.fmean(abs(d) for d in self.dts):.2f} ms, "
                  f"max {max(abs(d) for d in self.dts):.2f} ms")
        else:
            print("no pairs at all. ExactTime needs bit-identical stamps, which two independent\n"
                  "sensors never have. Use ApproximateTimeSynchronizer with a slop of about half\n"
                  "the period of the SLOWER topic.")
        raise SystemExit(0)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scan-topic", default="/scan")
    p.add_argument("--imu-topic", default="/imu/data_raw")
    p.add_argument("--slop", type=float, default=0.05, help="seconds")
    p.add_argument("--queue", type=int, default=30)
    p.add_argument("--exact", action="store_true", help="use ExactTime instead")
    p.add_argument("--seconds", type=float, default=6.0)
    return p


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if "--ros-args" in argv:
        cut = argv.index("--ros-args")
        argv, ros_args = argv[:cut], argv[cut:]
    args = build_parser().parse_args(argv)
    rclpy.init(args=ros_args or None)
    node = SyncDemo(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
