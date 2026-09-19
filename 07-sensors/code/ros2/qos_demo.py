#!/usr/bin/env python3
"""Show what a QoS mismatch looks like from both ends -- publisher and subscriber in one process.

    python3 qos_demo.py                                   # matched: reliable/reliable
    python3 qos_demo.py --pub best_effort --sub reliable   # the classic sensor-topic mismatch
    python3 qos_demo.py --pub best_effort --sub best_effort --size 921600   # big-message drops
    python3 qos_demo.py --pub reliable --sub best_effort   # compatible: works

The rule in one line: the subscriber may ask for *less* than the publisher offers, never more.
RELIABLE is stronger than BEST_EFFORT, TRANSIENT_LOCAL is stronger than VOLATILE, and a deadline
or liveliness the publisher does not promise is a mismatch too. ROS 2 does not fall back and does
not error -- it logs once and delivers nothing, which is why "the topic is there and my callback
never fires" is such a common first day in ROS.

Lessons: 07.10, 07.11 (and 04.11, which teaches QoS itself). Needs ROS 2 Jazzy.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Image

RELIABILITY = {"reliable": ReliabilityPolicy.RELIABLE, "best_effort": ReliabilityPolicy.BEST_EFFORT}
DURABILITY = {"volatile": DurabilityPolicy.VOLATILE,
              "transient_local": DurabilityPolicy.TRANSIENT_LOCAL}


def profile(reliability: str, durability: str, depth: int) -> QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=RELIABILITY[reliability], durability=DURABILITY[durability])


class QosDemo(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("qos_demo")
        self.args = args
        self.sent = 0
        self.got = 0
        self.pub = self.create_publisher(Image, "qos_demo/data",
                                         profile(args.pub, args.pub_durability, args.depth))
        self.create_subscription(Image, "qos_demo/data", self.cb,
                                 profile(args.sub, args.sub_durability, args.depth))
        self.payload = bytes(args.size)
        self.create_timer(1.0 / args.rate, self.tick)
        self.create_timer(args.seconds, self.finish)
        print(f"publisher offers {args.pub}/{args.pub_durability}, "
              f"subscriber requests {args.sub}/{args.sub_durability}, "
              f"depth {args.depth}, {args.size} bytes at {args.rate} Hz for {args.seconds} s")

    def tick(self) -> None:
        msg = Image()                       # an Image is just a convenient bag of bytes here
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = 1, self.args.size
        msg.encoding, msg.step = "mono8", self.args.size
        msg.data = self.payload
        self.pub.publish(msg)
        self.sent += 1

    def cb(self, _msg) -> None:
        self.got += 1

    def finish(self) -> None:
        lost = self.sent - self.got
        pct = 100.0 * self.got / self.sent if self.sent else 0.0
        print(f"\nsent {self.sent}, received {self.got}  ({pct:.1f}% delivered, {lost} lost)")
        if self.got == 0:
            print("verdict: INCOMPATIBLE -- the subscriber asked for a stronger policy than the\n"
                  "         publisher offered. ROS logged one warning and then said nothing.\n"
                  "         Diagnose it on a live system with:  ros2 topic info -v <topic>")
        elif lost > 1:
            print("verdict: compatible but lossy -- BEST_EFFORT or a shallow queue is dropping\n"
                  "         messages. Big messages drop first; raise the depth, use RELIABLE, or\n"
                  "         make the message smaller (compressed transport, fewer points).")
        else:
            print("verdict: compatible, nothing lost "
                  "(one message may still be in flight at the cut-off).")
        raise SystemExit(0)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pub", choices=sorted(RELIABILITY), default="reliable")
    p.add_argument("--sub", choices=sorted(RELIABILITY), default="reliable")
    p.add_argument("--pub-durability", choices=sorted(DURABILITY), default="volatile")
    p.add_argument("--sub-durability", choices=sorted(DURABILITY), default="volatile")
    p.add_argument("--depth", type=int, default=5)
    p.add_argument("--size", type=int, default=1024, help="payload bytes per message")
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--seconds", type=float, default=4.0)
    return p


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if "--ros-args" in argv:
        cut = argv.index("--ros-args")
        argv, ros_args = argv[:cut], argv[cut:]
    args = build_parser().parse_args(argv)
    rclpy.init(args=ros_args or None)
    node = QosDemo(args)
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
