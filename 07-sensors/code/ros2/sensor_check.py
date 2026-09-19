#!/usr/bin/env python3
"""Audit a live sensor topic: rate, jitter, stamp age, frame_id, units and covariance.

    python3 sensor_check.py /imu/data_raw --seconds 5
    python3 sensor_check.py /scan --seconds 5
    python3 sensor_check.py /camera/image_raw --seconds 5
    python3 sensor_check.py /depth/points --seconds 5

This is the first thing to run on any sensor topic, yours or someone else's, before you believe a
single number from it. It answers the six questions of the 07.11 method's "message" layer:

    Is it arriving?  At what rate, and how steady?  How old is the data when I get it?
    What frame is it in?  Are the units right?  Does it say how uncertain it is?

Exit code is 0 when every check passes and 1 when any check fails, so it works in a script.

Lessons: 07.06, 07.07, 07.08, 07.09, 07.10, 07.11. Needs ROS 2 Jazzy (course Docker image).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from imu_conventions import G, is_not_reported  # noqa: E402
from scan_geometry import ScanSpec, valid_mask  # noqa: E402
from sync_math import rate_report, stamp_age_stats  # noqa: E402

TYPES = {"imu": Imu, "scan": LaserScan, "image": Image, "cloud": PointCloud2}
OK, BAD = "  ok  ", " FAIL "


def stamp_seconds(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class SensorCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("sensor_check")
        self.args = args
        self.msgs: list = []
        self.stamps: list[float] = []
        self.received: list[float] = []
        qos = (QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.RELIABLE) if args.reliable
               else qos_profile_sensor_data)
        self.create_subscription(TYPES[args.kind], args.topic, self.cb, qos)

    def cb(self, msg) -> None:
        self.stamps.append(stamp_seconds(msg.header))
        self.received.append(self.get_clock().now().nanoseconds * 1e-9)
        if len(self.msgs) < 3:
            self.msgs.append(msg)


def report(node: SensorCheck, args: argparse.Namespace) -> int:
    fails = 0

    def check(name: str, ok: bool, detail: str) -> None:
        nonlocal fails
        fails += 0 if ok else 1
        print(f"[{OK if ok else BAD}] {name:<22} {detail}")

    n = len(node.stamps)
    print(f"\n{args.topic}  ({TYPES[args.kind].__name__}, {args.seconds:.0f} s window)")
    check("arriving", n > 0, f"{n} messages")
    if n == 0:
        print("\n  Nothing arrived. In order: is the publisher running (`ros2 node list`)? "
              "does the topic name match (`ros2 topic list`)? does the QoS match "
              "(`ros2 topic info -v` on both ends)? is ROS_DOMAIN_ID the same?")
        return 1

    rr = rate_report(node.stamps, args.expect_hz)
    check("rate", (args.expect_hz is None
                   or abs(rr.mean_hz - args.expect_hz) < 0.2 * args.expect_hz),
          f"{rr.mean_hz:.2f} Hz (expected {args.expect_hz}), jitter {rr.jitter_ms:.1f} ms, "
          f"worst gap {rr.max_gap_ms:.1f} ms, ~{rr.dropped_estimate} missing")

    age = stamp_age_stats(node.stamps, node.received)
    check("stamp age", 0.0 <= age["median_ms"] <= args.max_age_ms,
          f"median {age['median_ms']:.1f} ms, p95 {age['p95_ms']:.1f} ms, "
          f"max {age['max_ms']:.1f} ms (limit {args.max_age_ms:.0f} ms)")
    check("stamps increase", all(b > a for a, b in zip(node.stamps, node.stamps[1:])),
          "monotonic" if all(b > a for a, b in zip(node.stamps, node.stamps[1:]))
          else "stamps repeat or go backwards")
    check("stamp is set", node.stamps[0] > 1e8,
          f"{node.stamps[0]:.3f} s" + ("" if node.stamps[0] > 1e8 else "  <- zero/unset stamp"))

    m = node.msgs[0]
    frame = m.header.frame_id
    check("frame_id", bool(frame) and (args.frame is None or frame == args.frame),
          f"'{frame}'" + (f" (expected '{args.frame}')" if args.frame else ""))

    if args.kind == "imu":
        a = m.linear_acceleration
        mag = float(np.linalg.norm([a.x, a.y, a.z]))
        check("units (|a| ~ g)", abs(mag - G) < 1.0,
              f"|linear_acceleration| = {mag:.3f} m/s^2 (expect ~{G:.2f} at rest)")
        check("gyro covariance", not all(v == 0.0 for v in m.angular_velocity_covariance),
              f"diag = {[m.angular_velocity_covariance[i] for i in (0, 4, 8)]}")
        check("accel covariance", not all(v == 0.0 for v in m.linear_acceleration_covariance),
              f"diag = {[m.linear_acceleration_covariance[i] for i in (0, 4, 8)]}")
        orient_ok = is_not_reported(m.orientation_covariance) or \
            not all(v == 0.0 for v in m.orientation_covariance)
        check("orientation field", orient_ok,
              "not reported (-1), as REP-145 wants" if is_not_reported(m.orientation_covariance)
              else f"reported, diag = {[m.orientation_covariance[i] for i in (0, 4, 8)]}")
    elif args.kind == "scan":
        expect_n = int(round((m.angle_max - m.angle_min) / m.angle_increment)) + 1
        check("ranges length", abs(len(m.ranges) - expect_n) <= 1,
              f"{len(m.ranges)} ranges, angle span implies {expect_n}")
        check("angles in radians", abs(m.angle_max - m.angle_min) <= 2 * np.pi + 1e-3,
              f"span = {m.angle_max - m.angle_min:.4f} rad "
              f"({np.degrees(m.angle_max - m.angle_min):.1f} deg)")
        spec = ScanSpec(n=len(m.ranges), angle_min=m.angle_min, angle_max=m.angle_max,
                        range_min=m.range_min, range_max=m.range_max)
        r = np.array(m.ranges, dtype=float)
        good = valid_mask(r, spec)
        zeros = int(np.sum(r == 0.0))
        check("no-return marker", zeros == 0,
              f"{zeros} beams are exactly 0.0 (a driver bug; use inf), "
              f"{int(np.sum(~np.isfinite(r)))} are inf/NaN")
        check("valid fraction", good.mean() > args.min_valid,
              f"{100 * good.mean():.1f}% of {len(r)} beams valid "
              f"(min {100 * args.min_valid:.0f}%), nearest {np.min(r[good]):.3f} m")
    elif args.kind == "image":
        expected_step = {"rgb8": 3, "bgr8": 3, "mono8": 1, "16UC1": 2, "32FC1": 4}.get(m.encoding)
        check("encoding/step", expected_step is None or m.step == m.width * expected_step,
              f"{m.encoding} {m.width}x{m.height}, step {m.step} "
              f"(expect {m.width * (expected_step or 0)})")
        check("data length", len(m.data) == m.step * m.height,
              f"{len(m.data)} bytes, step*height = {m.step * m.height}")
        mbps = len(m.data) * 8 * rr.mean_hz / 1e6
        check("bandwidth", True, f"{mbps:.1f} Mbit/s raw on this topic")
    elif args.kind == "cloud":
        names = [f.name for f in m.fields]
        check("fields", {"x", "y", "z"} <= set(names), f"{names}, point_step {m.point_step}")
        check("row_step", m.row_step == m.point_step * m.width,
              f"{m.row_step} vs point_step*width = {m.point_step * m.width}")
        check("data length", len(m.data) == m.row_step * m.height,
              f"{len(m.data)} bytes for {m.width * m.height} points "
              f"({'organised' if m.height > 1 else 'unordered'})")
        xyz = np.frombuffer(m.data, dtype=np.float32).reshape(-1, m.point_step // 4)[:, :3]
        nans = int(np.isnan(xyz[:, 2]).sum())
        check("is_dense honest", (not m.is_dense) or nans == 0,
              f"is_dense={m.is_dense}, {nans} NaN points")
        finite = xyz[np.isfinite(xyz[:, 2])]
        if finite.size:
            check("z range", True, f"z from {finite[:, 2].min():.3f} to {finite[:, 2].max():.3f} m")

    print(f"\n{'PASS' if fails == 0 else str(fails) + ' CHECK(S) FAILED'}\n")
    return 0 if fails == 0 else 1


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("topic")
    p.add_argument("--kind", choices=sorted(TYPES), default=None,
                   help="message type; guessed from the topic name when omitted")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--expect-hz", type=float, default=None)
    p.add_argument("--frame", default=None, help="the frame_id you expect")
    p.add_argument("--max-age-ms", type=float, default=200.0)
    p.add_argument("--min-valid", type=float, default=0.5, help="LaserScan: valid-beam fraction")
    p.add_argument("--reliable", action="store_true",
                   help="subscribe RELIABLE (the default is the sensor-data QoS)")
    return p


def guess_kind(topic: str) -> str:
    t = topic.lower()
    for key, kind in (("imu", "imu"), ("scan", "scan"), ("point", "cloud"), ("cloud", "cloud"),
                      ("depth", "image"), ("image", "image")):
        if key in t:
            return kind
    return "imu"


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if "--ros-args" in argv:
        cut = argv.index("--ros-args")
        argv, ros_args = argv[:cut], argv[cut:]
    args = build_parser().parse_args(argv)
    args.kind = args.kind or guess_kind(args.topic)
    rclpy.init(args=ros_args or None)
    node = SensorCheck(args)
    end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = report(node, args)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
