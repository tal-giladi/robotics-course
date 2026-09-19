"""05.06 — REP-105 in numbers: how a localizer computes map -> odom, and why it jumps.

    python 05-frames-and-transforms/code/rep105_demo.py

Frame authorities (REP-105):
    odometry  publishes  odom -> base_link    (continuous, drifts)
    localizer publishes  map  -> odom         (jumps when a correction arrives)
    nobody    publishes  map  -> base_link    (TF2 chains map -> odom -> base_link)

    T_map_odom = T_map_base_link(localizer estimate) @ inverse(T_odom_base_link(odometry))
"""

from __future__ import annotations

import math

import frames_math as fm


def map_to_odom(T_map_base_estimate, T_odom_base):
    """What AMCL / slam_toolbox publish: the correction that makes the TF chain agree."""
    return T_map_base_estimate @ fm.se2_inverse(T_odom_base)


def fmt(T) -> str:
    x, y, th = fm.se2_params(T)
    return f"({x:+.4f}, {y:+.4f}, {math.degrees(th):+.2f} deg)"


def main() -> None:
    T_map_odom = fm.se2(0.0, 0.0, 0.0)                  # before the first correction
    T_odom_base = fm.se2(4.0, 0.0, 0.0)                  # odometry after a 4 m drive
    print("before correction: robot in map =", fmt(T_map_odom @ T_odom_base))

    T_map_base_estimate = fm.se2(3.8, 0.3, math.radians(5))   # scan matched against the map
    T_map_odom = map_to_odom(T_map_base_estimate, T_odom_base)
    print("localizer publishes T_map_odom  =", fmt(T_map_odom))
    print("after correction:  robot in map =", fmt(T_map_odom @ T_odom_base), "(the jump)")
    print("odom -> base_link did not change =", fmt(T_odom_base))

    T_odom_base = T_odom_base @ fm.se2(0.5, 0.0, 0.0)   # drive 0.5 m more, no new correction
    print("0.5 m later: odom -> base_link   =", fmt(T_odom_base))
    print("0.5 m later: robot in map        =", fmt(T_map_odom @ T_odom_base))

    # a goal stored in odom vs map, after a correction
    goal_map = (5.0, 0.3)
    goal_odom = fm.transform_points(fm.se2_inverse(T_map_odom), goal_map)
    print("goal (5.0, 0.3) in map is", goal_odom.round(4), "in odom (valid until the next correction)")


if __name__ == "__main__":
    main()
