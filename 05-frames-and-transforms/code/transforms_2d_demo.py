"""05.03 — 2D rigid transforms on karmel's running example, three ways.

    python 05-frames-and-transforms/code/transforms_2d_demo.py

1. by hand, with the closed-form formulas of the lesson
2. with 3x3 matrices from frames_math.py
3. with the course library robotlab.geometry.SE2 (labs/python)

Notation: T_a_b = pose of frame b expressed in frame a; p_a = T_a_b @ p_b.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

import frames_math as fm

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "labs" / "python"))
from robotlab.geometry import SE2  # noqa: E402


def compose_by_hand(a: tuple[float, float, float], b: tuple[float, float, float]):
    """T_a_b (x1, y1, th1) composed with T_b_c (x2, y2, th2) -> T_a_c."""
    x1, y1, th1 = a
    x2, y2, th2 = b
    c, s = math.cos(th1), math.sin(th1)
    return (x1 + c * x2 - s * y2, y1 + s * x2 + c * y2, fm.wrap_angle(th1 + th2))


def invert_by_hand(a: tuple[float, float, float]):
    """T_a_b -> T_b_a."""
    x, y, th = a
    c, s = math.cos(th), math.sin(th)
    return (-c * x - s * y, s * x - c * y, fm.wrap_angle(-th))


def apply_by_hand(a: tuple[float, float, float], p: tuple[float, float]):
    """p_a = T_a_b @ p_b."""
    x, y, th = a
    c, s = math.cos(th), math.sin(th)
    return (x + c * p[0] - s * p[1], y + s * p[0] + c * p[1])


def fmt(pose) -> str:
    x, y, th = pose
    return f"({x:+.4f}, {y:+.4f}, {math.degrees(th):+.2f} deg)"


def main() -> None:
    T_map_base = (2.0, 1.0, math.radians(90))        # karmel in the map
    T_base_camera = (0.10, 0.0, 0.0)                 # camera_link on the robot (top view)
    p_camera = (0.60, -0.05)                          # bottle in camera_link (top view, from 05.01)

    # 1. by hand
    T_map_camera = compose_by_hand(T_map_base, T_base_camera)
    print("by hand   T_map_camera   ", fmt(T_map_camera))
    print("by hand   T_base_map     ", fmt(invert_by_hand(T_map_base)))
    print("by hand   bottle in map   ({:+.4f}, {:+.4f})".format(*apply_by_hand(T_map_camera, p_camera)))

    # 2. matrices
    M_map_base, M_base_camera = fm.se2(*T_map_base), fm.se2(*T_base_camera)
    M_map_camera = M_map_base @ M_base_camera
    print("matrix    T_map_camera   ", fmt(fm.se2_params(M_map_camera)))
    print("matrix    T_base_map     ", fmt(fm.se2_params(fm.se2_inverse(M_map_base))))
    print("matrix    bottle in map  ", fm.transform_points(M_map_camera, p_camera).round(4))
    p_map = fm.transform_points(M_map_camera, p_camera)
    print("matrix    bottle back in base_link",
          fm.transform_points(fm.se2_inverse(M_map_base), p_map).round(4))

    # 3. robotlab.geometry.SE2 (the course library uses the same convention: world_T_robot)
    map_T_base, base_T_camera = SE2(*T_map_base), SE2(*T_base_camera)
    map_T_camera = map_T_base @ base_T_camera
    print("robotlab  T_map_camera   ", fmt(map_T_camera))
    print("robotlab  bottle in map  ", map_T_camera.apply(np.array(p_camera)).round(4))

    # order matters
    print("WRONG ORDER T_base_camera @ T_map_base", fmt(compose_by_hand(T_base_camera, T_map_base)))
    # relative pose of two robots
    robot_a, robot_b = (2.0, 1.0, math.radians(90)), (3.0, 1.0, math.radians(180))
    print("T_a_b (robot b seen from robot a)", fmt(compose_by_hand(invert_by_hand(robot_a), robot_b)))
    # motion in the robot's own frame: 0.5 m forward, then 30 deg right
    print("after moving in base_link", fmt(compose_by_hand(T_map_base, (0.5, 0.0, math.radians(-30)))))


if __name__ == "__main__":
    main()
