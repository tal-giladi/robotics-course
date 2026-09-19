"""05.02 — poses, wrapping and bearings. Standard library only."""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]. Same result as robotlab.geometry.wrap_angle."""
    return math.pi - ((math.pi - a) % (2.0 * math.pi))


@dataclass(frozen=True)
class Pose2D:
    x: float        # m, in the frame named by the variable (e.g. robot_in_map)
    y: float        # m
    theta: float    # rad, counter-clockwise from the frame's +x axis, wrapped to (-pi, pi]

    def __post_init__(self) -> None:
        object.__setattr__(self, "theta", wrap_angle(self.theta))


def bearing_to(pose: Pose2D, gx: float, gy: float) -> float:
    """Direction from the robot's position to (gx, gy), measured in the same frame as the pose."""
    return math.atan2(gy - pose.y, gx - pose.x)


def heading_error(pose: Pose2D, gx: float, gy: float) -> float:
    """How much to turn (positive = left/CCW) to face the goal. Always in (-pi, pi]."""
    return wrap_angle(bearing_to(pose, gx, gy) - pose.theta)


def circular_mean(angles: list[float]) -> float:
    """Mean direction: average the unit vectors, not the numbers."""
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


if __name__ == "__main__":
    deg = math.degrees
    for a in (190, -270, 540, -180, 360):
        print(f"wrap({a:5d} deg) = {deg(wrap_angle(math.radians(a))):7.2f} deg")
    print(f"wrap(7.0 rad)   = {wrap_angle(7.0):.4f} rad")

    robot = Pose2D(2.0, 1.0, math.radians(90))
    bottle = (2.05, 1.70)
    print(f"bottle: bearing {deg(bearing_to(robot, *bottle)):.2f} deg, "
          f"turn {deg(heading_error(robot, *bottle)):+.2f} deg, "
          f"distance {math.hypot(bottle[0] - robot.x, bottle[1] - robot.y):.4f} m")
    print(f"goal (1, 0): turn {deg(heading_error(robot, 1.0, 0.0)):+.2f} deg")

    theta = 2.5
    for _ in range(100):                  # 10 s of turning at 0.5 rad/s, dt = 0.1 s
        theta = wrap_angle(theta + 0.5 * 0.1)
    print(f"after 10 s at 0.5 rad/s from 2.5 rad: {theta:.4f} rad ({deg(theta):.2f} deg)")

    headings = [math.radians(a) for a in (350, 10, 20)]
    print(f"naive mean {deg(sum(headings) / 3):.2f} deg, circular mean {deg(circular_mean(headings)):.2f} deg")
