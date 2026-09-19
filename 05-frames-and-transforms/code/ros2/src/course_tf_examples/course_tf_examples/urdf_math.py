"""robot_state_publisher in miniature: read a URDF's joints and compute T_a_b between any two links.

No rclpy import: runs anywhere (lesson 05.08).

    ros2 run course_tf_examples urdf_chain /tmp/karmel.urdf base_link camera_optical_frame
    ros2 run course_tf_examples urdf_chain /tmp/karmel.urdf base_link left_wheel left_wheel_joint=1.5708
    python3 urdf_math.py karmel.urdf base_link laser                     (no ROS needed)

Notation (module 05): T_a_b is the pose of frame b expressed in frame a; p_a = T_a_b @ p_b.
A URDF joint's <origin> is T_parent_jointframe; the child link frame is the joint frame moved by
the joint value q about/along <axis>:  T_parent_child = T_origin @ motion(axis, q).
"""

from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

MOVABLE = ('revolute', 'continuous', 'prismatic')


@dataclass(frozen=True)
class Joint:
    name: str
    type: str                                   # fixed | continuous | revolute | prismatic | ...
    parent: str
    child: str
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: tuple[float, float, float] = (1.0, 0.0, 0.0)   # URDF default when <axis> is omitted
    lower: float | None = None
    upper: float | None = None


def _floats(text: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if text is None:
        return default
    values = tuple(float(v) for v in text.split())
    if len(values) != 3:
        raise ValueError(f'expected 3 numbers, got {text!r}')
    return values  # type: ignore[return-value]


def parse_urdf(xml_text: str) -> dict[str, Joint]:
    """All joints of a (xacro-rendered) URDF, keyed by CHILD link name (each link has one parent)."""
    robot = ET.fromstring(xml_text)
    links = {link.get('name') for link in robot.findall('link')}
    joints: dict[str, Joint] = {}
    for j in robot.findall('joint'):
        origin = j.find('origin')
        axis = j.find('axis')
        limit = j.find('limit')
        joint = Joint(
            name=j.get('name'),
            type=j.get('type'),
            parent=j.find('parent').get('link'),
            child=j.find('child').get('link'),
            xyz=_floats(origin.get('xyz') if origin is not None else None, (0.0, 0.0, 0.0)),
            rpy=_floats(origin.get('rpy') if origin is not None else None, (0.0, 0.0, 0.0)),
            axis=_floats(axis.get('xyz') if axis is not None else None, (1.0, 0.0, 0.0)),
            lower=float(limit.get('lower')) if limit is not None and limit.get('lower') else None,
            upper=float(limit.get('upper')) if limit is not None and limit.get('upper') else None,
        )
        for link in (joint.parent, joint.child):
            if link not in links:
                raise ValueError(f'joint {joint.name} refers to unknown link {link!r}')
        if joint.child in joints:
            raise ValueError(f'link {joint.child!r} has two parents: '
                             f'{joints[joint.child].parent!r} and {joint.parent!r}')
        joints[joint.child] = joint
    return joints


def root_links(xml_text: str) -> list[str]:
    """Links that are nobody's child. A valid URDF has exactly one."""
    robot = ET.fromstring(xml_text)
    children = {j.find('child').get('link') for j in robot.findall('joint')}
    return [link.get('name') for link in robot.findall('link') if link.get('name') not in children]


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF/ROS rpy: fixed axes x, y, z  ->  R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_matrix(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    """Rodrigues' formula: rotation by ``angle`` about the unit vector ``axis``."""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def joint_transform(joint: Joint, q: float = 0.0) -> np.ndarray:
    """T_parent_child for one joint at position q (rad for revolute/continuous, m for prismatic)."""
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*joint.rpy)
    T[:3, 3] = joint.xyz
    if joint.type in ('revolute', 'continuous') and q != 0.0:
        M = np.eye(4)
        M[:3, :3] = axis_angle_matrix(joint.axis, q)
        T = T @ M
    elif joint.type == 'prismatic' and q != 0.0:
        M = np.eye(4)
        M[:3, 3] = np.asarray(joint.axis, dtype=float) / np.linalg.norm(joint.axis) * q
        T = T @ M
    return T


def transform_from_root(joints: dict[str, Joint], link: str,
                        positions: dict[str, float] | None = None) -> tuple[str, np.ndarray]:
    """(root, T_root_link): walk parent pointers up to the root and compose the joint transforms."""
    positions = positions or {}
    T = np.eye(4)
    seen = set()
    while link in joints:
        if link in seen:
            raise ValueError(f'cycle through link {link!r}')
        seen.add(link)
        joint = joints[link]
        T = joint_transform(joint, positions.get(joint.name, 0.0)) @ T
        link = joint.parent
    return link, T


def chain_transform(joints: dict[str, Joint], frame_a: str, frame_b: str,
                    positions: dict[str, float] | None = None) -> np.ndarray:
    """T_a_b between any two links of the tree: (T_root_a)^-1 @ T_root_b."""
    root_a, T_root_a = transform_from_root(joints, frame_a, positions)
    root_b, T_root_b = transform_from_root(joints, frame_b, positions)
    if root_a != root_b:
        raise ValueError(f'{frame_a!r} and {frame_b!r} are not in the same tree '
                         f'(roots {root_a!r} and {root_b!r})')
    R, t = T_root_a[:3, :3], T_root_a[:3, 3]
    T_a_root = np.eye(4)
    T_a_root[:3, :3] = R.T
    T_a_root[:3, 3] = -R.T @ t
    return T_a_root @ T_root_b


def matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) with w >= 0, from a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0, R[1, 0] - R[0, 1])
    return x, y, z, w


# ---------------------------------------------------------------- inertia of simple solids
# The same formulas as labs/ros2_ws/src/karmel_description/urdf/inertial_macros.xacro
# (solid, uniform density, about the centre of mass). Derivation: FP.06.

def box_inertia(m: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    return m / 12.0 * (y * y + z * z), m / 12.0 * (x * x + z * z), m / 12.0 * (x * x + y * y)


def cylinder_inertia(m: float, r: float, h: float) -> tuple[float, float, float]:
    """Cylinder with its axis along z."""
    side = m / 12.0 * (3.0 * r * r + h * h)
    return side, side, m * r * r / 2.0


def sphere_inertia(m: float, r: float) -> tuple[float, float, float]:
    i = 2.0 / 5.0 * m * r * r
    return i, i, i


def format_transform(T: np.ndarray) -> str:
    """Print like `ros2 run tf2_ros tf2_echo` so the two can be compared line by line."""
    x, y, z, w = matrix_to_quaternion(T[:3, :3])
    sp = -T[2, 0]
    pitch = math.asin(max(-1.0, min(1.0, sp)))
    roll = math.atan2(T[2, 1], T[2, 2])
    yaw = math.atan2(T[1, 0], T[0, 0])
    lines = [
        '- Translation: [{:.3f}, {:.3f}, {:.3f}]'.format(*T[:3, 3]),
        f'- Rotation: in Quaternion (xyzw) [{x:.3f}, {y:.3f}, {z:.3f}, {w:.3f}]',
        f'- Rotation: in RPY (radian) [{roll:.3f}, {pitch:.3f}, {yaw:.3f}]',
        f'- Rotation: in RPY (degree) [{math.degrees(roll):.3f}, {math.degrees(pitch):.3f}, '
        f'{math.degrees(yaw):.3f}]',
        '- Matrix:',
    ]
    lines += ['  ' + ' '.join(f'{v:6.3f}' for v in row) for row in T]
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    args = [a for a in argv if not a.startswith('--ros-args')]
    if len(args) < 3:
        print('usage: urdf_chain FILE.urdf FRAME_A FRAME_B [joint_name=value ...]\n'
              '       prints T_a_b (the pose of FRAME_B in FRAME_A), like tf2_echo FRAME_A FRAME_B')
        return 2
    path, frame_a, frame_b, *assignments = args
    positions = {}
    for item in assignments:
        name, _, value = item.partition('=')
        positions[name] = float(value)
    with open(path, encoding='utf-8') as f:
        text = f.read()
    joints = parse_urdf(text)
    print(f'root link: {", ".join(root_links(text))}')
    print(f'T_{frame_a}_{frame_b}:')
    print(format_transform(chain_transform(joints, frame_a, frame_b, positions)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
