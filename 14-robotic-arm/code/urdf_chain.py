"""urdf_chain — read a base->tool kinematic chain straight from a URDF file (standard library XML).

This is an independent path to the same numbers as data/so101_kinematics.yaml: if the YAML was
mistyped, FK computed from the two disagrees and test_so101_data.py fails.

    py urdf_chain.py data/so101_new_calib.urdf base_link gripper_frame_link

Only the kinematic tags are read: <joint type>, <parent>, <child>, <origin xyz rpy>, <axis>,
<limit lower upper>. Meshes, inertias and ros2_control tags are ignored.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from arm_kinematics import Joint, SerialChain


def _floats(text: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if text is None:
        return default
    values = tuple(float(v) for v in text.split())
    if len(values) != 3:
        raise ValueError(f"expected 3 numbers, got {text!r}")
    return values  # type: ignore[return-value]


def parse_joints(urdf_path: Path | str) -> dict[str, Joint]:
    """All joints of a URDF, keyed by CHILD link name (each link has exactly one parent joint)."""
    root = ET.parse(urdf_path).getroot()
    joints: dict[str, Joint] = {}
    for el in root.findall("joint"):
        jtype = el.get("type", "")
        if jtype == "continuous":
            jtype = "revolute"
        origin = el.find("origin")
        axis = el.find("axis")
        limit = el.find("limit")
        child = el.find("child").get("link")  # type: ignore[union-attr]
        joints[child] = Joint(
            name=el.get("name", ""),
            type=jtype,
            parent=el.find("parent").get("link"),  # type: ignore[union-attr]
            child=child,
            xyz=_floats(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
            rpy=_floats(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
            axis=_floats(axis.get("xyz") if axis is not None else None, (1.0, 0.0, 0.0))
            if jtype != "fixed" else (0.0, 0.0, 1.0),
            lower=float(limit.get("lower", "-3.141592653589793")) if limit is not None else -np.pi,
            upper=float(limit.get("upper", "3.141592653589793")) if limit is not None else np.pi,
        )
    return joints


def chain_from_urdf(urdf_path: Path | str, base: str, tip: str) -> SerialChain:
    """Walk from ``tip`` up to ``base`` and return the chain in base->tip order."""
    by_child = parse_joints(urdf_path)
    path: list[Joint] = []
    link = tip
    while link != base:
        if link not in by_child:
            raise ValueError(f"link {link!r} has no parent joint: {base!r} is not an ancestor of {tip!r}")
        joint = by_child[link]
        if joint.type not in ("revolute", "fixed"):
            raise ValueError(f"joint {joint.name} has unsupported type {joint.type!r}")
        path.append(joint)
        link = joint.parent
    return SerialChain(tuple(reversed(path)), name=Path(urdf_path).stem)


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    args = sys.argv[1:] or [str(here / "data" / "so101_new_calib.urdf"), "base_link", "gripper_frame_link"]
    chain = chain_from_urdf(*args)
    np.set_printoptions(precision=4, suppress=True)
    print(f"{chain.name}: {chain.base} -> {chain.tool}, {chain.n_dof} DOF: {chain.joint_names}")
    print("T_base_tool(q = 0) =\n", chain.fk(np.zeros(chain.n_dof)))
