#!/usr/bin/env python3
"""Prove that karmel's simulated body and its real body are the same body (lesson 06.09).

    # render both and compare (needs xacro: run it in the course Docker image or on the robot)
    python3 06-simulation/code/ros2/sim_real_parity.py

    # compare two URDFs you already rendered (works anywhere, including Windows)
    python3 06-simulation/code/ros2/sim_real_parity.py /tmp/karmel_sim.urdf /tmp/karmel_real.urdf

Exit code 0 means: every difference between the two robot descriptions is on the allow-list.

The allow-list is short on purpose. `sim:=true` and `sim:=false` may differ in
  - the <hardware><plugin> inside <ros2_control>, and that plugin's <param>s
  - simulation-only <gazebo> extension blocks (friction, sensors, the gz_ros2_control plugin)
and in nothing else. A wheel radius, a mass, a joint origin or a sensor frame that differs
between the two is a bug you will meet later as "it worked in simulation": the controller
computes odometry from `wheel_radius` and `wheel_separation`, and those come from the same
config file, so a divergence here is a silent, constant odometry scale error on one target only.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

XACRO_DEFAULT = Path("labs/ros2_ws/src/karmel_description/urdf/karmel.urdf.xacro")


def render(xacro_path: Path, use_sim: bool, out: Path) -> Path:
    """Run xacro. `serial_device` must be set for the real build (the xacro requires a device)."""
    args = ["xacro", str(xacro_path), f"use_sim:={str(use_sim).lower()}"]
    if not use_sim:
        args.append("serial_device:=/dev/ttyACM0")
    out.write_text(subprocess.run(args, check=True, capture_output=True, text=True).stdout,
                   encoding="utf-8")
    return out


def _round(value: str | None, digits: int = 9) -> str:
    """Compare numbers as numbers: '0.045' and '0.0450000000001' are the same wheel."""
    if value is None:
        return ""
    parts = []
    for token in value.split():
        try:
            parts.append(f"{round(float(token), digits):+.9f}")
        except ValueError:
            parts.append(token)
    return " ".join(parts)


def kinematics(root: ET.Element) -> dict[str, str]:
    """Everything that decides where the robot's parts are and how heavy they are."""
    facts: dict[str, str] = {}
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is not None:
            facts[f"link[{name}].mass"] = _round(inertial.find("mass").get("value"))
            inertia = inertial.find("inertia")
            for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                facts[f"link[{name}].{key}"] = _round(inertia.get(key))
            origin = inertial.find("origin")
            facts[f"link[{name}].com"] = _round(origin.get("xyz") if origin is not None else "0 0 0")
        for kind in ("visual", "collision"):
            for index, element in enumerate(link.findall(kind)):
                geometry = element.find("geometry")
                shape = list(geometry)[0] if geometry is not None and len(geometry) else None
                if shape is not None:
                    facts[f"link[{name}].{kind}{index}.shape"] = shape.tag
                    for key, value in sorted(shape.attrib.items()):
                        facts[f"link[{name}].{kind}{index}.{key}"] = _round(value)
    for joint in root.findall("joint"):
        name = joint.get("name")
        facts[f"joint[{name}].type"] = joint.get("type", "")
        for tag in ("parent", "child"):
            element = joint.find(tag)
            if element is not None:
                facts[f"joint[{name}].{tag}"] = element.get("link", "")
        origin = joint.find("origin")
        facts[f"joint[{name}].xyz"] = _round(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0")
        facts[f"joint[{name}].rpy"] = _round(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0")
        axis = joint.find("axis")
        if axis is not None:
            facts[f"joint[{name}].axis"] = _round(axis.get("xyz"))
    return facts


def control(root: ET.Element) -> tuple[dict[str, str], str, dict[str, str]]:
    """The <ros2_control> contract: joints and interfaces (must match), plugin + params (may differ)."""
    facts: dict[str, str] = {}
    plugin = ""
    params: dict[str, str] = {}
    for block in root.findall("ros2_control"):
        facts[f"ros2_control[{block.get('name')}].type"] = block.get("type", "")
        hardware = block.find("hardware")
        if hardware is not None:
            plugin = (hardware.findtext("plugin") or "").strip()
            params = {p.get("name"): (p.text or "").strip() for p in hardware.findall("param")}
        for joint in block.findall("joint"):
            name = joint.get("name")
            for kind in ("command_interface", "state_interface"):
                names = sorted(i.get("name") for i in joint.findall(kind))
                facts[f"ros2_control.joint[{name}].{kind}"] = ",".join(names)
            for interface in joint.findall("command_interface"):
                for param in interface.findall("param"):
                    key = f"ros2_control.joint[{name}].{interface.get('name')}.{param.get('name')}"
                    facts[key] = _round(param.text)
    return facts, plugin, params


def compare(sim_urdf: Path, real_urdf: Path) -> int:
    sim = ET.parse(sim_urdf).getroot()
    real = ET.parse(real_urdf).getroot()

    print(f"sim  : {sim_urdf}  ({len(sim.findall('link'))} links, {len(sim.findall('joint'))} joints, "
          f"{len(sim.findall('gazebo'))} <gazebo> blocks)")
    print(f"real : {real_urdf}  ({len(real.findall('link'))} links, {len(real.findall('joint'))} joints, "
          f"{len(real.findall('gazebo'))} <gazebo> blocks)")

    problems: list[str] = []
    sim_facts, real_facts = kinematics(sim), kinematics(real)
    for key in sorted(set(sim_facts) | set(real_facts)):
        a, b = sim_facts.get(key, "<missing>"), real_facts.get(key, "<missing>")
        if a != b:
            problems.append(f"body differs: {key}: sim={a} real={b}")
    print(f"body    : {len(sim_facts)} facts compared (masses, inertias, geometry, joint origins) — "
          f"{'identical' if not problems else str(len(problems)) + ' DIFFER'}")

    sim_control, sim_plugin, sim_params = control(sim)
    real_control, real_plugin, real_params = control(real)
    control_problems = [f"ros2_control differs: {key}: sim={sim_control.get(key, '<missing>')} "
                        f"real={real_control.get(key, '<missing>')}"
                        for key in sorted(set(sim_control) | set(real_control))
                        if sim_control.get(key) != real_control.get(key)]
    problems += control_problems
    print(f"control : {len(sim_control)} facts compared (joints, command and state interfaces, limits) — "
          f"{'identical' if not control_problems else str(len(control_problems)) + ' DIFFER'}")

    print("allowed differences:")
    print(f"  hardware plugin   sim={sim_plugin!r}  real={real_plugin!r}")
    if sim_plugin == real_plugin:
        problems.append("the hardware plugin is the SAME in both builds — the sim/real switch is broken")
    print(f"  hardware params   sim={sim_params or '{}'}  real={real_params or '{}'}")
    print(f"  <gazebo> blocks   sim={len(sim.findall('gazebo'))}  real={len(real.findall('gazebo'))}")
    if real.findall("gazebo"):
        problems.append("the real build contains <gazebo> blocks: simulation-only content leaked "
                        "into the robot description")

    if problems:
        print(f"\nFAIL — {len(problems)} problem(s):")
        for problem in problems:
            print(f"  ! {problem}")
        return 1
    print("\nPASS — the two builds describe the same robot and differ only in the hardware component.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("urdfs", nargs="*", type=Path, metavar="SIM.urdf REAL.urdf")
    parser.add_argument("--xacro", type=Path, default=XACRO_DEFAULT)
    args = parser.parse_args()

    if len(args.urdfs) == 2:
        return compare(*args.urdfs)
    if args.urdfs:
        parser.error("give either two URDF paths or none")
    if shutil.which("xacro") is None:
        parser.error("xacro is not on PATH: run this inside the course Docker image, or pass two "
                     "already-rendered URDF files")
    with tempfile.TemporaryDirectory() as tmp:
        sim = render(args.xacro, True, Path(tmp) / "karmel_sim.urdf")
        real = render(args.xacro, False, Path(tmp) / "karmel_real.urdf")
        return compare(sim, real)


if __name__ == "__main__":
    sys.exit(main())
