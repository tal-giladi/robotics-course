#!/usr/bin/env python3
"""06.05 — Are this robot's inertias physically possible?

    xacro karmel.urdf.xacro use_sim:=true > /tmp/karmel.urdf
    python 06-simulation/code/inertia_check.py /tmp/karmel.urdf

Runs four tests on every link that has an <inertial> block, and prints one line per link:

1. mass > 0
2. the inertia tensor is symmetric positive definite (all three principal moments > 0)
3. the triangle inequality for principal moments:  I1 + I2 >= I3  for every ordering.
   This is not a convention, it is geometry: no rigid body can have a moment larger than the
   sum of the other two (FP.06).
4. plausibility: the radius of gyration k = sqrt(I / m) is compared with the link's collision
   (or visual) geometry. A 25 cm chassis with k = 1 mm is the classic copy-pasted
   "<inertia ixx="0.001" .../>" that makes Gazebo jitter or explode.

Exit code 1 if any link fails test 1-3, so it can be used in CI.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

TRIANGLE_TOLERANCE = 1e-12  # absolute slack, in kg m^2, for floating-point equality


@dataclass
class LinkReport:
    name: str
    mass: float
    principal: np.ndarray  # eigenvalues of the inertia tensor, ascending
    extent: float | None  # largest half-size of the collision/visual geometry, m
    problems: list[str] = field(default_factory=list)

    @property
    def gyration_m(self) -> float:
        """Radius of gyration about the axis with the LARGEST moment: k = sqrt(I / m)."""
        return math.sqrt(float(self.principal[-1]) / self.mass) if self.mass > 0 else math.nan


def geometry_extent(node: ET.Element | None) -> float | None:
    """Largest half-size of a <box>/<cylinder>/<sphere> geometry, in metres (None if unknown)."""
    if node is None:
        return None
    geometry = node.find("geometry")
    if geometry is None:
        return None
    box = geometry.find("box")
    if box is not None:
        return max(float(v) for v in box.get("size", "0 0 0").split()) / 2.0
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        return max(float(cylinder.get("radius", 0.0)), float(cylinder.get("length", 0.0)) / 2.0)
    sphere = geometry.find("sphere")
    if sphere is not None:
        return float(sphere.get("radius", 0.0))
    return None  # a <mesh>: we would have to load the file to know


def inertia_matrix(node: ET.Element) -> np.ndarray:
    g = node.get
    ixx, iyy, izz = float(g("ixx", 0)), float(g("iyy", 0)), float(g("izz", 0))
    ixy, ixz, iyz = float(g("ixy", 0)), float(g("ixz", 0)), float(g("iyz", 0))
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)


def check_link(link: ET.Element) -> LinkReport | None:
    inertial = link.find("inertial")
    if inertial is None:
        return None  # frames like base_footprint carry no mass, and that is fine
    name = link.get("name", "?")
    mass = float(inertial.find("mass").get("value", 0.0))
    principal = np.linalg.eigvalsh(inertia_matrix(inertial.find("inertia")))
    extent = geometry_extent(link.find("collision")) or geometry_extent(link.find("visual"))
    report = LinkReport(name=name, mass=mass, principal=principal, extent=extent)

    if mass <= 0.0:
        report.problems.append(f"mass {mass} kg is not positive")
    if principal[0] <= 0.0:
        report.problems.append(f"smallest principal moment {principal[0]:.3e} is not positive")
    i1, i2, i3 = principal  # ascending, so only the largest can break the inequality
    if i3 > i1 + i2 + TRIANGLE_TOLERANCE:
        report.problems.append(
            f"triangle inequality violated: {i3:.3e} > {i1:.3e} + {i2:.3e}")
    return report


def plausibility(report: LinkReport) -> str:
    """A human verdict on the radius of gyration against the link's own geometry."""
    if report.extent is None or report.mass <= 0 or not math.isfinite(report.gyration_m):
        return "?"
    ratio = report.gyration_m / report.extent
    # A solid body's radius of gyration is between ~0.4 and ~1.0 times its half-size
    # (solid sphere 0.63 R, solid cube about the face axis 0.58 a/2, thin ring 1.0 R).
    if ratio < 0.1:
        return f"SUSPICIOUS (k/size {ratio:.3f}: far too small — placeholder inertia?)"
    if ratio > 2.0:
        return f"SUSPICIOUS (k/size {ratio:.3f}: far too large)"
    return f"ok (k/size {ratio:.2f})"


def report_urdf(path: str) -> tuple[list[LinkReport], float]:
    root = ET.parse(path).getroot()
    reports = [r for link in root.findall("link") if (r := check_link(link)) is not None]
    return reports, sum(r.mass for r in reports)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("urdf", help="a rendered URDF file (xacro ... > file.urdf)")
    args = ap.parse_args()

    reports, total = report_urdf(args.urdf)
    print(f"{'link':22s} {'mass kg':>8} {'I1':>10} {'I2':>10} {'I3':>10} {'k (mm)':>8}  verdict")
    for r in reports:
        i1, i2, i3 = r.principal
        print(f"{r.name:22s} {r.mass:8.3f} {i1:10.3e} {i2:10.3e} {i3:10.3e} "
              f"{r.gyration_m * 1000:8.1f}  {plausibility(r)}")
    print(f"\n{len(reports)} links with mass, total {total:.3f} kg")

    failures = [r for r in reports if r.problems]
    for r in failures:
        for problem in r.problems:
            print(f"FAIL {r.name}: {problem}")
    if failures:
        return 1
    print("all links pass mass > 0, positive definite, and the triangle inequality")
    return 0


if __name__ == "__main__":
    sys.exit(main())
