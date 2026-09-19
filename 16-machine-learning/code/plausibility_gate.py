"""16.01 — an engineered check around a learned component: is this detection physically possible?

    py plausibility_gate.py

A detector says "bottle, 58 px tall". A range sensor (LiDAR beam or depth pixel along the box
centre) says how far away that direction is. The pinhole model (13.05) turns both into a real
height, which must be bottle-sized. Posters of bottles, reflections and bottle-shaped vases far away
fail the check. The network is not trusted blindly; geometry gets the last word.
"""
from __future__ import annotations

import math

FY_PX = (320 / 2) / math.tan(1.20 / 2)          # 320x240 stream, karmel.yaml horizontal FOV 1.20 rad, square pixels
REAL_HEIGHT_M = {"bottle": (0.15, 0.35), "cup": (0.07, 0.15)}
TOLERANCE = 0.2                                 # +-20 %: box looseness, range noise, camera tilt


def implied_height_m(box_height_px: float, range_m: float) -> float:
    """Pinhole model: h_px = fy * H / Z  =>  H = h_px * Z / fy."""
    return box_height_px * range_m / FY_PX


def plausible(label: str, box_height_px: float, range_m: float) -> tuple[bool, float]:
    lo, hi = REAL_HEIGHT_M[label]
    h = implied_height_m(box_height_px, range_m)
    return (lo * (1 - TOLERANCE) <= h <= hi * (1 + TOLERANCE)), h


if __name__ == "__main__":
    print(f"fy = {FY_PX:.1f} px")
    cases = [("bottle", 58, 1.00, "real bottle on the floor"),
             ("bottle", 58, 3.10, "bottle on a poster: the LiDAR sees the wall behind it"),
             ("cup", 40, 2.00, "'cup' that is really a vase across the room"),
             ("cup", 24, 1.20, "real mug on a low table")]
    for label, px, rng, story in cases:
        ok, h = plausible(label, px, rng)
        print(f"{label:6s} {px:3d} px at {rng:.2f} m -> {h:.2f} m tall -> {'ACCEPT' if ok else 'REJECT'}   ({story})")
