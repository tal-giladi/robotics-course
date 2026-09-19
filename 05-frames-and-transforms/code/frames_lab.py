"""frames_lab — draw frames, points and transform chains to PNG files (numpy + matplotlib).

Headless-safe: always renders with the Agg backend and writes PNGs, so it works over SSH, in
Docker, in CI and on Windows alike. Open the PNG to inspect it.

    py frames_lab.py list                     # scenes used by the module-05 exercises
    py frames_lab.py laser-point              # -> frames_out/laser-point.png
    py frames_lab.py all --out my_pngs

Or import the drawing helpers in your own exercise script:

    from frames_lab import new_axes_2d, draw_frame, draw_point, save
    from frames_math import se2, transform_points

Colors follow REP-103 / RViz: x red, y green, z blue. Notation (see frames_math.py):
T_a_b is the pose of frame b in frame a; p_a = T_a_b @ p_b.
"""

from __future__ import annotations

import argparse
import math
import os
from collections.abc import Callable, Sequence
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless: never needs a display

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.axes import Axes  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from numpy.typing import ArrayLike  # noqa: E402

import frames_math as fm  # noqa: E402

AXIS_COLORS = ("tab:red", "tab:green", "tab:blue")
POINT_COLOR = "tab:orange"
DEFAULT_OUT = Path(os.environ.get("FRAMES_LAB_OUT", "frames_out"))


# --------------------------------------------------------------------------------------------
# 2D drawing
# --------------------------------------------------------------------------------------------
def new_axes_2d(title: str = "", frame: str = "map", ax: Axes | None = None,
                xlim: tuple[float, float] | None = None,
                ylim: tuple[float, float] | None = None) -> tuple[Figure, Axes]:
    """An equal-aspect 2D plot whose axes are the coordinates of ``frame``."""
    if ax is None:
        fig, ax = plt.subplots(figsize=(6.5, 6.0))
    else:
        fig = ax.figure
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel(f"x in {frame} [m]")
    ax.set_ylabel(f"y in {frame} [m]")
    if title:
        ax.set_title(title)
    if xlim:
        ax.set_xlim(*xlim)
    if ylim:
        ax.set_ylim(*ylim)
    return fig, ax


def new_side_by_side(titles: Sequence[str], frames: Sequence[str]) -> tuple[Figure, list[Axes]]:
    """Several 2D panels next to each other — e.g. the same scene in base_link and in map."""
    fig, axes = plt.subplots(1, len(titles), figsize=(6.0 * len(titles), 6.0))
    axes = list(np.atleast_1d(axes))
    for ax, title, frame in zip(axes, titles, frames):
        new_axes_2d(title, frame, ax=ax)
    return fig, axes


def draw_frame(ax: Axes, T_ref_frame: ArrayLike, name: str, length: float = 0.3,
               linewidth: float = 2.0, label_offset: tuple[float, float] = (-0.04, -0.06)) -> None:
    """Draw frame ``name`` whose pose in the plot's reference frame is the 3x3 ``T_ref_frame``."""
    T = np.asarray(T_ref_frame, dtype=float)
    origin = T[:2, 2]
    for axis, color in zip((0, 1), AXIS_COLORS):
        direction = T[:2, axis] * length
        ax.annotate("", xy=origin + direction, xytext=origin,
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=linewidth))
    ax.plot(*origin, "k.", markersize=6)
    ax.text(origin[0] + label_offset[0], origin[1] + label_offset[1], name, fontsize=9,
            ha="right", va="top")


def draw_point(ax: Axes, p_ref: ArrayLike, label: str = "", color: str = POINT_COLOR,
               marker: str = "o") -> None:
    """Draw a point given in the plot's reference frame, with its coordinates in the label."""
    x, y = np.asarray(p_ref, dtype=float)[:2]
    ax.plot(x, y, marker, color=color, markersize=9, markeredgecolor="k")
    text = f"{label} ({x:.3f}, {y:.3f})" if label else f"({x:.3f}, {y:.3f})"
    ax.text(x + 0.05, y + 0.05, text, fontsize=9, color="k")


def draw_offset(ax: Axes, T_ref_parent: ArrayLike, T_ref_child: ArrayLike, label: str = "",
                color: str = "0.4") -> None:
    """A dashed arrow from the parent's origin to the child's origin: 'this is T_parent_child'."""
    a = np.asarray(T_ref_parent, dtype=float)[:2, 2]
    b = np.asarray(T_ref_child, dtype=float)[:2, 2]
    ax.annotate("", xy=b, xytext=a,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.2, linestyle="--"))
    if label:
        mid = (a + b) / 2
        ax.text(mid[0], mid[1] + 0.04, label, fontsize=8, color=color, ha="center")


def draw_ray(ax: Axes, T_ref_sensor: ArrayLike, p_ref: ArrayLike, color: str = POINT_COLOR) -> None:
    """A thin line from a sensor's origin to what it detected."""
    o = np.asarray(T_ref_sensor, dtype=float)[:2, 2]
    p = np.asarray(p_ref, dtype=float)[:2]
    ax.plot([o[0], p[0]], [o[1], p[1]], "-", color=color, lw=1.0, alpha=0.8)


def draw_robot(ax: Axes, T_ref_base: ArrayLike, length: float = 0.25, width: float = 0.20,
               wheel_radius: float = fm.KARMEL_WHEEL_RADIUS_M) -> None:
    """karmel's footprint (chassis rectangle and two wheels) at pose T_ref_base."""
    T = np.asarray(T_ref_base, dtype=float)
    hl, hw = length / 2, width / 2
    body = np.array([[hl, hw], [-hl, hw], [-hl, -hw], [hl, -hw], [hl, hw]])
    ax.fill(*fm.transform_points(T, body).T, color="0.85", edgecolor="0.3", zorder=0)
    for side in (1, -1):
        wheel = np.array([[wheel_radius, side * hw], [-wheel_radius, side * hw]])
        ax.plot(*fm.transform_points(T, wheel).T, "-", color="0.2", lw=5, zorder=1)


def autoscale(ax: Axes, points: ArrayLike, margin: float = 0.4) -> None:
    """Fit the view around a set of (N, 2) points with a margin, keeping equal aspect."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    lo, hi = p.min(axis=0) - margin, p.max(axis=0) + margin
    center, half = (lo + hi) / 2, max(hi - lo) / 2
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)


# --------------------------------------------------------------------------------------------
# 3D drawing
# --------------------------------------------------------------------------------------------
def new_axes_3d(title: str = "", frame: str = "map", ax: Axes | None = None) -> tuple[Figure, Axes]:
    if ax is None:
        fig = plt.figure(figsize=(7.5, 7.0))
        ax = fig.add_subplot(projection="3d")
    else:
        fig = ax.figure
    ax.set_xlabel(f"x in {frame} [m]")
    ax.set_ylabel(f"y in {frame} [m]")
    ax.set_zlabel(f"z in {frame} [m]")
    if title:
        ax.set_title(title)
    return fig, ax


def draw_frame_3d(ax: Axes, T_ref_frame: ArrayLike, name: str, length: float = 0.1,
                  linewidth: float = 2.0) -> None:
    """Draw a 4x4 frame: x red, y green, z blue."""
    T = np.asarray(T_ref_frame, dtype=float)
    o = T[:3, 3]
    for axis, color in zip(range(3), AXIS_COLORS):
        d = T[:3, axis] * length
        ax.plot([o[0], o[0] + d[0]], [o[1], o[1] + d[1]], [o[2], o[2] + d[2]], color=color,
                lw=linewidth)
    ax.text(o[0], o[1], o[2], "  " + name, fontsize=8)


def draw_point_3d(ax: Axes, p_ref: ArrayLike, label: str = "", color: str = POINT_COLOR) -> None:
    x, y, z = np.asarray(p_ref, dtype=float)
    ax.scatter([x], [y], [z], color=color, s=40, edgecolor="k", depthshade=False)
    ax.text(x, y, z, f"  {label} ({x:.3f}, {y:.3f}, {z:.3f})", fontsize=8)


def draw_offset_3d(ax: Axes, T_ref_parent: ArrayLike, T_ref_child: ArrayLike,
                   color: str = "0.5") -> None:
    a = np.asarray(T_ref_parent, dtype=float)[:3, 3]
    b = np.asarray(T_ref_child, dtype=float)[:3, 3]
    ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], "--", color=color, lw=1.0)


def set_equal_3d(ax: Axes, points: ArrayLike, margin: float = 0.05) -> None:
    """Equal scale on all three axes around (N, 3) points (matplotlib does not do it for you)."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    center = (p.min(axis=0) + p.max(axis=0)) / 2
    half = max(p.max(axis=0) - p.min(axis=0)) / 2 + margin
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1, 1, 1))


def save(fig: Figure, name: str, out_dir: Path | str | None = None) -> Path:
    """Save ``fig`` as <out_dir>/<name>.png, close it, print and return the path."""
    folder = Path(out_dir) if out_dir is not None else DEFAULT_OUT
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"{name}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    print(f"wrote {path}")
    return path


# --------------------------------------------------------------------------------------------
# karmel scene data (numbers from labs/config/karmel.yaml)
# --------------------------------------------------------------------------------------------
T_MAP_BASE_2D = fm.se2(2.0, 1.0, math.radians(90))        # the robot in the map
T_BASE_LASER_2D = fm.se2(0.0, 0.0, 0.0)                   # LiDAR at (0, 0, 0.12): same x, y
T_BASE_CAMERA_2D = fm.se2(0.10, 0.0, 0.0)                 # camera at (0.10, 0, 0.10)
T_BASE_TOF_2D = fm.se2(0.125, 0.0, 0.0)                   # ToF at (0.125, 0, 0.05)
BOTTLE_OPTICAL = np.array([0.05, -0.02, 0.60])            # what the camera reports


def polar_to_xy(range_m: float, bearing_rad: float) -> np.ndarray:
    """A LiDAR return in the laser frame: bearing measured counter-clockwise from laser +x."""
    return np.array([range_m * math.cos(bearing_rad), range_m * math.sin(bearing_rad)])


# --------------------------------------------------------------------------------------------
# Scenes
# --------------------------------------------------------------------------------------------
def scene_laser_point(out_dir: Path | None = None, range_m: float = 1.5,
                      bearing_deg: float = 30.0,
                      T_base_laser: ArrayLike = T_BASE_LASER_2D) -> Path:
    """05.01: base_link, laser and one LiDAR return, drawn in base_link and in map."""
    p_laser = polar_to_xy(range_m, math.radians(bearing_deg))
    T_base_laser = np.asarray(T_base_laser, dtype=float)
    p_base = fm.transform_points(T_base_laser, p_laser)
    T_map_laser = T_MAP_BASE_2D @ T_base_laser
    p_map = fm.transform_points(T_map_laser, p_laser)

    fig, (ax_b, ax_m) = new_side_by_side(
        [f"LiDAR return r={range_m} m, bearing={bearing_deg}° — seen in base_link",
         "the same return — seen in map"], ["base_link", "map"])
    I = np.eye(3)
    draw_robot(ax_b, I)
    draw_frame(ax_b, I, "base_link", 0.3)
    draw_frame(ax_b, T_base_laser, "laser", 0.18, label_offset=(0.2, 0.12))
    draw_ray(ax_b, T_base_laser, p_base)
    draw_point(ax_b, p_base, "p_base_link")
    autoscale(ax_b, [[0, 0], p_base], margin=0.5)

    draw_frame(ax_m, I, "map", 0.5)
    draw_robot(ax_m, T_MAP_BASE_2D)
    draw_frame(ax_m, T_MAP_BASE_2D, "base_link", 0.3)
    draw_frame(ax_m, T_map_laser, "laser", 0.18, label_offset=(0.2, 0.12))
    draw_offset(ax_m, I, T_MAP_BASE_2D, "T_map_base_link")
    draw_ray(ax_m, T_map_laser, p_map)
    draw_point(ax_m, p_map, "p_map")
    autoscale(ax_m, [[0, 0], T_MAP_BASE_2D[:2, 2], p_map], margin=0.5)
    return save(fig, "laser-point", out_dir)


def scene_bottle_2d(out_dir: Path | None = None) -> Path:
    """05.01 / 05.03: the camera's bottle (top view) in camera_link, base_link and map."""
    p_camera = (fm.R_CAMERA_LINK_OPTICAL @ BOTTLE_OPTICAL)[:2]
    p_base = fm.transform_points(T_BASE_CAMERA_2D, p_camera)
    p_map = fm.transform_points(T_MAP_BASE_2D @ T_BASE_CAMERA_2D, p_camera)
    fig, axes = new_side_by_side(
        ["top view in camera_link", "top view in base_link", "top view in map"],
        ["camera_link", "base_link", "map"])
    I = np.eye(3)
    T_camera_base = fm.se2_inverse(T_BASE_CAMERA_2D)
    draw_robot(axes[0], T_camera_base)
    draw_frame(axes[0], T_camera_base, "base_link", 0.2)
    draw_frame(axes[0], I, "camera_link", 0.15, label_offset=(0.25, 0.12))
    draw_point(axes[0], p_camera, "bottle")
    autoscale(axes[0], [[-0.1, 0], p_camera], 0.3)

    draw_robot(axes[1], I)
    draw_frame(axes[1], I, "base_link", 0.2)
    draw_frame(axes[1], T_BASE_CAMERA_2D, "camera_link", 0.15, label_offset=(0.25, 0.12))
    draw_ray(axes[1], T_BASE_CAMERA_2D, p_base)
    draw_point(axes[1], p_base, "bottle")
    autoscale(axes[1], [[0, 0], p_base], 0.3)

    T_map_camera = T_MAP_BASE_2D @ T_BASE_CAMERA_2D
    draw_frame(axes[2], I, "map", 0.5)
    draw_robot(axes[2], T_MAP_BASE_2D)
    draw_frame(axes[2], T_MAP_BASE_2D, "base_link", 0.2)
    draw_offset(axes[2], I, T_MAP_BASE_2D, "T_map_base_link")
    draw_ray(axes[2], T_map_camera, p_map)
    draw_point(axes[2], p_map, "bottle")
    autoscale(axes[2], [[0, 0], T_MAP_BASE_2D[:2, 2], p_map], 0.4)
    return save(fig, "bottle-2d", out_dir)


def scene_poses(out_dir: Path | None = None) -> Path:
    """05.02: poses as arrows, the bearing to a goal (atan2) and the heading error."""
    robot = (0.5, -1.0, math.radians(120))
    goal = (3.0, 2.0)
    fig, ax = new_axes_2d("pose (x, y, θ), bearing = atan2(dy, dx), heading error", "map")
    draw_frame(ax, np.eye(3), "map", 0.6)
    T = fm.se2(*robot)
    draw_robot(ax, T)
    draw_frame(ax, T, "base_link", 0.5)
    dx, dy = goal[0] - robot[0], goal[1] - robot[1]
    bearing = math.atan2(dy, dx)
    error = fm.wrap_angle(bearing - robot[2])
    ax.plot([robot[0], goal[0]], [robot[1], goal[1]], ":", color="0.3")
    draw_point(ax, goal, "goal", color="tab:purple", marker="*")
    arc = np.linspace(robot[2], robot[2] + error, 40)
    ax.plot(robot[0] + 0.7 * np.cos(arc), robot[1] + 0.7 * np.sin(arc), color="tab:purple")
    ax.text(robot[0] + 0.8, robot[1] + 0.4,
            f"θ = {math.degrees(robot[2]):.1f}°\nbearing = {math.degrees(bearing):.2f}°\n"
            f"error = wrap(bearing − θ) = {math.degrees(error):.2f}°", fontsize=9)
    for x, y, deg in ((-1.0, 1.0, 180.0), (-1.0, -0.5, -90.0), (2.5, -0.8, 45.0)):
        Tp = fm.se2(x, y, math.radians(deg))
        draw_frame(ax, Tp, f"({x}, {y}, {deg:.0f}°)", 0.35)
    autoscale(ax, [[-1.2, -1.2], [3.2, 2.2]], 0.3)
    return save(fig, "poses", out_dir)


def scene_compose(out_dir: Path | None = None) -> Path:
    """05.03: T_map_camera = T_map_base_link @ T_base_link_camera, and the inverse T_base_link_map."""
    I = np.eye(3)
    T_map_camera = T_MAP_BASE_2D @ T_BASE_CAMERA_2D
    T_base_map = fm.se2_inverse(T_MAP_BASE_2D)
    fig, (ax1, ax2) = new_side_by_side(
        ["compose: T_map_camera = T_map_base_link @ T_base_link_camera",
         "invert: the map origin seen from base_link (T_base_link_map)"], ["map", "base_link"])
    draw_frame(ax1, I, "map", 0.5)
    draw_robot(ax1, T_MAP_BASE_2D)
    draw_frame(ax1, T_MAP_BASE_2D, "base_link", 0.3)
    draw_frame(ax1, T_map_camera, "camera_link", 0.2, label_offset=(0.35, 0.2))
    draw_offset(ax1, I, T_MAP_BASE_2D, "T_map_base_link", color="tab:purple")
    draw_offset(ax1, T_MAP_BASE_2D, T_map_camera, "", color="tab:cyan")
    draw_offset(ax1, I, T_map_camera, "", color="0.6")
    x, y, th = fm.se2_params(T_map_camera)
    ax1.text(0.1, 1.6, f"T_map_camera = ({x:.2f}, {y:.2f}, {math.degrees(th):.0f}°)", fontsize=10)
    autoscale(ax1, [[0, 0], [2.2, 1.9]], 0.3)

    draw_robot(ax2, I)
    draw_frame(ax2, I, "base_link", 0.3)
    draw_frame(ax2, T_base_map, "map", 0.5)
    draw_offset(ax2, I, T_base_map, "T_base_link_map", color="tab:purple")
    x, y, th = fm.se2_params(T_base_map)
    ax2.text(-2.3, 2.3, f"T_base_link_map = ({x:.2f}, {y:.2f}, {math.degrees(th):.0f}°)",
             fontsize=10)
    autoscale(ax2, [[0, 0], [x, y]], 0.8)
    return save(fig, "compose", out_dir)


def _karmel_world_frames() -> dict[str, np.ndarray]:
    s = fm.karmel_static_transforms()
    T_map_fp = fm.se3_from_se2(T_MAP_BASE_2D)
    T_map_base = T_map_fp @ s[("base_footprint", "base_link")]
    T_map_cam = T_map_base @ s[("base_link", "camera_link")]
    return {
        "map": np.eye(4),
        "base_footprint": T_map_fp,
        "base_link": T_map_base,
        "laser": T_map_base @ s[("base_link", "laser")],
        "camera_link": T_map_cam,
        "camera_optical_frame": T_map_cam @ s[("camera_link", "camera_optical_frame")],
    }


def scene_chain_3d(out_dir: Path | None = None) -> Path:
    """05.04: the chain map -> base_footprint -> base_link -> camera_link -> camera_optical_frame."""
    frames = _karmel_world_frames()
    p_map = fm.transform_points(frames["camera_optical_frame"], BOTTLE_OPTICAL)
    fig = plt.figure(figsize=(13, 6.5))
    ax_full = fig.add_subplot(1, 2, 1, projection="3d")
    ax_zoom = fig.add_subplot(1, 2, 2, projection="3d")
    new_axes_3d("the whole chain, in map", "map", ax_full)
    new_axes_3d("zoom on the robot", "map", ax_zoom)
    order = ["map", "base_footprint", "base_link", "camera_link", "camera_optical_frame"]
    for ax, length, names in ((ax_full, 0.4, ["map", "base_link"]),
                              (ax_zoom, 0.08, list(frames))):
        for a, b in zip(order, order[1:]):
            draw_offset_3d(ax, frames[a], frames[b])
        for name in names:
            draw_frame_3d(ax, frames[name], name, length)
        draw_offset_3d(ax, frames["camera_optical_frame"], fm.se3(t=p_map), color=POINT_COLOR)
        draw_point_3d(ax, p_map, "bottle")
    set_equal_3d(ax_full, [[0, 0, 0], p_map, frames["base_link"][:3, 3]], 0.2)
    set_equal_3d(ax_zoom, [frames["base_footprint"][:3, 3], p_map], 0.05)
    ax_zoom.view_init(elev=25, azim=-150)
    return save(fig, "chain-3d", out_dir)


def scene_rpy(out_dir: Path | None = None) -> Path:
    """05.05: roll, pitch, yaw applied about the fixed axes, and 'order matters'."""
    r, p, y = math.radians(30), math.radians(45), math.radians(90)
    steps = [
        ("identity", np.eye(3)),
        ("Rx(roll=30°)", fm.rot_x(r)),
        ("Ry(pitch=45°) @ Rx(roll)", fm.rot_y(p) @ fm.rot_x(r)),
        ("Rz(yaw=90°) @ Ry @ Rx  (ROS rpy)", fm.rpy_to_matrix(r, p, y)),
        ("order matters: Rz(90°) @ Ry(90°)", fm.rot_z(math.pi / 2) @ fm.rot_y(math.pi / 2)),
        ("order matters: Ry(90°) @ Rz(90°)", fm.rot_y(math.pi / 2) @ fm.rot_z(math.pi / 2)),
    ]
    fig = plt.figure(figsize=(13, 8.5))
    for i, (title, R) in enumerate(steps):
        ax = fig.add_subplot(2, 3, i + 1, projection="3d")
        new_axes_3d(title, "parent", ax)
        draw_frame_3d(ax, np.eye(4), "", 1.0, linewidth=0.8)
        draw_frame_3d(ax, fm.se3(R), "child", 0.8, linewidth=3.0)
        set_equal_3d(ax, [[-1, -1, -1], [1, 1, 1]], 0.0)
        ax.view_init(elev=22, azim=35)
    return save(fig, "rpy", out_dir)


def scene_optical(out_dir: Path | None = None) -> Path:
    """05.06: camera_link vs camera_optical_frame, same origin, different axes."""
    s = fm.karmel_static_transforms()
    T_link_opt = s[("camera_link", "camera_optical_frame")]
    p_link = fm.transform_points(T_link_opt, BOTTLE_OPTICAL)
    fig = plt.figure(figsize=(12, 6))
    for i, (title, T, name) in enumerate((
            ("camera_link: x forward, y left, z up", np.eye(4), "camera_link"),
            ("camera_optical_frame: z forward, x right, y down", T_link_opt,
             "camera_optical_frame"))):
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        new_axes_3d(title, "camera_link", ax)
        draw_frame_3d(ax, T, name, 0.2, linewidth=3)
        draw_point_3d(ax, p_link, "bottle")
        ax.plot([0, p_link[0]], [0, p_link[1]], [0, p_link[2]], ":", color=POINT_COLOR)
        set_equal_3d(ax, [[-0.1, -0.3, -0.3], [0.65, 0.3, 0.3]], 0.0)
        ax.view_init(elev=20, azim=-130)
    return save(fig, "optical", out_dir)


def simulate_map_odom(duration_s: float = 60.0, dt: float = 0.05, correction_period_s: float = 5.0,
                      distance_scale: float = 0.97, yaw_rate_scale: float = 1.04
                      ) -> dict[str, np.ndarray]:
    """True path, drifting odometry and a localizer that publishes map->odom every few seconds.

    T_map_odom = T_map_base_link(localizer) @ inverse(T_odom_base_link(odometry))
    """
    n = int(duration_s / dt)
    t = np.arange(n) * dt
    v = 0.25
    omega = np.where((t % 15.0) < 11.0, 0.0, (math.pi / 2) / 4.0)   # straight 11 s, turn 4 s
    true_T = fm.se2(0.0, 0.0, 0.0)
    odom_T = fm.se2(0.0, 0.0, 0.0)
    T_map_odom = np.eye(3)
    rows = []
    for k in range(n):
        true_T = true_T @ fm.se2(v * dt, 0.0, omega[k] * dt)
        odom_T = odom_T @ fm.se2(distance_scale * v * dt, 0.0, yaw_rate_scale * omega[k] * dt)
        if k > 0 and k % int(correction_period_s / dt) == 0:
            T_map_odom = true_T @ fm.se2_inverse(odom_T)            # localizer "fixes" the drift
        est = T_map_odom @ odom_T
        rows.append((*fm.se2_params(true_T), *fm.se2_params(odom_T), *fm.se2_params(est),
                     *fm.se2_params(T_map_odom)))
    data = np.array(rows)
    return {"t": t, "true": data[:, 0:3], "odom_base": data[:, 3:6], "map_base": data[:, 6:9],
            "map_odom": data[:, 9:12]}


def scene_map_odom(out_dir: Path | None = None) -> Path:
    """05.06: why map->odom jumps while odom->base_link stays continuous."""
    sim = simulate_map_odom()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))
    new_axes_2d("paths", "map (odom path drawn as if odom = map)", ax=ax1)
    ax1.plot(*sim["true"][:, :2].T, "k-", lw=2, label="true path")
    ax1.plot(*sim["odom_base"][:, :2].T, "--", color="tab:blue", label="odom -> base_link (smooth, drifts)")
    ax1.plot(*sim["map_base"][:, :2].T, "-", color="tab:orange", lw=1,
             label="map -> odom -> base_link (jumps back)")
    ax1.legend(loc="upper left", fontsize=8)
    t = sim["t"]
    ax2.plot(t, sim["odom_base"][:, 0], color="tab:blue", label="odom->base_link x [m]")
    ax2.plot(t, sim["map_odom"][:, 0], color="tab:red", drawstyle="steps-post",
             label="map->odom x [m]")
    ax2.plot(t, sim["map_odom"][:, 1], color="tab:green", drawstyle="steps-post",
             label="map->odom y [m]")
    ax2.plot(t, np.degrees(sim["map_odom"][:, 2]) / 10, color="tab:purple",
             drawstyle="steps-post", label="map->odom yaw [deg/10]")
    ax2.set_xlabel("time [s]")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)
    ax2.set_title("map->odom changes in steps; odom->base_link is continuous")
    return save(fig, "map-odom", out_dir)


SCENES: dict[str, tuple[str, Callable[..., Path]]] = {
    "laser-point": ("05.01", scene_laser_point),
    "bottle-2d": ("05.01, 05.03", scene_bottle_2d),
    "poses": ("05.02", scene_poses),
    "compose": ("05.03", scene_compose),
    "chain-3d": ("05.04", scene_chain_3d),
    "rpy": ("05.05", scene_rpy),
    "optical": ("05.06", scene_optical),
    "map-odom": ("05.06", scene_map_odom),
}


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("scene", choices=["list", "all", *SCENES])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="folder for PNGs")
    args = parser.parse_args(argv)
    if args.scene == "list":
        for name, (lessons, func) in SCENES.items():
            print(f"{name:12s} [{lessons}] {func.__doc__.split(':', 1)[1].strip()}")
        return
    names = list(SCENES) if args.scene == "all" else [args.scene]
    for name in names:
        SCENES[name][1](args.out)


if __name__ == "__main__":
    main()
