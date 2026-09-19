"""Matplotlib helpers for the simulator: worlds, robots, trajectories, scans, uncertainty, maps.

Every ``draw_*`` function takes an ``Axes`` first and returns the artist(s) it created, so
lessons can compose plots freely. Nothing here picks a backend: in scripts and tests without a
display, call ``matplotlib.use("Agg")`` before importing pyplot.

>>> fig, ax = new_axes(world)
>>> draw_trajectory(ax, true_poses, label="truth")
>>> draw_trajectory(ax, odom_poses, label="odometry", linestyle="--")
>>> ax.legend(); fig.savefig("run.png")
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.artist import Artist
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Ellipse
from numpy.typing import ArrayLike, NDArray

from robotlab.geometry import SE2
from robotlab.sim.occupancy import FREE, OCCUPIED, OccupancyGrid
from robotlab.sim.sensors import LaserScan
from robotlab.sim.world import World

PoseLike = SE2 | Sequence[float] | NDArray[np.floating]


def poses_to_array(poses: Sequence[PoseLike] | NDArray[np.floating]) -> NDArray[np.floating]:
    """A list of ``SE2`` / ``(x, y, theta)`` -> ``(N, 3)`` array."""
    if isinstance(poses, np.ndarray):
        return poses.reshape(-1, 3).astype(float)
    return np.array([SE2.from_tuple(p).as_tuple() for p in poses], dtype=float).reshape(-1, 3)


def new_axes(world: World | None = None, *, figsize: tuple[float, float] = (7.0, 6.0), title: str | None = None) -> tuple[Figure, Axes]:
    """A figure with equal-aspect axes in meters, with the world drawn if given."""
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if title:
        ax.set_title(title)
    if world is not None:
        draw_world(ax, world)
    return fig, ax


def draw_world(ax: Axes, world: World, *, color: str = "0.2", show_landmarks: bool = True, label_landmarks: bool = False) -> list[Artist]:
    """Walls as lines, round obstacles as filled circles, landmarks as stars."""
    artists: list[Artist] = []
    if len(world.segments):
        lines = LineCollection(world.segments.reshape(-1, 2, 2), colors=color, linewidths=2.0)
        artists.append(ax.add_collection(lines))
    for cx, cy, r in world.circles:
        artists.append(ax.add_patch(Circle((cx, cy), r, facecolor="0.75", edgecolor=color)))
    if show_landmarks and len(world.landmarks):
        (stars,) = ax.plot(world.landmarks[:, 0], world.landmarks[:, 1], "*", color="goldenrod", markersize=11, label="landmarks")
        artists.append(stars)
        if label_landmarks:
            for (x, y), lid in zip(world.landmarks, world.landmark_ids):
                artists.append(ax.annotate(str(lid), (x, y), textcoords="offset points", xytext=(4, 4), fontsize=8))
    x_min, x_max, y_min, y_max = world.bounds
    if x_max > x_min and y_max > y_min:
        pad = 0.1 * max(x_max - x_min, y_max - y_min)
        ax.set_xlim(x_min - pad, x_max + pad)
        ax.set_ylim(y_min - pad, y_max + pad)
    return artists


def draw_robot(ax: Axes, pose: PoseLike, *, radius: float = 0.16, color: str = "tab:blue", label: str | None = None, alpha: float = 0.9) -> list[Artist]:
    """A circle with a heading line."""
    p = SE2.from_tuple(pose)
    body = ax.add_patch(Circle((p.x, p.y), radius, fill=False, edgecolor=color, linewidth=2.0, alpha=alpha, label=label))
    tip = p.apply([radius, 0.0])
    (heading,) = ax.plot([p.x, tip[0]], [p.y, tip[1]], color=color, linewidth=2.0, alpha=alpha)
    return [body, heading]


def draw_trajectory(ax: Axes, poses: Sequence[PoseLike] | NDArray[np.floating], **plot_kwargs: object) -> Artist:
    """A path through the ``(x, y)`` of each pose. Keyword args go to ``ax.plot``."""
    xy = poses_to_array(poses)
    (line,) = ax.plot(xy[:, 0], xy[:, 1], **plot_kwargs)
    return line


def draw_scan(ax: Axes, sensor_pose: PoseLike, scan: LaserScan, *, color: str = "tab:red", size: float = 4.0, rays: bool = False) -> list[Artist]:
    """Valid scan returns as world-frame points; ``sensor_pose`` is the LiDAR's world pose."""
    pose = SE2.from_tuple(sensor_pose)
    pts = pose.apply(scan.points())
    artists: list[Artist] = []
    if rays and len(pts):
        segs = np.stack([np.broadcast_to(pose.translation, pts.shape), pts], axis=1)
        artists.append(ax.add_collection(LineCollection(segs, colors=color, linewidths=0.3, alpha=0.3)))
    artists.append(ax.scatter(pts[:, 0], pts[:, 1], s=size, color=color, zorder=3))
    return artists


def covariance_ellipse(mean_xy: ArrayLike, cov: ArrayLike, n_sigma: float = 2.0) -> tuple[tuple[float, float], float, float, float]:
    """``(center, width, height, angle_deg)`` of the ``n_sigma`` ellipse of a 2x2 covariance."""
    eigvals, eigvecs = np.linalg.eigh(np.asarray(cov, dtype=float)[:2, :2])
    eigvals = np.maximum(eigvals, 0.0)
    angle = math.degrees(math.atan2(eigvecs[1, 1], eigvecs[0, 1]))  # direction of the major axis
    width, height = 2.0 * n_sigma * np.sqrt(eigvals[::-1])
    cx, cy = np.asarray(mean_xy, dtype=float)[:2]
    return (float(cx), float(cy)), float(width), float(height), angle


def draw_covariance_ellipse(ax: Axes, mean_xy: ArrayLike, cov: ArrayLike, *, n_sigma: float = 2.0, color: str = "tab:purple", **patch_kwargs: object) -> Artist:
    """Draw the ``n_sigma`` uncertainty ellipse of the x-y block of ``cov``."""
    center, width, height, angle = covariance_ellipse(mean_xy, cov, n_sigma)
    ellipse = Ellipse(center, width, height, angle=angle, fill=False, edgecolor=color, **patch_kwargs)
    return ax.add_patch(ellipse)


def draw_occupancy_grid(ax: Axes, grid: OccupancyGrid, *, alpha: float = 1.0) -> Artist:
    """Occupied black, free white, unknown gray; intermediate probabilities in between."""
    shade = np.full(grid.data.shape, 0.5)
    known = grid.data >= 0
    shade[known] = 1.0 - grid.data[known] / 100.0
    shade[grid.data == FREE] = 1.0
    shade[grid.data == OCCUPIED] = 0.0
    return ax.imshow(shade, cmap="gray", vmin=0.0, vmax=1.0, origin="lower", extent=grid.extent, alpha=alpha, interpolation="nearest")


def animate(
    world: World,
    poses: Sequence[PoseLike] | NDArray[np.floating],
    *,
    scans: Sequence[LaserScan | None] | None = None,
    estimates: Sequence[PoseLike] | NDArray[np.floating] | None = None,
    interval_ms: int = 40,
    robot_radius: float = 0.16,
    on_frame: Callable[[Axes, int], None] | None = None,
) -> FuncAnimation:
    """Animate a recorded run: the robot, its trail, optional estimate trail and scans.

    Save with ``anim.save("run.gif", writer="pillow")`` or show with ``plt.show()``.
    Keep a reference to the returned object until the animation is shown or saved.
    """
    truth = poses_to_array(poses)
    est = poses_to_array(estimates) if estimates is not None else None
    fig, ax = new_axes(world)
    (trail,) = ax.plot([], [], color="tab:blue", label="truth")
    (est_trail,) = ax.plot([], [], "--", color="tab:orange", label="estimate")
    body = ax.add_patch(Circle((0.0, 0.0), robot_radius, fill=False, edgecolor="tab:blue", linewidth=2.0))
    (heading,) = ax.plot([], [], color="tab:blue", linewidth=2.0)
    scan_dots = ax.scatter([], [], s=4.0, color="tab:red", zorder=3)
    ax.legend(loc="upper right")

    def update(i: int) -> list[Artist]:
        pose = SE2.from_tuple(truth[i])
        trail.set_data(truth[: i + 1, 0], truth[: i + 1, 1])
        if est is not None:
            est_trail.set_data(est[: i + 1, 0], est[: i + 1, 1])
        body.set_center((pose.x, pose.y))
        tip = pose.apply([robot_radius, 0.0])
        heading.set_data([pose.x, tip[0]], [pose.y, tip[1]])
        if scans is not None and i < len(scans) and scans[i] is not None:
            scan_dots.set_offsets(pose.apply(scans[i].points()))  # type: ignore[union-attr]
        if on_frame is not None:
            on_frame(ax, i)
        return [trail, est_trail, body, heading, scan_dots]

    return FuncAnimation(fig, update, frames=len(truth), interval=interval_ms, blit=False)
