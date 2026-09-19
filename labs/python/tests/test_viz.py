from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pytest

from robotlab.geometry import SE2
from robotlab.sim import DiffDriveSim, World, viz


def test_draw_everything_and_save(tmp_path: Path, ideal):
    world = World.apartment()
    sim = DiffDriveSim(world, *ideal, pose=(1.0, 1.3, 0.3))
    fig, ax = viz.new_axes(world, title="test")
    viz.draw_occupancy_grid(ax, world.to_occupancy_grid(0.1), alpha=0.3)
    viz.draw_world(ax, world, label_landmarks=True)
    viz.draw_robot(ax, sim.pose, label="robot")
    viz.draw_trajectory(ax, [SE2(1, 1.3, 0), (1.5, 1.4, 0.2), np.array([2.0, 1.6, 0.4])], label="path")
    viz.draw_scan(ax, sim.lidar_pose, sim.lidar_scan(), rays=True)
    viz.draw_covariance_ellipse(ax, [1.0, 1.3], [[0.04, 0.01], [0.01, 0.02]])
    out = tmp_path / "plot.png"
    fig.savefig(out)
    plt.close(fig)
    assert out.stat().st_size > 1000


def test_covariance_ellipse_axes():
    center, width, height, angle = viz.covariance_ellipse([1.0, 2.0], np.diag([0.01, 0.04]), n_sigma=2.0)
    assert center == (1.0, 2.0)
    assert width == pytest.approx(0.8) and height == pytest.approx(0.4)  # 2 sigma each side
    assert abs(abs(angle) - 90.0) < 1e-6  # major axis along y


def test_animate_headless(tmp_path: Path, ideal):
    sim = DiffDriveSim(World.rectangle_room(3, 3), *ideal, pose=(1.5, 1.5, 0.0))
    poses, scans = [], []
    for _ in range(10):
        sim.set_duty(0.5, 0.6)
        sim.step(0.1)
        poses.append(sim.pose)
        scans.append(sim.lidar_scan())
    anim = viz.animate(sim.world, poses, scans=scans, estimates=poses)
    out = tmp_path / "run.gif"
    anim.save(out, writer="pillow", fps=10)
    plt.close("all")
    assert out.stat().st_size > 1000
