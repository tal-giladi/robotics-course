"""Smoke tests for frames_lab.py: every scene renders headless, the map->odom story holds."""

from __future__ import annotations

import math

import numpy as np
import pytest

import frames_lab
import frames_math as fm


@pytest.mark.parametrize("name", list(frames_lab.SCENES))
def test_scene_writes_png(name: str, tmp_path) -> None:
    path = frames_lab.SCENES[name][1](tmp_path)
    assert path.exists() and path.stat().st_size > 10_000
    assert path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_cli_list_and_one_scene(tmp_path, capsys) -> None:
    frames_lab.main(["list"])
    assert "laser-point" in capsys.readouterr().out
    frames_lab.main(["poses", "--out", str(tmp_path)])
    assert (tmp_path / "poses.png").exists()


def test_laser_point_numbers() -> None:
    p_laser = frames_lab.polar_to_xy(1.5, math.radians(30))
    assert p_laser == pytest.approx([1.299038, 0.75])
    p_map = fm.transform_points(frames_lab.T_MAP_BASE_2D @ frames_lab.T_BASE_LASER_2D, p_laser)
    assert p_map == pytest.approx([1.25, 2.299038])


def test_map_odom_jumps_but_odom_is_continuous() -> None:
    sim = frames_lab.simulate_map_odom()
    odom_steps = np.linalg.norm(np.diff(sim["odom_base"][:, :2], axis=0), axis=1)
    assert odom_steps.max() < 0.02                      # 0.25 m/s * 0.05 s = 0.0125 m per step
    map_odom_changes = np.count_nonzero(np.linalg.norm(np.diff(sim["map_odom"], axis=0), axis=1) > 1e-9)
    assert 5 <= map_odom_changes <= 12                  # one change per correction, nothing else
    # right after each correction the estimate equals the truth
    err = np.linalg.norm(sim["map_base"][:, :2] - sim["true"][:, :2], axis=1)
    k = int(5.0 / 0.05)
    assert err[k] == pytest.approx(0.0, abs=1e-9)
