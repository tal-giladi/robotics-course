"""Host-runnable tests for the lessons 06.06-06.10 scripts (no ROS, no Gazebo).

    py -m pytest 06-simulation/code

The Gazebo-side behaviour these scripts measure is exercised in the course Docker image by
``labs/ros2_ws/tools/smoke_test.sh``; what is checked here is the analysis that turns those
measurements into the numbers the lessons quote.
"""

from __future__ import annotations

import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import sysid_fit
from gazebo import world_check

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
WORLDS = ROOT / "labs" / "ros2_ws" / "src" / "karmel_gazebo" / "worlds"


# ------------------------------------------------------------------ 06.07 / 06.08 the world checker
def test_apartment_is_a_good_slam_world():
    report = world_check.check(WORLDS / "apartment.sdf")
    assert report.ok, report.errors
    assert not report.warnings, report.warnings


def test_corridor_is_flagged_as_translation_ambiguous():
    """The whole point of the metric: a featureless corridor must score badly, the apartment well."""
    corridor = world_check.check(HERE / "gazebo" / "corridor.sdf", grid=0.4)
    assert corridor.ok, corridor.errors
    assert any("translation-ambiguous" in w for w in corridor.warnings)


def test_ambiguity_metric_separates_corridor_from_apartment():
    def fraction(path: Path, grid: float) -> float:
        root = ET.parse(path).getroot()
        report = world_check.Report(str(path))
        rects = world_check.collect_rects(root.find("world"), report)
        points = world_check.free_points(rects, grid)
        return world_check.ambiguity(rects, points)[0]

    assert fraction(HERE / "gazebo" / "corridor.sdf", 0.4) > 0.5
    assert fraction(WORLDS / "apartment.sdf", 0.5) < 0.05


def test_slippery_floor_is_an_error_not_a_warning():
    """06.05's deliberately broken world must fail the check that would have caught it."""
    report = world_check.check(HERE / "gazebo" / "slippery_room.sdf")
    assert not report.ok
    assert any("friction" in e for e in report.errors)


def test_missing_sensor_systems_are_errors():
    report = world_check.check(HERE / "gazebo" / "first_world.sdf")
    assert any("Sensors" in e for e in report.errors)
    assert any("Imu" in e for e in report.errors)


@pytest.mark.parametrize("name", ["corridor.sdf", "materials_test.sdf"])
def test_new_worlds_load_the_systems_karmel_needs(name):
    world = ET.parse(HERE / "gazebo" / name).getroot().find("world")
    plugins = {p.get("name") for p in world.findall("plugin")}
    assert set(world_check.REQUIRED_SYSTEMS) <= plugins
    assert float(world.find("physics/max_step_size").text) <= 0.001


def test_materials_walls_are_all_the_same_distance_from_the_origin():
    """The experiment only proves anything if the four walls really are equidistant."""
    report = world_check.Report("materials")
    root = ET.parse(HERE / "gazebo" / "materials_test.sdf").getroot()
    rects = world_check.collect_rects(root.find("world"), report)
    for angle in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        assert world_check.raycast(rects, 0.0, 0.0, angle) == pytest.approx(1.0, abs=1e-6)


def test_raycast_matches_hand_geometry():
    rect = world_check.Rect("wall", x=2.0, y=0.0, hx=0.05, hy=1.0, yaw=0.0)
    assert rect.ray_distance(0.0, 0.0, 0.0) == pytest.approx(1.95)
    assert rect.ray_distance(0.0, 0.0, math.pi) == math.inf
    # 45 degrees: the ray leaves the wall's y half-extent before reaching x = 1.95
    assert rect.ray_distance(0.0, 0.0, math.pi / 4) == math.inf


# ------------------------------------------------------------------ 06.10 system identification
def _write_step(path: Path, tau: float, dead: float = 0.0, gain: float = 1.0,
                target: float = 6.6667, dt: float = 0.02, hold: float = 3.0) -> Path:
    """A synthetic first-order step, so the fit can be checked against a known answer."""
    rows, velocity, t = [], 0.0, 0.0
    while t <= hold + 1.0:
        command = target if 0.5 <= t < 0.5 + hold else 0.0
        # The integration uses the command held over the PREVIOUS interval, the same zero-order
        # hold convention as sysid_fit.simulate — otherwise the fit absorbs half a sample into tau.
        previous = t - dt
        delayed = target if 0.5 + dead <= previous < 0.5 + hold + dead else 0.0
        velocity += (gain * delayed - velocity) * (1.0 - math.exp(-dt / tau))
        rows.append((round(t, 4), command, round(velocity, 5), 0.0))
        t += dt
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "cmd_rad_s", "meas_rad_s", "pos_rad"])
        writer.writerows(rows)
    return path


def test_sysid_recovers_a_known_time_constant(tmp_path):
    result = sysid_fit.fit(_write_step(tmp_path / "s.csv", tau=0.08))
    assert result.tau_s == pytest.approx(0.08, abs=0.02)
    assert result.dead_time_s == pytest.approx(0.0, abs=0.02)
    assert result.gain == pytest.approx(1.0, abs=0.01)


def test_sysid_recovers_dead_time_and_a_low_gain(tmp_path):
    result = sysid_fit.fit(_write_step(tmp_path / "s.csv", tau=0.08, dead=0.06, gain=0.9))
    assert result.dead_time_s == pytest.approx(0.06, abs=0.02)
    assert result.gain == pytest.approx(0.9, abs=0.02)


def test_a_sluggish_plant_has_a_later_t99(tmp_path):
    fast = sysid_fit.fit(_write_step(tmp_path / "fast.csv", tau=0.05))
    slow = sysid_fit.fit(_write_step(tmp_path / "slow.csv", tau=0.30))
    assert slow.t99 > fast.t99 + 0.5
    assert slow.tau_s > 4 * fast.tau_s
