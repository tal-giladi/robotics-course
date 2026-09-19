"""The module-06 scripts run headless and produce the numbers the lessons quote.

    py -m pytest 06-simulation/code

Gazebo is not exercised here (it needs Linux + ros_gz); the SDF and bridge files are only
checked for well-formedness. The full Gazebo path is covered by
``labs/ros2_ws/tools/smoke_test.sh`` inside the course Docker image.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

import inertia_check
import mini_sim_tour
import timestep_experiment

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent


# ------------------------------------------------------------------ 06.02 the mini simulator
def test_timestep_euler_blows_up_when_dt_exceeds_two_tau():
    """Forward Euler on a first-order lag is unstable for dt > 2*tau; the exact form never is."""
    tau, target = 0.08, 9.27
    stable = timestep_experiment.motor_speed(0.02, 1.0, target, tau, "euler")
    unstable = timestep_experiment.motor_speed(0.2, 1.0, target, tau, "euler")
    exact = timestep_experiment.motor_speed(0.2, 1.0, target, tau, "exact")
    assert stable == pytest.approx(target, abs=0.05)
    assert unstable > 2 * target            # dt/tau = 2.5: oscillates and diverges
    assert exact == pytest.approx(target, abs=0.05)


def test_course_simulator_converges_faster_than_the_naive_one():
    from robotlab.sim import DiffDriveParams

    params = DiffDriveParams.ideal()
    reference = timestep_experiment.run_script(
        lambda: timestep_experiment.DiffDriveSim(
            timestep_experiment.World(), params, timestep_experiment.SensorParams.ideal(), seed=0), 0.0001)
    errors = timestep_experiment.robot_errors(0.02, reference, params)
    assert errors["course"][0] < 0.001      # sub-millimetre at 50 Hz
    assert errors["naive"][0] > 10 * errors["course"][0]


def test_mini_sim_tour_parts_run(capsys):
    for part in sorted(mini_sim_tour.PARTS):
        mini_sim_tour.PARTS[part]()
    assert "stalled wheel" in capsys.readouterr().out


def test_square_drift_ordering():
    """A wrong wheel radius hurts a square drive more than a mismatched motor gain."""
    from dataclasses import replace

    from robotlab.sim import DiffDriveParams

    ideal = DiffDriveParams.ideal()
    end = {label: mini_sim_tour.drive_square(params, seed=7).pose
           for label, params in (
               ("ideal", ideal),
               ("gain", replace(ideal, motor_gain_right=0.94)),
               ("radius", replace(ideal, wheel_radius_scale_right=1.008)))}
    error = {k: math.hypot(p.x, p.y) for k, p in end.items()}
    # The firmware's velocity loop hides a motor-gain mismatch; it cannot hide a geometry error.
    assert error["gain"] == pytest.approx(error["ideal"], abs=0.005)
    assert error["radius"] > 5 * error["ideal"]


# ------------------------------------------------------------------ 06.05 inertia
def test_inertia_check_passes_on_a_valid_body(tmp_path):
    urdf = tmp_path / "ok.urdf"
    urdf.write_text(_box_urdf(ixx=8.33e-4, iyy=8.33e-4, izz=8.33e-4), encoding="utf-8")
    reports, total = inertia_check.report_urdf(str(urdf))
    assert total == pytest.approx(0.5)
    assert reports[0].problems == []
    assert "ok" in inertia_check.plausibility(reports[0])


def test_inertia_check_catches_the_triangle_inequality(tmp_path):
    urdf = tmp_path / "bad.urdf"
    urdf.write_text(_box_urdf(ixx=1e-6, iyy=1e-6, izz=1e-3), encoding="utf-8")
    reports, _ = inertia_check.report_urdf(str(urdf))
    assert any("triangle inequality" in p for p in reports[0].problems)


def test_inertia_check_flags_placeholder_inertia(tmp_path):
    """The copy-pasted 'ixx=0.001' on a big light link: legal, but not this body."""
    urdf = tmp_path / "placeholder.urdf"
    urdf.write_text(_box_urdf(ixx=1e-3, iyy=1e-3, izz=1e-3, size="0.25 0.18 0.07", mass=1.23),
                    encoding="utf-8")
    reports, _ = inertia_check.report_urdf(str(urdf))
    assert reports[0].problems == []                     # physically possible...
    assert "SUSPICIOUS" in inertia_check.plausibility(reports[0])   # ...but not for this shape


def _box_urdf(ixx: float, iyy: float, izz: float, size: str = "0.1 0.1 0.1", mass: float = 0.5) -> str:
    return f"""<?xml version="1.0"?>
<robot name="t">
  <link name="box">
    <collision><geometry><box size="{size}"/></geometry></collision>
    <inertial>
      <mass value="{mass}"/>
      <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/>
    </inertial>
  </link>
</robot>
"""


# ------------------------------------------------------------------ 06.03 / 06.04 static checks
@pytest.mark.parametrize("name", ["first_world.sdf", "slippery_room.sdf"])
def test_worlds_are_well_formed_and_load_the_required_systems(name):
    world = ET.parse(HERE / "gazebo" / name).getroot().find("world")
    plugins = {p.get("name") for p in world.findall("plugin")}
    assert "gz::sim::systems::Physics" in plugins
    assert "gz::sim::systems::UserCommands" in plugins        # needed to spawn anything
    assert "gz::sim::systems::SceneBroadcaster" in plugins    # needed by the GUI
    assert float(world.find("physics/max_step_size").text) <= 0.001


def test_bridge_yaml_matches_the_world():
    entries = yaml.safe_load((HERE / "ros2" / "first_world_bridge.yaml").read_text(encoding="utf-8"))
    topics = {e.get("ros_topic_name") for e in entries}
    assert "/clock" in topics, "every simulation needs /clock bridged for use_sim_time"
    world_name = ET.parse(HERE / "gazebo" / "first_world.sdf").getroot().find("world").get("name")
    services = [e["service_name"] for e in entries if "service_name" in e]
    assert services == [f"/world/{world_name}/control"]
