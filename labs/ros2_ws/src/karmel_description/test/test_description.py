"""Tests for karmel_description: the xacro renders for sim and real, and the config copy is in sync."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
XACRO = PKG / 'urdf' / 'karmel.urdf.xacro'
COPY = PKG / 'config' / 'robot.yaml'
# labs/ros2_ws/src/karmel_description/test -> labs/config/karmel.yaml
SOURCE = PKG.parent.parent.parent / 'config' / 'karmel.yaml'


def render(**mappings):
    xacro = pytest.importorskip('xacro')
    mappings.setdefault('robot_config', str(COPY))
    doc = xacro.process_file(str(XACRO), mappings={k: str(v) for k, v in mappings.items()})
    return ET.fromstring(doc.toxml())


def load(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.mark.skipif(not SOURCE.is_file(), reason='labs/config/karmel.yaml not present (installed copy only)')
def test_config_copy_matches_labs_config():
    assert load(COPY) == load(SOURCE), 'run labs/ros2_ws/check_config_sync.py --fix'


def test_real_robot_urdf():
    robot = render(use_sim='false')
    links = {link.get('name') for link in robot.findall('link')}
    for name in ['base_footprint', 'base_link', 'left_wheel', 'right_wheel', 'caster_link', 'imu_link',
                 'laser', 'camera_link', 'camera_optical_frame', 'range_front_link']:
        assert name in links
    plugin = robot.find('ros2_control/hardware/plugin').text.strip()
    assert plugin == 'karmel_hardware/KarmelSystem'
    params = {p.get('name'): p.text for p in robot.findall('ros2_control/hardware/param')}
    cfg = load(COPY)
    d = cfg['drive']
    assert int(params['ticks_per_rev']) == int(d['encoder_cpr_motor'] * d['gear_ratio'] * d['quadrature_multiplier'])
    assert robot.find('gazebo') is None


def test_sim_urdf_has_gazebo_plugins():
    robot = render(use_sim='true', controllers_file='/tmp/controllers.yaml')
    assert robot.find('ros2_control/hardware/plugin').text.strip() == 'gz_ros2_control/GazeboSimSystem'
    sensor_types = {s.get('type') for s in robot.iter('sensor')}
    assert {'gpu_lidar', 'imu', 'camera'} <= sensor_types
    plugins = [p.get('filename') for p in robot.iter('plugin') if p.get('filename')]
    assert 'gz_ros2_control-system' in plugins


def test_dimensions_follow_config():
    robot = render(use_sim='false')
    cfg = load(COPY)
    joints = {j.get('name'): j for j in robot.findall('joint')}
    left_y = float(joints['left_wheel_joint'].find('origin').get('xyz').split()[1])
    assert math.isclose(2 * left_y, cfg['drive']['wheel_separation_m'])
    base_z = float(joints['base_joint'].find('origin').get('xyz').split()[2])
    assert math.isclose(base_z, cfg['drive']['wheel_radius_m'])
    lidar_xyz = [float(v) for v in joints['laser_joint'].find('origin').get('xyz').split()]
    assert lidar_xyz == pytest.approx([cfg['sensors']['lidar'][k] for k in ('x_m', 'y_m', 'z_m')])


def test_every_massive_link_has_positive_inertia():
    robot = render(use_sim='false')
    total = 0.0
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        m = float(inertial.find('mass').get('value'))
        total += m
        inertia = inertial.find('inertia')
        ixx, iyy, izz = (float(inertia.get(k)) for k in ('ixx', 'iyy', 'izz'))
        assert m > 0 and ixx > 0 and iyy > 0 and izz > 0, link.get('name')
        # triangle inequality holds for any physical inertia tensor
        assert ixx + iyy >= izz and iyy + izz >= ixx and ixx + izz >= iyy, link.get('name')
    assert math.isclose(total, load(COPY)['robot']['mass_kg'], rel_tol=1e-6)


def test_camera_optical_frame_convention():
    robot = render(use_sim='false')
    joint = next(j for j in robot.findall('joint') if j.get('name') == 'camera_optical_joint')
    rpy = [float(v) for v in joint.find('origin').get('rpy').split()]
    assert rpy == pytest.approx([-math.pi / 2, 0.0, -math.pi / 2])
