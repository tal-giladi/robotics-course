from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from robotlab.sim import DiffDriveSim, World


def test_lidar_square_room_from_center(cfg, ideal):
    sim = DiffDriveSim(World.rectangle_room(4.0, 4.0), *ideal, pose=(2.0, 2.0, 0.0))
    scan = sim.lidar_scan()
    n = cfg.sensors.lidar.samples
    assert len(scan.ranges) == n
    assert scan.angle_min == pytest.approx(-math.pi)
    assert scan.angle_increment == pytest.approx(2 * math.pi / n)
    # Distance to the walls of a square from its center: 2 / max(|cos a|, |sin a|).
    a = scan.angles
    expected = 2.0 / np.maximum(np.abs(np.cos(a)), np.abs(np.sin(a)))
    assert np.allclose(scan.ranges, expected)
    assert scan.points().shape == (n, 2)


def test_lidar_is_in_sensor_frame(ideal):
    room = World.rectangle_room(4.0, 2.0)
    scan = DiffDriveSim(room, *ideal, pose=(1.0, 1.0, math.pi / 2)).lidar_scan()
    forward = np.argmin(np.abs(scan.angles))  # beam along the robot's +x = world +y
    assert scan.ranges[forward] == pytest.approx(1.0)


def test_lidar_special_values(ideal):
    params = dataclasses.replace(ideal[1], lidar_max_range_m=3.0, lidar_dropout_prob=0.3)
    sim = DiffDriveSim(World.rectangle_room(10.0, 0.2), ideal[0], params, pose=(1.0, 0.1, 0.0), seed=1)
    scan = sim.lidar_scan()
    assert np.isnan(scan.ranges).any()  # dropouts
    assert np.isposinf(scan.ranges).any()  # beyond max range
    assert np.isneginf(scan.ranges).any()  # side walls closer than range_min
    assert not scan.valid[np.isnan(scan.ranges) | np.isinf(scan.ranges)].any()


def test_lidar_noise_statistics(realistic):
    sim = DiffDriveSim(World.rectangle_room(4.0, 4.0), *realistic, pose=(2.0, 2.0, 0.0), seed=2)
    errors = []
    for _ in range(20):
        scan = sim.lidar_scan()
        a = scan.angles
        expected = 2.0 / np.maximum(np.abs(np.cos(a)), np.abs(np.sin(a)))
        errors.append((scan.ranges - expected)[scan.valid])
    errors = np.concatenate(errors)
    assert np.std(errors) == pytest.approx(sim.sensor_params.lidar_noise_std_m, rel=0.1)
    assert abs(np.mean(errors)) < 0.002


def test_front_range(cfg, ideal):
    room = World.rectangle_room(3.0, 3.0)
    sim = DiffDriveSim(room, *ideal, pose=(1.0, 1.5, 0.0))
    assert sim.front_range() == pytest.approx(2.0 - cfg.sensors.range_front.x_m)
    far = DiffDriveSim(World.rectangle_room(20.0, 20.0), *ideal, pose=(10.0, 10.0, 0.0))
    assert far.front_range() is None


def test_front_range_cone_sees_off_axis_obstacle(ideal):
    world = World.from_segments(np.zeros((0, 4)), circles=[[1.5, 0.25, 0.1]])
    sim = DiffDriveSim(world, *ideal, pose=(0.0, 0.0, 0.0))
    assert sim.front_range() is not None  # a single center ray would miss it


def test_landmarks_fov_range_and_occlusion(ideal):
    landmarks = [[3.0, 0.0], [0.0, 3.0], [3.0, 0.5], [9.0, 0.0]]
    world = World.from_segments([[1.5, 0.1, 1.5, 0.8]], landmarks=landmarks, landmark_ids=[10, 11, 12, 13])
    sim = DiffDriveSim(world, *ideal, pose=(0.0, 0.0, 0.0))
    obs = sim.observe_landmarks()
    assert [o.id for o in obs] == [10]  # 11 outside FOV, 12 behind the wall, 13 too far
    assert obs[0].range_m == pytest.approx(3.0) and obs[0].bearing_rad == pytest.approx(0.0)


def test_landmark_bearing_sign(ideal):
    world = World.from_segments(np.zeros((0, 4)), landmarks=[[2.0, 0.5]])
    obs = DiffDriveSim(world, *ideal, pose=(0.0, 0.0, 0.2)).observe_landmarks()
    assert obs[0].bearing_rad == pytest.approx(math.atan2(0.5, 2.0) - 0.2)


def test_gyro_bias_and_noise(realistic):
    sim = DiffDriveSim(World(), *realistic, seed=4)
    samples = np.array([sim.gyro_z() for _ in range(4000)])
    assert samples.mean() == pytest.approx(sim.sensor_params.gyro_bias_rad_s, abs=5e-4)
    assert samples.std() == pytest.approx(sim.sensor_params.gyro_noise_std_rad_s, rel=0.1)
