"""karmel_bringup configs must agree with the robot config (karmel_description/config/robot.yaml)."""
import ast
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
ROBOT = PKG.parent / 'karmel_description' / 'config' / 'robot.yaml'


def load(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def robot():
    if not ROBOT.is_file():
        pytest.skip('karmel_description source not next to karmel_bringup')
    return load(ROBOT)


def test_diff_drive_matches_robot(robot):
    ddc = load(PKG / 'config' / 'controllers.yaml')['diff_drive_controller']['ros__parameters']
    drive = robot['drive']
    assert ddc['wheel_separation'] == pytest.approx(drive['wheel_separation_m'])
    assert ddc['wheel_radius'] == pytest.approx(drive['wheel_radius_m'])
    assert ddc['linear.x.max_velocity'] == pytest.approx(drive['max_linear_speed_m_s'])
    assert ddc['angular.z.max_velocity'] == pytest.approx(drive['max_angular_speed_rad_s'])
    assert ddc['linear.x.max_acceleration'] == pytest.approx(drive['max_linear_accel_m_s2'])
    assert ddc['left_wheel_names'] == ['left_wheel_joint']
    # odometry must attach to the URDF root, or base_link gets two TF parents and the tree splits
    assert ddc['base_frame_id'] == 'base_footprint'
    assert ddc['right_wheel_names'] == ['right_wheel_joint']


def test_nav2_footprint_matches_robot(robot):
    nav2 = load(PKG / 'config' / 'nav2_params.yaml')
    half_length = robot['chassis']['length_m'] / 2
    # overall width = outer wheel edge to outer wheel edge
    half_width =(robot['drive']['wheel_separation_m'] + robot['drive']['wheel_width_m']) / 2
    for costmap in ('local_costmap', 'global_costmap'):
        footprint = ast.literal_eval(nav2[costmap][costmap]['ros__parameters']['footprint'])
        xs = sorted({abs(p[0]) for p in footprint})
        ys = sorted({abs(p[1]) for p in footprint})
        assert xs == [pytest.approx(half_length)]
        assert ys == [pytest.approx(half_width)]


def test_nav2_speed_limit_is_course_limit():
    nav2 = load(PKG / 'config' / 'nav2_params.yaml')
    smoother = nav2['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][0] <= 0.3
    assert nav2['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] <= 0.3
    for node in ('controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor'):
        assert nav2[node]['ros__parameters']['enable_stamped_cmd_vel'] is True, node


def test_ekf_override_disables_controller_tf():
    override = load(PKG / 'config' / 'diff_drive_ekf.yaml')
    assert override['diff_drive_controller']['ros__parameters']['enable_odom_tf'] is False
    assert load(PKG / 'config' / 'ekf.yaml')['ekf_filter_node']['ros__parameters']['publish_tf'] is True


def test_diff_drive_uses_current_acceleration_parameters():
    """min_acceleration is deprecated in ros2_controllers 4.x; max_deceleration replaces it."""
    ddc = load(PKG / 'config' / 'controllers.yaml')['diff_drive_controller']['ros__parameters']
    for axis in ('linear.x', 'angular.z'):
        assert f'{axis}.min_acceleration' not in ddc, f'{axis}.min_acceleration is deprecated'
        assert ddc[f'{axis}.max_acceleration'] > 0, axis      # forward, m/s^2 or rad/s^2
        assert ddc[f'{axis}.max_deceleration'] < 0, axis      # braking, same sign convention


def test_global_costmap_marks_only_nearby_lidar_returns():
    """A far return lands in the wrong cell (localization error grows with range).

    At obstacle_max_range 3.5 m the smeared marks closed the apartment's 0.8 m doorways and
    every cross-room goal failed with NO_VALID_PATH (208). Mark near, clear far.
    """
    nav2 = load(PKG / 'config' / 'nav2_params.yaml')
    scan = nav2['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']
    assert scan['obstacle_max_range'] <= 1.5
    assert scan['raytrace_max_range'] > scan['obstacle_max_range'], 'clearing must outrange marking'


def test_map_thresholds_keep_unknown_cells_unknown():
    """map_server turns pixel 205 (unknown grey) into occ 0.196; free_thresh must stay below it.

    With free_thresh 0.25 the 2,607 unknown cells of the apartment map were served as FREE,
    so the planner happily routed through unexplored space.
    """
    unknown_occ = (255 - 205) / 255.0
    for path in (PKG / 'maps').glob('*.yaml'):
        meta = load(path)
        assert meta['free_thresh'] <= unknown_occ, f'{path.name}: unknown cells would be free'
        assert meta['occupied_thresh'] > unknown_occ, path.name
    saver = load(PKG / 'config' / 'nav2_params.yaml')['map_saver']['ros__parameters']
    assert saver['free_thresh_default'] <= unknown_occ


def test_all_yaml_parses():
    for path in list((PKG / 'config').glob('*.yaml')) + list((PKG / 'maps').glob('*.yaml')):
        assert load(path) is not None, path
