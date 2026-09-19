"""Real robot, simple path: base_node (protocol v1) + robot_state_publisher.

    ros2 launch karmel_base base.launch.py                          # serial.device from karmel.yaml
    ros2 launch karmel_base base.launch.py port:=/tmp/ttyKARMEL     # fake Pico over a pty
    ros2 launch karmel_base base.launch.py port:=socket://localhost:5760
    ros2 launch karmel_base base.launch.py publish_tf:=false        # when an EKF publishes odom->base_footprint

All physical parameters are read from karmel_description/config/robot.yaml (the copy of
labs/config/karmel.yaml), so the driver, the URDF and the controllers agree.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def robot_params(config_path):
    """Map karmel.yaml onto base_node parameters."""
    with open(config_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    d, s, b = cfg['drive'], cfg['serial'], cfg['battery']
    rng = cfg['sensors']['range_front']
    params = {
        'baud': int(s['baud']),
        'wheel_radius': float(d['wheel_radius_m']),
        'wheel_separation': float(d['wheel_separation_m']),
        'ticks_per_rev': int(d['encoder_cpr_motor'] * d['gear_ratio'] * d['quadrature_multiplier']),
        'max_wheel_speed': float(d['max_wheel_speed_rad_s']),
        'max_linear_speed': float(d['max_linear_speed_m_s']),
        'max_angular_speed': float(d['max_angular_speed_rad_s']),
        'max_linear_accel': float(d['max_linear_accel_m_s2']),
        'watchdog_ms': int(s['watchdog_ms']),
        'telemetry_hz': int(s['telemetry_hz']),
        'battery_full_v': float(b['full_v']),
        'battery_cutoff_v': float(b['cutoff_v']),
        'battery_cells': int(b['cells_series']),
        'battery_capacity_ah': float(b['capacity_ah']),
        'range_type': 'infrared' if str(rng['type']).startswith('vl53') else 'ultrasound',
        'range_fov': float(rng['fov_rad']),
        'range_max': float(rng['max_range_m']),
    }
    return params, str(s['device'])


def launch_setup(context):
    params, default_port = robot_params(LaunchConfiguration('robot_config').perform(context))
    params['port'] = LaunchConfiguration('port').perform(context) or default_port
    params['publish_tf'] = LaunchConfiguration('publish_tf').perform(context).lower() == 'true'
    # Odometry attaches to the URDF root. base_link already has a parent (base_footprint, fixed joint);
    # a second parent would split the TF tree.
    params['base_frame'] = 'base_footprint'
    params['use_stamped_cmd_vel'] = LaunchConfiguration('use_stamped_cmd_vel').perform(context).lower() == 'true'

    robot_description = ParameterValue(
        Command(['xacro ',
                 PathJoinSubstitution([FindPackageShare('karmel_description'), 'urdf', 'karmel.urdf.xacro']),
                 ' use_sim:=false']),
        value_type=str)

    return [
        Node(package='karmel_base', executable='base_node', name='base_node', output='screen',
             parameters=[params]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
    ]


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('karmel_description'), 'config', 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='',
                              description='serial device or pyserial URL (default: serial.device in robot.yaml)'),
        DeclareLaunchArgument('publish_tf', default_value='true', description='publish odom -> base_footprint TF'),
        DeclareLaunchArgument('use_stamped_cmd_vel', default_value='true',
                              description='subscribe geometry_msgs/TwistStamped (false: Twist)'),
        DeclareLaunchArgument('robot_config', default_value=default_config,
                              description='robot YAML (a copy of labs/config/karmel.yaml)'),
        OpaqueFunction(function=launch_setup),
    ])
