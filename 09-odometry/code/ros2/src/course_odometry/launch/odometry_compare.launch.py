"""Run your wheel odometry next to whatever already publishes /odom, and compare them.

    ros2 launch course_odometry odometry_compare.launch.py
    ros2 launch course_odometry odometry_compare.launch.py use_sim_time:=true            # with Gazebo
    ros2 launch course_odometry odometry_compare.launch.py robot_config:=/path/karmel.yaml

Start the robot first, one of:
    ros2 launch karmel_base base.launch.py                           # simple driver (04.16): /odom from base_node
    ros2 launch karmel_bringup robot.launch.py sim:=true             # Gazebo + diff_drive_controller
    ros2 launch karmel_bringup robot.launch.py sim:=false            # real robot + diff_drive_controller
"""
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params = {'use_sim_time': use_sim_time}
    config_path = LaunchConfiguration('robot_config').perform(context)
    if config_path:
        with open(config_path, encoding='utf-8') as f:
            drive = yaml.safe_load(f)['drive']
        params.update(wheel_radius=float(drive['wheel_radius_m']), wheel_separation=float(drive['wheel_separation_m']))
    return [
        Node(package='course_odometry', executable='wheel_odometry', output='screen', parameters=[params]),
        Node(package='course_odometry', executable='odom_compare', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'topics': ['/odom', '/odom_mine']}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_config', default_value='',
                              description='karmel.yaml to read wheel_radius_m / wheel_separation_m from'),
        OpaqueFunction(function=launch_setup),
    ])
