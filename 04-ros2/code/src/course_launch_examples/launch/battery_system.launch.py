"""battery_system.launch.py — the whole battery + drive example from 04.04–04.09 in one command (lesson 04.10).

    ros2 launch course_launch_examples battery_system.launch.py
    ros2 launch course_launch_examples battery_system.launch.py --show-args
    ros2 launch course_launch_examples battery_system.launch.py namespace:=karmel1 use_monitor:=false
    ros2 launch course_launch_examples battery_system.launch.py voltage_topic:=/power/pack_voltage log_level:=debug

Starts (all inside the optional namespace):
  battery_sim             karmel_tutorial          publishes battery/voltage
  battery_health          course_params_examples   parameters from params_file
  battery_monitor         karmel_tutorial          only if use_monitor:=true
  drive_distance_server   karmel_tutorial          unless sim_only:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # 1. Arguments: the launch file's public API. Each has a default and a description (--show-args).
    args = [
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace for every node, e.g. karmel1 (empty = none)'),
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution(
                                  [FindPackageShare('course_launch_examples'), 'config', 'battery_health.yaml']),
                              description='YAML parameter file for battery_health'),
        DeclareLaunchArgument('voltage_topic', default_value='battery/voltage',
                              description='Topic the voltage flows on (remapped for every node)'),
        DeclareLaunchArgument('sim_rate_hz', default_value='2.0', description='battery_sim publish rate [Hz]'),
        DeclareLaunchArgument('use_monitor', default_value='true', choices=['true', 'false'],
                              description='Also start the simple battery_monitor from 04.04'),
        DeclareLaunchArgument('sim_only', default_value='false', choices=['true', 'false'],
                              description='Skip the drive_distance action server'),
        DeclareLaunchArgument('log_level', default_value='info', choices=['debug', 'info', 'warn', 'error'],
                              description='Log level for all nodes'),
    ]

    namespace = LaunchConfiguration('namespace')
    voltage_topic = LaunchConfiguration('voltage_topic')
    log_level = LaunchConfiguration('log_level')

    # 2. Remapping: every node in this file was written against 'battery/voltage'.
    #    One list rewires all of them without touching their code.
    voltage_remap = [('battery/voltage', voltage_topic)]
    common = {'output': 'screen', 'ros_arguments': ['--log-level', log_level]}

    nodes = GroupAction([
        PushRosNamespace(namespace),  # everything below gets the namespace; relative names only!
        Node(package='karmel_tutorial', executable='battery_sim', name='battery_sim',
             parameters=[{'rate_hz': LaunchConfiguration('sim_rate_hz')}],
             remappings=voltage_remap, **common),
        Node(package='course_params_examples', executable='battery_health_params', name='battery_health',
             parameters=[LaunchConfiguration('params_file')],
             remappings=voltage_remap, **common),
        Node(package='karmel_tutorial', executable='battery_monitor', name='battery_monitor',
             condition=IfCondition(LaunchConfiguration('use_monitor')),
             remappings=voltage_remap, **common),
        Node(package='karmel_tutorial', executable='drive_distance_server', name='drive_distance_server',
             condition=UnlessCondition(LaunchConfiguration('sim_only')), **common),
    ])

    return LaunchDescription(args + [
        LogInfo(msg=['battery_system: namespace="', namespace, '" voltage_topic=', voltage_topic]),
        nodes,
    ])
