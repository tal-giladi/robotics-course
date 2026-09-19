"""broken_battery_system.launch.py — a battery system with FOUR planted faults (lesson 04.13, exercise E1).

    ros2 launch course_debug_examples broken_battery_system.launch.py

What it is supposed to do:
  battery_sim (10 Hz) -> battery/voltage -> battery_health (low at 11.0 V) -> battery/health
                                         -> battery_monitor (logs LOW)
What it does: find out with the method from the lesson. Don't read this file for the answers —
debug the running system. (The faults are all realistic: each one has cost real teams days.)
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution([FindPackageShare('course_debug_examples'), 'config', 'battery_health.yaml'])
    return LaunchDescription([
        Node(package='course_qos_examples', executable='voltage_publisher_qos', name='battery_sim',
             parameters=[{'rate_hz': 0.25, 'drain_v_per_s': 0.02}], output='screen'),
        Node(package='course_params_examples', executable='battery_health_params', name='battery_health',
             parameters=[params], remappings=[('battery/voltage', 'battery/voltag')], output='screen'),
        Node(package='karmel_tutorial', executable='battery_monitor', name='battery_monitor', output='screen'),
    ])
