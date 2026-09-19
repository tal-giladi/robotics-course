"""two_robots.launch.py — include battery_system.launch.py twice, in two namespaces (lesson 04.10).

    ros2 launch course_launch_examples two_robots.launch.py
    ros2 node list        # /karmel1/battery_health, /karmel2/battery_health, ...

karmel2 gets a bench-test parameter file (warns early) and no action server: the included
file's arguments are its interface, exactly like calling a function with keyword arguments.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare('course_launch_examples')
    system_file = PathJoinSubstitution([share, 'launch', 'battery_system.launch.py'])

    # One source object per include: launch refuses to execute the same action object twice.
    karmel1 = IncludeLaunchDescription(PythonLaunchDescriptionSource(system_file), launch_arguments={
        'namespace': 'karmel1',
        'use_monitor': 'false',
    }.items())

    karmel2 = IncludeLaunchDescription(PythonLaunchDescriptionSource(system_file), launch_arguments={
        'namespace': 'karmel2',
        'use_monitor': 'false',
        'sim_only': 'true',
        'sim_rate_hz': '5.0',
        'params_file': PathJoinSubstitution([share, 'config', 'battery_health_bench.yaml']),
    }.items())

    return LaunchDescription([karmel1, karmel2])
