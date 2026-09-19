"""Online SLAM with slam_toolbox (async). Start the robot first (robot.launch.py or karmel_base).

    ros2 launch karmel_bringup robot.launch.py sim:=true world:=apartment
    ros2 launch karmel_bringup slam.launch.py                 # use_sim_time defaults to true
    ros2 launch karmel_bringup slam.launch.py use_sim_time:=false rviz:=true   # real robot

Drive around (teleop), then save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('karmel_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_params_file',
                              default_value=PathJoinSubstitution([bringup, 'config', 'slam_toolbox_online_async.yaml'])),
        DeclareLaunchArgument('rviz', default_value='false'),

        # slam_toolbox is a lifecycle node on Jazzy; its own launch file configures and activates it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items()),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'slam.rviz'])],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
