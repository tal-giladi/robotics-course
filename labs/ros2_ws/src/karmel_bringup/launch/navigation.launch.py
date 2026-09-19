"""Nav2 on a saved map: map_server + AMCL + the Nav2 servers, with karmel's parameters.

Start the robot first, then:

    ros2 launch karmel_bringup navigation.launch.py                                   # apartment map, sim time
    ros2 launch karmel_bringup navigation.launch.py map:=$HOME/maps/my_room.yaml use_sim_time:=false
    ros2 launch karmel_bringup navigation.launch.py initial_pose:=true x:=-1.0 y:=0.0 yaw:=0.0

Without initial_pose:=true, give AMCL a pose with RViz's "2D Pose Estimate" before sending goals.
Send a goal from RViz ("Nav2 Goal") or the command line:

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: -1.0, y: -1.0}, orientation: {w: 1.0}}}}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def nav2(context):
    params = LaunchConfiguration('params_file').perform(context)
    if LaunchConfiguration('initial_pose').perform(context).lower() == 'true':
        # Set AMCL's initial pose from launch arguments without editing the YAML file.
        params = RewrittenYaml(
            source_file=params,
            param_rewrites={
                'amcl.ros__parameters.set_initial_pose': 'true',
                'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('x').perform(context),
                'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('y').perform(context),
                'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('yaw').perform(context),
            },
            convert_types=True)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': params,
            'autostart': 'true',
            'use_composition': 'False',
            'slam': 'False',
        }.items())]


def generate_launch_description():
    bringup = FindPackageShare('karmel_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([bringup, 'maps', 'apartment.yaml'])),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([bringup, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('initial_pose', default_value='false', description='set AMCL initial pose from x/y/yaw'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false'),

        OpaqueFunction(function=nav2),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'navigation.rviz'])],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
