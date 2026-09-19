"""View karmel in RViz and move its wheel joints with sliders.

    ros2 launch karmel_description display.launch.py
    ros2 launch karmel_description display.launch.py gui:=false rviz:=false   # just publish TF
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('karmel_description')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # xacro runs at launch time; ParameterValue(..., value_type=str) stops launch from trying to
    # parse the URDF XML as YAML.
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'karmel.urdf.xacro']),
                 ' use_sim:=', LaunchConfiguration('use_sim')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='joint_state_publisher_gui (sliders) instead of joint_state_publisher'),
        DeclareLaunchArgument('rviz', default_value='true', description='start RViz'),
        DeclareLaunchArgument('use_sim', default_value='false',
                              description='render the simulation variant of the URDF'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', arguments=['-d', LaunchConfiguration('rviz_config')],
             condition=IfCondition(rviz), output='screen'),
    ])
