"""Show the SO-101 in RViz with sliders — no controllers, no hardware (lesson 14.08).

    ros2 launch so101_description display.launch.py

robot_state_publisher turns /joint_states + the URDF into the TF tree; joint_state_publisher_gui
gives you a slider per joint so you can drive that tree by hand. This is the first thing to run
after editing the xacro: if the arm looks wrong here, nothing downstream can be right.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " hardware:=", LaunchConfiguration("hardware")]),
        value_type=str,
    )
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mock",
                              description="mock | topic | gazebo"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="run joint_state_publisher_gui"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            # turn this off (gui:=false) as soon as a controller publishes /joint_states —
            # two publishers on that topic is the classic "the arm twitches in RViz" bug
            condition=IfCondition(LaunchConfiguration("gui")),
            output="screen",
        ),
        Node(package="rviz2", executable="rviz2", output="screen"),
    ])
