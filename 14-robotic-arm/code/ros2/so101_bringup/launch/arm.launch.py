"""Bring up the SO-101 under ros2_control (lesson 14.08).

    ros2 launch so101_bringup arm.launch.py                       # mock hardware, no arm needed
    ros2 launch so101_bringup arm.launch.py hardware:=topic \\
         port:=/dev/ttyACM0 calibration:=$HOME/so101_joint_map.json     # the real arm

What starts, and why:

  robot_state_publisher   URDF + /joint_states  ->  the TF tree RViz and MoveIt read
  ros2_control_node       the controller_manager: loads the hardware plugin and the controllers
  spawner joint_state_broadcaster   reads the hardware's state interfaces -> /joint_states
  spawner arm_controller            JointTrajectoryController -> the FollowJointTrajectory action
  spawner gripper_controller        GripperActionController
  joint_bridge (hardware:=topic)    the Python servo driver behind TopicBasedSystem

The spawners are deliberately ORDERED: joint_state_broadcaster first, so that by the time a
trajectory controller activates there is already a published state for it to start from.

> SAFETY (14.11): with hardware:=topic this energises the arm. Clamp it down, clear the
> workspace, keep the PSU switch in reach.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    hardware = LaunchConfiguration("hardware")
    controllers = PathJoinSubstitution(
        [FindPackageShare("so101_bringup"), "config", "controllers.yaml"])
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101.urdf.xacro"])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " hardware:=", hardware]), value_type=str)

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers],
        output="screen",
    )

    def spawner(name: str) -> Node:
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager"],
                    output="screen")

    broadcaster = spawner("joint_state_broadcaster")
    arm = spawner("arm_controller")
    gripper = spawner("gripper_controller")

    bridge = Node(
        package="so101_bringup",
        executable="joint_bridge",
        condition=IfCondition(PythonExpression(["'", hardware, "' == 'topic'"])),
        parameters=[{
            "port": LaunchConfiguration("port"),
            "robot_id": LaunchConfiguration("robot_id"),
            "calibration": LaunchConfiguration("calibration"),
            "fake": LaunchConfiguration("fake"),
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mock", description="mock | topic | gazebo"),
        DeclareLaunchArgument("port", default_value="", description="serial port of the arm"),
        DeclareLaunchArgument("robot_id", default_value="karmel_follower"),
        DeclareLaunchArgument("calibration", default_value="",
                              description="joint_map.json from 14.08-E1"),
        DeclareLaunchArgument("fake", default_value="false",
                              description="joint_bridge simulates the servos instead of opening the port"),
        state_publisher,
        control_node,
        bridge,
        broadcaster,
        # start the motion controllers only once the state broadcaster is up
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=[arm, gripper])),
    ])
