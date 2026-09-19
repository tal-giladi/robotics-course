"""karmel's frames from the REAL URDF (robot_state_publisher) + fake odometry/localization + a fake bottle detector.

Lessons 05.08-05.11. Needs karmel_description (labs/ros2_ws) built and sourced before this workspace.

    ros2 launch course_tf_examples karmel_urdf_demo.launch.py                           # headless
    ros2 launch course_tf_examples karmel_urdf_demo.launch.py rviz:=true                # with RViz
    ros2 launch course_tf_examples karmel_urdf_demo.launch.py moving:=false jumps:=false
    ros2 launch course_tf_examples karmel_urdf_demo.launch.py robot_config:=/tmp/robot.yaml

Unlike karmel_tf_demo.launch.py, nothing here hard-codes a sensor position: every fixed transform
comes from karmel.urdf.xacro, which reads config/robot.yaml. Don't run karmel_static_tf together
with this launch file: two publishers for the same frames.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description = FindPackageShare('karmel_description')
    moving = LaunchConfiguration('moving')
    jumps = LaunchConfiguration('jumps')

    # xacro runs at launch time. value_type=str: the result is XML, not YAML.
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([description, 'urdf', 'karmel.urdf.xacro']),
                 ' use_sim:=false robot_config:=', LaunchConfiguration('robot_config')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('moving', default_value='true', description='drive a 0.5 m circle'),
        DeclareLaunchArgument('jumps', default_value='true', description='map->odom corrections every 5 s'),
        DeclareLaunchArgument('detector', default_value='true', description='bottle_detector + bottle_to_map'),
        DeclareLaunchArgument('scan', default_value='true', description='fake_scan: a LiDAR in a 4 m x 3 m room'),
        DeclareLaunchArgument('rviz', default_value='false', description='start RViz with karmel_frames.rviz'),
        DeclareLaunchArgument('robot_config',
                              default_value=PathJoinSubstitution([description, 'config', 'robot.yaml']),
                              description='robot YAML read by the xacro (sensor positions, wheel size)'),

        # URDF -> /robot_description (latched) and /tf_static (fixed joints) + /tf (movable joints)
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        # Publishes /joint_states (0 rad) for the two continuous wheel joints, so their TF exists.
        Node(package='joint_state_publisher', executable='joint_state_publisher'),

        Node(package='course_tf_examples', executable='fake_odometry', parameters=[{
            'linear_speed': PythonExpression(["0.1 if '", moving, "' == 'true' else 0.0"]),
            'angular_speed': PythonExpression(["0.2 if '", moving, "' == 'true' else 0.0"]),
        }]),
        Node(package='course_tf_examples', executable='fake_localization', parameters=[{
            'jump_every_s': PythonExpression(["5.0 if '", jumps, "' == 'true' else 0.0"]),
        }]),
        Node(package='course_tf_examples', executable='fake_scan',
             condition=IfCondition(LaunchConfiguration('scan'))),
        Node(package='course_tf_examples', executable='bottle_detector',
             condition=IfCondition(LaunchConfiguration('detector'))),
        Node(package='course_tf_examples', executable='bottle_to_map', output='screen', emulate_tty=True,
             condition=IfCondition(LaunchConfiguration('detector'))),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('course_tf_examples'), 'rviz',
                                                    'karmel_frames.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
