"""karmel's TF tree without a robot: map -> odom -> base_footprint -> base_link -> sensors.

    ros2 launch course_tf_examples karmel_tf_demo.launch.py
    ros2 launch course_tf_examples karmel_tf_demo.launch.py moving:=false jumps:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    moving = LaunchConfiguration('moving')
    jumps = LaunchConfiguration('jumps')
    listener = LaunchConfiguration('listener')
    return LaunchDescription([
        DeclareLaunchArgument('moving', default_value='true', description='drive a 0.5 m circle'),
        DeclareLaunchArgument('jumps', default_value='true', description='map->odom corrections'),
        DeclareLaunchArgument('listener', default_value='true', description='start bottle_listener'),
        Node(package='course_tf_examples', executable='karmel_static_tf'),
        Node(package='course_tf_examples', executable='fake_odometry', parameters=[{
            'linear_speed': PythonExpression(["0.1 if '", moving, "' == 'true' else 0.0"]),
            'angular_speed': PythonExpression(["0.2 if '", moving, "' == 'true' else 0.0"]),
        }]),
        Node(package='course_tf_examples', executable='fake_localization', parameters=[{
            'jump_every_s': PythonExpression(["5.0 if '", jumps, "' == 'true' else 0.0"]),
        }]),
        Node(package='course_tf_examples', executable='bottle_listener', output='screen', emulate_tty=True,
             condition=IfCondition(listener)),
    ])
