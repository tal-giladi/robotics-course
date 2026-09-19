"""lifecycle_battery.launch.py — start battery_sim and the lifecycle battery_health, then bring it up in order (04.15).

    ros2 launch course_lifecycle_examples lifecycle_battery.launch.py                    # configure + activate
    ros2 launch course_lifecycle_examples lifecycle_battery.launch.py autostart:=false    # stays unconfigured

This is a hand-written, one-node version of what nav2_lifecycle_manager does for Nav2's servers:
emit "configure"; when the node reports it reached "inactive", emit "activate".
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    autostart = LaunchConfiguration('autostart')

    health = LifecycleNode(package='course_lifecycle_examples', executable='lifecycle_battery_health',
                           name='battery_health', namespace='', output='screen')

    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(health), transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))

    activate_when_inactive = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=health, goal_state='inactive',
        entities=[
            LogInfo(msg='battery_health is inactive (configured) -> activating'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(health), transition_id=Transition.TRANSITION_ACTIVATE)),
        ]), condition=IfCondition(autostart))

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false'],
                              description='configure and activate battery_health automatically'),
        Node(package='karmel_tutorial', executable='battery_sim', name='battery_sim',
             ros_arguments=['--log-level', 'warn'], output='screen'),
        activate_when_inactive,  # register the handler BEFORE the node can reach inactive
        health,
        configure,
    ])
