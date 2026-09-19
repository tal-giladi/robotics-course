"""Bring up karmel with ros2_control — in simulation or on the real robot, with ONE argument.

    ros2 launch karmel_bringup robot.launch.py sim:=true                       # Gazebo, empty world
    ros2 launch karmel_bringup robot.launch.py sim:=true world:=apartment headless:=true
    ros2 launch karmel_bringup robot.launch.py sim:=false                      # real robot, /dev/ttyACM0
    ros2 launch karmel_bringup robot.launch.py sim:=false serial_device:=/dev/ttyACM1 use_ekf:=true

The teaching point: everything from the controller_manager upward is identical.

                   sim:=true                              sim:=false
    hardware       gz_ros2_control/GazeboSimSystem        karmel_hardware/KarmelSystem (serial)
    controller_mgr runs inside Gazebo (plugin)            ros2_control_node
    sensors        Gazebo + ros_gz_bridge                 real drivers (sllidar_ros2, v4l2_camera, bno055)
    ─────────────────────────────── same from here up ───────────────────────────────
    controllers    joint_state_broadcaster + diff_drive_controller (config/controllers.yaml)
    topics         /cmd_vel (TwistStamped), /odom, /tf odom->base_footprint, /joint_states
    optional       robot_localization EKF (use_ekf:=true), RViz (rviz:=true)

Drive it:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('karmel_bringup')
    sim = LaunchConfiguration('sim')
    use_ekf = LaunchConfiguration('use_ekf')
    controllers_file = LaunchConfiguration('controllers_file')
    use_sim_time = {'use_sim_time': sim}

    # ---------------------------------------------------------------- simulation: Gazebo + bridge
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('karmel_gazebo'), 'launch', 'sim.launch.py'])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
            'enable_camera': LaunchConfiguration('enable_camera'),
            'controllers_file': controllers_file,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
        condition=IfCondition(sim))

    # ---------------------------------------------------------------- real robot: RSP + ros2_control_node
    real_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([FindPackageShare('karmel_description'), 'urdf', 'karmel.urdf.xacro']),
                 ' use_sim:=false serial_device:=', LaunchConfiguration('serial_device')]),
        value_type=str)

    real_robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': real_description}],
        condition=UnlessCondition(sim))

    # Since Jazzy the controller_manager reads the URDF from the /robot_description topic.
    control_node = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[controllers_file],
        remappings=[('~/robot_description', '/robot_description')],
        condition=UnlessCondition(sim))

    # ---------------------------------------------------------------- controllers (both worlds)
    # Generous timeouts: a Gazebo that starts slowly (CPU rendering, Docker, CI) needs more than the
    # default 5 s to perform the first controller switch.
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120', '--switch-timeout', '60'])

    # diff_drive_controller's topics are ~/cmd_vel and ~/odom; expose them as /cmd_vel and /odom so
    # teleop, Nav2 and the karmel_base path all use the same names.
    diff_drive_args = ['diff_drive_controller', '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '120', '--switch-timeout', '60',
                       '--param-file', controllers_file,
                       '--controller-ros-args',
                       '-r /diff_drive_controller/cmd_vel:=/cmd_vel -r /diff_drive_controller/odom:=/odom']
    diff_drive = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=diff_drive_args,
        condition=UnlessCondition(use_ekf))
    # With the EKF, the EKF owns odom -> base_footprint; the controller must not publish it too.
    diff_drive_for_ekf = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=diff_drive_args + ['--param-file', PathJoinSubstitution([bringup, 'config', 'diff_drive_ekf.yaml'])],
        condition=IfCondition(use_ekf))

    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[PathJoinSubstitution([bringup, 'config', 'ekf.yaml']), use_sim_time],
        condition=IfCondition(use_ekf))

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'robot.rviz'])],
        parameters=[use_sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', description='true: Gazebo, false: real robot'),
        DeclareLaunchArgument('world', default_value='empty', description='[sim] empty | apartment | tabletop | path'),
        DeclareLaunchArgument('headless', default_value='false', description='[sim] no Gazebo GUI'),
        DeclareLaunchArgument('enable_camera', default_value='true', description='[sim] simulate the camera'),
        DeclareLaunchArgument('x', default_value='0.0', description='[sim] spawn x'),
        DeclareLaunchArgument('y', default_value='0.0', description='[sim] spawn y'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='[sim] spawn yaw'),
        DeclareLaunchArgument('serial_device', default_value='',
                              description='[real] Pico serial port (default: serial.device in karmel.yaml)'),
        DeclareLaunchArgument('controllers_file', default_value=PathJoinSubstitution([bringup, 'config', 'controllers.yaml'])),
        DeclareLaunchArgument('use_ekf', default_value='false',
                              description='fuse wheel odometry + IMU with robot_localization (publishes odom->base_footprint)'),
        DeclareLaunchArgument('rviz', default_value='false', description='start RViz'),

        simulation,
        real_robot_state_publisher,
        control_node,
        joint_state_broadcaster,
        diff_drive,
        diff_drive_for_ekf,
        ekf,
        rviz,
    ])
