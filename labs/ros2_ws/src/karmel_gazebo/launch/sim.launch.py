"""Start Gazebo Harmonic with karmel in a world.

    ros2 launch karmel_gazebo sim.launch.py                         # empty world, with GUI
    ros2 launch karmel_gazebo sim.launch.py world:=apartment
    ros2 launch karmel_gazebo sim.launch.py world:=tabletop headless:=true
    ros2 launch karmel_gazebo sim.launch.py world:=/abs/path/my_world.sdf x:=1.0 yaw:=1.57

What it starts:
  gz sim (server + GUI, or server only with headless:=true)
  robot_state_publisher   robot_description from xacro (use_sim:=true), use_sim_time
  ros_gz_sim create       spawns the robot from the /robot_description topic
  ros_gz_bridge           /clock, /scan, /imu, /camera/image_raw, /camera/camera_info (config/bridge.yaml)

It does NOT start the controllers — karmel_bringup/robot.launch.py sim:=true does that (and
includes this file), so the simulated and real robot share one controller launch path.
The gz_ros2_control plugin inside Gazebo reads controllers_file (default: karmel_bringup's).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def resolve_world(world):
    """'apartment' -> <share>/worlds/apartment.sdf; absolute paths pass through."""
    if os.path.isabs(world) or world.endswith('.sdf'):
        return world
    return os.path.join(get_package_share_directory('karmel_gazebo'), 'worlds', world + '.sdf')


def gazebo(context):
    world = resolve_world(LaunchConfiguration('world').perform(context))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    verbosity = LaunchConfiguration('gz_verbosity').perform(context)
    # -r: start running immediately.  -s: server only.  --headless-rendering: render sensors with
    # EGL and no display (needed for gpu_lidar/camera in Docker/CI without X).
    gz_args = f'-r -v {verbosity} ' + ('-s --headless-rendering ' if headless else '') + world
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    description_pkg = FindPackageShare('karmel_description')
    gazebo_pkg = FindPackageShare('karmel_gazebo')
    use_sim_time = {'use_sim_time': True}

    robot_description = ParameterValue(
        Command([
            'xacro ', PathJoinSubstitution([description_pkg, 'urdf', 'karmel.urdf.xacro']),
            ' use_sim:=true',
            ' enable_camera:=', LaunchConfiguration('enable_camera'),
            ' controllers_file:=', LaunchConfiguration('controllers_file'),
        ]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description='empty | apartment | tabletop | absolute path to an .sdf'),
        DeclareLaunchArgument('headless', default_value='false', description='server only, no GUI'),
        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='simulate the camera (false = faster on CPU-only machines)'),
        DeclareLaunchArgument('controllers_file',
                              default_value=PathJoinSubstitution(
                                  [FindPackageShare('karmel_bringup'), 'config', 'controllers.yaml']),
                              description='ros2_control controller YAML loaded by gz_ros2_control'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gz_verbosity', default_value='2'),

        # Lets Gazebo find resources installed by our packages (meshes/models if you add them).
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), os.pathsep,
             os.path.join(get_package_share_directory('karmel_description'), '..')]),

        OpaqueFunction(function=gazebo),

        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}, use_sim_time]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'karmel',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
             parameters=[{'config_file': PathJoinSubstitution([gazebo_pkg, 'config', 'bridge.yaml'])},
                         use_sim_time]),
    ])
