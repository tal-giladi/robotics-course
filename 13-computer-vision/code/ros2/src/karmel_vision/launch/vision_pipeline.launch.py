"""The whole 13.15 + 13.16 pipeline: camera -> detector -> 3D -> object map.

    ros2 launch karmel_vision vision_pipeline.launch.py spin_rad_s:=0.4 inference_ms:=150.0 method:=depth

Arguments (all typed, see ARGS below): rate_hz, width, height, spin_rad_s, drive_m_s, inference_ms,
image_qos, method (depth|size|ground), use_latest — the stale-transform bug of 05.10, on a switch.

Note the `ParameterValue(..., value_type=...)` wrapper: a launch argument is a *string*, and a node
that declared `rate_hz` as a double rejects the string "10.0" with a confusing type error.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {                          # name: (default, type)
    "rate_hz": ("10.0", float),
    "width": ("320", int),        # 320x240 keeps a raw frame inside one UDP-friendly message;
    "height": ("240", int),       # 13.15 measures what happens at 640x480
    "spin_rad_s": ("0.4", float),
    "drive_m_s": ("0.0", float),
    "inference_ms": ("150.0", float),
    "image_qos": ("sensor", str),
    "method": ("depth", str),
    "use_latest": ("false", bool),
}


def generate_launch_description() -> LaunchDescription:
    cfg = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
           for name, (_default, kind) in ARGS.items()}
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, (default, _k) in ARGS.items()]
        + [
            Node(package="karmel_vision", executable="fake_camera", name="fake_camera", output="screen",
                 parameters=[{"rate_hz": cfg["rate_hz"], "spin_rad_s": cfg["spin_rad_s"],
                              "drive_m_s": cfg["drive_m_s"], "width": cfg["width"],
                              "height": cfg["height"]}]),
            Node(package="karmel_vision", executable="detector_node", name="detector_node", output="screen",
                 parameters=[{"inference_ms": cfg["inference_ms"], "image_qos": cfg["image_qos"]}]),
            Node(package="karmel_vision", executable="detection_3d_node", name="detection_3d_node",
                 output="screen", parameters=[{"method": cfg["method"]}]),
            Node(package="karmel_vision", executable="object_map_node", name="object_map_node",
                 output="screen", parameters=[{"use_latest": cfg["use_latest"],
                                               "truth": [2.05, 1.70, 0.12]}]),
        ]
    )
