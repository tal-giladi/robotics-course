"""16.02 — the same bag reading, on the robot, with ROS 2 Jazzy's own rosbag2_py (no extra installs).

    source /opt/ros/jazzy/setup.bash
    python3 bag_frames_rosbag2py.py data/bags/kitchen-day-1

Prints the session info and the first few image stamps. bag_to_dataset.py does the same with the
pure-Python `rosbags` library so it also runs on Windows/macOS without ROS.
"""
import json
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main(bag: str) -> None:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/karmel/session_info", "/camera/image_raw/compressed"]))
    shown = 0
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == "/karmel/session_info":
            print("session:", json.loads(msg.data))
        elif shown < 3:
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            print(f"image stamp {stamp} recorded {t_ns} shape {img.shape}")
            shown += 1


if __name__ == "__main__":
    main(sys.argv[1])
