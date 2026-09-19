# The ROS 2 container for lessons 13.15 and 13.16.
#
# `ros:jazzy-ros-base` has rclpy and tf2 but not the vision packages, so this image adds
# cv_bridge, vision_msgs, image_transport and OpenCV from apt — never from pip, because
# cv_bridge is compiled against Ubuntu 24.04's python3-opencv (versions.yaml, 13.07).
#
#   docker build -t karmel-vision:jazzy 13-computer-vision/code/ros2
#   docker run -it --rm --name vision -e ROS_DOMAIN_ID=73 \
#       -v "${PWD}/13-computer-vision/code/ros2:/ws" karmel-vision:jazzy bash
#
# Inside: cd /ws && colcon build --symlink-install && source install/setup.bash
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-cv-bridge \
      ros-jazzy-vision-msgs \
      ros-jazzy-tf2-ros-py \
      ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-image-transport \
      ros-jazzy-compressed-image-transport \
      ros-jazzy-image-transport-plugins \
      python3-opencv \
      python3-numpy \
      python3-pytest \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
