# syntax=docker/dockerfile:1
#
# karmel course image: ROS 2 Jazzy + Gazebo Harmonic + ros2_control + Nav2 + slam_toolbox.
#
# Build from the labs/ directory (the build context must contain ros2_ws/):
#   docker build -f docker/Dockerfile -t karmel-ros:jazzy .
# or simply:
#   docker compose -f docker/compose.yaml build
#
# Stages
#   deps  – ROS + Gazebo + every apt package the labs need, non-root user. Use it as a
#           dev container with the workspace bind-mounted (compose does this).
#   final – deps + a colcon build of labs/ros2_ws baked into /home/ros/labs/ros2_ws (for CI,
#           for the Pi-less "just run it" path, and to prove the workspace builds).
#
# osrf/ros:jazzy-desktop-full is amd64 only. On a Raspberry Pi / Jetson (arm64) use
# ros:jazzy-ros-base as BASE_IMAGE and drop the GUI packages (see docker/README.md).

ARG BASE_IMAGE=osrf/ros:jazzy-desktop-full
FROM ${BASE_IMAGE} AS deps

ARG ROS_DISTRO=jazzy
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONUNBUFFERED=1

# Versions are whatever packages.ros.org ships for noble today (see curriculum/versions.yaml).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-ament-lint-auto \
        ros-${ROS_DISTRO}-ament-lint-common \
        ros-${ROS_DISTRO}-ament-cmake-pytest \
        python3-serial \
        python3-pytest \
        python3-yaml \
        python3-colcon-common-extensions \
        python3-rosdep \
        bash-completion \
        less \
        nano \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Non-root user. Ubuntu 24.04 images ship a user "ubuntu" with UID 1000; remove it so our
# user gets UID 1000 and files created in bind mounts are owned by the host user on Linux.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout,video,plugdev ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
# Inside the container the repo layout is mirrored: ~/labs/{config,python,ros2_ws}.
# build/ install/ log/ exist (owned by the user) so compose's named volumes inherit that ownership.
RUN mkdir -p ~/labs/ros2_ws/build ~/labs/ros2_ws/install ~/labs/ros2_ws/log \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo '[ -f ~/labs/ros2_ws/install/setup.bash ] && source ~/labs/ros2_ws/install/setup.bash' >> ~/.bashrc
WORKDIR /home/${USERNAME}/labs/ros2_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ----------------------------------------------------------------------------------------
FROM deps AS final
ARG ROS_DISTRO=jazzy
ARG USERNAME=ros

COPY --chown=${USERNAME}:${USERNAME} ros2_ws/src /home/${USERNAME}/labs/ros2_ws/src
COPY --chown=${USERNAME}:${USERNAME} ros2_ws/tools /home/${USERNAME}/labs/ros2_ws/tools
COPY --chown=${USERNAME}:${USERNAME} ros2_ws/check_config_sync.py /home/${USERNAME}/labs/ros2_ws/check_config_sync.py
COPY --chown=${USERNAME}:${USERNAME} config /home/${USERNAME}/labs/config
# The canonical protocol module, so karmel_base can test that its copy is identical.
COPY --chown=${USERNAME}:${USERNAME} python/robotlab/protocol.py /home/${USERNAME}/labs/python/robotlab/protocol.py

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct- \
       --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
