#!/bin/bash
# Source ROS 2 and (if built) the course workspace, then run the given command.
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
WS="${KARMEL_WS:-$HOME/labs/ros2_ws}"
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi
exec "$@"
