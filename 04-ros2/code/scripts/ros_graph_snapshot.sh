#!/usr/bin/env bash
# ros_graph_snapshot.sh — save the current ROS 2 graph to a text file (lesson 04.03).
# Nodes, topics, services and actions with their types, plus every node's info and parameters.
# Usage:  ros_graph_snapshot.sh [output_file]       (ROS must be sourced)
# Diff two snapshots to see what changed:  diff before.txt after.txt

if ! command -v ros2 >/dev/null; then
  echo "ros2 not found: source /opt/ros/jazzy/setup.bash first" >&2
  exit 1
fi

out="${1:-ros_graph_$(date +%Y%m%d_%H%M%S).txt}"
{
  echo "# ROS graph snapshot $(date -Iseconds)"
  echo "# ROS_DISTRO=${ROS_DISTRO} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}"
  echo "## nodes";    ros2 node list
  echo "## topics";   ros2 topic list -t
  echo "## services"; ros2 service list -t
  echo "## actions";  ros2 action list -t
  for node in $(ros2 node list); do
    echo "## node ${node}"
    ros2 node info "${node}"
    echo "### parameters of ${node}"
    ros2 param dump "${node}"
  done
} > "${out}" 2>&1

echo "wrote ${out}: $(ros2 node list | wc -l) nodes, $(ros2 topic list | wc -l) topics, $(wc -l < "${out}") lines"
