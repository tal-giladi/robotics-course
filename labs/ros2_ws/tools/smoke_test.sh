#!/usr/bin/env bash
# Headless end-to-end smoke test of the simulation stack (what TESTED.md records, automated).
#
#   tools/smoke_test.sh            # sim + controllers + sensors + drive + SLAM   (~3-4 min)
#   tools/smoke_test.sh --nav      # ... then Nav2 on the ground-truth map, one NavigateToPose goal
#
# Needs a built and sourced workspace. Uses ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST so it never
# talks to other ROS systems on the network. Logs go to $LOG_DIR (default /tmp/karmel_smoke).
set -uo pipefail

NAV=false
[[ "${1:-}" == "--nav" ]] && NAV=true
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${LOG_DIR:-/tmp/karmel_smoke}"
mkdir -p "$LOG_DIR"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
set +u; source "${KARMEL_WS:-$HOME/labs/ros2_ws}/install/setup.bash"; set -u

PIDS=()
FAILED=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILED=1; }
start() {  # start <name> <command...> in its own process group
  local name=$1; shift
  setsid "$@" > "$LOG_DIR/$name.log" 2>&1 &
  PIDS+=($!)
}
cleanup() {
  for pid in "${PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null; done
  sleep 5
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done
}
trap cleanup EXIT

odom_field() {  # ros2 topic echo may print 'A message was lost!!!' lines; keep the number only
  timeout 15 ros2 topic echo --once /odom --field "$1" 2>/dev/null | grep -E '^-?[0-9]' | head -1
}
odom_x() { odom_field pose.pose.position.x; }

echo "== 1. Gazebo (headless) + controllers, apartment world"
start sim ros2 launch karmel_bringup robot.launch.py sim:=true headless:=true world:=apartment x:=-1.5 y:=-0.5
for _ in $(seq 1 60); do
  [[ $(ros2 control list_controllers 2>/dev/null | grep -c " active") -ge 2 ]] && break
  sleep 3
done
if [[ $(ros2 control list_controllers 2>/dev/null | grep -c " active") -ge 2 ]]; then
  pass "joint_state_broadcaster and diff_drive_controller active"
else
  fail "controllers not active (see $LOG_DIR/sim.log)"; exit 1
fi

echo "== 2. Sensors"
rate=$(timeout 15 ros2 topic hz /scan 2>/dev/null | awk '/average rate/ {r=$3} END {print r}')
[[ -n "$rate" ]] && pass "/scan publishing (${rate} Hz wall clock)" || fail "/scan not publishing"
frame=$(timeout 15 ros2 topic echo --once /scan --field header.frame_id 2>/dev/null | grep -Ev 'message was lost|total count' | head -1)
[[ "$frame" == "laser" ]] && pass "/scan frame_id = laser" || fail "/scan frame_id = '$frame'"
frame=$(timeout 15 ros2 topic echo --once /imu --field header.frame_id 2>/dev/null | grep -Ev 'message was lost|total count' | head -1)
[[ "$frame" == "imu_link" ]] && pass "/imu frame_id = imu_link" || fail "/imu frame_id = '$frame'"
frame=$(timeout 30 ros2 topic echo --once /camera/camera_info --field header.frame_id 2>/dev/null | grep -Ev 'message was lost|total count' | head -1)
[[ "$frame" == "camera_optical_frame" ]] && pass "/camera/camera_info frame_id = camera_optical_frame" \
  || fail "/camera/camera_info frame_id = '$frame'"

echo "== 3. Drive with TwistStamped on /cmd_vel"
x0=$(odom_x)
python3 "$HERE/drive_pattern.py" --pattern "0.2,0,4;0,0,1" --ros-args -p use_sim_time:=true > "$LOG_DIR/drive1.log" 2>&1
x1=$(odom_x)
if python3 -c "import sys; sys.exit(0 if float('$x1') - float('$x0') > 0.3 else 1)" 2>/dev/null; then
  pass "/odom x moved from $x0 to $x1"
else
  fail "/odom x did not move enough ($x0 -> $x1)"
fi

echo "== 4. SLAM (slam_toolbox online async)"
start slam ros2 launch karmel_bringup slam.launch.py
sleep 15
python3 "$HERE/drive_pattern.py" --ros-args -p use_sim_time:=true > "$LOG_DIR/drive2.log" 2>&1
width=$(timeout 30 ros2 topic echo --once /map --field info.width 2>/dev/null | grep -Ev 'message was lost|total count' | head -1)
[[ -n "$width" && "$width" -gt 0 ]] && pass "/map published (width $width cells)" || fail "/map not published"

if $NAV; then
  echo "== 5. Nav2 on the ground-truth map"
  kill -INT -- "-${PIDS[1]}" 2>/dev/null; sleep 8
  # odom starts at the spawn pose (-1.5, -0.5, yaw 0) of the map frame, so map pose = spawn + odom
  read -r mx my myaw < <(python3 - <<'PY'
import math, rclpy
from nav_msgs.msg import Odometry
rclpy.init(); node = rclpy.create_node('odom_probe'); msgs = []
node.create_subscription(Odometry, '/odom', msgs.append, 10)
while not msgs:
    rclpy.spin_once(node, timeout_sec=0.5)
p, q = msgs[0].pose.pose.position, msgs[0].pose.pose.orientation
print(-1.5 + p.x, -0.5 + p.y, 2 * math.atan2(q.z, q.w))
PY
)
  start nav ros2 launch karmel_bringup navigation.launch.py initial_pose:=true x:="$mx" y:="$my" yaw:="$myaw"
  for _ in $(seq 1 40); do
    [[ $(grep -c "Managed nodes are active" "$LOG_DIR/nav.log") -ge 2 ]] && break
    sleep 3
  done
  sleep 10
  result=$(timeout 300 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: -1.8, y: 0.3}, orientation: {w: 1.0}}}}" 2>&1 \
    | grep "Goal finished with status")
  [[ "$result" == *SUCCEEDED* ]] && pass "NavigateToPose ($mx, $my) -> (-1.8, 0.3): $result" \
    || fail "NavigateToPose: ${result:-no result} (see $LOG_DIR/nav.log)"
fi

echo
[[ $FAILED -eq 0 ]] && echo "SMOKE TEST PASSED" || echo "SMOKE TEST FAILED (logs in $LOG_DIR)"
exit $FAILED
