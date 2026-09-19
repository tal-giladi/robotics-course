# ROS 2 packages for the SO-101 arm (lessons 14.08–14.11)

Two ordinary ament packages. They live here rather than in `labs/ros2_ws/src/` because the arm is
optional hardware — copy or symlink them into the workspace when you reach
[14.08](../../14.08-arm-urdf-and-ros2-control.md).

```text
so101_description/          ament_cmake
  urdf/so101.urdf.xacro           links, joints and limits, copied from data/so101_kinematics.yaml
  urdf/so101.ros2_control.xacro   the three hardware variants (mock | topic | gazebo), one macro
  launch/display.launch.py        URDF + sliders in RViz, no controllers

so101_bringup/              ament_python
  config/controllers.yaml               joint_state_broadcaster, arm_controller, gripper_controller
  so101_bringup/joint_map.py            URDF radians <-> LeRobot degrees/percent  (no ROS: unit-tested)
  so101_bringup/joint_bridge.py         the rclpy servo driver behind TopicBasedSystem
  launch/arm.launch.py                  bring the whole stack up
```

## Build and run

```bash
cp -r 14-robotic-arm/code/ros2/* labs/ros2_ws/src/
cd labs/ros2_ws
colcon build --symlink-install --packages-select so101_description so101_bringup
source install/setup.bash

ros2 launch so101_description display.launch.py     # just the model, with sliders
ros2 launch so101_bringup arm.launch.py             # mock hardware + the three controllers
ros2 launch so101_bringup arm.launch.py hardware:=topic \
     port:=/dev/ttyACM0 calibration:=$HOME/so101_joint_map.json
```

`joint_bridge` imports `servo_tools` and `so101_bus` from `14-robotic-arm/code`, which are
deliberately ROS-free. Put that directory on `PYTHONPATH`, or copy the two files into the package.

> [!CAUTION]
> `hardware:=topic` energises the arm. Read [14.11](../../14.11-arm-safety.md) first: clamp the
> base, clear the workspace, keep the PSU switch in reach. The arm goes **limp and falls** when
> the driver exits.

## Version-sensitive

Verified 2026-09 against ROS 2 Jazzy, `ros2_control` 4.48.x, `ros2_controllers` 4.42.x.

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
                 ros-jazzy-topic-based-ros2-control ros-jazzy-xacro
```

Controller type strings and parameter names move between distros — on Jazzy
`position_controllers/GripperActionController` is deprecated in favour of
`parallel_gripper_action_controller/GripperActionController`, and `open_loop_control` has been
replaced by `interpolate_from_desired_state`. Check https://control.ros.org/jazzy/index.html
before copying any YAML.

## Checking the URDF

The xacro's numbers are copied from `../data/so101_kinematics.yaml`. Prove they have not drifted:

```bash
xacro so101_description/urdf/so101.urdf.xacro hardware:=mock > /tmp/so101.urdf
check_urdf /tmp/so101.urdf
cd 14-robotic-arm/code && py -c "
from urdf_chain import chain_from_urdf
import arm_kinematics as ak, numpy as np
a, b = chain_from_urdf('/tmp/so101.urdf', 'base_link', 'gripper_frame_link'), ak.load_so101()
rng = np.random.default_rng(0)
print(max(float(np.abs(a.fk(q) - b.fk(q)).max()) for q in rng.uniform(b.lower, b.upper, size=(300, 5))))
"
```

It prints `0.0` — bit-for-bit identical, for all three `hardware:=` values. Exercise
[14.08-E3](../../14.08-arm-urdf-and-ros2-control.md) turns that into a test.
