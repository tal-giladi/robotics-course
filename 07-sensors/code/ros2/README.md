# 07-sensors/code/ros2 — sensor messages in ROS 2, without the sensors

Everything here supports lessons [07.06](../../07.06-imu-in-ros2.md) to
[07.11](../../07.11-sensor-troubleshooting.md). The point is that you can learn
`sensor_msgs/Imu`, `LaserScan`, `Image`, `CameraInfo` and `PointCloud2` — their fields, their
conventions, their failure modes — before any of that hardware exists, and then recognise the same
symptoms the day it does.

| File | Needs rclpy | What it is |
|---|---|---|
| `fake_imu.py` | yes | synthetic `Imu` + `MagneticField` at 100 Hz, with 7 injectable faults |
| `fake_lidar.py` | yes | synthetic 360-beam `LaserScan` of a rectangular room, 6 injectable faults |
| `fake_camera.py` | yes | synthetic `Image` + `CameraInfo`, 5 injectable faults |
| `fake_depth.py` | yes | synthetic depth `Image` (16UC1/32FC1) + organised `PointCloud2`, 4 faults |
| `sensor_check.py` | yes | audit any of those topics: rate, jitter, stamp age, frame, units, covariance |
| `sync_demo.py` | yes | `message_filters` ApproximateTime vs ExactTime, live |
| `qos_demo.py` | yes | publisher and subscriber with mismatched QoS, in one process |
| `scan_geometry.py` | no | LaserScan geometry: angles, polar→Cartesian, sector minima, resolution |
| `imu_conventions.py` | no | quaternions (x, y, z, w!), covariance, axis remap, gravity check |
| `sync_math.py` | no | stamp pairing, rate/jitter/gap report, stamp-age statistics |
| `test_ros2_code.py` | no | tests for the three ROS-free modules |

## Run the ROS-free tests anywhere

```bash
py -m pytest 07-sensors/code/ros2        # 20 tests, no ROS, no hardware
```

## Run the nodes

They need ROS 2 Jazzy, so run them in the course Docker image
([`labs/docker`](../../../labs/docker/README.md)) or on Ubuntu 24.04 with Jazzy installed. They are
plain scripts, not a ROS package — `python3 <file>`, not `ros2 run`.

```bash
cd labs
docker compose -f docker/compose.yaml run --rm wslg      # or your own container
# inside:
source /opt/ros/jazzy/setup.bash
cd /path/to/robotics-course/07-sensors/code/ros2
python3 fake_imu.py --yaw-rate 0.5 &
python3 sensor_check.py /imu/data_raw --expect-hz 100 --frame imu_link --seconds 5
```

Every script takes `--help`, and anything after `--ros-args` is passed to ROS
(`--ros-args -p use_sim_time:=true`, `--ros-args -r /scan:=/scan_filtered`, …).

> When you run several ROS containers on one machine, give each an isolated DDS domain
> (`-e ROS_DOMAIN_ID=<1..101>`) or no network at all (`--network none`), or your nodes will
> discover someone else's robot.
