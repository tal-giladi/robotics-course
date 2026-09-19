# Replaying the simulated tour into real ROS 2 nodes (10.07 and 10.09)

These two scripts let you exercise the *real* `robot_localization` and Nav2 AMCL binaries against the
`robotlab` simulator, headlessly, without Gazebo. Everything the lessons quote as an "Expected result"
was produced this way.

`rclpy` only exists inside a ROS 2 environment, so these files are never imported by
`python -m pytest 10-localization/code`.

## 1. Export the data (anywhere Python + numpy runs)

```bash
python 10-localization/code/export_replay.py --out localization_out/replay
```

```text
map   : 132 x 112 cells at 0.05 m, origin (-0.3, -0.3), 2195 occupied / 12589 free
tour  : 49.8 s, 2493 steps, 249 scans of 360 beams
truth : start (1.0, 1.3, 0.0) -> end (1.002, 1.417, -1.596)
odom  : end (0.954, 2.23, -2.16)  (dead reckoning RMSE 46.9 cm, final 81.4 cm)
```

`apartment.yaml` + `apartment.pgm` are a plain `map_server` map — the same format `map_saver_cli`
writes, so you can open them in any ROS tool.

## 2. A container to run them in

Use the course image if you built it, or the stock one:

```bash
docker run -d --name loc10-lab --network none \
  -e ROS_DOMAIN_ID=77 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$PWD:/course" ros:jazzy-ros-base sleep infinity
docker exec loc10-lab bash -lc \
  'apt-get update && apt-get install -y ros-jazzy-robot-localization ros-jazzy-nav2-amcl \
     ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager python3-numpy'
```

> `--network none` plus a private `ROS_DOMAIN_ID` keeps this container's DDS traffic away from any
> other ROS 2 you are running. Remove only the container you created: `docker rm -f loc10-lab`.

## 3. robot_localization (lesson 10.07)

```bash
docker exec loc10-lab bash -lc '
  source /opt/ros/jazzy/setup.bash
  ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node \
    --params-file /course/labs/ros2_ws/src/karmel_bringup/config/ekf.yaml -p use_sim_time:=false &
  sleep 4
  python3 /course/10-localization/code/ros2/replay_odom_imu.py --data /course/localization_out/replay'
```

Add `--zero-cov --label zerocov` for the "driver forgot the covariances" run.

## 4. map_server + AMCL (lesson 10.09)

```bash
docker exec loc10-lab bash -lc '
  source /opt/ros/jazzy/setup.bash
  D=/course/localization_out/replay
  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$D/apartment.yaml -p use_sim_time:=false &
  ros2 run nav2_amcl amcl --ros-args \
    --params-file /course/labs/ros2_ws/src/karmel_bringup/config/nav2_params.yaml -p use_sim_time:=false &
  sleep 3
  ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true \
    -p use_sim_time:=false -p "node_names:=[\"map_server\",\"amcl\"]" &
  sleep 6
  ros2 lifecycle get /map_server; ros2 lifecycle get /amcl
  python3 /course/10-localization/code/ros2/replay_scan.py --data $D --label good --init 1.0 1.3 0.0'
```

Both lifecycle nodes must print `active [3]` before the replay starts. Change `--init` to move the
initial pose: `--init 2.0 2.3 1.05` (1.4 m and 60° wrong) or `--init 4.8 4.2 0.0` (the wrong room).

Each run also writes a CSV next to the data (`amcl_good.csv`, `rl_fuse.csv`, …) with the estimate, the
truth, the per-sample errors and the reported covariance, so you can plot them yourself.
