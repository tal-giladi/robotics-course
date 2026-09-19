#!/usr/bin/env python3
"""Measure the noise of karmel's simulated sensors while the robot stands still.

    ros2 run ... : python3 sensor_stats.py --ros-args -p use_sim_time:=true

Collects N messages from /scan, /imu and /camera/camera_info and prints, for each,
what the simulator promised (the <noise> block in karmel.gazebo.xacro) next to what it
actually delivered.
"""
import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

N_SCANS = 60
N_IMU = 400


class Collector(Node):
    def __init__(self):
        super().__init__('sensor_stats')
        self.scans, self.imus, self.infos, self.images = [], [], [], []
        self.create_subscription(LaserScan, '/scan', self.scans.append, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.imus.append, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.infos.append, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.images.append, qos_profile_sensor_data)

    def done(self):
        return len(self.scans) >= N_SCANS and len(self.imus) >= N_IMU and self.infos and self.images


def main():
    rclpy.init()
    node = Collector()
    for _ in range(4000):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.done():
            break

    s = node.scans
    print(f'--- /scan  ({len(s)} messages) ---')
    if s:
        first = s[0]
        n = len(first.ranges)
        print(f'frame_id={first.header.frame_id}  samples={n}  '
              f'angle_min={first.angle_min:.5f} angle_max={first.angle_max:.5f} '
              f'increment={first.angle_increment:.5f} rad ({math.degrees(first.angle_increment):.3f} deg)')
        print(f'range_min={first.range_min} range_max={first.range_max}')
        finite = [i for i in range(n)
                  if all(math.isfinite(m.ranges[i]) and m.ranges[i] < m.range_max for m in s)]
        stds = [statistics.pstdev([m.ranges[i] for m in s]) for i in finite]
        means = {i: statistics.fmean([m.ranges[i] for m in s]) for i in finite}
        print(f'rays with a finite return in every scan: {len(finite)} / {n}')
        print(f'per-ray std dev: median {statistics.median(stds)*1000:.2f} mm, '
              f'min {min(stds)*1000:.2f} mm, max {max(stds)*1000:.2f} mm  (configured stddev = 10 mm)')
        ahead = n // 2      # angle 0 = straight ahead, because angle_min = -pi
        print(f'ray {ahead} (straight ahead): mean {means.get(ahead, float("nan")):.4f} m, '
              f'std {statistics.pstdev([m.ranges[ahead] for m in s])*1000:.2f} mm')
        inf_rays = sum(1 for i in range(n) if not math.isfinite(first.ranges[i]))
        print(f'rays reported as inf (nothing within range_max) in scan 0: {inf_rays}')

    im = node.imus
    print(f'\n--- /imu  ({len(im)} messages) ---')
    if im:
        print(f'frame_id={im[0].header.frame_id}')
        for axis in ('x', 'y', 'z'):
            w = [getattr(m.angular_velocity, axis) for m in im]
            print(f'  angular_velocity.{axis}: mean {statistics.fmean(w):+.5f} rad/s, '
                  f'std {statistics.pstdev(w):.5f} rad/s   (configured stddev 2e-3, bias_mean 1e-4)')
        for axis in ('x', 'y', 'z'):
            a = [getattr(m.linear_acceleration, axis) for m in im]
            print(f'  linear_acceleration.{axis}: mean {statistics.fmean(a):+.4f} m/s2, '
                  f'std {statistics.pstdev(a):.4f} m/s2  (configured stddev 1.7e-2, bias_mean 0.05)')
        print(f'  orientation covariance[0] = {im[0].orientation_covariance[0]}')

    if node.infos:
        ci = node.infos[0]
        print(f'\n--- /camera/camera_info ---')
        print(f'frame_id={ci.header.frame_id}  {ci.width}x{ci.height}  distortion_model={ci.distortion_model!r}')
        print(f'  K = [{ci.k[0]:.2f} {ci.k[1]:.2f} {ci.k[2]:.2f}; {ci.k[3]:.2f} {ci.k[4]:.2f} {ci.k[5]:.2f}; '
              f'{ci.k[6]:.2f} {ci.k[7]:.2f} {ci.k[8]:.2f}]')
        print(f'  D = {[round(v, 6) for v in ci.d]}')
        fx = ci.k[0]
        print(f'  implied horizontal FOV = {2*math.atan(ci.width/(2*fx)):.4f} rad '
              f'({math.degrees(2*math.atan(ci.width/(2*fx))):.2f} deg)')
    if node.images:
        img = node.images[0]
        print(f'\n--- /camera/image_raw ---')
        print(f'frame_id={img.header.frame_id}  {img.width}x{img.height} encoding={img.encoding} '
              f'step={img.step} bytes={len(img.data)}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
