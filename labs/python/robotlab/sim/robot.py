"""The simulated differential-drive robot: parameters, physics step and sensor access.

Two kinds of parameters:

* :class:`DiffDriveParams` — the body: motors, wheels, encoders, battery, firmware PID. The
  ``wheel_radius_m`` / ``wheel_separation_m`` fields are the *nominal* values from karmel.yaml
  (what your odometry believes); the ``*_scale`` and ``motor_gain_*`` knobs make the *true*
  robot different, for calibration and drift labs.
* :class:`SensorParams` — sensor mounts, ranges and noise.

Both come in two presets built from the config: ``ideal()`` (no mismatch, no noise — motor lag,
deadband, saturation and encoder quantization remain, because they are physics) and
``realistic()``. Tweak any field with :func:`dataclasses.replace`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from numpy.typing import NDArray

from robotlab.config import KarmelConfig, load_config
from robotlab.geometry import SE2, angle_diff
from robotlab.sim import sensors
from robotlab.sim.components import Battery, WheelVelocityController, deadband_speed, first_order_alpha, low_pass
from robotlab.sim.world import World

TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class DiffDriveParams:
    """Physical parameters of the simulated base (SI units)."""

    wheel_radius_m: float  # nominal
    wheel_separation_m: float  # nominal
    ticks_per_wheel_rev: int
    max_wheel_speed_rad_s: float  # steady state at |duty| = 1 and battery_nominal_v
    motor_time_constant_s: float
    duty_deadband: float
    robot_radius_m: float  # collision disc around base_link
    # imperfections: 1.0 / 0.0 means perfect
    motor_gain_left: float = 1.0
    motor_gain_right: float = 1.0
    wheel_radius_scale_left: float = 1.0  # true radius = nominal * scale
    wheel_radius_scale_right: float = 1.0
    wheel_separation_scale: float = 1.0
    slip_std: float = 0.0  # per-step multiplicative noise on each wheel's ground travel
    encoder_bits: int | None = None  # wrap tick counters like an N-bit hardware counter
    # battery
    battery_nominal_v: float = 11.1
    battery_full_v: float = 12.6
    battery_empty_v: float = 9.0
    battery_low_v: float = 10.5
    battery_capacity_ah: float = 3.5
    battery_internal_resistance_ohm: float = 0.0
    battery_initial_soc: float = 0.9
    idle_current_a: float = 0.6  # Raspberry Pi + sensors, drawn from the pack
    motor_current_a: float = 0.8  # per motor at full duty
    # firmware velocity loop (defaults = labs/firmware/pico/config.py)
    velocity_filter_alpha: float = 0.5  # low-pass on the tick-based speed estimate, per step
    velocity_kp: float = 0.02
    velocity_ki: float = 0.3
    velocity_kd: float = 0.0
    velocity_ff: float = 1.0

    @property
    def true_wheel_radius_left_m(self) -> float:
        return self.wheel_radius_m * self.wheel_radius_scale_left

    @property
    def true_wheel_radius_right_m(self) -> float:
        return self.wheel_radius_m * self.wheel_radius_scale_right

    @property
    def true_wheel_separation_m(self) -> float:
        return self.wheel_separation_m * self.wheel_separation_scale

    @classmethod
    def ideal(cls, config: KarmelConfig | None = None) -> DiffDriveParams:
        """The robot exactly as karmel.yaml describes it."""
        cfg = config or load_config()
        d, b = cfg.drive, cfg.battery
        return cls(
            wheel_radius_m=d.wheel_radius_m,
            wheel_separation_m=d.wheel_separation_m,
            ticks_per_wheel_rev=d.ticks_per_wheel_rev,
            max_wheel_speed_rad_s=d.max_wheel_speed_rad_s,
            motor_time_constant_s=d.motor_time_constant_s,
            duty_deadband=d.duty_deadband,
            robot_radius_m=cfg.chassis.footprint_radius_m,
            battery_nominal_v=b.nominal_v,
            battery_full_v=b.full_v,
            battery_empty_v=3.0 * b.cells_series,
            battery_low_v=b.low_warning_v,
            battery_capacity_ah=b.capacity_ah,
        )

    @classmethod
    def realistic(cls, config: KarmelConfig | None = None) -> DiffDriveParams:
        """A robot like the one on your desk: mismatched motors and wheels, slip, battery sag."""
        return replace(
            cls.ideal(config),
            motor_gain_left=1.0,
            motor_gain_right=0.94,
            wheel_radius_scale_left=0.995,
            wheel_radius_scale_right=1.008,
            wheel_separation_scale=1.04,
            slip_std=0.02,
            battery_internal_resistance_ohm=0.15,
        )


@dataclass(frozen=True)
class SensorParams:
    """Sensor mounts (x, y in base_link), limits and noise."""

    range_x_m: float
    range_y_m: float
    range_max_m: float
    range_fov_rad: float
    lidar_x_m: float
    lidar_y_m: float
    lidar_samples: int
    lidar_max_range_m: float
    landmark_fov_rad: float
    range_rays: int = 5  # the ToF cone is approximated by this many rays
    range_noise_std_m: float = 0.0
    lidar_min_range_m: float = 0.15
    lidar_noise_std_m: float = 0.0
    lidar_dropout_prob: float = 0.0
    landmark_max_range_m: float = 4.0
    landmark_range_std_m: float = 0.0
    landmark_bearing_std_rad: float = 0.0
    gyro_bias_rad_s: float = 0.0
    gyro_noise_std_rad_s: float = 0.0

    @classmethod
    def ideal(cls, config: KarmelConfig | None = None) -> SensorParams:
        """Noise-free sensors mounted as in karmel.yaml."""
        s = (config or load_config()).sensors
        return cls(
            range_x_m=s.range_front.x_m,
            range_y_m=s.range_front.y_m,
            range_max_m=s.range_front.max_range_m,
            range_fov_rad=s.range_front.fov_rad,
            lidar_x_m=s.lidar.x_m,
            lidar_y_m=s.lidar.y_m,
            lidar_samples=s.lidar.samples,
            lidar_max_range_m=s.lidar.max_range_m,
            landmark_fov_rad=s.camera.horizontal_fov_rad,
        )

    @classmethod
    def realistic(cls, config: KarmelConfig | None = None) -> SensorParams:
        """Typical hobby-grade noise: VL53L1X, RPLIDAR-class scanner, AprilTags, MPU-class gyro."""
        return replace(
            cls.ideal(config),
            range_noise_std_m=0.01,
            lidar_noise_std_m=0.01,
            lidar_dropout_prob=0.02,
            landmark_range_std_m=0.05,
            landmark_bearing_std_rad=0.03,
            gyro_bias_rad_s=0.01,
            gyro_noise_std_rad_s=0.005,
        )


def arc_update(pose: SE2, d_left: float, d_right: float, wheel_separation: float) -> SE2:
    """Move ``pose`` along the exact circular arc traced by the wheel travels ``d_left``/``d_right``."""
    ds = 0.5 * (d_left + d_right)  # distance travelled by base_link
    dtheta = (d_right - d_left) / wheel_separation  # heading change
    if abs(dtheta) < 1e-9:  # straight line: the arc radius is infinite
        dx, dy = ds * math.cos(pose.theta), ds * math.sin(pose.theta)
    else:
        radius = ds / dtheta
        dx = radius * (math.sin(pose.theta + dtheta) - math.sin(pose.theta))
        dy = -radius * (math.cos(pose.theta + dtheta) - math.cos(pose.theta))
    return SE2(pose.x + dx, pose.y + dy, pose.theta + dtheta)


class DiffDriveSim:
    """A differential-drive robot moving in a :class:`World`.

    Ground truth is public (``pose``, ``wheel_rad_s``, ``yaw_rate``, ``collided``); the sensor
    methods return what a real robot would measure.
    """

    def __init__(
        self,
        world: World | None = None,
        params: DiffDriveParams | None = None,
        sensor_params: SensorParams | None = None,
        pose: SE2 | tuple[float, float, float] = (0.0, 0.0, 0.0),
        seed: int | None = None,
        config: KarmelConfig | None = None,
    ) -> None:
        if params is None or sensor_params is None:
            config = config or load_config()
        self.world = world if world is not None else World()
        self.params = params or DiffDriveParams.ideal(config)
        self.sensor_params = sensor_params or SensorParams.ideal(config)
        self.rng = np.random.default_rng(seed)
        self.reset(pose)

    def reset(self, pose: SE2 | tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        """Put the robot at ``pose``, at rest, with zeroed encoders and a fresh battery."""
        p = self.params
        self.t = 0.0
        self.pose = SE2.from_tuple(pose)
        self.wheel_rad_s = np.zeros(2)  # true wheel speeds [left, right]
        self.yaw_rate = 0.0  # true body yaw rate
        self.collided = False
        self.duty = np.zeros(2)
        self.velocity_setpoint: NDArray[np.floating] | None = None  # None = open-loop duty mode
        self._wheel_angle = np.zeros(2)  # total rotation of each wheel, rad
        self._wheel_estimate = np.zeros(2)  # encoder-based speed estimate, rad/s
        self._true_radii = np.array([p.true_wheel_radius_left_m, p.true_wheel_radius_right_m])
        self._motor_gains = np.array([p.motor_gain_left, p.motor_gain_right])
        self._controller = WheelVelocityController(
            p.velocity_kp, p.velocity_ki, p.velocity_kd, p.velocity_ff, p.max_wheel_speed_rad_s, p.duty_deadband
        )
        self._battery = Battery(
            p.battery_full_v, p.battery_empty_v, p.battery_capacity_ah, p.battery_internal_resistance_ohm,
            p.idle_current_a, p.motor_current_a, soc=p.battery_initial_soc,
        )

    # --- commands --------------------------------------------------------------------------------
    def set_duty(self, left: float, right: float) -> None:
        """Open-loop motor duty, each clipped to -1..1."""
        self.velocity_setpoint = None
        self.duty = np.clip([left, right], -1.0, 1.0).astype(float)

    def set_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        """Closed-loop wheel speed setpoints, tracked by the simulated firmware PID."""
        if self.velocity_setpoint is None:
            self._controller.reset()
        self.velocity_setpoint = np.array([left_rad_s, right_rad_s], dtype=float)

    def stop(self) -> None:
        self.set_duty(0.0, 0.0)

    # --- physics ---------------------------------------------------------------------------------
    def step(self, dt: float) -> None:
        """Advance the simulation by one time step of ``dt`` seconds."""
        p = self.params

        # 1. Motor command. In velocity mode the simulated firmware PID picks the duty from the
        #    same encoder-based speed estimate the real Pico has.
        if self.velocity_setpoint is not None:
            self.duty = self._controller.update(self.velocity_setpoint, self._wheel_estimate, dt)

        # 2. Duty -> the speed each wheel would settle at: deadband, saturation, motor gain
        #    mismatch, and proportionally less speed when the battery voltage sags.
        volts = self._battery.voltage / p.battery_nominal_v
        target = deadband_speed(self.duty, p.duty_deadband, p.max_wheel_speed_rad_s) * self._motor_gains * volts

        # 3. First-order motor lag (exact discretization); use the average speed over the step.
        w_prev = self.wheel_rad_s
        w_next = w_prev + (target - w_prev) * first_order_alpha(dt, p.motor_time_constant_s)
        wheel_turn = 0.5 * (w_prev + w_next) * dt  # rad each wheel turned

        # 4. Wheel rotation -> ground travel, using the TRUE radii and random slip.
        slip = 1.0 + self.rng.normal(0.0, p.slip_std, 2) if p.slip_std > 0 else 1.0
        travel = wheel_turn * self._true_radii * slip

        # 5. Move along the exact arc, unless that drives into an obstacle: then the wheels stall.
        new_pose = arc_update(self.pose, travel[0], travel[1], p.true_wheel_separation_m)
        self.collided = self._blocked(new_pose)
        if self.collided:
            new_pose, w_next, wheel_turn = self.pose, np.zeros(2), np.zeros(2)
        self.yaw_rate = angle_diff(new_pose.theta, self.pose.theta) / dt
        self.pose, self.wheel_rad_s = new_pose, w_next

        # 6. Encoders count wheel rotation (slip fools them) in whole ticks; the speed estimate
        #    is low-pass-filtered ticks-per-step, like the firmware computes it.
        ticks_before = self._unwrapped_ticks()
        self._wheel_angle = self._wheel_angle + wheel_turn
        raw_speed = (self._unwrapped_ticks() - ticks_before) * TWO_PI / p.ticks_per_wheel_rev / dt
        self._wheel_estimate = low_pass(self._wheel_estimate, raw_speed, p.velocity_filter_alpha)

        # 7. Battery drain and the clock.
        self._battery.update(self.duty, dt)
        self.t += dt

    def advance(self, duration: float, dt: float = 0.02) -> None:
        """Call :meth:`step` repeatedly for ``duration`` seconds."""
        for _ in range(max(0, round(duration / dt))):
            self.step(dt)

    def _blocked(self, new_pose: SE2) -> bool:
        """Reject a move that ends in contact and doesn't increase clearance (so you can back out)."""
        clearance = self.world.distance_to_obstacles([[new_pose.x, new_pose.y], [self.pose.x, self.pose.y]])
        return bool(clearance[0] < self.params.robot_radius_m and clearance[0] <= clearance[1])

    def _unwrapped_ticks(self) -> NDArray[np.int64]:
        revolutions = self._wheel_angle / TWO_PI
        return np.floor(revolutions * self.params.ticks_per_wheel_rev + 1e-9).astype(np.int64)

    # --- proprioception --------------------------------------------------------------------------
    @property
    def ticks(self) -> tuple[int, int]:
        """Cumulative encoder counts (left, right), wrapped if ``params.encoder_bits`` is set."""
        raw = self._unwrapped_ticks()
        bits = self.params.encoder_bits
        if bits is not None:
            half = 1 << (bits - 1)
            raw = (raw + half) % (1 << bits) - half
        return int(raw[0]), int(raw[1])

    @property
    def wheel_velocity_estimate(self) -> tuple[float, float]:
        """Encoder-derived wheel speeds over the last step (rad/s), as the firmware reports them."""
        return float(self._wheel_estimate[0]), float(self._wheel_estimate[1])

    @property
    def battery_v(self) -> float:
        return self._battery.voltage

    @property
    def velocity_mode(self) -> bool:
        return self.velocity_setpoint is not None

    # --- exteroception ---------------------------------------------------------------------------
    def gyro_z(self) -> float:
        """Yaw-rate gyro sample (rad/s): true rate + bias + noise."""
        s = self.sensor_params
        return sensors.gyro_z(self.yaw_rate, s.gyro_bias_rad_s, s.gyro_noise_std_rad_s, self.rng)

    def front_range(self) -> float | None:
        """Front ToF distance from the sensor (m), or ``None`` when out of range."""
        s = self.sensor_params
        mount = self.pose @ SE2(s.range_x_m, s.range_y_m, 0.0)
        return sensors.cone_range(
            self.world, mount, s.range_fov_rad, s.range_rays, s.range_max_m, s.range_noise_std_m, self.rng
        )

    def lidar_scan(self) -> sensors.LaserScan:
        """A full 360-degree scan in the LiDAR frame (see :class:`LaserScan` for inf/nan)."""
        s = self.sensor_params
        return sensors.lidar_scan(
            self.world, self.lidar_pose, s.lidar_samples, s.lidar_min_range_m, s.lidar_max_range_m,
            s.lidar_noise_std_m, s.lidar_dropout_prob, self.rng, stamp=self.t,
        )

    @property
    def lidar_pose(self) -> SE2:
        """True world pose of the LiDAR frame (use it to plot scan points)."""
        return self.pose @ SE2(self.sensor_params.lidar_x_m, self.sensor_params.lidar_y_m, 0.0)

    def observe_landmarks(self) -> list[sensors.LandmarkObservation]:
        """Range-bearing to every visible landmark, measured from base_link."""
        s = self.sensor_params
        return sensors.observe_landmarks(
            self.world, self.pose, s.landmark_max_range_m, s.landmark_fov_rad,
            s.landmark_range_std_m, s.landmark_bearing_std_rad, self.rng,
        )
