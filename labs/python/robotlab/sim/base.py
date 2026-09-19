"""``SimBase`` — the simulator behind the :class:`robotlab.hal.DifferentialBase` interface.

Code written against ``DifferentialBase`` (odometry, PID, teleop) runs on the simulator
unchanged. Two timing modes:

* **lockstep** (default): every :meth:`SimBase.read` advances simulated time by ``dt``. Fast,
  deterministic — ideal for tests and lessons.
* **realtime**: ``read()`` also sleeps so simulated time tracks the wall clock, like the robot's
  50 Hz telemetry stream.

The firmware watchdog is simulated too: if no ``set_wheel_duty``/``set_wheel_velocity`` arrives
for ``watchdog_s`` of simulated time, the motors stop and ``FLAG_WATCHDOG`` is set. Send your
command every loop iteration, as on the real robot (or pass ``watchdog_s=None``).
"""

from __future__ import annotations

import time

from robotlab.geometry import SE2
from robotlab.hal import FLAG_LOW_BATTERY, FLAG_VELOCITY_MODE, FLAG_WATCHDOG, BaseState
from robotlab.sim.robot import DiffDriveSim
from robotlab.sim.sensors import LandmarkObservation, LaserScan


class SimBase:
    """A :class:`DifferentialBase` backed by a :class:`DiffDriveSim`.

    >>> base = SimBase(DiffDriveSim(World.apartment(), pose=SE2(1.0, 1.3, 0.0), seed=0))
    >>> base.set_wheel_velocity(5.0, 5.0)
    >>> state = base.read()           # t = 0.02
    >>> truth = base.sim.pose         # ground truth for comparison
    """

    def __init__(
        self,
        sim: DiffDriveSim | None = None,
        *,
        dt: float = 0.02,
        realtime: bool = False,
        watchdog_s: float | None = 0.3,
    ) -> None:
        self._sim = sim if sim is not None else DiffDriveSim()
        self.dt = dt
        self.realtime = realtime
        self.watchdog_s = watchdog_s
        self._last_command_t = self._sim.t
        self._watchdog_tripped = False
        self._closed = False
        self._wall_start = time.monotonic() - self._sim.t

    @property
    def sim(self) -> DiffDriveSim:
        """The underlying simulator — ground truth (``sim.pose``) and extra sensors."""
        return self._sim

    # --- DifferentialBase ------------------------------------------------------------------------
    def set_wheel_duty(self, left: float, right: float) -> None:
        self._command()
        self._sim.set_duty(left, right)

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        self._command()
        self._sim.set_velocity(left_rad_s, right_rad_s)

    def stop(self) -> None:
        self._check_open()
        self._sim.stop()

    def read(self) -> BaseState:
        self._check_open()
        self._apply_watchdog()
        self._sim.step(self.dt)
        if self.realtime:
            self._sleep_until_sim_time()
        return self._state()

    def close(self) -> None:
        if not self._closed:
            self._sim.stop()
            self._closed = True

    def __enter__(self) -> SimBase:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # --- extra simulated sensors (not part of DifferentialBase) ------------------------------------
    def scan(self) -> LaserScan:
        """A LiDAR scan at the current time (call it at your scan rate, e.g. every 5th read)."""
        return self._sim.lidar_scan()

    def gyro_z(self) -> float:
        return self._sim.gyro_z()

    def landmarks(self) -> list[LandmarkObservation]:
        return self._sim.observe_landmarks()

    @property
    def true_pose(self) -> SE2:
        return self._sim.pose

    # --- internals ---------------------------------------------------------------------------------
    def _command(self) -> None:
        self._check_open()
        self._last_command_t = self._sim.t
        self._watchdog_tripped = False

    def _apply_watchdog(self) -> None:
        if self.watchdog_s is None or self._watchdog_tripped:
            return
        if self._sim.t - self._last_command_t >= self.watchdog_s - 1e-9:
            self._sim.stop()
            self._watchdog_tripped = True

    def _state(self) -> BaseState:
        sim = self._sim
        left_ticks, right_ticks = sim.ticks
        left_rad_s, right_rad_s = sim.wheel_velocity_estimate
        flags = 0
        if self._watchdog_tripped:
            flags |= FLAG_WATCHDOG
        if sim.battery_v < sim.params.battery_low_v:
            flags |= FLAG_LOW_BATTERY
        if sim.velocity_mode:
            flags |= FLAG_VELOCITY_MODE
        return BaseState(
            t=sim.t,
            left_ticks=left_ticks,
            right_ticks=right_ticks,
            left_rad_s=left_rad_s,
            right_rad_s=right_rad_s,
            battery_v=sim.battery_v,
            range_m=sim.front_range(),
            flags=flags,
        )

    def _sleep_until_sim_time(self) -> None:
        delay = self._wall_start + self._sim.t - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    def _check_open(self) -> None:
        if self._closed:
            raise RuntimeError("SimBase is closed")
