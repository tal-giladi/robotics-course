"""Reference solution for 03.05 — A fake DifferentialBase that passes the conformance suite.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math

from robotlab.hal import FLAG_VELOCITY_MODE, FLAG_WATCHDOG, BaseState, DifferentialBase


class FakeBase:
    """An in-memory, ideal differential base: wheels reach their commanded speed instantly."""

    def __init__(
        self,
        *,
        ticks_per_rev: int = 2464,
        max_wheel_speed_rad_s: float = 17.0,
        dt: float = 0.02,
        watchdog_s: float | None = 0.3,
        battery_v: float | None = 12.0,
        range_m: float | None = None,
    ) -> None:
        self.ticks_per_rev = ticks_per_rev
        self.max_wheel_speed_rad_s = max_wheel_speed_rad_s
        self.dt = dt
        self.watchdog_s = watchdog_s
        self.battery_v = battery_v
        self.range_m = range_m
        self.commands: list[tuple[str, float, float]] = []
        self.t = 0.0
        self._angle = [0.0, 0.0]  # wheel angles, rad
        self._speed = [0.0, 0.0]  # rad/s
        self._velocity_mode = False
        self._last_command_t = 0.0
        self._watchdog_tripped = False
        self._closed = False

    # --- DifferentialBase ------------------------------------------------------------------------
    def set_wheel_duty(self, left: float, right: float) -> None:
        self._command("duty", left, right)
        clamp = lambda d: max(-1.0, min(1.0, d))  # noqa: E731
        self._speed = [clamp(left) * self.max_wheel_speed_rad_s, clamp(right) * self.max_wheel_speed_rad_s]
        self._velocity_mode = False

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        self._command("velocity", left_rad_s, right_rad_s)
        limit = self.max_wheel_speed_rad_s
        self._speed = [max(-limit, min(limit, left_rad_s)), max(-limit, min(limit, right_rad_s))]
        self._velocity_mode = True

    def stop(self) -> None:
        self._check_open()
        self.commands.append(("stop", 0.0, 0.0))
        self._speed = [0.0, 0.0]
        self._velocity_mode = False

    def read(self) -> BaseState:
        self._check_open()
        if (
            self.watchdog_s is not None
            and not self._watchdog_tripped
            and self.t - self._last_command_t >= self.watchdog_s - 1e-9
        ):
            self._speed = [0.0, 0.0]
            self._velocity_mode = False
            self._watchdog_tripped = True
        self.t += self.dt
        for i in range(2):
            self._angle[i] += self._speed[i] * self.dt
        flags = (FLAG_WATCHDOG if self._watchdog_tripped else 0) | (FLAG_VELOCITY_MODE if self._velocity_mode else 0)
        return BaseState(
            t=self.t,
            left_ticks=round(self._angle[0] / (2 * math.pi) * self.ticks_per_rev),
            right_ticks=round(self._angle[1] / (2 * math.pi) * self.ticks_per_rev),
            left_rad_s=self._speed[0],
            right_rad_s=self._speed[1],
            battery_v=self.battery_v,
            range_m=self.range_m,
            flags=flags,
        )

    def close(self) -> None:
        if not self._closed:
            self._speed = [0.0, 0.0]
            self._closed = True

    def __enter__(self) -> FakeBase:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # --- internals ---------------------------------------------------------------------------------
    def _command(self, kind: str, left: float, right: float) -> None:
        self._check_open()
        if not (math.isfinite(left) and math.isfinite(right)):
            raise ValueError(f"{kind} command must be finite, got ({left!r}, {right!r})")
        self.commands.append((kind, left, right))
        self._last_command_t = self.t
        self._watchdog_tripped = False

    def _check_open(self) -> None:
        if self._closed:
            raise RuntimeError("FakeBase is closed")


def drive_distance(
    base: DifferentialBase,
    distance_m: float,
    wheel_speed_rad_s: float,
    wheel_radius_m: float,
    ticks_per_rev: int,
    timeout_s: float = 30.0,
) -> float:
    """Drive straight until the mean wheel travel reaches ``distance_m``, then stop.

    Works on any DifferentialBase. Returns the distance travelled by the encoders' account.
    ``timeout_s`` is measured in robot time (``BaseState.t``), so it works in lockstep simulation too.
    """
    if distance_m == 0:
        return 0.0
    if wheel_speed_rad_s <= 0:
        raise ValueError("wheel_speed_rad_s must be positive; the sign comes from distance_m")
    speed = math.copysign(wheel_speed_rad_s, distance_m)
    meters_per_tick = 2 * math.pi * wheel_radius_m / ticks_per_rev
    start = base.read()
    travelled = 0.0
    try:
        while True:
            base.set_wheel_velocity(speed, speed)  # every cycle: feeds the watchdog
            state = base.read()
            ticks = ((state.left_ticks - start.left_ticks) + (state.right_ticks - start.right_ticks)) / 2
            travelled = ticks * meters_per_tick
            if abs(travelled) >= abs(distance_m):
                return travelled
            if state.t - start.t > timeout_s:
                raise TimeoutError(f"drove {travelled:.3f} m of {distance_m:.3f} m in {timeout_s} s")
    finally:
        base.stop()  # SAFETY: always stop, also on exceptions

