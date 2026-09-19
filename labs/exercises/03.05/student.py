"""03.05 — A fake DifferentialBase that passes the conformance suite.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 03.05``.
Standard library + robotlab.hal only.

The contract (``labs/python/robotlab/hal.py``)::

    class DifferentialBase(Protocol):
        def set_wheel_duty(self, left: float, right: float) -> None: ...        # -1..1, open loop
        def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None: ...
        def stop(self) -> None: ...
        def read(self) -> BaseState: ...
        def close(self) -> None: ...

You don't inherit from it: ``isinstance(FakeBase(), DifferentialBase)`` is True as soon as the
five methods exist (``typing.Protocol`` + ``@runtime_checkable`` = structural typing).
"""

from __future__ import annotations

import math

from robotlab.hal import FLAG_VELOCITY_MODE, FLAG_WATCHDOG, BaseState, DifferentialBase  # noqa: F401


class FakeBase:
    """An in-memory, *ideal* differential base for fast unit tests.

    Behaviour (the tests check each point):
    * Wheels reach the commanded speed instantly. ``set_wheel_velocity`` clamps each wheel to
      ±``max_wheel_speed_rad_s`` and sets velocity mode; ``set_wheel_duty`` clamps duty to -1..1,
      speed = duty × ``max_wheel_speed_rad_s``, velocity mode off. Non-finite values -> ValueError.
    * ``read()`` advances ``t`` by ``dt``, integrates each wheel angle (speed × dt) and returns a
      ``BaseState``: ticks = round(angle / 2π × ticks_per_rev), speeds, ``battery_v``, ``range_m``,
      flags = FLAG_WATCHDOG if tripped | FLAG_VELOCITY_MODE if in velocity mode.
    * Watchdog: at the start of ``read()``, if ``watchdog_s`` is not None and no drive command came
      for ``watchdog_s`` seconds of fake time (``t - last_command_t >= watchdog_s``), stop the wheels,
      leave velocity mode and set the watchdog flag until the next drive command.
    * ``stop()``: speeds 0, velocity mode off.
    * ``commands``: a list of ``("velocity" | "duty", left, right)`` and ``("stop", 0.0, 0.0)`` in call order
      (the arguments as given, before clamping) — a *spy* for behaviour tests.
    * ``close()``: stops, is idempotent; afterwards every other method raises ``RuntimeError``.
    * Context manager: ``with FakeBase() as base:`` closes on exit.
    """

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
        # TODO(student): the rest of the state (wheel angles, speeds, mode, watchdog, closed).

    def set_wheel_duty(self, left: float, right: float) -> None:
        # TODO(student)
        raise NotImplementedError("FakeBase.set_wheel_duty")

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        # TODO(student)
        raise NotImplementedError("FakeBase.set_wheel_velocity")

    def stop(self) -> None:
        # TODO(student)
        raise NotImplementedError("FakeBase.stop")

    def read(self) -> BaseState:
        # TODO(student): watchdog check, advance time, integrate, build the BaseState.
        raise NotImplementedError("FakeBase.read")

    def close(self) -> None:
        # TODO(student)
        raise NotImplementedError("FakeBase.close")

    def __enter__(self) -> FakeBase:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def drive_distance(
    base: DifferentialBase,
    distance_m: float,
    wheel_speed_rad_s: float,
    wheel_radius_m: float,
    ticks_per_rev: int,
    timeout_s: float = 30.0,
) -> float:
    """Drive straight until the mean wheel travel reaches ``|distance_m|``, then stop. Any base.

    * ``distance_m == 0`` -> return 0.0 without moving. ``wheel_speed_rad_s <= 0`` -> ValueError
      (the direction comes from the sign of ``distance_m``).
    * Take a first ``read()`` as the start. Then loop: send ``set_wheel_velocity`` EVERY cycle (it
      feeds the watchdog), ``read()``, compute the mean tick change of both wheels since the start
      and convert it to meters. Return that distance as soon as its magnitude reaches ``|distance_m|``.
    * If robot time (``state.t - start.t``) exceeds ``timeout_s``, raise ``TimeoutError``.
    * SAFETY: ``base.stop()`` must be called on EVERY exit path, including exceptions (try/finally).
    """
    # TODO(student)
    raise NotImplementedError("drive_distance")


_ = math  # you will need it
