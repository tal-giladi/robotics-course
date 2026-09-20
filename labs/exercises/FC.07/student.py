"""FC.07 — is this loop real-time? A deadline monitor, a schedulability test and a watchdog.

Fill in every ``TODO(student)``. Check your work with ``python course.py check FC.07``.
Standard library only.

These are the three questions you ask about any control loop:
  1. *Did* it meet its deadline?  -> ``LoopMonitor``
  2. *Can* it meet its deadline?  -> ``total_utilisation`` / ``rate_monotonic_bound`` / ``is_schedulable``
  3. What happens when it doesn't? -> ``CommandWatchdog``
"""

from __future__ import annotations

from dataclasses import dataclass


# --------------------------------------------------------------------------- 1. did it meet it?
@dataclass(frozen=True)
class LoopStats:
    count: int              # number of periods = len(timestamps) - 1
    mean_ms: float
    min_ms: float
    max_ms: float
    p99_ms: float           # 99th percentile of the *period*, not of the error
    worst_jitter_ms: float  # largest |period - nominal|
    misses: int             # periods strictly longer than nominal * (1 + tolerance)
    miss_fraction: float    # misses / count, 0.0 when count == 0


def percentile(values: list[float], fraction: float) -> float:
    """Nearest-rank percentile of a list: sort it, then take index ``round(fraction * (n-1))``.

    ``fraction`` is in [0, 1]. An empty list raises ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("percentile")


def analyse_loop(timestamps_s: list[float], nominal_hz: float, tolerance: float = 0.10) -> LoopStats:
    """Turn a list of loop timestamps (seconds, monotonic) into :class:`LoopStats`.

    * Periods are the differences between consecutive timestamps, converted to **milliseconds**.
    * ``nominal_ms = 1000 / nominal_hz``.
    * A *miss* is a period **strictly greater** than ``nominal_ms * (1 + tolerance)``. A period
      that is too short is not a miss — it is the loop catching up after a late one.
    * Fewer than two timestamps: return ``LoopStats(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0)``.
    * ``nominal_hz <= 0`` raises ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("analyse_loop")


# --------------------------------------------------------------------------- 2. can it meet it?
@dataclass(frozen=True)
class Task:
    """A periodic task: it needs ``wcet_ms`` of CPU every ``period_ms``."""
    name: str
    period_ms: float
    wcet_ms: float          # worst-case execution time


def total_utilisation(tasks: list[Task]) -> float:
    """Sum of ``wcet_ms / period_ms``. 1.0 means the CPU is exactly full."""
    # TODO(student)
    raise NotImplementedError("total_utilisation")


def rate_monotonic_bound(n: int) -> float:
    """Liu & Layland's sufficient bound for ``n`` tasks under rate-monotonic priorities:

    .. math:: U_{bound}(n) = n \\left(2^{1/n} - 1\\right)

    n = 1 -> 1.000, n = 2 -> 0.828, n = 3 -> 0.780, and it decreases to ln 2 = 0.693.
    ``n <= 0`` raises ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("rate_monotonic_bound")


def is_schedulable(tasks: list[Task]) -> tuple[bool, str]:
    """Rate-monotonic schedulability, as (verdict, one-line reason).

    * ``U > 1.0``            -> (False, "overloaded: U = ... > 1.0")
    * ``U <= bound(n)``      -> (True,  "guaranteed: U = ... <= bound ...")
    * in between             -> (True,  "possible: U = ... is above the bound ... — needs response-time analysis")

    Format every number with three decimals, e.g. ``"overloaded: U = 1.180 > 1.0"``,
    ``"guaranteed: U = 0.450 <= bound 0.780"``,
    ``"possible: U = 0.900 is above the bound 0.780 - needs response-time analysis"``.
    An empty task list is schedulable: ``(True, "guaranteed: U = 0.000 <= bound 1.000")``.
    """
    # TODO(student)
    raise NotImplementedError("is_schedulable")


# --------------------------------------------------------------------------- 3. when it doesn't
TICKS_PERIOD = 1 << 30
TICKS_HALF = TICKS_PERIOD // 2


def ticks_diff(new_ms: int, old_ms: int) -> int:
    """``time.ticks_diff`` for the rp2 port: signed distance on a counter that wraps at 2**30."""
    # TODO(student)
    raise NotImplementedError("ticks_diff")


class CommandWatchdog:
    """The rule from ``labs/firmware/pico/watchdog.py``, reimplemented from its specification.

        wd = CommandWatchdog(timeout_ms=300, now_ms=0)
        wd.feed(now_ms)            # a valid drive command arrived
        if wd.check(now_ms):       # call once per control step
            motors.brake()

    * It starts **tripped**: after boot the motors stay stopped until the first command.
    * ``feed`` records the time and clears ``tripped``.
    * ``check`` returns ``True`` **exactly once**, on the step where the timeout is first reached
      (``elapsed >= timeout_ms``). While already tripped it returns ``False`` — you stopped the
      motors on that first step and must not keep re-stopping them every step.
    * ``tripped`` stays ``True`` until the next ``feed``.
    * ``elapsed_ms(now)`` uses :func:`ticks_diff`, never plain subtraction.
    """

    def __init__(self, timeout_ms: int, now_ms: int) -> None:
        # TODO(student)
        raise NotImplementedError("CommandWatchdog.__init__")

    def feed(self, now_ms: int) -> None:
        # TODO(student)
        raise NotImplementedError("CommandWatchdog.feed")

    def elapsed_ms(self, now_ms: int) -> int:
        # TODO(student)
        raise NotImplementedError("CommandWatchdog.elapsed_ms")

    def check(self, now_ms: int) -> bool:
        # TODO(student)
        raise NotImplementedError("CommandWatchdog.check")


def stopping_distance_m(speed_m_s: float, timeout_ms: float, control_period_ms: float,
                        decel_m_s2: float) -> float:
    """How far the robot travels between the last command and standing still.

    Three pieces:
      * the watchdog waits ``timeout_ms`` before it notices,
      * it is only checked once per control step, so add up to ``control_period_ms``,
      * then braking at ``decel_m_s2`` takes ``speed / decel`` seconds and covers
        ``speed**2 / (2 * decel)`` metres.

    Return the total in metres. ``decel_m_s2 <= 0`` raises ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("stopping_distance_m")
