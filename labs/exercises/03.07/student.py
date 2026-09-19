"""03.07 — Turn a telemetry log into answers: parse it, reduce it, find the episodes.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 03.07``.
Standard library only (``csv``, ``statistics``, ``dataclasses``, ``math``) — no numpy, no pandas:
this code has to run on the robot, straight after a run, with nothing installed.

The input is what ``labs/robot/log_telemetry.py`` writes — one row per telemetry sample:

    host_time_s,t_s,left_ticks,right_ticks,left_rad_s,right_rad_s,battery_v,range_m,flags,left_cmd,right_cmd
    0.0200,0.020,0,0,0.000,0.000,12.240,,0,,
    2.0000,2.000,4581,4581,7.964,7.964,12.239,,8,8.0,8.0

Empty cells mean "no reading" (``None``), never zero. ``flags`` is the bitfield from
``robotlab.hal``: 1 watchdog, 2 low battery, 4 range error, 8 velocity mode.
"""

from __future__ import annotations

import csv  # noqa: F401
import math  # noqa: F401
import statistics  # noqa: F401
from dataclasses import dataclass
from pathlib import Path

# The columns log_telemetry.py writes, in order. A file missing any of them is not a telemetry log.
COLUMNS = ["host_time_s", "t_s", "left_ticks", "right_ticks", "left_rad_s", "right_rad_s",
           "battery_v", "range_m", "flags", "left_cmd", "right_cmd"]

GAP_FACTOR = 1.5
"""An interval longer than GAP_FACTOR x the median interval counts as a gap (lost telemetry)."""


@dataclass(frozen=True)
class TelemetrySample:
    """One row. ``None`` means the robot had no valid reading — never 0.0, never -1."""

    host_time_s: float
    t_s: float
    left_ticks: int
    right_ticks: int
    left_rad_s: float
    right_rad_s: float
    battery_v: float | None
    range_m: float | None
    flags: int
    left_cmd: float | None
    right_cmd: float | None


@dataclass(frozen=True)
class Episode:
    """A contiguous stretch of samples in which a flag bit was set."""

    start_t: float
    end_t: float
    samples: int

    @property
    def duration_s(self) -> float:
        return self.end_t - self.start_t


@dataclass(frozen=True)
class RunSummary:
    """Everything you want to know about a run before you open a plot."""

    samples: int
    duration_s: float            # last t_s - first t_s
    rate_hz: float               # (samples - 1) / duration_s
    median_interval_s: float
    gaps: int                    # intervals longer than GAP_FACTOR x median
    max_gap_s: float             # the largest interval (NOT only gaps)
    lost_samples: int            # how many samples the gaps swallowed
    distance_m: float            # path length from the encoders (reversals add, not cancel)
    max_wheel_speed_rad_s: float
    battery_start_v: float | None
    battery_min_v: float | None
    range_valid_fraction: float  # samples with a range reading / samples


def read_telemetry_csv(path: str | Path, *, strict: bool = False) -> list[TelemetrySample]:
    """Parse a telemetry CSV into samples.

    * The header must contain every name in ``COLUMNS``; otherwise raise ``ValueError`` naming
      the missing ones (a file you cannot trust is worse than no file).
    * An empty cell for ``battery_v``, ``range_m``, ``left_cmd`` or ``right_cmd`` becomes ``None``.
    * A row that cannot be parsed — a short last line from a robot that lost power mid-write, a
      truncated number — is SKIPPED when ``strict`` is false (the default), and re-raised as
      ``ValueError`` when ``strict`` is true.
    * Ticks and flags are ``int``; the rest are ``float``.
    """
    # TODO(student)
    raise NotImplementedError("read_telemetry_csv")


def summarize(samples: list[TelemetrySample], meters_per_tick: float) -> RunSummary:
    """Reduce a run to one line of numbers. Raise ``ValueError`` for fewer than two samples.

    Definitions the tests use, so be exact:

    * ``duration_s``  = ``samples[-1].t_s - samples[0].t_s``
    * ``rate_hz``     = ``(len(samples) - 1) / duration_s``
    * intervals       = the differences between consecutive ``t_s``
    * ``median_interval_s`` = the median of those intervals (``statistics.median``)
    * ``gaps``        = how many intervals exceed ``GAP_FACTOR * median_interval_s``
    * ``max_gap_s``   = the largest interval
    * ``lost_samples``= for each gap interval, ``round(interval / median) - 1``, summed
    * ``distance_m``  = sum over consecutive samples of
      ``abs((d_left_ticks + d_right_ticks) / 2) * meters_per_tick``
      (the robot centre's path length: driving back does not undo driving out)
    * ``max_wheel_speed_rad_s`` = the largest ``abs()`` of either wheel speed
    * ``battery_start_v`` = the first reading that is not ``None`` (``None`` if there is none);
      ``battery_min_v`` = the smallest such reading
    * ``range_valid_fraction`` = samples with a range reading / total samples
    """
    # TODO(student)
    raise NotImplementedError("summarize")


def flag_episodes(samples: list[TelemetrySample], flag: int) -> list[Episode]:
    """Every contiguous stretch in which ``flag`` was set, in time order.

    One ``Episode`` per stretch: ``start_t`` and ``end_t`` are the ``t_s`` of its first and last
    sample (so a single-sample episode has ``duration_s == 0.0``), and ``samples`` counts them.
    An empty input, or a flag that never appears, gives an empty list.
    """
    # TODO(student)
    raise NotImplementedError("flag_episodes")
