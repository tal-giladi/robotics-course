"""03.07 — reference solution. Read it only after you have tried student.py."""

from __future__ import annotations

import csv
import statistics
from dataclasses import dataclass
from pathlib import Path

COLUMNS = ["host_time_s", "t_s", "left_ticks", "right_ticks", "left_rad_s", "right_rad_s",
           "battery_v", "range_m", "flags", "left_cmd", "right_cmd"]

GAP_FACTOR = 1.5

OPTIONAL = ("battery_v", "range_m", "left_cmd", "right_cmd")


@dataclass(frozen=True)
class TelemetrySample:
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
    start_t: float
    end_t: float
    samples: int

    @property
    def duration_s(self) -> float:
        return self.end_t - self.start_t


@dataclass(frozen=True)
class RunSummary:
    samples: int
    duration_s: float
    rate_hz: float
    median_interval_s: float
    gaps: int
    max_gap_s: float
    lost_samples: int
    distance_m: float
    max_wheel_speed_rad_s: float
    battery_start_v: float | None
    battery_min_v: float | None
    range_valid_fraction: float


def _optional_float(value: str | None) -> float | None:
    """'' or missing -> None; anything else must parse as a float."""
    if value is None or value.strip() == "":
        return None
    return float(value)


def _row_to_sample(row: dict[str, str]) -> TelemetrySample:
    return TelemetrySample(
        host_time_s=float(row["host_time_s"]),
        t_s=float(row["t_s"]),
        left_ticks=int(row["left_ticks"]),
        right_ticks=int(row["right_ticks"]),
        left_rad_s=float(row["left_rad_s"]),
        right_rad_s=float(row["right_rad_s"]),
        battery_v=_optional_float(row["battery_v"]),
        range_m=_optional_float(row["range_m"]),
        flags=int(row["flags"]),
        left_cmd=_optional_float(row["left_cmd"]),
        right_cmd=_optional_float(row["right_cmd"]),
    )


def read_telemetry_csv(path: str | Path, *, strict: bool = False) -> list[TelemetrySample]:
    with Path(path).open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        missing = [name for name in COLUMNS if name not in header]
        if missing:
            raise ValueError(f"{path}: not a telemetry log, missing column(s): {', '.join(missing)}")
        samples: list[TelemetrySample] = []
        for row in reader:
            try:
                samples.append(_row_to_sample(row))
            except (TypeError, ValueError, KeyError) as exc:
                # A robot that loses power mid-write leaves a half-row. One bad row must not
                # cost you the other 20,000.
                if strict:
                    raise ValueError(f"{path}: cannot parse row {row!r}: {exc}") from exc
        return samples


def summarize(samples: list[TelemetrySample], meters_per_tick: float) -> RunSummary:
    if len(samples) < 2:
        raise ValueError(f"need at least two samples to summarize a run, got {len(samples)}")

    intervals = [b.t_s - a.t_s for a, b in zip(samples, samples[1:])]
    median = statistics.median(intervals)
    threshold = GAP_FACTOR * median
    gap_intervals = [d for d in intervals if d > threshold]
    lost = sum(max(0, round(d / median) - 1) for d in gap_intervals) if median > 0 else 0

    distance = 0.0
    for a, b in zip(samples, samples[1:]):
        mean_ticks = ((b.left_ticks - a.left_ticks) + (b.right_ticks - a.right_ticks)) / 2.0
        distance += abs(mean_ticks) * meters_per_tick

    duration = samples[-1].t_s - samples[0].t_s
    batteries = [s.battery_v for s in samples if s.battery_v is not None]
    ranges = sum(1 for s in samples if s.range_m is not None)

    return RunSummary(
        samples=len(samples),
        duration_s=duration,
        rate_hz=(len(samples) - 1) / duration if duration > 0 else 0.0,
        median_interval_s=median,
        gaps=len(gap_intervals),
        max_gap_s=max(intervals),
        lost_samples=lost,
        distance_m=distance,
        max_wheel_speed_rad_s=max(max(abs(s.left_rad_s), abs(s.right_rad_s)) for s in samples),
        battery_start_v=batteries[0] if batteries else None,
        battery_min_v=min(batteries) if batteries else None,
        range_valid_fraction=ranges / len(samples),
    )


def flag_episodes(samples: list[TelemetrySample], flag: int) -> list[Episode]:
    episodes: list[Episode] = []
    start_index: int | None = None
    for index, sample in enumerate(samples):
        if sample.flags & flag:
            if start_index is None:
                start_index = index
        elif start_index is not None:
            episodes.append(_episode(samples, start_index, index - 1))
            start_index = None
    if start_index is not None:
        episodes.append(_episode(samples, start_index, len(samples) - 1))
    return episodes


def _episode(samples: list[TelemetrySample], first: int, last: int) -> Episode:
    # Index-based, because two identical frozen dataclasses compare equal: list.index would lie.
    return Episode(samples[first].t_s, samples[last].t_s, last - first + 1)
