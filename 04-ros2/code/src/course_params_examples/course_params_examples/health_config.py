"""HealthConfig — the battery_health settings as one immutable value, with its validation rules.

No ROS imports: the rules are unit-tested with plain pytest (test/test_health_config.py) and
used by battery_health_params.py both at start-up and in the on-set-parameters callback.
"""
from __future__ import annotations

from dataclasses import dataclass, fields, replace
from typing import Any

# Li-ion cell voltage window used to derive sane per-pack ranges (lesson 01.14).
CELL_MIN_V = 3.0
CELL_MAX_V = 4.25


@dataclass(frozen=True)
class HealthConfig:
    full_v: float = 12.6            # labs/config/karmel.yaml battery.full_v
    low_threshold_v: float = 10.5   # battery.low_warning_v
    critical_v: float = 9.9         # battery.cutoff_v
    hysteresis_v: float = 0.2
    filter_alpha: float = 0.3       # low-pass filter weight of the newest sample

    @classmethod
    def names(cls) -> list[str]:
        return [f.name for f in fields(cls)]

    def with_changes(self, changes: dict[str, Any]) -> HealthConfig:
        """Return a copy with the given fields replaced (unknown keys are ignored)."""
        known = {k: float(v) for k, v in changes.items() if k in self.names()}
        return replace(self, **known)

    def problems(self) -> list[str]:
        """Every rule this configuration breaks. Empty list = valid."""
        found = []
        if self.hysteresis_v < 0.0:
            found.append(f'hysteresis_v must be >= 0, got {self.hysteresis_v:.2f}')
        if self.critical_v + self.hysteresis_v > self.low_threshold_v:
            found.append(
                f'low_threshold_v ({self.low_threshold_v:.2f} V) must be >= critical_v + hysteresis_v '
                f'({self.critical_v:.2f} + {self.hysteresis_v:.2f} V)')
        if self.low_threshold_v >= self.full_v:
            found.append(f'low_threshold_v ({self.low_threshold_v:.2f} V) must be < full_v ({self.full_v:.2f} V)')
        if not 0.0 < self.filter_alpha <= 1.0:
            found.append(f'filter_alpha must be in (0, 1], got {self.filter_alpha}')
        return found


def voltage_range(cells_series: int) -> tuple[float, float]:
    """Physically possible pack voltages for a Li-ion pack with this many cells in series."""
    return CELL_MIN_V * cells_series, CELL_MAX_V * cells_series
