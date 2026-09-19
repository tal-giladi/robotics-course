"""
Battery logic for the karmel tutorial nodes, with no ROS imports so it is unit-testable.

Numbers come from labs/config/karmel.yaml: a 3S Li-ion pack, 12.6 V full, 10.5 V low
warning, 9.9 V cutoff. Used by battery_health.py (lessons 04.06 and 04.07).
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

FULL_V = 12.6
LOW_WARNING_V = 10.5
CUTOFF_V = 9.9
HYSTERESIS_V = 0.2


class HealthState(IntEnum):
    """Same numeric values as the STATE_* constants in BatteryHealth.msg."""

    OK = 0
    LOW = 1
    CRITICAL = 2


def estimate_percent(voltage_v: float, empty_v: float = CUTOFF_V,
                     full_v: float = FULL_V) -> float:
    """Crude linear state-of-charge estimate, clamped to 0..100 %."""
    fraction = (voltage_v - empty_v) / (full_v - empty_v)
    return 100.0 * min(1.0, max(0.0, fraction))


@dataclass
class LowPassFilter:
    """Exponential moving average: y = alpha * x + (1 - alpha) * y_previous."""

    alpha: float = 0.3
    value: float | None = None

    def update(self, sample: float) -> float:
        """Add a sample and return the filtered value."""
        if self.value is None:
            self.value = sample
        else:
            self.value = self.alpha * sample + (1.0 - self.alpha) * self.value
        return self.value


@dataclass
class BatteryHealthTracker:
    """
    OK / LOW / CRITICAL with hysteresis.

    Getting worse happens immediately. Getting better requires the voltage to clear the
    boundary by hysteresis_v, so a noisy voltage near a threshold does not flap.
    """

    low_threshold_v: float = LOW_WARNING_V
    critical_v: float = CUTOFF_V
    hysteresis_v: float = HYSTERESIS_V
    state: HealthState = HealthState.OK

    def classify(self, voltage_v: float) -> HealthState:
        """Classify a voltage without hysteresis."""
        if voltage_v < self.critical_v:
            return HealthState.CRITICAL
        if voltage_v < self.low_threshold_v:
            return HealthState.LOW
        return HealthState.OK

    def update(self, voltage_v: float) -> HealthState:
        """Feed a (filtered) voltage and return the new state."""
        candidate = self.classify(voltage_v)
        if candidate < self.state:  # improving: demand a margin above the boundary
            boundary = (self.critical_v if self.state == HealthState.CRITICAL
                        else self.low_threshold_v)
            if voltage_v < boundary + self.hysteresis_v:
                candidate = self.state
        self.state = candidate
        return self.state

    def check_threshold(self, low_threshold_v: float) -> str | None:
        """Return None when the threshold is acceptable, else the reason it is not."""
        minimum = self.critical_v + self.hysteresis_v
        if not minimum <= low_threshold_v < FULL_V:
            return (f'threshold must be in [{minimum:.2f}, {FULL_V:.2f}) V, '
                    f'got {low_threshold_v:.2f} V')
        return None
