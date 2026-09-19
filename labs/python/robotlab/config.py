"""Load ``labs/config/karmel.yaml`` — the robot's single source of physical parameters.

>>> cfg = load_config()
>>> cfg.drive.wheel_radius_m, cfg.drive.ticks_per_wheel_rev

Which file is loaded, in priority order:

1. the ``path`` argument,
2. the ``KARMEL_CONFIG`` environment variable,
3. ``labs/config/karmel.yaml`` located relative to this package.

Keys that the dataclasses don't know are ignored (add your own freely); missing keys raise
:class:`ConfigError` naming the key.
"""

from __future__ import annotations

import dataclasses
import math
import os
import typing
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

ENV_VAR = "KARMEL_CONFIG"


class ConfigError(ValueError):
    """The config file is missing, unreadable or lacks a required key."""


@dataclass(frozen=True)
class RobotInfo:
    name: str
    mass_kg: float


@dataclass(frozen=True)
class DriveConfig:
    wheel_radius_m: float
    wheel_separation_m: float
    wheel_width_m: float
    encoder_cpr_motor: int
    gear_ratio: float
    quadrature_multiplier: int
    max_wheel_speed_rad_s: float
    motor_time_constant_s: float
    duty_deadband: float
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    max_linear_accel_m_s2: float

    @property
    def ticks_per_wheel_rev(self) -> int:
        """Encoder ticks per wheel revolution = CPR x gear ratio x quadrature multiplier."""
        return round(self.encoder_cpr_motor * self.gear_ratio * self.quadrature_multiplier)

    @property
    def meters_per_tick(self) -> float:
        """Distance the wheel rim travels per encoder tick."""
        return 2.0 * math.pi * self.wheel_radius_m / self.ticks_per_wheel_rev


@dataclass(frozen=True)
class ChassisConfig:
    length_m: float
    width_m: float
    height_m: float
    ground_clearance_m: float
    caster_offset_x_m: float

    @property
    def footprint_radius_m(self) -> float:
        """Radius of a circle around base_link that covers the chassis rectangle."""
        return math.hypot(self.length_m / 2.0, self.width_m / 2.0)


@dataclass(frozen=True)
class RangeSensorConfig:
    type: str
    x_m: float
    y_m: float
    z_m: float
    max_range_m: float
    fov_rad: float


@dataclass(frozen=True)
class ImuConfig:
    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True)
class LidarConfig:
    x_m: float
    y_m: float
    z_m: float
    max_range_m: float
    samples: int
    rate_hz: float


@dataclass(frozen=True)
class CameraConfig:
    x_m: float
    y_m: float
    z_m: float
    width_px: int
    height_px: int
    horizontal_fov_rad: float


@dataclass(frozen=True)
class SensorsConfig:
    range_front: RangeSensorConfig
    imu: ImuConfig
    lidar: LidarConfig
    camera: CameraConfig


@dataclass(frozen=True)
class BatteryConfig:
    chemistry: str
    cells_series: int
    nominal_v: float
    full_v: float
    low_warning_v: float
    cutoff_v: float
    capacity_ah: float


@dataclass(frozen=True)
class SerialConfig:
    device: str
    baud: int
    watchdog_ms: int
    telemetry_hz: float


@dataclass(frozen=True)
class KarmelConfig:
    """The whole robot description. ``source`` is the file it was loaded from."""

    robot: RobotInfo
    drive: DriveConfig
    chassis: ChassisConfig
    sensors: SensorsConfig
    battery: BatteryConfig
    serial: SerialConfig
    pins: dict[str, int] = field(default_factory=dict)
    source: Path | None = None


def default_config_path() -> Path:
    """``labs/config/karmel.yaml`` next to this package (``labs/python/robotlab``)."""
    return Path(__file__).resolve().parents[2] / "config" / "karmel.yaml"


def find_config_path(path: str | os.PathLike[str] | None = None) -> Path:
    """Resolve which config file to use (argument > ``KARMEL_CONFIG`` > package default)."""
    if path is not None:
        return Path(path)
    env = os.environ.get(ENV_VAR)
    return Path(env) if env else default_config_path()


def load_config(path: str | os.PathLike[str] | None = None) -> KarmelConfig:
    """Read and validate the robot config file."""
    resolved = find_config_path(path)
    try:
        raw = yaml.safe_load(resolved.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ConfigError(
            f"cannot read robot config {resolved} ({exc}); pass a path or set {ENV_VAR}"
        ) from exc
    if not isinstance(raw, dict):
        raise ConfigError(f"{resolved}: expected a YAML mapping at the top level")
    parsed = _build(KarmelConfig, {**raw, "source": None}, section="")
    return dataclasses.replace(parsed, source=resolved)


# --- parsing helpers ---------------------------------------------------------------------------
def _build(cls: type[Any], data: Any, section: str) -> Any:
    """Instantiate dataclass ``cls`` from a mapping, recursing into nested dataclasses."""
    if not isinstance(data, dict):
        raise ConfigError(f"'{section}' must be a mapping, got {type(data).__name__}")
    hints = typing.get_type_hints(cls)
    kwargs: dict[str, Any] = {}
    for f in dataclasses.fields(cls):
        key = f"{section}.{f.name}" if section else f.name
        if f.name not in data:
            if f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING:
                raise ConfigError(f"missing required config key '{key}'")
            continue
        kwargs[f.name] = _coerce(hints[f.name], data[f.name], key)
    return cls(**kwargs)


def _coerce(hint: Any, value: Any, key: str) -> Any:
    if dataclasses.is_dataclass(hint):
        return _build(hint, value, key)
    try:
        if hint is float:
            return float(value)
        if hint is int:
            if float(value) != int(value):
                raise ValueError("not an integer")
            return int(value)
        if hint is str:
            return str(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"config key '{key}': cannot convert {value!r} ({exc})") from exc
    if typing.get_origin(hint) is dict:
        return {str(k): int(v) for k, v in (value or {}).items()}
    return value
