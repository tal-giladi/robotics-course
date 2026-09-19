from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from robotlab.config import ENV_VAR, ConfigError, KarmelConfig, default_config_path, load_config


def raw_yaml() -> dict:
    return yaml.safe_load(default_config_path().read_text(encoding="utf-8"))


def test_default_path_is_labs_config():
    path = default_config_path()
    assert path.parts[-3:] == ("labs", "config", "karmel.yaml")
    assert path.is_file()


def test_loads_every_section_with_file_values(cfg: KarmelConfig):
    raw = raw_yaml()
    assert cfg.robot.name == raw["robot"]["name"]
    assert cfg.drive.wheel_radius_m == raw["drive"]["wheel_radius_m"]
    assert cfg.drive.wheel_separation_m == raw["drive"]["wheel_separation_m"]
    assert cfg.sensors.lidar.samples == raw["sensors"]["lidar"]["samples"]
    assert cfg.battery.capacity_ah == raw["battery"]["capacity_ah"]
    assert cfg.serial.watchdog_ms == raw["serial"]["watchdog_ms"]
    assert cfg.pins == raw["pins"]
    assert cfg.source == default_config_path()


def test_derived_ticks_per_wheel_rev(cfg: KarmelConfig):
    d = raw_yaml()["drive"]
    expected = d["encoder_cpr_motor"] * d["gear_ratio"] * d["quadrature_multiplier"]
    assert cfg.drive.ticks_per_wheel_rev == expected
    assert isinstance(cfg.drive.ticks_per_wheel_rev, int)
    assert cfg.drive.meters_per_tick == pytest.approx(2 * math.pi * d["wheel_radius_m"] / expected)


def test_types_are_coerced(cfg: KarmelConfig):
    assert isinstance(cfg.drive.gear_ratio, float)
    assert isinstance(cfg.drive.encoder_cpr_motor, int)
    assert isinstance(cfg.sensors.range_front.max_range_m, float)


def modified_config(tmp_path: Path, mutate) -> Path:
    raw = raw_yaml()
    mutate(raw)
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


def test_explicit_path_and_env_var(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    path = modified_config(tmp_path, lambda r: r["drive"].update(wheel_radius_m=0.1, custom_key="ignored"))
    assert load_config(path).drive.wheel_radius_m == 0.1
    monkeypatch.setenv(ENV_VAR, str(path))
    assert load_config().drive.wheel_radius_m == 0.1
    assert load_config().source == path


def test_missing_key_is_named(tmp_path: Path):
    path = modified_config(tmp_path, lambda r: r["drive"].pop("wheel_radius_m"))
    with pytest.raises(ConfigError, match=r"drive\.wheel_radius_m"):
        load_config(path)


def test_missing_file_is_a_config_error(tmp_path: Path):
    with pytest.raises(ConfigError, match=ENV_VAR):
        load_config(tmp_path / "nope.yaml")
