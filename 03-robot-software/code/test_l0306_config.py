"""Tests for the lesson 03.06 config lab — no hardware, no network.

    python -m pytest 03-robot-software/code/test_l0306_config.py
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

import l0306_config_lab as lab
from robotlab.config import ConfigError, default_config_path, load_config


def raw() -> dict:
    return yaml.safe_load(default_config_path().read_text(encoding="utf-8"))


def write(tmp_path: Path, mutate) -> Path:
    data = raw()
    mutate(data)
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


# --- the shipped config must be plausible, not just parseable ------------------------------------
def test_karmel_yaml_has_no_validation_errors() -> None:
    problems = lab.plausibility_problems(load_config())
    assert [str(p) for p in problems if p.severity == "error"] == []


def test_derived_values_match_the_hand_calculation() -> None:
    cfg = load_config()
    d = cfg.drive
    assert d.ticks_per_wheel_rev == 11 * 56 * 4 == 2464
    assert d.meters_per_tick == pytest.approx(2 * math.pi * 0.045 / 2464)
    assert d.meters_per_tick * 1000 == pytest.approx(0.1147, abs=0.0001)  # 0.115 mm per tick
    rows = lab.derived(cfg)
    assert "2464 = 11 x 56 x 4" in rows["drive.ticks_per_wheel_rev"]
    assert cfg.chassis.footprint_radius_m == pytest.approx(math.hypot(0.125, 0.10))


# --- validation catches the mistakes people actually make -----------------------------------------
@pytest.mark.parametrize(
    ("mutate", "expect_key"),
    [
        (lambda r: r["drive"].update(wheel_radius_m=45.0), "drive.wheel_radius_m"),          # mm, not m
        (lambda r: r["drive"].update(max_linear_speed_m_s=3.0), "drive.max_linear_speed_m_s"),
        (lambda r: r["drive"].update(max_angular_speed_rad_s=25.0), "drive.max_angular_speed_rad_s"),
        (lambda r: r["battery"].update(low_warning_v=9.0), "battery.cutoff_v"),              # warning below cutoff
        (lambda r: r["serial"].update(watchdog_ms=10), "serial.watchdog_ms"),
        (lambda r: r["pins"].update(status_led=r["pins"]["encoder_left_a"]), "pins"),         # two functions, one GPIO
        (lambda r: r["pins"].update(status_led=99), "pins.status_led"),
    ],
)
def test_plausibility_errors(tmp_path: Path, mutate, expect_key: str) -> None:
    cfg = load_config(write(tmp_path, mutate))
    errors = [p for p in lab.plausibility_problems(cfg) if p.severity == "error"]
    assert any(p.key == expect_key for p in errors), [str(p) for p in errors]


def test_duplicate_pin_message_names_both_functions(tmp_path: Path) -> None:
    cfg = load_config(write(tmp_path, lambda r: r["pins"].update(status_led=r["pins"]["i2c_sda"])))
    (problem,) = [p for p in lab.plausibility_problems(cfg) if p.key == "pins"]
    assert "i2c_sda" in problem.message and "status_led" in problem.message


def test_missing_key_is_a_config_error_naming_the_key(tmp_path: Path) -> None:
    path = write(tmp_path, lambda r: r["drive"].pop("wheel_radius_m"))
    with pytest.raises(ConfigError, match="wheel_radius_m"):
        load_config(path)


# --- sensitivity ------------------------------------------------------------------------------------
def test_odometry_sensitivity_is_the_linear_relation() -> None:
    rows = dict(lab.odometry_sensitivity(load_config(), distance_m=10.0, turns=1.0))
    assert "10.0 cm" in rows["wheel_radius_m off by 1 %"]          # 10 m * 1 % = 10 cm
    assert "3.6 deg" in rows["wheel_separation_m off by 1 %"]      # 360 deg * 1 % = 3.6 deg
    assert "50.0 cm" in rows["wheel_radius_m off by 5 %"]


# --- calibration overlay ------------------------------------------------------------------------------
def test_deep_merge_replaces_leaves_and_keeps_siblings() -> None:
    base = {"drive": {"wheel_radius_m": 0.045, "gear_ratio": 56.0}, "robot": {"name": "karmel"}}
    overlay = {"drive": {"wheel_radius_m": 0.0446}}
    assert lab.deep_merge(base, overlay) == {
        "drive": {"wheel_radius_m": 0.0446, "gear_ratio": 56.0},
        "robot": {"name": "karmel"},
    }


def test_changes_reports_dotted_keys_with_old_and_new() -> None:
    base = {"drive": {"wheel_radius_m": 0.045, "duty_deadband": 0.12}}
    overlay = {"drive": {"wheel_radius_m": 0.0446}}
    assert lab.changes(base, overlay) == [("drive.wheel_radius_m", 0.045, 0.0446)]


def test_example_calibration_overlay_loads_and_changes_the_right_numbers() -> None:
    cfg, diff = lab.load_with_overlay(default_config_path(), lab.EXAMPLE_CALIBRATION)
    keys = {key for key, _, _ in diff}
    assert keys == {"drive.wheel_radius_m", "drive.wheel_separation_m", "drive.duty_deadband"}
    assert cfg.drive.wheel_radius_m == 0.0446
    assert cfg.drive.wheel_separation_m == 0.2032
    # the derived value follows the measurement, because it is computed and never stored
    assert cfg.drive.meters_per_tick == pytest.approx(2 * math.pi * 0.0446 / 2464)
    assert [p for p in lab.plausibility_problems(cfg) if p.severity == "error"] == []


def test_the_calibration_block_carries_provenance() -> None:
    prov = lab.calibration_provenance(lab.EXAMPLE_CALIBRATION)
    assert {"date", "method", "operator", "uncertainty"} <= set(prov)
    assert prov["uncertainty"]["wheel_radius_m"] > 0


# --- the command line --------------------------------------------------------------------------------
def test_main_reports_a_clean_config(capsys: pytest.CaptureFixture[str]) -> None:
    assert lab.main(["--sensitivity"]) == 0
    out = capsys.readouterr().out
    assert "ticks_per_wheel_rev" in out and "no problems" in out


def test_main_exits_nonzero_on_an_implausible_config(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    path = write(tmp_path, lambda r: r["drive"].update(max_linear_speed_m_s=3.0))
    assert lab.main(["--config", str(path), "--quiet"]) == 1
    assert "max_linear_speed_m_s" in capsys.readouterr().out


def test_main_prints_the_overlay_provenance(capsys: pytest.CaptureFixture[str]) -> None:
    assert lab.main(["--overlay", str(lab.EXAMPLE_CALIBRATION)]) == 0
    out = capsys.readouterr().out
    assert "karmel-001" in out and "0.045 -> 0.0446" in out
