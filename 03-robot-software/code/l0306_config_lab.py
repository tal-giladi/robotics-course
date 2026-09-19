"""Lesson 03.06 — what a robot config file is worth: derived values, validation, calibration, diffs.

    python 03-robot-software/code/l0306_config_lab.py                       # summary + validation
    python 03-robot-software/code/l0306_config_lab.py --sensitivity
    python 03-robot-software/code/l0306_config_lab.py --overlay 03-robot-software/code/l0306_calibration_example.yaml
    python 03-robot-software/code/l0306_config_lab.py --diff other-robot.yaml
    python 03-robot-software/code/l0306_config_lab.py --quiet                # exit 1 on any error

Four things a config file needs that a dict of constants does not:

summary       the DERIVED values (ticks/rev, m/tick, footprint radius) printed next to the raw
              ones, so you can see that nobody stored two versions of the same truth.
--validate    (default) plausibility, not just types: cross-field consistency, duplicate GPIO
              pins, battery thresholds in the right order. robotlab.config checks that keys EXIST
              and have the right type; it does not check that the numbers make sense.
--sensitivity how much a 1 % error in wheel radius or wheelbase costs you in metres and degrees.
              This is the argument for calibration (lessons 01.12 and 09.05).
--overlay     merge a dated calibration file over the design config, and show what changed.
              Calibration is measured, belongs to ONE robot, and has a provenance; config is a
              choice. Keeping them in separate files keeps both honest.

Exit code 0 when there are no errors, 1 otherwise — usable as a pre-flight check on the robot.
Nothing here needs hardware. Tests: test_l0306_config.py
"""

from __future__ import annotations

import argparse
import math
import sys
import tempfile
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "labs" / "python"))

from robotlab.config import ConfigError, KarmelConfig, find_config_path, load_config  # noqa: E402

EXAMPLE_CALIBRATION = Path(__file__).resolve().parent / "l0306_calibration_example.yaml"


@dataclass(frozen=True)
class Problem:
    severity: str  # "error" or "warning"
    key: str
    message: str

    def __str__(self) -> str:
        return f"{self.severity.upper():<7} {self.key:<34} {self.message}"


# --- validation beyond types -----------------------------------------------------------------------
def plausibility_problems(cfg: KarmelConfig) -> list[Problem]:
    """Physical and cross-field checks. robotlab.config already guarantees presence and type."""
    p: list[Problem] = []
    d, ch, b, s = cfg.drive, cfg.chassis, cfg.battery, cfg.serial

    def err(key: str, message: str) -> None:
        p.append(Problem("error", key, message))

    def warn(key: str, message: str) -> None:
        p.append(Problem("warning", key, message))

    # --- ranges a wheeled indoor robot must be inside
    if not 0.01 <= d.wheel_radius_m <= 0.30:
        err("drive.wheel_radius_m", f"{d.wheel_radius_m} m is not a plausible wheel radius (0.01-0.30)")
    if not 0.05 <= d.wheel_separation_m <= 1.50:
        err("drive.wheel_separation_m", f"{d.wheel_separation_m} m is not a plausible track width")
    if d.wheel_separation_m <= d.wheel_width_m:
        err("drive.wheel_separation_m", "track width must exceed one wheel width")
    if d.ticks_per_wheel_rev <= 0:
        err("drive.ticks_per_wheel_rev", "derived ticks per wheel revolution must be positive")
    if not 0.0 <= d.duty_deadband < 0.6:
        err("drive.duty_deadband", f"{d.duty_deadband} is outside 0.0-0.6")
    if d.motor_time_constant_s <= 0.0:
        err("drive.motor_time_constant_s", "must be positive")

    # --- cross-field consistency: software limits the motors cannot actually deliver are a lie
    rim_speed = d.max_wheel_speed_rad_s * d.wheel_radius_m
    if d.max_linear_speed_m_s > rim_speed + 1e-9:
        err("drive.max_linear_speed_m_s",
            f"{d.max_linear_speed_m_s} m/s > what the wheels can do ({rim_speed:.3f} m/s)")
    omega_max = 2.0 * rim_speed / d.wheel_separation_m
    if d.max_angular_speed_rad_s > omega_max + 1e-9:
        err("drive.max_angular_speed_rad_s",
            f"{d.max_angular_speed_rad_s} rad/s > geometric maximum ({omega_max:.2f} rad/s)")
    if d.max_linear_speed_m_s > 0.7 * rim_speed:
        warn("drive.max_linear_speed_m_s",
             f"only {100 * (1 - d.max_linear_speed_m_s / rim_speed):.0f} % headroom over the wheel limit")

    # --- battery thresholds must be ordered, or the low-battery logic is nonsense
    order = [("cutoff_v", b.cutoff_v), ("low_warning_v", b.low_warning_v),
             ("nominal_v", b.nominal_v), ("full_v", b.full_v)]
    for (name_a, a), (name_b, bb) in zip(order, order[1:]):
        if not a < bb:
            err(f"battery.{name_a}", f"{name_a} ({a}) must be below {name_b} ({bb})")
    if b.cells_series > 0:
        per_cell = b.full_v / b.cells_series
        if not 1.2 <= per_cell <= 4.3:
            warn("battery.full_v", f"{per_cell:.2f} V per cell is unusual for {b.chemistry}")

    # --- serial / timing
    if not 50 <= s.watchdog_ms <= 5000:
        err("serial.watchdog_ms", f"{s.watchdog_ms} ms outside the firmware's accepted 50-5000")
    if not 1.0 <= s.telemetry_hz <= 200.0:
        err("serial.telemetry_hz", f"{s.telemetry_hz} Hz outside the firmware's accepted 1-200")
    if s.telemetry_hz > 0 and s.watchdog_ms < 3 * 1000.0 / s.telemetry_hz:
        warn("serial.watchdog_ms",
             f"only {s.watchdog_ms * s.telemetry_hz / 1000.0:.1f} telemetry periods of margin")

    # --- pins: two peripherals on one GPIO is a wiring bug you want to find on a laptop
    duplicates = [pin for pin, n in Counter(cfg.pins.values()).items() if n > 1]
    for pin in sorted(duplicates):
        names = sorted(k for k, v in cfg.pins.items() if v == pin)
        err("pins", f"GPIO {pin} is assigned to {len(names)} functions: {', '.join(names)}")
    for name, pin in sorted(cfg.pins.items()):
        if not 0 <= pin <= 29:
            err(f"pins.{name}", f"GPIO {pin} is not a Pico 2 GPIO (0-29)")

    # --- sensors must be mounted on the robot
    front = cfg.sensors.range_front
    if front.x_m > ch.length_m / 2 + 0.02:
        warn("sensors.range_front.x_m",
             f"{front.x_m} m is ahead of the chassis front ({ch.length_m / 2:.3f} m)")
    if cfg.sensors.lidar.samples <= 0 or cfg.sensors.lidar.rate_hz <= 0:
        err("sensors.lidar", "samples and rate_hz must be positive")
    return p


# --- derived values and sensitivity -----------------------------------------------------------------
def derived(cfg: KarmelConfig) -> dict[str, str]:
    d, ch = cfg.drive, cfg.chassis
    rim = d.max_wheel_speed_rad_s * d.wheel_radius_m
    return {
        "drive.ticks_per_wheel_rev": f"{d.ticks_per_wheel_rev} = {d.encoder_cpr_motor} x {d.gear_ratio:g} x {d.quadrature_multiplier}",
        "drive.meters_per_tick": f"{d.meters_per_tick * 1000:.4f} mm = 2*pi*r / ticks_per_rev",
        "drive.wheel rim speed": f"{rim:.3f} m/s at max_wheel_speed_rad_s",
        "drive.max omega (geometry)": f"{2 * rim / d.wheel_separation_m:.2f} rad/s",
        "chassis.footprint_radius_m": f"{ch.footprint_radius_m:.4f} m (circle covering {ch.length_m} x {ch.width_m})",
        "serial.telemetry period": f"{1000.0 / cfg.serial.telemetry_hz:.1f} ms",
        "serial.watchdog in periods": f"{cfg.serial.watchdog_ms * cfg.serial.telemetry_hz / 1000.0:.1f}",
    }


def odometry_sensitivity(cfg: KarmelConfig, distance_m: float = 10.0, turns: float = 1.0) -> list[tuple[str, str]]:
    """What a percentage error in the two calibrated numbers costs, in metres and degrees.

    Straight line: reported distance scales with the wheel radius, so a relative radius error
    ``e`` gives ``distance_m * e`` of error. In-place rotation: theta = (d_right - d_left) / b,
    so a relative wheelbase error ``e`` gives ``turns * 360 * e`` degrees.
    """
    rows = []
    for percent in (0.5, 1.0, 2.0, 5.0):
        e = percent / 100.0
        rows.append((
            f"wheel_radius_m off by {percent:g} %",
            f"{distance_m * e * 100:6.1f} cm after {distance_m:g} m straight",
        ))
        rows.append((
            f"wheel_separation_m off by {percent:g} %",
            f"{turns * 360.0 * e:6.1f} deg after {turns:g} full turn(s) in place",
        ))
    return rows


# --- calibration overlay -----------------------------------------------------------------------------
def deep_merge(base: Any, overlay: Any) -> Any:
    """Overlay wins, recursively; mappings merge, everything else is replaced."""
    if isinstance(base, dict) and isinstance(overlay, dict):
        merged = dict(base)
        for key, value in overlay.items():
            merged[key] = deep_merge(base.get(key), value) if key in base else value
        return merged
    return overlay


def changes(base: Any, overlay: Any, path: str = "") -> list[tuple[str, Any, Any]]:
    """[(dotted key, old, new)] for every leaf the overlay changes."""
    out: list[tuple[str, Any, Any]] = []
    if isinstance(base, dict) and isinstance(overlay, dict):
        for key, value in overlay.items():
            sub = f"{path}.{key}" if path else str(key)
            out.extend(changes(base.get(key), value, sub))
    elif base != overlay:
        out.append((path, base, overlay))
    return out


def load_with_overlay(base_path: Path, overlay_path: Path) -> tuple[KarmelConfig, list[tuple[str, Any, Any]]]:
    """Merge a calibration file over the design config and load the result.

    ``robotlab.config.load_config`` takes a *path*, not a mapping, so the merged document is
    written to a temporary file. Making ``load_config`` accept a mapping is Exercise 03.06-E3.
    """
    base_raw = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    overlay_raw = yaml.safe_load(overlay_path.read_text(encoding="utf-8")) or {}
    provenance = overlay_raw.pop("calibration", None)
    merged = deep_merge(base_raw, overlay_raw)
    diff = changes(base_raw, overlay_raw)
    if provenance is not None:
        merged["calibration"] = provenance  # kept in the document; the dataclasses ignore it
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8") as f:
        yaml.safe_dump(merged, f, sort_keys=False)
        temp = Path(f.name)
    try:
        return load_config(temp), diff
    finally:
        temp.unlink(missing_ok=True)


def calibration_provenance(overlay_path: Path) -> dict[str, Any]:
    """The ``calibration:`` block: who measured what, when, how, and how well."""
    raw = yaml.safe_load(overlay_path.read_text(encoding="utf-8")) or {}
    return raw.get("calibration", {})


# --- reporting ---------------------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, help="robot config (default: $KARMEL_CONFIG or labs/config/karmel.yaml)")
    parser.add_argument("--overlay", type=Path, help="calibration file to merge over the config")
    parser.add_argument("--diff", type=Path, help="another config file to compare against")
    parser.add_argument("--sensitivity", action="store_true", help="what a calibration error costs")
    parser.add_argument("--quiet", action="store_true", help="only print problems")
    args = parser.parse_args(argv)

    base_path = find_config_path(args.config)
    try:
        cfg = load_config(base_path)
    except ConfigError as exc:
        print(f"ERROR   {exc}")
        return 1

    overlay_changes: list[tuple[str, Any, Any]] = []
    if args.overlay:
        cfg, overlay_changes = load_with_overlay(base_path, args.overlay)

    if not args.quiet:
        print(f"config   {base_path}")
        print(f"robot    {cfg.robot.name}, {cfg.robot.mass_kg} kg")
        print("\nderived values (computed, never stored twice):")
        for key, value in derived(cfg).items():
            print(f"  {key:<30} {value}")

        if args.overlay:
            prov = calibration_provenance(args.overlay)
            print(f"\ncalibration overlay {args.overlay}")
            for key in ("date", "operator", "robot_serial", "method", "uncertainty", "notes"):
                if key in prov:
                    print(f"  {key:<12} {prov[key]}")
            for key, old, new in overlay_changes:
                delta = f"  ({100 * (new - old) / old:+.2f} %)" if _numeric(old, new) and old else ""
                print(f"  {key:<34} {old!r} -> {new!r}{delta}")
            if not overlay_changes:
                print("  (the overlay changes nothing)")

        if args.diff:
            other = yaml.safe_load(args.diff.read_text(encoding="utf-8"))
            base_raw = yaml.safe_load(base_path.read_text(encoding="utf-8"))
            print(f"\ndiff against {args.diff}")
            for key, old, new in changes(base_raw, other) or [("(identical)", None, None)]:
                print(f"  {key:<34} {old!r} -> {new!r}")

        if args.sensitivity:
            print("\nwhat a calibration error costs (lessons 01.12, 09.05):")
            for label, cost in odometry_sensitivity(cfg):
                print(f"  {label:<34} {cost}")

    problems = plausibility_problems(cfg)
    errors = [p for p in problems if p.severity == "error"]
    if problems:
        print("\nvalidation:" if not args.quiet else "validation:")
        for problem in problems:
            print(f"  {problem}")
    elif not args.quiet:
        print("\nvalidation: no problems")
    return 1 if errors else 0


def _numeric(*values: Any) -> bool:
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in values)


_ = math

if __name__ == "__main__":
    raise SystemExit(main())
