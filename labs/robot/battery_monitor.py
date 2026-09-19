"""Watch the battery: voltage, estimated state of charge, low-voltage warning, safe shutdown.

Lesson 01.14 (measure and report battery state; low-battery shutdown).

    python labs/robot/battery_monitor.py --fake --once
    python labs/robot/battery_monitor.py --interval 5 --shutdown-command "sudo shutdown -h now"

Why care: a Li-ion cell discharged far below ~3.0 V is permanently damaged and can become
unsafe to charge. The 3S pack is protected by its BMS, but the robot should stop driving and
power down cleanly long before the BMS has to cut the power (which would also corrupt the Pi's
SD card).

State of charge from voltage is an ESTIMATE:
* the voltage-to-charge curve of Li-ion is flat in the middle (3.6-3.9 V covers most of it);
* under load the voltage sags (internal resistance), so it reads low while driving.
We therefore average several readings, and only act when the voltage stays low for
--confirm consecutive samples - a short sag when the motors start is not an empty battery.

Levels (from karmel.yaml, battery): full 12.6 V, low warning 10.5 V, cutoff 9.9 V for 3 cells.
At the cutoff this script stops the motors and runs --shutdown-command if you gave one
(otherwise it only prints what it would do).
"""

from __future__ import annotations

import argparse
import shlex
import subprocess
import time
from collections import deque
from dataclasses import dataclass

from robot_common import add_connection_args, connect

# Approximate resting (open-circuit) voltage per Li-ion cell vs state of charge, %.
# Typical of NMC 18650 cells at room temperature; real curves vary by cell and age.
CELL_VOLTAGE_TO_SOC = (
    (3.00, 0), (3.30, 2), (3.45, 5), (3.60, 12), (3.68, 20), (3.74, 30), (3.79, 40),
    (3.84, 50), (3.90, 60), (3.97, 70), (4.05, 80), (4.12, 90), (4.20, 100),
)


def estimate_soc_percent(pack_voltage: float, cells: int) -> float:
    """Linear interpolation in CELL_VOLTAGE_TO_SOC, clamped to 0..100 %."""
    cell_v = pack_voltage / cells
    table = CELL_VOLTAGE_TO_SOC
    if cell_v <= table[0][0]:
        return 0.0
    if cell_v >= table[-1][0]:
        return 100.0
    for (v_low, soc_low), (v_high, soc_high) in zip(table, table[1:]):
        if v_low <= cell_v <= v_high:
            return soc_low + (cell_v - v_low) / (v_high - v_low) * (soc_high - soc_low)
    raise AssertionError("unreachable")


@dataclass
class BatteryStatus:
    voltage: float
    soc_percent: float
    level: str  # "ok", "low", "critical"


class BatteryMonitor:
    """Averages readings and decides ok / low / critical with a confirmation count."""

    def __init__(self, cells: int, low_v: float, cutoff_v: float, window: int = 5, confirm: int = 3) -> None:
        self.cells = cells
        self.low_v = low_v
        self.cutoff_v = cutoff_v
        self.confirm = confirm
        self._readings: deque[float] = deque(maxlen=window)
        self._below_cutoff = 0

    def update(self, voltage: float) -> BatteryStatus:
        self._readings.append(voltage)
        average = sum(self._readings) / len(self._readings)
        self._below_cutoff = self._below_cutoff + 1 if average < self.cutoff_v else 0
        if self._below_cutoff >= self.confirm:
            level = "critical"
        elif average < self.low_v:
            level = "low"
        else:
            level = "ok"
        return BatteryStatus(average, estimate_soc_percent(average, self.cells), level)


def safe_shutdown(base: object, command: str | None) -> None:
    """Stop the motors, then (optionally) shut the computer down."""
    base.stop()  # type: ignore[attr-defined]  # SAFETY: first make sure nothing moves
    if not command:
        print("CRITICAL battery: motors stopped. Run with --shutdown-command to power off automatically.")
        return
    print(f"CRITICAL battery: motors stopped, running: {command}")
    subprocess.run(shlex.split(command), check=False)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interval", type=float, default=2.0, help="seconds between samples (default 2)")
    parser.add_argument("--confirm", type=int, default=3, help="consecutive low samples before shutdown")
    parser.add_argument("--shutdown-command", help='e.g. "sudo shutdown -h now" (default: only print)')
    parser.add_argument("--once", action="store_true", help="print one reading and exit")
    add_connection_args(parser)
    args = parser.parse_args(argv)

    with connect(args) as conn:
        p = conn.params
        monitor = BatteryMonitor(p.battery_cells, p.battery_low_warning_v, p.battery_cutoff_v, confirm=args.confirm)
        while True:
            state = conn.base.read()
            if state.battery_v is None:
                print("no battery reading (is the INA219 / divider connected?)")
            else:
                status = monitor.update(state.battery_v)
                print(f"{time.strftime('%H:%M:%S')}  {status.voltage:5.2f} V  "
                      f"({status.voltage / p.battery_cells:.2f} V/cell)  ~{status.soc_percent:3.0f} %  {status.level.upper()}")
                if status.level == "low":
                    print(f"  warning: below {p.battery_low_warning_v} V - finish up and charge the battery")
                if status.level == "critical":
                    safe_shutdown(conn.base, args.shutdown_command)
                    return
            if args.once:
                return
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
