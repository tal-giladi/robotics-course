"""02.08 — a design-rule check (DRC) for karmel's soldered robot board, written as data.

Describe the board as nets (what is connected to what) before you pick up the soldering iron,
then let the checker find the mistakes that are expensive to find with a meter later:

  * a net with only one connection (a wire to nowhere)
  * a pin in two nets (a short)
  * a pin exposed to more voltage than its datasheet allows
  * a power or bus net without a test point
  * a power net routed through a connector rated below its current
  * a supply net without its capacitor
  * a Pico pin that disagrees with labs/config/karmel.yaml

Run:  python 02-robot-electronics/code/board_drc.py
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]

# Maximum voltage each kind of pin may see (datasheets; the course rule for GPIO is 3.3 V).
PIN_LIMITS = {
    "pico_gpio": 3.3,          # FT pins take 5.5 V powered, but course rule: never above 3.3 V
    "pico_adc": 3.3,
    "pico_power_3v3": 3.3,
    "pico_vsys": 5.5,          # Pico 2 datasheet: VSYS 1.8-5.5 V
    "drv_vin": 37.0,           # Pololu 4035 operating range
    "drv_logic": 5.5,          # TI DRV8874 recommended logic input max
    "drv_cs": 5.75,
    "sensor_vin": 5.5,
    "ground": 0.0,
    "passive": 100.0,          # resistors, capacitors (check the capacitor's rating separately)
    "connector": 100.0,
    "testpoint": 100.0,
    "motor": 16.0,             # Yahboom: 11-16 V OK
}


@dataclass
class Net:
    name: str
    volts: float
    amps: float
    nodes: list[str]                              # "Component.pin"
    kind: str = "signal"                          # signal | power | ground | bus
    connectors: list[tuple[str, float]] = field(default_factory=list)   # (name, rated A)


# component -> {pin: pin kind}
COMPONENTS: dict[str, dict[str, str]] = {
    "PICO": {"GP2": "pico_gpio", "GP3": "pico_gpio", "GP6": "pico_gpio", "GP7": "pico_gpio",
             "GP4": "pico_gpio", "GP5": "pico_gpio", "GP10": "pico_gpio", "GP11": "pico_gpio",
             "GP12": "pico_gpio", "GP13": "pico_gpio", "GP14": "pico_gpio", "GP15": "pico_gpio",
             "GP27": "pico_adc", "3V3": "pico_power_3v3", "GND": "ground", "VSYS": "pico_vsys"},
    "DRV_L": {"VIN": "drv_vin", "GND": "ground", "IN1": "drv_logic", "IN2": "drv_logic",
              "SLEEP": "drv_logic", "PMODE": "drv_logic", "CS": "drv_cs"},
    "DRV_R": {"VIN": "drv_vin", "GND": "ground", "IN1": "drv_logic", "IN2": "drv_logic",
              "SLEEP": "drv_logic", "PMODE": "drv_logic", "CS": "drv_cs"},
    "INA219": {"VCC": "sensor_vin", "GND": "ground", "SDA": "sensor_vin", "SCL": "sensor_vin"},
    "TOF": {"VIN": "sensor_vin", "GND": "ground", "SDA": "sensor_vin", "SCL": "sensor_vin"},
    "US100": {"VCC": "sensor_vin", "GND": "ground", "TRIG": "sensor_vin", "ECHO": "sensor_vin"},
    "C_BULK": {"+": "passive", "-": "passive"},          # 470 uF 25 V
    "C_3V3": {"+": "passive", "-": "passive"},           # 10 uF + 100 nF at the sensor header
    "R_CS": {"1": "passive", "2": "passive"},            # 10 k series, CS -> GP27
    "TP": {f"{i}": "testpoint" for i in range(1, 12)},
}

# What each capacitor is expected to sit on.
REQUIRED_CAPS = {"VBAT_SW": "C_BULK", "3V3": "C_3V3"}


def karmel_board() -> list[Net]:
    return [
        Net("VBAT_SW", 12.6, 8.0, ["DRV_L.VIN", "DRV_R.VIN", "C_BULK.+", "TP.1"], "power", [("XT30 in", 15.0)]),
        Net("GND", 0.0, 8.0, ["DRV_L.GND", "DRV_R.GND", "C_BULK.-", "PICO.GND", "INA219.GND", "TOF.GND",
                              "US100.GND", "C_3V3.-", "TP.2"], "ground", [("XT30 in", 15.0)]),
        Net("3V3", 3.3, 0.11, ["PICO.3V3", "DRV_L.SLEEP", "DRV_L.PMODE", "DRV_R.SLEEP", "DRV_R.PMODE",
                               "INA219.VCC", "TOF.VIN", "US100.VCC", "C_3V3.+", "TP.3"], "power"),
        Net("L_IN1", 3.3, 0.0, ["PICO.GP2", "DRV_L.IN1", "TP.4"]),
        Net("L_IN2", 3.3, 0.0, ["PICO.GP3", "DRV_L.IN2", "TP.5"]),
        Net("R_IN1", 3.3, 0.0, ["PICO.GP6", "DRV_R.IN1", "TP.6"]),
        Net("R_IN2", 3.3, 0.0, ["PICO.GP7", "DRV_R.IN2", "TP.7"]),
        Net("SDA", 3.3, 0.0, ["PICO.GP4", "INA219.SDA", "TOF.SDA", "TP.8"], "bus"),
        Net("SCL", 3.3, 0.0, ["PICO.GP5", "INA219.SCL", "TOF.SCL", "TP.9"], "bus"),
        Net("L_CS", 3.3, 0.0, ["DRV_L.CS", "R_CS.1", "TP.10"]),
        Net("L_CS_ADC", 3.3, 0.0, ["R_CS.2", "PICO.GP27"]),
        Net("TRIG", 3.3, 0.0, ["PICO.GP14", "US100.TRIG"]),
        Net("ECHO", 3.3, 0.0, ["PICO.GP15", "US100.ECHO", "TP.11"]),
    ]


def pico_pins_from_yaml() -> dict[str, int]:
    cfg = yaml.safe_load((ROOT / "labs" / "config" / "karmel.yaml").read_text(encoding="utf-8"))
    return cfg["pins"]


# which yaml pin each net should use
YAML_EXPECT = {"L_IN1": "motor_left_in1", "L_IN2": "motor_left_in2", "R_IN1": "motor_right_in1",
               "R_IN2": "motor_right_in2", "SDA": "i2c_sda", "SCL": "i2c_scl",
               "TRIG": "ultrasonic_trig", "ECHO": "ultrasonic_echo", "L_CS_ADC": "motor_left_ipropi_adc"}


def drc(nets: list[Net], components: dict[str, dict[str, str]] = COMPONENTS,
        yaml_pins: dict[str, int] | None = None) -> list[str]:
    problems: list[str] = []
    seen: dict[str, str] = {}
    for net in nets:
        if len(net.nodes) < 2:
            problems.append(f"{net.name}: only one connection ({net.nodes})")
        for node in net.nodes:
            comp, _, pin = node.partition(".")
            if comp not in components or pin not in components[comp]:
                problems.append(f"{net.name}: unknown pin {node}")
                continue
            if node in seen:
                problems.append(f"{node} is in {seen[node]} AND {net.name}: short circuit")
            seen[node] = net.name
            limit = PIN_LIMITS[components[comp][pin]]
            kind = components[comp][pin]
            if kind != "ground" and net.volts > limit:
                problems.append(f"{net.name}: {net.volts} V exceeds {node} limit {limit} V")
        if net.kind in ("power", "ground", "bus") and not any(n.startswith("TP.") for n in net.nodes):
            problems.append(f"{net.name}: {net.kind} net without a test point")
        for conn, rated in net.connectors:
            if net.amps > 0.8 * rated:
                problems.append(f"{net.name}: {net.amps} A through {conn} rated {rated} A (keep <= 80 %)")
    names = {n.name: n for n in nets}
    for net_name, cap in REQUIRED_CAPS.items():
        if net_name in names and not any(x.startswith(cap + ".") for x in names[net_name].nodes):
            problems.append(f"{net_name}: missing {cap}")
    if yaml_pins is not None:
        for net_name, key in YAML_EXPECT.items():
            if net_name not in names:
                continue
            gpio = [x for x in names[net_name].nodes if x.startswith("PICO.GP")]
            want = f"PICO.GP{yaml_pins[key]}"
            if gpio and gpio[0] != want:
                problems.append(f"{net_name}: uses {gpio[0]} but karmel.yaml {key} = {yaml_pins[key]}")
    return problems


def main(argv: list[str] | None = None) -> list[str]:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.parse_args(argv)
    nets = karmel_board()
    problems = drc(nets, yaml_pins=pico_pins_from_yaml())
    print(f"{len(nets)} nets, {sum(len(n.nodes) for n in nets)} connections")
    print("DRC clean" if not problems else "\n".join("PROBLEM: " + p for p in problems))
    tps = sorted((x, n.name) for n in nets for x in n.nodes if x.startswith("TP."))
    print("Test points: " + ", ".join(f"{tp}={name}" for tp, name in sorted(tps, key=lambda t: int(t[0][3:]))))
    return problems


if __name__ == "__main__":
    main()
