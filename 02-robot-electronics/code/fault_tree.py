"""02.09 — "the motor doesn't move": a fault tree you walk with a multimeter, as code.

The power reaches the motor through a CHAIN (battery -> fuse -> switch -> driver VIN), the
commands reach it through another chain (Pico -> GPIO -> wire -> IN pins), and the driver also
needs its enable and mode pins right. Both chains meet at OUT1/OUT2.

Two strategies are compared on simulated faults:
  * walk:   measure every point from the source to the load, stop at the first bad one
  * bisect: measure in the middle of a chain; good -> the fault is downstream, bad -> upstream

    python 02-robot-electronics/code/fault_tree.py                  simulate every fault
    python 02-robot-electronics/code/fault_tree.py --interactive    you type the meter readings
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class Check:
    key: str
    how: str                   # what to do with the meter
    lo: float                  # accepted range
    hi: float
    unit: str = "V"

    def ok(self, value: float) -> bool:
        return self.lo <= value <= self.hi


# Command: 50 % forward, drive/brake PWM (IN1 = 100 %, IN2 = 50 %), pack at ~11-12.6 V.
POWER_CHAIN = [
    Check("pack", "DC V across the battery pack output (BMS P+ to P-), switch ON", 9.9, 12.6),
    Check("after_fuse", "DC V from fuse output to battery -", 9.9, 12.6),
    Check("after_switch", "DC V from switch output to battery -", 9.9, 12.6),
    Check("drv_vin", "DC V at the DRV8874 carrier VIN to its GND pin", 9.9, 12.6),
]
PICO_ALIVE = Check("pico_alive", "Pico runs code: `mpremote exec \"print(1)\"` answers (1 = yes, 0 = no)", 1, 1, "")
IN1_CHAIN = [
    PICO_ALIVE,
    Check("gpio_in1_at_pico", "DC V at Pico GP2 to Pico GND (a meter averages PWM)", 3.0, 3.4),
    Check("in1_at_driver", "DC V at carrier IN1 to carrier GND", 3.0, 3.4),
]
IN2_CHAIN = [
    PICO_ALIVE,
    Check("gpio_in2_at_pico", "DC V at Pico GP3 to Pico GND", 1.4, 1.9),
    Check("in2_at_driver", "DC V at carrier IN2 to carrier GND", 1.4, 1.9),
]
ENABLE_CHECKS = [
    Check("sleep", "DC V at carrier SLEEP to GND (must be logic high)", 1.5, 5.5),
    Check("pmode", "DC V at carrier PMODE to GND when the driver WOKE UP (latched!)", 1.5, 5.5),
]
OUTPUT = Check("out", "DC V across OUT1-OUT2 at the carrier (about duty x VIN)", 4.0, 7.0)
LOAD_CHAIN = [
    Check("motor_terminals", "DC V across the motor's own terminals (after connectors)", 4.0, 7.0),
    Check("motor_current", "current into the motor, A (INA219 or meter in series)", 0.05, 1.5, "A"),
]

Measure = Callable[[Check], float]


def walk(chain: list[Check], measure: Measure, log: list[str]) -> Check | None:
    for c in chain:
        v = measure(c)
        log.append(f"{c.key}={v}")
        if not c.ok(v):
            return c
    return None


def bisect(chain: list[Check], measure: Measure, log: list[str]) -> Check | None:
    """Assumes a series chain: once a point is bad, every point after it is bad too."""
    lo, hi = 0, len(chain) - 1
    last = measure(chain[hi])
    log.append(f"{chain[hi].key}={last}")
    if chain[hi].ok(last):
        return None
    while lo < hi:
        mid = (lo + hi) // 2
        v = measure(chain[mid])
        log.append(f"{chain[mid].key}={v}")
        if chain[mid].ok(v):
            lo = mid + 1
        else:
            hi = mid
    return chain[lo]


def diagnose(measure: Measure, strategy: str = "bisect") -> tuple[str, list[str]]:
    find = bisect if strategy == "bisect" else walk
    log: list[str] = []
    out_v = measure(OUTPUT)
    log.append(f"out={out_v}")
    if OUTPUT.ok(out_v):
        bad = find(LOAD_CHAIN, measure, log)
        if bad is None:
            return "electrically fine: mechanical (gearbox, hub set screw, wheel jammed)", log
        if bad.key == "motor_terminals":
            return "open circuit between driver OUT and motor: connector, crimp or wire", log
        return "voltage at motor but no current: open winding/brushes, or current reads > limit: seized", log
    bad = find(POWER_CHAIN, measure, log)
    if bad is not None:
        prev = {"pack": "battery empty or BMS tripped", "after_fuse": "fuse blown or holder open",
                "after_switch": "switch off or broken", "drv_vin": "wire/connector between switch and driver VIN"}
        return prev[bad.key], log
    for c in ENABLE_CHECKS:
        v = measure(c)
        log.append(f"{c.key}={v}")
        if not c.ok(v):
            return ("SLEEP low/floating: driver asleep" if c.key == "sleep"
                    else "PMODE was low at wake-up: PH/EN mode latched; tie PMODE high and power-cycle"), log
    bad = find(IN1_CHAIN, measure, log) or find(IN2_CHAIN, measure, log)
    if bad is not None:
        causes = {"pico_alive": "Pico not running (USB, firmware, main.py crashed)",
                  "gpio_in1_at_pico": "firmware not driving GP2 (wrong pin number, watchdog stopped motors)",
                  "gpio_in2_at_pico": "firmware not driving GP3 (wrong pin number)",
                  "in1_at_driver": "IN1 wire/Dupont open between Pico and carrier, or missing common GND",
                  "in2_at_driver": "IN2 wire/Dupont open between Pico and carrier, or missing common GND"}
        return causes[bad.key], log
    return "inputs, power and enables are good but OUT is wrong: driver fault (nFAULT? thermal? dead chip)", log


# ------------------------------------------------------------------------------------------------
# A simulated karmel with one injected fault
# ------------------------------------------------------------------------------------------------
GOOD = {"pack": 11.8, "after_fuse": 11.8, "after_switch": 11.8, "drv_vin": 11.7, "sleep": 3.3, "pmode": 3.3,
        "pico_alive": 1, "gpio_in1_at_pico": 3.3, "gpio_in2_at_pico": 1.65, "in1_at_driver": 3.3,
        "in2_at_driver": 1.65, "out": 5.8, "motor_terminals": 5.8, "motor_current": 0.25}

# fault name: (readings that differ from GOOD, a phrase the verdict must contain)
FAULTS: dict[str, tuple[dict[str, float], str]] = {
    "blown fuse": ({"after_fuse": 0.0, "after_switch": 0.0, "drv_vin": 0.0, "out": 0.0,
                    "motor_terminals": 0.0, "motor_current": 0.0}, "fuse blown"),
    "switch off": ({"after_switch": 0.0, "drv_vin": 0.0, "out": 0.0, "motor_terminals": 0.0,
                    "motor_current": 0.0}, "switch off"),
    "VIN wire loose": ({"drv_vin": 0.0, "out": 0.0, "motor_terminals": 0.0, "motor_current": 0.0}, "driver VIN"),
    "SLEEP floating": ({"sleep": 0.0, "out": 0.0, "motor_terminals": 0.0, "motor_current": 0.0}, "SLEEP"),
    "PMODE low at power-up": ({"pmode": 0.0, "out": 0.0, "motor_terminals": 0.0, "motor_current": 0.0}, "PMODE"),
    "IN2 on wrong GPIO in config.py": ({"gpio_in2_at_pico": 0.0, "in2_at_driver": 0.0, "out": 11.7,
                                "motor_terminals": 11.7, "motor_current": 3.0}, "GP3"),
    "IN1 Dupont open": ({"in1_at_driver": 0.0, "out": 0.0, "motor_terminals": 0.0, "motor_current": 0.0}, "IN1 wire"),
    "no common ground": ({"in1_at_driver": 0.4, "in2_at_driver": 0.2, "out": 0.0, "motor_terminals": 0.0,
                          "motor_current": 0.0}, "IN1 wire"),
    "motor connector open": ({"motor_terminals": 0.0, "motor_current": 0.0}, "open circuit"),
    "hub set screw loose": ({}, "mechanical"),
    "empty battery / BMS tripped": ({"pack": 0.0, "after_fuse": 0.0, "after_switch": 0.0, "drv_vin": 0.0,
                                     "out": 0.0, "motor_terminals": 0.0, "motor_current": 0.0}, "BMS"),
}


def simulated(fault: str) -> Measure:
    values = dict(GOOD)
    values.update(FAULTS[fault][0])
    return lambda check: values[check.key]


def interactive() -> None:
    def ask(c: Check) -> float:
        while True:
            raw = input(f"{c.how}  [{c.lo}-{c.hi} {c.unit} is good] > ")
            try:
                return float(raw)
            except ValueError:
                print("type a number")
    print("Command 50 % forward (wheels in the air) and keep it running while you measure.")
    verdict, log = diagnose(ask)
    print("\nVERDICT:", verdict)
    print("measurements:", ", ".join(log))


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--interactive", action="store_true")
    args = ap.parse_args(argv)
    if args.interactive:
        interactive()
        return {}
    out = {}
    print(f"{'injected fault':30} {'walk':>5} {'bisect':>6}  verdict")
    for fault, (_, expect) in FAULTS.items():
        v_walk, log_walk = diagnose(simulated(fault), "walk")
        v_bis, log_bis = diagnose(simulated(fault), "bisect")
        assert v_walk == v_bis, (fault, v_walk, v_bis)
        out[fault] = (v_bis, len(log_walk), len(log_bis))
        print(f"{fault:30} {len(log_walk):5d} {len(log_bis):6d}  {v_bis}")
    return out


if __name__ == "__main__":
    main()
