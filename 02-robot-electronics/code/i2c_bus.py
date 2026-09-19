"""02.04 — bus arithmetic for karmel: I2C pull-ups from bus capacitance, transaction timing,
and UART/SPI budgets (US-100 in UART mode, an SPI loopback).

Run:  python 02-robot-electronics/code/i2c_bus.py [--wire-cm 30] [--pullups 10000,10000]

I2C limits (NXP UM10204 / TI SLVA689): rise time 30 %->70 % of VDD <= 1000 ns (standard mode,
100 kHz) and <= 300 ns (fast mode, 400 kHz); bus capacitance <= 400 pF; V_OL <= 0.4 V at 3 mA.
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

T_RISE_MAX_S = {"standard 100 kHz": 1000e-9, "fast 400 kHz": 300e-9}
V_OL_MAX = 0.4
I_OL = 3e-3


@dataclass(frozen=True)
class BusNode:
    name: str
    pin_capacitance_pf: float
    note: str


KARMEL_I2C = [
    BusNode("Pico 2 GP4/GP5 pads", 5.0, "estimate"),
    BusNode("INA219 module", 3.0 + 5.0, "chip 3 pF (TI datasheet) + board traces (estimate)"),
    BusNode("Pololu VL53L1X carrier (level shifter)", 15.0, "estimate: MOSFET shifter + traces"),
]
WIRE_PF_PER_CM = 0.7    # loose Dupont jumpers, signal next to ground: ~50-100 pF/m (estimate)


def bus_capacitance_pf(nodes: list[BusNode], wire_cm: float) -> float:
    return sum(n.pin_capacitance_pf for n in nodes) + WIRE_PF_PER_CM * wire_cm


def rise_time_s(r_pullup: float, c_bus_pf: float) -> float:
    """30 % -> 70 % rise of an RC edge: t = RC * ln(0.7 / 0.3) = 0.8473 RC."""
    return 0.8473 * r_pullup * c_bus_pf * 1e-12


def r_pullup_range(c_bus_pf: float, vdd: float = 3.3) -> dict[str, tuple[float, float]]:
    r_min = (vdd - V_OL_MAX) / I_OL
    return {mode: (r_min, t / (0.8473 * c_bus_pf * 1e-12)) for mode, t in T_RISE_MAX_S.items()}


def parallel(*resistors: float) -> float:
    return 1.0 / sum(1.0 / r for r in resistors)


def register_read_s(freq_hz: float, n_bytes: int, addr_bytes: int = 1) -> float:
    """START + addr(9) + register address bytes (9 each) + repeated START + addr(9) + data (9 each) + STOP."""
    bits = 1 + 9 + 9 * addr_bytes + 1 + 9 + 9 * n_bytes + 1
    return bits / freq_hz


def uart_frame_s(baud: int, n_bytes: int, bits_per_byte: int = 10) -> float:
    return n_bytes * bits_per_byte / baud


def us100_uart_distance_mm(high: int, low: int) -> int:
    """US-100 UART mode: send 0x55, receive two bytes, distance = high*256 + low in mm."""
    return (high << 8) | low


def us100_uart_temperature_c(raw: int) -> int:
    """US-100 UART mode: send 0x50, receive one byte, temperature = raw - 45 degrees C."""
    return raw - 45


def speed_of_sound_m_s(temp_c: float) -> float:
    return 331.3 * math.sqrt(1 + temp_c / 273.15)


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--wire-cm", type=float, default=30.0, help="total SDA wire length on the bus")
    ap.add_argument("--pullups", default="10000,10000", help="pull-up resistors already on the boards, ohms")
    args = ap.parse_args(argv)

    c = bus_capacitance_pf(KARMEL_I2C, args.wire_cm)
    print(f"Bus capacitance estimate: {c:.0f} pF ({args.wire_cm:.0f} cm of wire)")
    ranges = r_pullup_range(c)
    for mode, (lo, hi) in ranges.items():
        print(f"  {mode:17}: pull-up between {lo:6.0f} and {hi:6.0f} ohm")
    existing = [float(x) for x in args.pullups.split(",") if x]
    rp = parallel(*existing)
    tr = rise_time_s(rp, c)
    print(f"Existing pull-ups {existing} in parallel = {rp:.0f} ohm -> rise time {tr * 1e9:.0f} ns "
          f"({'OK' if tr <= 300e-9 else 'too slow'} for 400 kHz, {'OK' if tr <= 1000e-9 else 'too slow'} for 100 kHz)")
    add = 4700.0
    rp2 = parallel(*existing, add)
    print(f"Add one 4.7 k pair: {rp2:.0f} ohm -> rise time {rise_time_s(rp2, c) * 1e9:.0f} ns, "
          f"low-level sink current {(3.3 - 0.4) / rp2 * 1000:.2f} mA (limit 3 mA)")
    c_max_fast = 300e-9 / (0.8473 * rp)
    print(f"With {rp:.0f} ohm the bus tolerates at most {c_max_fast * 1e12:.0f} pF at 400 kHz "
          f"= about {(c_max_fast * 1e12 - sum(n.pin_capacitance_pf for n in KARMEL_I2C)) / WIRE_PF_PER_CM:.0f} cm of jumper wire")

    print("\nTransaction timing")
    for f in (100_000, 400_000):
        print(f"  {f // 1000:3d} kHz: INA219 2-byte register read {register_read_s(f, 2) * 1e3:.2f} ms; "
              f"VL53L1X 17-byte result block (16-bit register address) {register_read_s(f, 17, 2) * 1e3:.2f} ms")
    print("  INA219 12-bit conversion takes 532 us typ (586 max): polling faster than ~1 kHz re-reads old data")

    t = uart_frame_s(9600, 1) + uart_frame_s(9600, 2)
    print(f"\nUS-100 UART mode at 9600 8N1: 0x55 out + 2 bytes back = {t * 1e3:.2f} ms on the wire; "
          f"example reply 0x02 0x0B = {us100_uart_distance_mm(0x02, 0x0B)} mm; temperature byte 0x45 = {us100_uart_temperature_c(0x45)} C")
    for temp in (10, 20, 35):
        print(f"  speed of sound at {temp:2d} C = {speed_of_sound_m_s(temp):.1f} m/s -> a 2.000 m target timed with 343 m/s "
              f"reads {2.0 * 343.0 / speed_of_sound_m_s(temp):.3f} m")

    for baud in (1_000_000, 10_000_000):
        kb = baud / 8 / 1000
        print(f"SPI at {baud / 1e6:.0f} MHz: {kb:.0f} kB/s raw; 64-byte loopback transfer {64 * 8 / baud * 1e6:.0f} us of clocking")
    return {"c_bus_pf": c, "ranges": ranges, "rp_existing": rp, "tr_existing_s": tr, "rp_with_4k7": rp2,
            "c_max_fast_pf": c_max_fast * 1e12}


if __name__ == "__main__":
    main()
