"""02.10 — when does a robot outgrow I2C and UART? Timing budgets for a Feetech bus-servo arm, CAN
and CAN FD, and an RS-485 cable-length check.

Sources: Feetech STS/SMS control table and baud table (LeRobot `feetech/tables.py`); packet
framing FF FF ID LEN INSTR PARAMS CHK (FE.11); CAN frame sizes (FE.19); RS-485 rule of thumb
length[m] x rate[bit/s] < 1e7 and 32 unit loads (TI SLLA272).

Run:  python 02-robot-electronics/code/bus_budget.py
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass


# --------------------------------------------------------------------------------------------
# Feetech STS bus: half-duplex UART, 8N1, one wire shared by the host and every servo
# --------------------------------------------------------------------------------------------
def sts_sync_write_bytes(n_servos: int, data_len: int) -> int:
    """FF FF FE LEN 83 ADDR DLEN [ID d0..d(n-1)] * n CHK"""
    return 2 + 1 + 1 + 1 + 1 + 1 + n_servos * (1 + data_len) + 1


def sts_sync_read_request_bytes(n_servos: int) -> int:
    """FF FF FE LEN 82 ADDR DLEN ID1..IDn CHK"""
    return 2 + 1 + 1 + 1 + 1 + 1 + n_servos + 1


def sts_status_bytes(data_len: int) -> int:
    """FF FF ID LEN ERR data CHK"""
    return 2 + 1 + 1 + 1 + data_len + 1


@dataclass(frozen=True)
class ArmCycle:
    n_servos: int = 6
    baud: int = 1_000_000
    return_delay_s: float = 0.0          # per servo reply; check the servo's Return_Delay_Time (addr 7)
    usb_latency_s: float = 0.002         # USB-serial adapter round trip (estimate; measure it)

    def bits_s(self, n_bytes: int) -> float:
        return n_bytes * 10 / self.baud

    def cycle_s(self) -> dict[str, float]:
        write = self.bits_s(sts_sync_write_bytes(self.n_servos, 2))           # goal positions
        request = self.bits_s(sts_sync_read_request_bytes(self.n_servos))     # ask for present positions
        replies = self.n_servos * (self.bits_s(sts_status_bytes(2)) + self.return_delay_s)
        wire = write + request + replies
        total = wire + 2 * self.usb_latency_s                                   # write and read transactions
        return {"wire_s": wire, "total_s": total, "max_hz": 1 / total}


# --------------------------------------------------------------------------------------------
# CAN classic vs CAN FD
# --------------------------------------------------------------------------------------------
def can_classic_frame_bits(payload: int, worst_stuffing: bool = True) -> int:
    """11-bit ID data frame. 34 stuffable header/CRC bits + 8*payload; worst case adds 1 stuff bit
    per 4 bits after the first; +10 unstuffed trailer bits, +3 interframe space."""
    stuffable = 34 + 8 * payload
    stuff = (stuffable - 1) // 4 if worst_stuffing else 0
    return stuffable + stuff + 10 + 3


def can_fd_frame_time_s(payload: int, nominal_bps: int, data_bps: int) -> float:
    """Approximation: ~30 bits of arbitration/control/ACK/EOF at the nominal rate; data + CRC
    (17 or 21 bits) + ~20 % stuffing at the data rate."""
    crc = 17 if payload <= 16 else 21
    slow_bits = 30
    fast_bits = (8 * payload + crc + 4) * 1.2
    return slow_bits / nominal_bps + fast_bits / data_bps


def joints_fit(n_joints: int, rate_hz: float, frame_s: float, frames_per_joint: int = 2,
               max_utilisation: float = 0.7) -> tuple[bool, float]:
    load = n_joints * frames_per_joint * rate_hz * frame_s
    return load <= max_utilisation, load


# --------------------------------------------------------------------------------------------
# RS-485
# --------------------------------------------------------------------------------------------
def rs485_ok(length_m: float, bps: float) -> bool:
    return length_m * bps < 1e7


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.parse_args(argv)
    out: dict = {}

    print("1) SO-101 follower: 6 STS3215 servos on one half-duplex wire")
    for baud, delay, usb in ((1_000_000, 0.0, 0.002), (1_000_000, 0.0, 0.016), (115_200, 0.0, 0.002), (1_000_000, 0.002, 0.002)):
        c = ArmCycle(baud=baud, return_delay_s=delay, usb_latency_s=usb).cycle_s()
        out[("arm", baud, delay, usb)] = c
        print(f"   {baud:>9} baud, reply delay {delay * 1000:3.0f} ms, USB latency {usb * 1000:3.0f} ms: "
              f"wire {c['wire_s'] * 1000:5.2f} ms, cycle {c['total_s'] * 1000:5.1f} ms -> max {c['max_hz']:5.0f} Hz")
    print(f"   packet sizes: sync write {sts_sync_write_bytes(6, 2)} B, sync read request {sts_sync_read_request_bytes(6)} B, "
          f"one status reply {sts_status_bytes(2)} B")

    print("\n2) 12-joint legged robot, command + status per joint at 500 Hz and 1 kHz")
    classic = can_classic_frame_bits(8) / 1_000_000
    fd8 = can_fd_frame_time_s(8, 1_000_000, 5_000_000)
    fd64 = can_fd_frame_time_s(64, 1_000_000, 5_000_000)
    for label, frame_s in (("CAN 1 Mbit/s, 8 B frames", classic), ("CAN FD 1/5 Mbit/s, 8 B frames", fd8)):
        for hz in (500, 1000):
            ok, load = joints_fit(12, hz, frame_s)
            out[(label, hz)] = load
            print(f"   {label:40} {hz:5d} Hz: frame {frame_s * 1e6:5.0f} us, bus load {100 * load:5.0f} % "
                  f"{'fits' if ok else 'split across buses'}")
    for hz in (500, 1000):
        load = hz * (fd64 + 12 * fd8)            # one 64 B frame with all commands + 12 short status frames
        out[("fd packed", hz)] = load
        print(f"   {'CAN FD, 12 commands packed in one 64 B frame':40} {hz:5d} Hz: bus load {100 * load:5.0f} % "
              f"{'fits' if load <= 0.7 else 'split across buses'}")
    print(f"   buses needed at 1 kHz, classic CAN: {math.ceil(12 * 2 * 1000 * classic / 0.7)}")

    print("\n3) RS-485 cable check (length x rate < 1e7)")
    for length, bps in ((2, 1_000_000), (15, 1_000_000), (50, 115_200), (1000, 9_600)):
        ok = rs485_ok(length, bps)
        out[("rs485", length, bps)] = ok
        print(f"   {length:5d} m at {bps:>9} bit/s: {'OK' if ok else 'too long/fast: slow down or check the cable graph'}")
    return out


if __name__ == "__main__":
    main()
