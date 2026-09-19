"""How often does a corrupted telemetry line slip past XOR vs CRC-16? A Monte Carlo experiment.

Lesson 03.03.   python 03-robot-software/code/l0303_error_detection.py [--trials 200000]
"""

from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "labs" / "python"))

from robotlab import protocol  # noqa: E402

from l0303_cobs import crc16_ccitt_false  # noqa: E402


def xor(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def flip_bits(data: bytes, n: int, rng: random.Random) -> bytes:
    out = bytearray(data)
    for bit in rng.sample(range(len(data) * 8), n):
        out[bit // 8] ^= 1 << (bit % 8)
    return bytes(out)


def burst(data: bytes, length: int, rng: random.Random) -> bytes:
    out = bytearray(data)
    start = rng.randrange(len(data) - length)
    for i in range(start, start + length):
        out[i] = rng.randrange(256)
    return bytes(out)


def miss_rates(corrupt, payload: bytes, trials: int, rng: random.Random) -> tuple[float, float]:
    """Fraction of corrupted payloads that XOR / CRC-16 fail to detect."""
    good_x, good_c = xor(payload), crc16_ccitt_false(payload)
    miss_x = miss_c = corrupted = 0
    for _ in range(trials):
        bad = corrupt(payload, rng)
        if bad == payload:
            continue
        corrupted += 1
        miss_x += xor(bad) == good_x
        miss_c += crc16_ccitt_false(bad) == good_c
    return miss_x / corrupted, miss_c / corrupted


def main(argv: list[str] | None = None) -> dict[str, tuple[float, float]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=200_000)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    line = protocol.encode(protocol.Telemetry(123456, -5, 2464, 6283, -6283, 11850, -1, 9))
    payload = line[:-4].encode("ascii")
    print(f"line {line!r}: {len(line)} bytes, {len(line) * 50} bytes/s at 50 Hz")
    rng = random.Random(args.seed)
    cases = {
        "1 bit flipped": lambda p, r: flip_bits(p, 1, r),
        "2 bits flipped": lambda p, r: flip_bits(p, 2, r),
        "3 bits flipped": lambda p, r: flip_bits(p, 3, r),
        "8-byte garbage burst": lambda p, r: burst(p, 8, r),
    }
    results = {}
    for name, corrupt in cases.items():
        results[name] = miss_rates(corrupt, payload, args.trials, rng)
        print(f"{name:22s} undetected: XOR {results[name][0]:7.3%}   CRC-16 {results[name][1]:8.4%}")
    return results


if __name__ == "__main__":
    main()
