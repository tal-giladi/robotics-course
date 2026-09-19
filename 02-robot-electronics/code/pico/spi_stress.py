# spi_stress.py - SPI0 loopback bit-error test: find the speed where your wiring stops working.
#
# Lesson 02.04. MicroPython on the Pico 2, from the repository root:
#     mpremote run 02-robot-electronics/code/pico/spi_stress.py
#
# Wiring: one jumper from GP19 (SPI0 TX / MOSI) to GP16 (SPI0 RX / MISO). GP18 is SCK (leave it
# unconnected, or clip a scope probe on it). Nothing else. These pins are free on karmel.
# Run it twice: with a 5 cm jumper, then with a 30-50 cm jumper, and compare the tables.

from machine import SPI, Pin

try:
    import time
    ticks_ms, ticks_diff = time.ticks_ms, time.ticks_diff
except AttributeError:                  # CPython test shims
    import utime
    ticks_ms, ticks_diff = utime.ticks_ms, utime.ticks_diff

BAUDS = (1_000_000, 8_000_000, 24_000_000, 48_000_000)
BLOCK = 64


def lcg_bytes(seed, n):
    """Deterministic pseudo-random test pattern (no `random` needed on the Pico)."""
    out = bytearray(n)
    x = seed & 0xFFFFFFFF
    for i in range(n):
        x = (1103515245 * x + 12345) & 0xFFFFFFFF
        out[i] = (x >> 16) & 0xFF
    return out


def bit_errors(sent, received):
    errors = 0
    for a, b in zip(sent, received):
        v = a ^ b
        while v:
            errors += v & 1
            v >>= 1
    return errors


def run(blocks=200, bauds=BAUDS):
    results = {}
    print("%10s %10s %10s %12s" % ("baud", "bits sent", "bit errors", "kbit/s real"))
    for baud in bauds:
        spi = SPI(0, baudrate=baud, polarity=0, phase=0, bits=8, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
        rx = bytearray(BLOCK)
        errors = 0
        t0 = ticks_ms()
        for k in range(blocks):
            tx = lcg_bytes(k + 1, BLOCK)
            spi.write_readinto(tx, rx)
            errors += bit_errors(tx, rx)
        ms = max(1, ticks_diff(ticks_ms(), t0))
        bits = blocks * BLOCK * 8
        results[baud] = errors
        print("%10d %10d %10d %12.0f" % (baud, bits, errors, bits / ms))
        spi.deinit()
    return results


if __name__ == "__main__":
    run()
