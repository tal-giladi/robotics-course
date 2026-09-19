"""Tests for the module-03.03 scripts: py -m pytest 03-robot-software/code/test_l0303_serial.py"""

from __future__ import annotations

import os
import random

import pytest

import l0303_cobs as cobs
import l0303_link_monitor as monitor


# --- COBS + CRC ----------------------------------------------------------------------------------
def test_cobs_wikipedia_example():
    assert cobs.cobs_encode(bytes([0x11, 0x22, 0x00, 0x33])) == bytes([0x03, 0x11, 0x22, 0x02, 0x33])


@pytest.mark.parametrize("n", [0, 1, 2, 253, 254, 255, 508, 1000])
def test_cobs_round_trip_and_no_zeros(n):
    for data in (os.urandom(n), bytes(n), bytes([7]) * n):
        encoded = cobs.cobs_encode(data)
        assert 0 not in encoded
        assert cobs.cobs_decode(encoded) == data
        assert len(encoded) <= n + 1 + n // 254  # the COBS overhead bound


def test_crc_check_value():
    assert cobs.crc16_ccitt_false(b"123456789") == 0x29B1


def test_frame_reader_resynchronizes_after_garbage_and_rejects_corruption():
    frames = [cobs.encode_frame(f"T {i} 5000 -5000".encode()) for i in range(50)]
    corrupted = bytearray(frames[10])
    corrupted[3] ^= 0x10
    stream = b"\x13\x37garbage" + b"".join(frames[:10]) + bytes(corrupted) + b"".join(frames[11:])
    rng = random.Random(1)
    reader = cobs.FrameReader()
    out, i = [], 0
    while i < len(stream):
        n = rng.randrange(1, 40)
        out += reader.feed(stream[i:i + n])
        i += n
    # Frame 0 is glued to the garbage prefix (no delimiter in between), frame 10 is corrupted.
    expected = [f"T {i} 5000 -5000".encode() for i in range(50) if i not in (0, 10)]
    assert out == expected
    assert reader.bad_frames == 2  # the garbage prefix glued to frame 0, and the corrupted frame


# --- the link monitor against a fake Pico whose cable is "pulled" every 0.7 s ------------------------
def test_link_monitor_sees_disconnects_and_reconnects(capsys):
    stats = monitor.main(["--fake", "--chaos", "0.8", "--duration", "3.2", "--interval", "0.25"])
    out = capsys.readouterr().out
    assert stats["reconnects"] >= 2, out
    assert stats["telemetry"] > 50, "telemetry resumes after every reconnect"
    assert "UP" in out


# --- XOR vs CRC experiment -------------------------------------------------------------------------
def test_error_detection_experiment_shapes():
    import l0303_error_detection as experiment

    results = experiment.main(["--trials", "4000", "--seed", "1"])
    assert results["1 bit flipped"] == (0.0, 0.0)  # both detect every single-bit error
    assert results["3 bits flipped"][0] == 0.0  # an odd number of flips always changes the XOR parity
    assert 0.09 < results["2 bits flipped"][0] < 0.16  # same bit position in two bytes: ~1/8 of pairs
    assert results["2 bits flipped"][1] == 0.0  # CRC-16 catches all double-bit errors at this length
