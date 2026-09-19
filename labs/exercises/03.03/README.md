# 03.03 — A line-framing parser that survives a real serial link

Lesson: [03.03 Robust serial communication — framing, timeouts, reconnection](../../../03-robot-software/03.03-robust-serial-communication.md)

A USB serial port delivers a *byte stream*, not messages. `read()` returns half a line, three
lines, or the tail of a line that started before you opened the port. This exercise builds the
receive side of protocol v1 so it copes with all of that, and adds the CRC you would use in a
protocol v2.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `xor_checksum(payload)` | protocol v1's checksum: XOR of the payload bytes |
| `crc16_ccitt_false(data)` | CRC-16/CCITT-FALSE, check value `0x29B1` for `b"123456789"` |
| `frame(payload)` | `"S 9"` → `b"S 9*4A\n"`, refusing payloads that would break the framing |
| `LineReader.feed(chunk)` | bytes in any split → list of verified payloads; counts `bad_lines` and `overflows`; bounded memory |

## Check

```bash
python course.py check 03.03              # your code
python course.py check 03.03 --solution   # the reference
```

The tests feed lines byte by byte, several lines in one chunk, CRLF endings, corrupted lines,
a partial first line, a 500-byte flood without newlines, a line of exactly the maximum length,
and a seeded fuzz stream of 200 telemetry lines with noise lines and bit flips cut into random
chunks. Every valid line must come out, in order, and nothing else. Two tests show why a CRC is
stronger than XOR: XOR can't see swapped bytes, and the CRC catches every single-bit flip.

## Hints

* Keep a `bytearray` buffer between calls. Search for `b"\n"` with `bytes.find`, don't loop byte by byte in Python.
* After an overflow you are in the middle of a line you can't trust: stay in "discarding" mode until the next newline.
* `int(b"4a", 16)` works on bytes. Iterating over `bytes` yields `int`s.
