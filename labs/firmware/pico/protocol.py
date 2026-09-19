# protocol.py - Pi <-> Pico serial protocol v1, the firmware side.
#
# Lesson 01.10 (designing the protocol), 03.03 (making it robust).
#
# One message per line of ASCII text, with an NMEA-style checksum:
#
#     <payload>*<hh>\n        hh = XOR of every payload byte, as two uppercase hex digits
#
#     M 7 500 -500*77         "open-loop duty, sequence number 7, left +50 %, right -50 %"
#
# Why text and not binary? You can watch and type it in any serial terminal while debugging.
# Why a checksum? USB is reliable, but a line can still be cut in half when a cable is
# unplugged or the host program restarts mid-write; the checksum lets us reject it.
#
# This file is pure Python that runs unchanged on MicroPython (on the Pico) and on CPython
# (in the tests). The canonical, fully-typed host implementation is
# labs/python/robotlab/protocol.py; tests round-trip lines between the two so they can't drift.
#
# Host -> Pico commands:  H <seq> | M <seq> <l> <r> | V <seq> <l> <r> | S <seq> | R <seq>
#                         P <seq> <key> <value>
# Pico -> host replies:   I <fw> <proto> | A <seq> OK | A <seq> ERR <reason>
#                         T <ms> <lticks> <rticks> <l_mrad_s> <r_mrad_s> <batt_mV> <range_mm> <flags>

PROTOCOL_VERSION = 1
SEQ_MAX = 65535
DUTY_SCALE = 1000
PARAM_KEYS = ("kp", "ki", "kd", "ff", "watchdog_ms", "telemetry_hz")
INTEGER_PARAMS = ("watchdog_ms", "telemetry_hz")

FLAG_WATCHDOG = 1
FLAG_LOW_BATTERY = 2
FLAG_RANGE_ERROR = 4
FLAG_VELOCITY_MODE = 8

# Short error codes sent back in "A <seq> ERR <reason>".
ERR_BAD_ARGS = "bad_args"
ERR_OUT_OF_RANGE = "out_of_range"
ERR_UNKNOWN_COMMAND = "unknown_command"
ERR_UNKNOWN_PARAM = "unknown_param"

_HEX_DIGITS = "0123456789ABCDEFabcdef"


class ProtocolError(ValueError):
    """A line we cannot use. `reason` is one of the ERR_* codes (or a framing problem)."""

    def __init__(self, reason, seq=None):
        super().__init__(reason)
        self.reason = reason
        self.seq = seq      # known only if the line was framed correctly and had a valid seq


# --- framing ----------------------------------------------------------------------------------
def checksum(payload):
    """XOR of all payload characters (0..255)."""
    value = 0
    for ch in payload:
        value ^= ord(ch)
    return value


def frame(payload):
    """'A 7 OK' -> 'A 7 OK*72\\n'"""
    return "%s*%02X\n" % (payload, checksum(payload))


def unframe(line):
    """Check the checksum and return the payload. Raises ProtocolError on any problem.

    The firmware silently drops lines that fail here: if the line is corrupted we cannot
    trust the sequence number in it, so there is nobody to reply to.
    """
    line = line.rstrip("\r\n")
    if len(line) < 4 or line[-3] != "*":
        raise ProtocolError("no_checksum")
    payload = line[:-3]
    hex_digits = line[-2:]
    for ch in hex_digits:
        if ch not in _HEX_DIGITS:
            raise ProtocolError("bad_checksum_digits")
    for ch in payload:
        if ch < " " or ch > "~" or ch == "*":
            raise ProtocolError("bad_character")
    if int(hex_digits, 16) != checksum(payload):
        raise ProtocolError("bad_checksum")
    return payload


# --- field parsing ----------------------------------------------------------------------------
def parse_int(token):
    """Plain decimal integer ('-12' ok; '+12', '1.0', '' rejected). Returns None if invalid."""
    digits = token[1:] if token.startswith("-") else token
    if not digits:
        return None
    for ch in digits:
        if ch < "0" or ch > "9":
            return None
    return int(token)


def parse_number(token):
    """A finite decimal number ('0.02', '-3', '1e-05'). Returns None if invalid."""
    has_digit = False
    for ch in token:
        if ch in "0123456789":
            has_digit = True
        elif ch not in ".-+eE":
            return None
    if not has_digit:
        return None
    try:
        value = float(token)
    except ValueError:
        return None
    if value != value or value in (float("inf"), float("-inf")):   # NaN is not equal to itself
        return None
    return value


def parse_command(payload):
    """Parse a host -> Pico payload into a tuple (kind, seq, args).

        'M 7 500 -500'      -> ('M', 7, (500, -500))
        'V 8 1500 1500'     -> ('V', 8, (1500, 1500))
        'P 9 kp 0.05'       -> ('P', 9, ('kp', 0.05))
        'H 1' / 'S 2' / 'R 3' -> (kind, seq, ())

    Raises ProtocolError. When the seq could be read, error.seq is set so the caller can
    still reply 'A <seq> ERR <reason>'.
    """
    fields = payload.split(" ")
    for field in fields:
        if field == "":
            raise ProtocolError(ERR_BAD_ARGS)
    kind = fields[0]
    if len(fields) < 2:
        raise ProtocolError(ERR_BAD_ARGS)
    seq = parse_int(fields[1])
    if seq is None or seq < 0 or seq > SEQ_MAX:
        raise ProtocolError(ERR_BAD_ARGS)
    args = fields[2:]

    if kind in ("H", "S", "R"):
        if args:
            raise ProtocolError(ERR_BAD_ARGS, seq)
        return (kind, seq, ())

    if kind in ("M", "V"):
        if len(args) != 2:
            raise ProtocolError(ERR_BAD_ARGS, seq)
        left = parse_int(args[0])
        right = parse_int(args[1])
        if left is None or right is None:
            raise ProtocolError(ERR_BAD_ARGS, seq)
        if kind == "M" and (abs(left) > DUTY_SCALE or abs(right) > DUTY_SCALE):
            raise ProtocolError(ERR_OUT_OF_RANGE, seq)
        return (kind, seq, (left, right))

    if kind == "P":
        if len(args) != 2:
            raise ProtocolError(ERR_BAD_ARGS, seq)
        key = args[0]
        if key not in PARAM_KEYS:
            raise ProtocolError(ERR_UNKNOWN_PARAM, seq)
        if key in INTEGER_PARAMS:
            value = parse_int(args[1])
        else:
            value = parse_number(args[1])
        if value is None:
            raise ProtocolError(ERR_BAD_ARGS, seq)
        return (kind, seq, (key, value))

    raise ProtocolError(ERR_UNKNOWN_COMMAND, seq)


# --- formatting replies -----------------------------------------------------------------------
def format_hello_reply(firmware_version):
    return frame("I %s %d" % (firmware_version, PROTOCOL_VERSION))


def format_ack(seq):
    return frame("A %d OK" % seq)


def format_error(seq, reason):
    return frame("A %d ERR %s" % (seq, reason))


def format_telemetry(ms, left_ticks, right_ticks, left_mrad_s, right_mrad_s,
                     battery_mv, range_mm, flags):
    """All arguments are integers. Use -1 for "no reading" in battery_mv and range_mm."""
    return frame("T %d %d %d %d %d %d %d %d" % (
        ms, left_ticks, right_ticks, left_mrad_s, right_mrad_s, battery_mv, range_mm, flags))
