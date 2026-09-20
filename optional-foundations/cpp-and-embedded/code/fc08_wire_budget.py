"""FC.08 - what micro-ROS costs: bytes on the wire, memory on the chip, entities in the pool.

Run it on any machine:   py fc08_wire_budget.py      (or python3 fc08_wire_budget.py)
Standard library only. Nothing here needs a Pico or a ROS 2 installation.

Four questions, answered with numbers:
  1. How big is a ROS 2 message on the wire compared with karmel's ASCII protocol line?
  2. What fraction of the link does each one use, at karmel's rates?
  3. How much flash and RAM does micro-ROS add to the firmware? (measured, see the lesson)
  4. Which micro-ROS build-time entity limits would karmel's node hit?

The CDR sizes are a *budgeting model*, not a protocol specification: the body of the message is
counted with plain CDR alignment rules, and the XRCE submessage header and the serial framing are
added on top. It is accurate enough to choose a rate and wrong enough that you should measure the
real thing before you claim a number in a paper.
"""

from __future__ import annotations

from dataclasses import dataclass

# --------------------------------------------------------------------------- CDR sizing
def align(offset: int, boundary: int) -> int:
    """CDR pads every primitive to its own size, counted from the start of the body."""
    remainder = offset % boundary
    return offset if remainder == 0 else offset + (boundary - remainder)


def cdr_string(offset: int, text: str) -> int:
    """uint32 length (including the terminating NUL) followed by the bytes."""
    offset = align(offset, 4) + 4
    return offset + len(text.encode()) + 1


def cdr_sequence_of_double(offset: int, count: int) -> int:
    offset = align(offset, 4) + 4          # uint32 element count
    if count:
        offset = align(offset, 8) + 8 * count
    return offset


def joint_state_body(names: list[str], frame_id: str, efforts: int = 0) -> int:
    """sensor_msgs/msg/JointState: header, name[], position[], velocity[], effort[]."""
    offset = 0
    offset = align(offset, 4) + 4          # header.stamp.sec  (int32)
    offset = align(offset, 4) + 4          # header.stamp.nanosec (uint32)
    offset = cdr_string(offset, frame_id)
    offset = align(offset, 4) + 4          # name sequence length
    for name in names:
        offset = cdr_string(offset, name)
    offset = cdr_sequence_of_double(offset, len(names))   # position
    offset = cdr_sequence_of_double(offset, len(names))   # velocity
    offset = cdr_sequence_of_double(offset, efforts)      # effort
    return offset


def twist_body() -> int:
    """geometry_msgs/msg/Twist: six float64. No strings, no sequences: 48 bytes, always."""
    return 6 * 8


# --------------------------------------------------------------------------- framing
XRCE_SUBMESSAGE_HEADER = 4        # submessage id, flags, length
XRCE_WRITE_DATA_OVERHEAD = 4      # DATA payload header for the WRITE_DATA submessage
SERIAL_FRAME_OVERHEAD = 7         # FLAG + SADD + RADD + LEN(2) + CRC(2), before byte stuffing
UXR_SESSION_HEADER = 8            # session id, stream id, sequence number


def xrce_frame_bytes(body_bytes: int) -> int:
    """One topic sample as it leaves the Pico's USB port (stuffing ignored: see the docstring)."""
    return (body_bytes + XRCE_WRITE_DATA_OVERHEAD + XRCE_SUBMESSAGE_HEADER
            + UXR_SESSION_HEADER + SERIAL_FRAME_OVERHEAD)


def ascii_telemetry_line(ms: int = 123456, lticks: int = -20481, rticks: int = 20477,
                         l_mrad: int = 8500, r_mrad: int = 8500, batt_mv: int = 11480,
                         range_mm: int = 742, flags: int = 8) -> str:
    """karmel's protocol v1 telemetry line (labs/README.md), checksum included."""
    payload = f"T {ms} {lticks} {rticks} {l_mrad} {r_mrad} {batt_mv} {range_mm} {flags}"
    checksum = 0
    for byte in payload.encode():
        checksum ^= byte
    return f"{payload}*{checksum:02X}\n"


def link_utilisation(bytes_per_second: float, baud: int, bits_per_byte: int = 10) -> float:
    """8N1 serial: one start bit and one stop bit per byte, so 10 bits per byte."""
    return bytes_per_second * bits_per_byte / baud


# --------------------------------------------------------------------------- entity limits
@dataclass(frozen=True)
class EntityLimits:
    """micro-ROS build-time pool, from colcon.meta of micro_ros_raspberrypi_pico_sdk (jazzy)."""
    nodes: int = 1
    publishers: int = 10
    subscriptions: int = 5
    services: int = 1
    clients: int = 1
    history: int = 4


@dataclass(frozen=True)
class NodeDesign:
    name: str
    publishers: int
    subscriptions: int
    services: int = 0
    clients: int = 0
    nodes: int = 1
    notes: str = ""


def violations(design: NodeDesign, limits: EntityLimits = EntityLimits()) -> list[str]:
    out: list[str] = []
    for what in ("nodes", "publishers", "subscriptions", "services", "clients"):
        want, have = getattr(design, what), getattr(limits, what)
        if want > have:
            out.append(f"{what}: wants {want}, pool has {have} -> rebuild libmicroros")
    return out


# --------------------------------------------------------------------------- firmware size
@dataclass(frozen=True)
class Build:
    what: str
    text: int
    bss: int
    uf2: int = 0


# Measured in this course with Pico SDK 2.3.1, arm-none-eabi-gcc 13.2.1, RelWithDebInfo,
# PICO_BOARD=pico2. See the lesson's "Expected result".
BUILDS = [
    Build("FC.06 plain C firmware (blink + PWM + benchmark)", 31056, 2664, 54784),
    Build("micro-ROS Int32 publisher (upstream example)", 99932, 22272, 192512),
    Build("FC.08 karmel_wheels (Twist in, JointState out)", 120620, 23104, 233984),
]

PICO2_FLASH = 4 * 1024 * 1024
PICO2_SRAM = 520 * 1024


def main() -> None:
    names = ["left_wheel_joint", "right_wheel_joint"]
    joint_body = joint_state_body(names, "base_link")
    cmd_body = twist_body()
    line = ascii_telemetry_line()

    print("1. One sample on the wire")
    print(f"  sensor_msgs/JointState, 2 joints, frame_id 'base_link'")
    print(f"     CDR body                         {joint_body:5d} B")
    print(f"     + XRCE + session + serial frame  {xrce_frame_bytes(joint_body):5d} B")
    print(f"  karmel protocol v1 telemetry line")
    print(f"     {line!r}")
    print(f"     ASCII, checksum and newline      {len(line):5d} B")
    print(f"  ratio: {xrce_frame_bytes(joint_body) / len(line):.1f}x more bytes for the same information")
    v_line = "V 4321 8500 8500*4C\n"
    print(f"  geometry_msgs/Twist command          {xrce_frame_bytes(cmd_body):5d} B"
          f"   vs  V command line {len(v_line):2d} B")

    print("\n2. The link, at karmel's rates")
    for rate in (20, 50, 100, 200):
        ros_bps = rate * xrce_frame_bytes(joint_body)
        ascii_bps = rate * len(line)
        print(f"  {rate:3d} Hz  micro-ROS {ros_bps:7.0f} B/s ({100 * link_utilisation(ros_bps, 115200):5.1f} % of 115200 baud)"
              f"   ASCII {ascii_bps:7.0f} B/s ({100 * link_utilisation(ascii_bps, 115200):5.1f} %)")
    print("  Over USB CDC the baud rate is fiction - the port is a USB endpoint, not a UART, and")
    print("  full-speed USB moves 12 Mbit/s in 1 ms frames. The numbers above are what you need")
    print("  the day you move the link to real UART pins, a radio or an RS-485 pair.")

    print("\n3. What micro-ROS costs in the chip (measured, Pico 2, RelWithDebInfo)")
    print(f"  {'build':<48} {'flash':>9} {'static RAM':>11} {'flash %':>8} {'RAM %':>7}")
    for build in BUILDS:
        print(f"  {build.what:<48} {build.text:8d} B {build.bss:10d} B"
              f" {100 * build.text / PICO2_FLASH:7.2f} % {100 * build.bss / PICO2_SRAM:6.2f} %")
    delta_flash = BUILDS[2].text - BUILDS[0].text
    delta_ram = BUILDS[2].bss - BUILDS[0].bss
    print(f"  micro-ROS costs {delta_flash / 1024:.0f} kB of flash and {delta_ram / 1024:.0f} kB of"
          f" static RAM over the same job in plain C.")

    print("\n4. Entity pool (colcon.meta defaults of the Pico port)")
    limits = EntityLimits()
    designs = [
        NodeDesign("karmel_wheels (this lesson)", publishers=1, subscriptions=1),
        NodeDesign("wheels + range + IMU + diagnostics", publishers=4, subscriptions=1),
        NodeDesign("one node per sensor", publishers=4, subscriptions=1, nodes=4,
                   notes="RMW_UXRCE_MAX_NODES is 1"),
        NodeDesign("wheels + a parameter service + 6 subscriptions",
                   publishers=1, subscriptions=6, services=2),
    ]
    print(f"  pool: nodes {limits.nodes}, publishers {limits.publishers}, "
          f"subscriptions {limits.subscriptions}, services {limits.services}, "
          f"clients {limits.clients}, history {limits.history}")
    for design in designs:
        problems = violations(design, limits)
        verdict = "fits" if not problems else "; ".join(problems)
        print(f"  {design.name:<46} {verdict}")


if __name__ == "__main__":
    main()
