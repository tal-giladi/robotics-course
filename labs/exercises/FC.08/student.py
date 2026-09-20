"""FC.08 — does ROS 2 fit on this microcontroller? Wire budget, entity pool, agent link.

Fill in every ``TODO(student)``. Check your work with ``python course.py check FC.08``.
Standard library only.

Three questions you must be able to answer before putting micro-ROS on a robot:
  1. Does the message fit through the link at the rate I want?   -> the wire budget
  2. Do my entities fit in the pool the library was built with?  -> ``check_design``
  3. What does the robot do when the agent disappears?           -> ``AgentLink``
"""

from __future__ import annotations

from dataclasses import dataclass

# --------------------------------------------------------------------------- 1. the wire
# Framing added to every topic sample on its way out of the Pico, in bytes.
XRCE_SUBMESSAGE_HEADER = 4      # submessage id + flags + length
XRCE_WRITE_DATA_OVERHEAD = 4    # WRITE_DATA payload header
UXR_SESSION_HEADER = 8          # session id + stream id + sequence number
SERIAL_FRAME_OVERHEAD = 7       # FLAG + SADD + RADD + LEN(2) + CRC(2)


def align(offset: int, boundary: int) -> int:
    """CDR alignment: round ``offset`` up to the next multiple of ``boundary``.

    ``align(8, 4) == 8``, ``align(9, 4) == 12``, ``align(20, 8) == 24``.
    A negative offset or a non-positive boundary raises ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("align")


def cdr_string_size(offset: int, text: str) -> int:
    """Append a CDR string at ``offset`` and return the new offset.

    A string is a uint32 length (aligned to 4) **including the terminating NUL**, followed by
    the bytes and the NUL. ``cdr_string_size(0, "base_link") == 14``.
    """
    # TODO(student)
    raise NotImplementedError("cdr_string_size")


def joint_state_body_bytes(names: list[str], frame_id: str, efforts: int = 0) -> int:
    """Serialized size of ``sensor_msgs/msg/JointState``, body only, in bytes.

    Fields in order: ``header.stamp`` (int32 sec + uint32 nanosec = 8 bytes at offset 0),
    ``header.frame_id`` (string), ``name`` (sequence of strings), then ``position``,
    ``velocity`` and ``effort`` — each a sequence of float64: a uint32 count aligned to 4,
    then, if the count is non-zero, the doubles aligned to 8.

    karmel's telemetry message — two joints, ``frame_id="base_link"``, no efforts — is
    **124 bytes**.
    """
    # TODO(student)
    raise NotImplementedError("joint_state_body_bytes")


def xrce_frame_bytes(body_bytes: int) -> int:
    """Body plus all four framing layers above. Negative input raises ``ValueError``."""
    # TODO(student)
    raise NotImplementedError("xrce_frame_bytes")


def link_utilisation(bytes_per_second: float, baud: int, bits_per_byte: int = 10) -> float:
    """Fraction of a serial link used. 8N1 costs 10 bits per byte (start + 8 + stop).

    ``link_utilisation(2350, 115200)`` is about 0.204. ``baud <= 0`` raises ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("link_utilisation")


def max_rate_hz(frame_bytes: int, baud: int, budget: float = 0.5,
                bits_per_byte: int = 10) -> int:
    """Highest whole-number rate whose traffic stays within ``budget`` of the link.

    Truncate, never round up: 46.6 Hz of headroom means 46 Hz.
    ``frame_bytes <= 0`` or ``budget`` outside (0, 1] raises ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("max_rate_hz")


# --------------------------------------------------------------------------- 2. the entity pool
@dataclass(frozen=True)
class EntityLimits:
    """micro-ROS is built with a FIXED pool. These are the Pico port's colcon.meta defaults."""
    nodes: int = 1
    publishers: int = 10
    subscriptions: int = 5
    services: int = 1
    clients: int = 1


@dataclass(frozen=True)
class NodeDesign:
    """What you want to run on the microcontroller."""
    name: str
    publishers: int = 0
    subscriptions: int = 0
    services: int = 0
    clients: int = 0
    nodes: int = 1


FIELDS = ("nodes", "publishers", "subscriptions", "services", "clients")
CMAKE_FLAG = {
    "nodes": "RMW_UXRCE_MAX_NODES",
    "publishers": "RMW_UXRCE_MAX_PUBLISHERS",
    "subscriptions": "RMW_UXRCE_MAX_SUBSCRIPTIONS",
    "services": "RMW_UXRCE_MAX_SERVICES",
    "clients": "RMW_UXRCE_MAX_CLIENTS",
}


def check_design(design: NodeDesign, limits: EntityLimits = EntityLimits()) -> list[str]:
    """Which pool limits this design exceeds, in the order of ``FIELDS``.

    One string per exceeded limit, formatted exactly as
    ``"subscriptions: 6 > 5 (RMW_UXRCE_MAX_SUBSCRIPTIONS)"`` — use ``CMAKE_FLAG``. An empty
    list means it fits. A negative count raises ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("check_design")


def executor_handles(design: NodeDesign, timers: int = 0) -> int:
    """Handles an rclc executor must be sized for: subscriptions + services + clients + timers.

    Publishers are **not** handles — nothing waits on them. ``timers < 0`` raises ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("executor_handles")


# --------------------------------------------------------------------------- 3. the agent link
WAITING_AGENT = "WAITING_AGENT"
AGENT_AVAILABLE = "AGENT_AVAILABLE"
AGENT_CONNECTED = "AGENT_CONNECTED"
AGENT_DISCONNECTED = "AGENT_DISCONNECTED"


class AgentLink:
    """The agent-connection state machine from ``code/fc08_micro_ros_pico/karmel_wheels.c``.

    Attributes you must maintain: ``state``, ``motors_enabled``, ``entities``.
    ``update(now_ms, ping_ok)`` is called from the superloop and returns the list of actions it
    performed, in the order it performed them. It makes **at most one state transition per call**.

    * ``__init__(ping_interval_ms=500, connected_ping_interval_ms=200, now_ms=0)`` starts in
      ``WAITING_AGENT`` with the motors disabled, no entities, and the ping clock set so that the
      **first** ``update`` pings.
    * ``WAITING_AGENT``: ping when ``now_ms - last_ping >= ping_interval_ms``; the ping resets the
      clock and returns ``["ping"]``; a successful ping moves to ``AGENT_AVAILABLE``. Otherwise
      return ``[]``.
    * ``AGENT_AVAILABLE``: create the entities (``entities = True``), enable the motors, move to
      ``AGENT_CONNECTED``, return ``["create_entities"]``. ``ping_ok`` is ignored here.
    * ``AGENT_CONNECTED``: ping when ``now_ms - last_ping >= connected_ping_interval_ms``; a good
      ping returns ``["ping", "spin"]``, a failed one returns ``["ping"]`` and moves to
      ``AGENT_DISCONNECTED``. Between pings, return ``["spin"]``.
    * ``AGENT_DISCONNECTED``: ``["brake", "destroy_entities"]`` — in that order, because the robot
      must stop before you spend time tearing down the session — then disable the motors, clear
      ``entities``, reset the ping clock and go back to ``WAITING_AGENT``.

    The invariant the tests check hardest: ``motors_enabled`` is true only while the session is
    up — from ``create_entities`` until the ``brake`` — and the brake always follows a failed ping
    within one ``update``.
    """

    def __init__(self, ping_interval_ms: int = 500, connected_ping_interval_ms: int = 200,
                 now_ms: int = 0) -> None:
        # TODO(student)
        raise NotImplementedError("AgentLink.__init__")

    def update(self, now_ms: int, ping_ok: bool) -> list[str]:
        # TODO(student)
        raise NotImplementedError("AgentLink.update")


def blind_travel_m(speed_m_s: float, ping_interval_ms: float, ping_timeout_ms: float,
                   control_period_ms: float) -> float:
    """How far the robot travels before a dead agent is noticed, in metres.

    Worst case: the agent dies just after a successful ping, so you wait a full
    ``ping_interval_ms``, then the ping itself takes ``ping_timeout_ms`` to fail, and the brake
    happens on the next control step (``control_period_ms``). A negative speed raises
    ``ValueError``.
    """
    # TODO(student)
    raise NotImplementedError("blind_travel_m")
