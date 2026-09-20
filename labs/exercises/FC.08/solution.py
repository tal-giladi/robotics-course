"""FC.08 — reference solution: wire budget, entity pool and the agent-link state machine."""

from __future__ import annotations

from dataclasses import dataclass

# --------------------------------------------------------------------------- 1. the wire
XRCE_SUBMESSAGE_HEADER = 4
XRCE_WRITE_DATA_OVERHEAD = 4
UXR_SESSION_HEADER = 8
SERIAL_FRAME_OVERHEAD = 7       # FLAG + SADD + RADD + LEN(2) + CRC(2)
FRAMING_BYTES = (XRCE_SUBMESSAGE_HEADER + XRCE_WRITE_DATA_OVERHEAD
                 + UXR_SESSION_HEADER + SERIAL_FRAME_OVERHEAD)


def align(offset: int, boundary: int) -> int:
    if offset < 0:
        raise ValueError("offset must be >= 0")
    if boundary <= 0:
        raise ValueError("boundary must be positive")
    remainder = offset % boundary
    return offset if remainder == 0 else offset + (boundary - remainder)


def cdr_string_size(offset: int, text: str) -> int:
    offset = align(offset, 4) + 4
    return offset + len(text.encode()) + 1


def joint_state_body_bytes(names: list[str], frame_id: str, efforts: int = 0) -> int:
    offset = 8                                   # header.stamp: int32 sec + uint32 nanosec
    offset = cdr_string_size(offset, frame_id)
    offset = align(offset, 4) + 4                # name sequence length
    for name in names:
        offset = cdr_string_size(offset, name)
    for count in (len(names), len(names), efforts):
        offset = align(offset, 4) + 4
        if count:
            offset = align(offset, 8) + 8 * count
    return offset


def xrce_frame_bytes(body_bytes: int) -> int:
    if body_bytes < 0:
        raise ValueError("body_bytes must be >= 0")
    return body_bytes + FRAMING_BYTES


def link_utilisation(bytes_per_second: float, baud: int, bits_per_byte: int = 10) -> float:
    if baud <= 0:
        raise ValueError("baud must be positive")
    return bytes_per_second * bits_per_byte / baud


def max_rate_hz(frame_bytes: int, baud: int, budget: float = 0.5,
                bits_per_byte: int = 10) -> int:
    if frame_bytes <= 0:
        raise ValueError("frame_bytes must be positive")
    if not 0.0 < budget <= 1.0:
        raise ValueError("budget must be in (0, 1]")
    return int(budget * baud / (frame_bytes * bits_per_byte))


# --------------------------------------------------------------------------- 2. the entity pool
@dataclass(frozen=True)
class EntityLimits:
    nodes: int = 1
    publishers: int = 10
    subscriptions: int = 5
    services: int = 1
    clients: int = 1


@dataclass(frozen=True)
class NodeDesign:
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
    out: list[str] = []
    for what in FIELDS:
        want = getattr(design, what)
        have = getattr(limits, what)
        if want < 0:
            raise ValueError(f"{what} must be >= 0")
        if want > have:
            out.append(f"{what}: {want} > {have} ({CMAKE_FLAG[what]})")
    return out


def executor_handles(design: NodeDesign, timers: int = 0) -> int:
    if timers < 0:
        raise ValueError("timers must be >= 0")
    return design.subscriptions + design.services + design.clients + timers


# --------------------------------------------------------------------------- 3. the agent link
WAITING_AGENT = "WAITING_AGENT"
AGENT_AVAILABLE = "AGENT_AVAILABLE"
AGENT_CONNECTED = "AGENT_CONNECTED"
AGENT_DISCONNECTED = "AGENT_DISCONNECTED"


class AgentLink:
    """See student.py for the contract; this mirrors the state machine in karmel_wheels.c."""

    def __init__(self, ping_interval_ms: int = 500, connected_ping_interval_ms: int = 200,
                 now_ms: int = 0) -> None:
        self.ping_interval_ms = ping_interval_ms
        self.connected_ping_interval_ms = connected_ping_interval_ms
        self.state = WAITING_AGENT
        self.motors_enabled = False
        self.entities = False
        self._last_ping_ms = now_ms - ping_interval_ms

    def update(self, now_ms: int, ping_ok: bool) -> list[str]:
        if self.state == WAITING_AGENT:
            if now_ms - self._last_ping_ms >= self.ping_interval_ms:
                self._last_ping_ms = now_ms
                if ping_ok:
                    self.state = AGENT_AVAILABLE
                return ["ping"]
            return []

        if self.state == AGENT_AVAILABLE:
            self.entities = True
            self.motors_enabled = True
            self.state = AGENT_CONNECTED
            return ["create_entities"]

        if self.state == AGENT_CONNECTED:
            if now_ms - self._last_ping_ms >= self.connected_ping_interval_ms:
                self._last_ping_ms = now_ms
                if not ping_ok:
                    self.state = AGENT_DISCONNECTED
                    return ["ping"]
                return ["ping", "spin"]
            return ["spin"]

        # AGENT_DISCONNECTED
        self.motors_enabled = False
        self.entities = False
        self.state = WAITING_AGENT
        self._last_ping_ms = now_ms
        return ["brake", "destroy_entities"]


def blind_travel_m(speed_m_s: float, ping_interval_ms: float, ping_timeout_ms: float,
                   control_period_ms: float) -> float:
    if speed_m_s < 0:
        raise ValueError("speed must be >= 0")
    return speed_m_s * (ping_interval_ms + ping_timeout_ms + control_period_ms) / 1000.0
