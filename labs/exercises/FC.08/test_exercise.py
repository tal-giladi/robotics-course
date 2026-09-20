"""Checker for FC.08 — does ROS 2 fit on this microcontroller?

Run: ``python course.py check FC.08`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import pytest

KARMEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]


# --- CDR sizing -------------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("offset", "boundary", "expected"),
    [(0, 4, 0), (8, 4, 8), (9, 4, 12), (20, 8, 24), (24, 8, 24), (5, 1, 5)],
)
def test_align(impl, offset, boundary, expected):
    assert impl.align(offset, boundary) == expected


def test_align_rejects_nonsense(impl):
    with pytest.raises(ValueError):
        impl.align(-1, 4)
    with pytest.raises(ValueError):
        impl.align(8, 0)


def test_cdr_string_size(impl):
    assert impl.cdr_string_size(0, "base_link") == 14        # 4 + 9 + NUL
    assert impl.cdr_string_size(0, "") == 5                  # 4 + NUL
    assert impl.cdr_string_size(1, "ab") == 11               # pad 1->4, +4, +2, +NUL


def test_joint_state_is_124_bytes_for_karmel(impl):
    assert impl.joint_state_body_bytes(KARMEL_JOINTS, "base_link") == 124


def test_joint_state_grows_with_joints_and_efforts(impl):
    two = impl.joint_state_body_bytes(KARMEL_JOINTS, "base_link")
    with_effort = impl.joint_state_body_bytes(KARMEL_JOINTS, "base_link", efforts=2)
    assert with_effort - two == 20, "16 bytes of doubles plus 4 bytes of alignment padding"
    one = impl.joint_state_body_bytes(["left_wheel_joint"], "base_link")
    assert one < two


def test_framing_is_23_bytes(impl):
    assert impl.xrce_frame_bytes(124) == 147
    assert impl.xrce_frame_bytes(48) == 71                   # geometry_msgs/Twist
    assert impl.xrce_frame_bytes(0) == 23
    with pytest.raises(ValueError):
        impl.xrce_frame_bytes(-1)


# --- link budget ------------------------------------------------------------------------------------
def test_link_utilisation(impl):
    assert impl.link_utilisation(2350, 115200) == pytest.approx(0.2040, abs=1e-4)
    assert impl.link_utilisation(147 * 50, 115200) == pytest.approx(0.6380, abs=1e-4)
    assert impl.link_utilisation(11520, 115200) == pytest.approx(1.0)
    with pytest.raises(ValueError):
        impl.link_utilisation(100, 0)


def test_max_rate_truncates(impl):
    assert impl.max_rate_hz(147, 115200, 0.5) == 39          # 39.18 Hz of headroom
    assert impl.max_rate_hz(47, 115200, 0.5) == 122          # the ASCII line, same link
    assert impl.max_rate_hz(147, 115200, 1.0) == 78
    with pytest.raises(ValueError):
        impl.max_rate_hz(0, 115200)
    with pytest.raises(ValueError):
        impl.max_rate_hz(147, 115200, budget=1.5)


# --- the entity pool --------------------------------------------------------------------------------
def test_karmel_node_fits_the_default_pool(impl):
    design = impl.NodeDesign("karmel_wheels", publishers=1, subscriptions=1)
    assert impl.check_design(design) == []


def test_too_many_subscriptions_and_services(impl):
    design = impl.NodeDesign("greedy", publishers=1, subscriptions=6, services=2)
    assert impl.check_design(design) == [
        "subscriptions: 6 > 5 (RMW_UXRCE_MAX_SUBSCRIPTIONS)",
        "services: 2 > 1 (RMW_UXRCE_MAX_SERVICES)",
    ]


def test_one_node_per_sensor_hits_the_node_limit(impl):
    design = impl.NodeDesign("four nodes", publishers=4, subscriptions=1, nodes=4)
    assert impl.check_design(design) == ["nodes: 4 > 1 (RMW_UXRCE_MAX_NODES)"]


def test_a_bigger_pool_accepts_the_same_design(impl):
    design = impl.NodeDesign("greedy", publishers=1, subscriptions=6, services=2)
    roomy = impl.EntityLimits(nodes=2, publishers=10, subscriptions=8, services=4, clients=2)
    assert impl.check_design(design, roomy) == []


def test_negative_counts_are_a_bug(impl):
    with pytest.raises(ValueError):
        impl.check_design(impl.NodeDesign("bad", publishers=-1))


def test_publishers_are_not_executor_handles(impl):
    design = impl.NodeDesign("karmel_wheels", publishers=3, subscriptions=1)
    assert impl.executor_handles(design, timers=1) == 2
    assert impl.executor_handles(impl.NodeDesign("idle")) == 0
    with pytest.raises(ValueError):
        impl.executor_handles(design, timers=-1)


# --- the agent link ---------------------------------------------------------------------------------
def test_it_starts_disconnected_and_pings_immediately(impl):
    link = impl.AgentLink(now_ms=0)
    assert link.state == impl.WAITING_AGENT
    assert link.motors_enabled is False
    assert link.entities is False
    assert link.update(0, False) == ["ping"]
    assert link.update(100, False) == [], "one ping per ping_interval_ms, not one per loop"
    assert link.update(500, False) == ["ping"]


def test_a_good_ping_creates_entities_on_the_next_update(impl):
    link = impl.AgentLink(now_ms=0)
    assert link.update(0, True) == ["ping"]
    assert link.state == impl.AGENT_AVAILABLE
    assert link.motors_enabled is False, "nothing may drive before the entities exist"
    assert link.update(1, True) == ["create_entities"]
    assert link.state == impl.AGENT_CONNECTED
    assert link.motors_enabled is True and link.entities is True


def test_connected_spins_and_pings_on_its_own_interval(impl):
    link = impl.AgentLink(ping_interval_ms=500, connected_ping_interval_ms=200, now_ms=0)
    link.update(0, True)
    link.update(0, True)                       # -> AGENT_CONNECTED, last ping at t=0
    assert link.update(100, True) == ["spin"]
    assert link.update(200, True) == ["ping", "spin"]
    assert link.update(300, True) == ["spin"]


def test_a_lost_agent_brakes_before_it_tidies_up(impl):
    link = impl.AgentLink(now_ms=0)
    link.update(0, True)
    link.update(0, True)
    assert link.update(200, False) == ["ping"]
    assert link.state == impl.AGENT_DISCONNECTED
    assert link.update(205, False) == ["brake", "destroy_entities"]
    assert link.state == impl.WAITING_AGENT
    assert link.motors_enabled is False and link.entities is False


def test_the_full_timeline(impl):
    link = impl.AgentLink(ping_interval_ms=500, connected_ping_interval_ms=200, now_ms=0)
    timeline = [
        (0, False, ["ping"], impl.WAITING_AGENT),
        (100, False, [], impl.WAITING_AGENT),
        (500, False, ["ping"], impl.WAITING_AGENT),
        (1000, True, ["ping"], impl.AGENT_AVAILABLE),
        (1000, True, ["create_entities"], impl.AGENT_CONNECTED),
        (1005, True, ["spin"], impl.AGENT_CONNECTED),
        (1200, True, ["ping", "spin"], impl.AGENT_CONNECTED),
        (1400, False, ["ping"], impl.AGENT_DISCONNECTED),
        (1400, False, ["brake", "destroy_entities"], impl.WAITING_AGENT),
        (1500, False, [], impl.WAITING_AGENT),
        (1900, False, ["ping"], impl.WAITING_AGENT),
    ]
    for now_ms, ping_ok, actions, state in timeline:
        assert link.update(now_ms, ping_ok) == actions, f"at t={now_ms}"
        assert link.state == state, f"at t={now_ms}"


def test_motors_are_enabled_only_while_the_session_is_up(impl):
    """The motors may only turn while entities exist, and never after the brake."""
    link = impl.AgentLink(ping_interval_ms=100, connected_ping_interval_ms=50, now_ms=0)
    ping_ok = True
    for tick in range(0, 4000, 10):
        if tick in (1000, 2500):
            ping_ok = not ping_ok
        link.update(tick, ping_ok)
        if link.motors_enabled:
            assert link.entities is True
            assert link.state in (impl.AGENT_CONNECTED, impl.AGENT_DISCONNECTED)


def test_the_brake_follows_a_failed_ping_within_one_update(impl):
    link = impl.AgentLink(now_ms=0)
    link.update(0, True)
    link.update(0, True)
    link.update(200, False)                    # the ping fails: still driving, for one update
    assert link.motors_enabled is True
    assert "brake" in link.update(201, False)
    assert link.motors_enabled is False


def test_blind_travel(impl):
    assert impl.blind_travel_m(0.5, 200, 100, 10) == pytest.approx(0.155)
    assert impl.blind_travel_m(0.5, 500, 100, 10) == pytest.approx(0.305)
    assert impl.blind_travel_m(0.0, 500, 100, 10) == 0.0
    with pytest.raises(ValueError):
        impl.blind_travel_m(-1.0, 200, 100, 10)
