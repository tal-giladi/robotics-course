"""Checker for 20.01 — the integration review, as code.

Run: ``python course.py check 20.01`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "20-final-robot" / "code",):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import system_contracts as sc  # noqa: E402

RELIABLE, BEST_EFFORT = "reliable", "best_effort"
VOLATILE, TRANSIENT_LOCAL = "volatile", "transient_local"


# --- qos_incompatibility ----------------------------------------------------------------------
def test_identical_profiles_match(impl):
    assert impl.qos_incompatibility(sc.Qos(), sc.Qos()) is None
    assert impl.qos_incompatibility(sc.SENSOR_DATA, sc.SENSOR_DATA) is None


def test_a_reliable_subscriber_gets_nothing_from_a_best_effort_publisher(impl):
    why = impl.qos_incompatibility(sc.SENSOR_DATA, sc.Qos(RELIABLE, VOLATILE, 10))
    assert why is not None
    assert "best_effort" in why and "reliable" in why


def test_asking_for_less_than_is_offered_is_fine(impl):
    assert impl.qos_incompatibility(sc.Qos(RELIABLE), sc.Qos(BEST_EFFORT)) is None
    assert impl.qos_incompatibility(sc.LATCHED, sc.Qos(RELIABLE, VOLATILE, 1)) is None


def test_a_late_joiner_needs_a_transient_local_publisher(impl):
    why = impl.qos_incompatibility(sc.Qos(RELIABLE, VOLATILE, 1), sc.LATCHED)
    assert why is not None and "transient_local" in why


def test_deadlines_are_request_versus_offered(impl):
    assert impl.qos_incompatibility(sc.Qos(deadline_s=0.05), sc.Qos(deadline_s=0.10)) is None
    slow = impl.qos_incompatibility(sc.Qos(deadline_s=0.20), sc.Qos(deadline_s=0.10))
    none_offered = impl.qos_incompatibility(sc.Qos(), sc.Qos(deadline_s=0.10))
    assert slow is not None and "deadline" in slow
    assert none_offered is not None, "a publisher that offers no deadline cannot satisfy one"


def test_reliability_is_reported_before_durability(impl):
    both = impl.qos_incompatibility(sc.Qos(BEST_EFFORT, VOLATILE), sc.LATCHED)
    assert both is not None and "best_effort" in both


# --- interface_rules --------------------------------------------------------------------------
def test_the_course_system_is_clean(impl):
    for contract in sc.karmel_system():
        assert impl.interface_rules(contract) == [], contract.topic


def test_two_publishers_on_an_actuator_topic(impl):
    cmd = next(c for c in sc.karmel_system() if c.topic == "/cmd_vel")
    doubled = sc.replace(cmd, publishers=cmd.publishers + (sc.Endpoint("agent_bridge", sc.Qos(), 2.0),))
    assert impl.interface_rules(doubled) == ["single-writer"]


def test_multi_writer_topics_are_allowed_to_have_many_publishers(impl):
    tf = next(c for c in sc.karmel_system() if c.topic == "/tf")
    assert len(tf.publishers) > 1 and impl.interface_rules(tf) == []


def test_a_frame_that_is_not_in_the_tf_tree(impl):
    imu = next(c for c in sc.karmel_system() if c.topic == "/imu/data")
    assert impl.interface_rules(sc.replace(imu, frame_id="imu")) == ["frame"]
    assert impl.interface_rules(sc.replace(imu, frame_id=None)) == []


def test_a_topic_nobody_subscribes_to(impl):
    scan = next(c for c in sc.karmel_system() if c.topic == "/scan")
    assert impl.interface_rules(sc.replace(scan, subscribers=())) == ["orphan"]


def test_a_subscriber_that_needs_more_than_the_publisher_gives(impl):
    odom = next(c for c in sc.karmel_system() if c.topic == "/odom")
    slow = sc.replace(odom, publishers=(sc.replace(odom.publishers[0], rate_hz=10.0),))
    assert impl.interface_rules(slow) == ["rate"]


def test_a_subscriber_with_no_stated_rate_is_not_a_rate_problem(impl):
    goal = next(c for c in sc.karmel_system() if c.topic == "/goal_pose")
    assert goal.subscribers[0].rate_hz == 0.0
    assert impl.interface_rules(goal) == []


def test_an_undocumented_timeout_is_only_a_problem_on_actuation(impl):
    cmd = next(c for c in sc.karmel_system() if c.topic == "/cmd_vel")
    assert impl.interface_rules(sc.replace(cmd, on_timeout="—")) == ["timeout"]
    scan = next(c for c in sc.karmel_system() if c.topic == "/scan")
    assert impl.interface_rules(sc.replace(scan, on_timeout="—")) == []


def test_several_rules_at_once_are_reported_together_and_sorted(impl):
    cmd = next(c for c in sc.karmel_system() if c.topic == "/cmd_vel")
    bad = sc.replace(cmd,
                     publishers=cmd.publishers + (sc.Endpoint("agent_bridge", sc.Qos(), 2.0),),
                     frame_id="base",
                     on_timeout="—")
    assert impl.interface_rules(bad) == ["frame", "single-writer", "timeout"]


def test_the_student_review_agrees_with_the_course_one(impl):
    """The same four planted faults, found by the same rules."""
    mine = sorted({rule for c in sc.broken_system() for rule in impl.interface_rules(c)})
    theirs = sorted({p.rule for p in sc.check_system(sc.broken_system())})
    assert mine == theirs == ["frame", "qos", "rate", "single-writer"]


def test_every_returned_name_is_a_known_rule(impl):
    for contract in sc.broken_system():
        assert set(impl.interface_rules(contract)) <= set(impl.RULES)


@pytest.mark.parametrize("topic", ["/scan", "/map", "/estop", "/diagnostics"])
def test_no_false_positives_on_the_tricky_interfaces(impl, topic):
    contract = next(c for c in sc.karmel_system() if c.topic == topic)
    assert impl.interface_rules(contract) == []
