"""Checker for 19.05 — the behavior-tree tick engine.

Run: ``python course.py check 19.05`` (``--solution`` runs the reference).
No robot and no py_trees: these tests are about control flow only.
"""

from __future__ import annotations

import pytest


def scripted(impl, name, script, journal=None):
    """A leaf that returns ``script`` statuses in order (repeating the last), and journals its
    lifecycle as ('init'|'term', name) so the tests can see entries and halts."""
    S = impl.Status
    codes = [{"S": S.SUCCESS, "F": S.FAILURE, "R": S.RUNNING}[c] for c in script]
    journal = journal if journal is not None else []

    class Leaf(impl.Behaviour):
        def __init__(self):
            super().__init__(name)
            self.i = 0
            self.journal = journal

        def initialise(self):
            self.journal.append(("init", self.name))

        def update(self):
            out = codes[min(self.i, len(codes) - 1)]
            self.i += 1
            self.journal.append(("tick", self.name))
            return out

        def terminate(self, new_status):
            self.journal.append(("term", self.name, new_status.name))

    return Leaf()


# --- the tick protocol ------------------------------------------------------------------------
def test_initialise_runs_on_entry_not_on_every_tick(impl):
    journal = []
    leaf = scripted(impl, "A", "RRS", journal)
    for _ in range(3):
        leaf.tick()
    assert [e for e in journal if e[0] == "init"] == [("init", "A")], "a RUNNING node is not re-entered"
    assert journal[-1] == ("term", "A", "SUCCESS")


def test_a_finished_node_is_re_entered_on_the_next_tick(impl):
    journal = []
    leaf = scripted(impl, "A", "SS", journal)
    leaf.tick()
    leaf.tick()
    assert [e for e in journal if e[0] == "init"] == [("init", "A"), ("init", "A")]


def test_status_starts_invalid_and_tracks_the_last_tick(impl):
    leaf = scripted(impl, "A", "RS")
    assert leaf.status is impl.Status.INVALID
    assert leaf.tick() is impl.Status.RUNNING and leaf.status is impl.Status.RUNNING
    assert leaf.tick() is impl.Status.SUCCESS and leaf.status is impl.Status.SUCCESS


def test_stop_halts_a_running_node_and_its_children(impl):
    journal = []
    a, b = scripted(impl, "A", "R", journal), scripted(impl, "B", "R", journal)
    seq = impl.Sequence("SEQ", [a, b], memory=True)
    seq.tick()
    seq.stop(impl.Status.INVALID)
    assert ("term", "A", "INVALID") in journal, "the running child must be told it was halted"
    assert a.status is impl.Status.INVALID and seq.status is impl.Status.INVALID


def test_stop_does_not_terminate_a_node_that_was_not_running(impl):
    journal = []
    leaf = scripted(impl, "A", "S", journal)
    leaf.tick()
    journal.clear()
    leaf.stop(impl.Status.INVALID)
    assert journal == [], "terminate is the halt path for RUNNING nodes only"


# --- Sequence ---------------------------------------------------------------------------------
def test_sequence_succeeds_when_every_child_succeeds(impl):
    seq = impl.Sequence("SEQ", [scripted(impl, "A", "S"), scripted(impl, "B", "S")], memory=True)
    assert seq.tick() is impl.Status.SUCCESS


def test_sequence_fails_at_the_first_failure_and_skips_the_rest(impl):
    journal = []
    seq = impl.Sequence("SEQ", [scripted(impl, "A", "F", journal), scripted(impl, "B", "S", journal)],
                        memory=True)
    assert seq.tick() is impl.Status.FAILURE
    assert ("tick", "B") not in journal, "B must never run after A failed"


def test_sequence_with_memory_resumes_the_running_child(impl):
    journal = []
    a = scripted(impl, "A", "S", journal)
    b = scripted(impl, "B", "RRS", journal)
    seq = impl.Sequence("SEQ", [a, b], memory=True)
    assert [seq.tick() for _ in range(3)] == [impl.Status.RUNNING, impl.Status.RUNNING, impl.Status.SUCCESS]
    assert [e[1] for e in journal if e[0] == "tick"] == ["A", "B", "B", "B"], "A runs once, not three times"


def test_sequence_without_memory_re_ticks_from_the_first_child(impl):
    """The guard pattern: a condition to the left is re-asked while the work to its right runs."""
    journal = []
    guard = scripted(impl, "GUARD", "SSF", journal)
    work = scripted(impl, "WORK", "RRR", journal)
    seq = impl.Sequence("ROOT", [guard, work], memory=False)
    assert seq.tick() is impl.Status.RUNNING
    assert seq.tick() is impl.Status.RUNNING
    assert seq.tick() is impl.Status.FAILURE, "the guard failed on the third tick"
    assert [e[1] for e in journal if e[0] == "tick"] == ["GUARD", "WORK", "GUARD", "WORK", "GUARD"]
    assert ("term", "WORK", "INVALID") in journal, "the abandoned child must be halted (goal cancelled)"


def test_a_guard_under_a_memory_true_sequence_is_never_re_asked(impl):
    """The bug this exercise exists to make visible."""
    journal = []
    guard = scripted(impl, "GUARD", "SF", journal)
    work = scripted(impl, "WORK", "RRS", journal)
    seq = impl.Sequence("ROOT", [guard, work], memory=True)
    assert [seq.tick() for _ in range(3)] == [impl.Status.RUNNING, impl.Status.RUNNING, impl.Status.SUCCESS]
    assert [e[1] for e in journal if e[0] == "tick"].count("GUARD") == 1


# --- Selector ---------------------------------------------------------------------------------
def test_selector_takes_the_first_child_that_succeeds(impl):
    journal = []
    sel = impl.Selector("SEL", [scripted(impl, "A", "F", journal), scripted(impl, "B", "S", journal),
                                scripted(impl, "C", "S", journal)], memory=True)
    assert sel.tick() is impl.Status.SUCCESS
    assert ("tick", "C") not in journal


def test_selector_fails_only_when_every_child_fails(impl):
    sel = impl.Selector("SEL", [scripted(impl, "A", "F"), scripted(impl, "B", "F")], memory=True)
    assert sel.tick() is impl.Status.FAILURE


def test_selector_without_memory_halts_a_lower_priority_running_child(impl):
    """A higher-priority option becoming available must interrupt the fallback that is running."""
    journal = []
    high = scripted(impl, "HIGH", "FS", journal)
    low = scripted(impl, "LOW", "RRR", journal)
    sel = impl.Selector("SEL", [high, low], memory=False)
    assert sel.tick() is impl.Status.RUNNING
    assert sel.tick() is impl.Status.SUCCESS
    assert ("term", "LOW", "INVALID") in journal


# --- decorators -------------------------------------------------------------------------------
def test_retry_re_enters_the_child_after_a_failure(impl):
    journal = []
    child = scripted(impl, "PICK", "FFS", journal)
    retry = impl.Retry("PickWithRetries", child, num_failures=3)
    assert [retry.tick() for _ in range(3)] == [impl.Status.RUNNING, impl.Status.RUNNING, impl.Status.SUCCESS]
    assert [e[1] for e in journal if e[0] == "init"] == ["PICK", "PICK", "PICK"], "each attempt is an entry"


def test_retry_gives_up_after_the_budget(impl):
    retry = impl.Retry("R", scripted(impl, "A", "F"), num_failures=2)
    assert retry.tick() is impl.Status.RUNNING
    assert retry.tick() is impl.Status.FAILURE


def test_retry_passes_running_through(impl):
    retry = impl.Retry("R", scripted(impl, "A", "R"), num_failures=3)
    assert retry.tick() is impl.Status.RUNNING and retry.failures == 0


def test_retry_resets_its_counter_on_re_entry(impl):
    child = scripted(impl, "A", "FSFS")
    retry = impl.Retry("R", child, num_failures=2)
    assert retry.tick() is impl.Status.RUNNING   # failure 1
    assert retry.tick() is impl.Status.SUCCESS   # succeeded -> the decorator finished
    assert retry.tick() is impl.Status.RUNNING   # re-entered: the counter started again
    assert retry.failures == 1


def test_inverter_and_succeeder(impl):
    S = impl.Status
    assert impl.Inverter("I", scripted(impl, "A", "S")).tick() is S.FAILURE
    assert impl.Inverter("I", scripted(impl, "A", "F")).tick() is S.SUCCESS
    assert impl.Inverter("I", scripted(impl, "A", "R")).tick() is S.RUNNING
    assert impl.Succeeder("S", scripted(impl, "A", "F")).tick() is S.SUCCESS
    assert impl.Succeeder("S", scripted(impl, "A", "R")).tick() is S.RUNNING


def test_a_condition_never_returns_running(impl):
    flag = {"ok": True}
    c = impl.Condition("Ok?", lambda: flag["ok"])
    assert c.tick() is impl.Status.SUCCESS
    flag["ok"] = False
    assert c.tick() is impl.Status.FAILURE


# --- put it together --------------------------------------------------------------------------
def test_a_fetch_shaped_tree_runs_to_success(impl):
    """FetchTask = Sequence(memory=False)[ EStopReleased?, Sequence(memory=True)[ find, pick ] ]"""
    journal = []
    estop = {"pressed": False}
    find = impl.Selector("Find", [impl.Condition("Known?", lambda: False),
                                  impl.Retry("Look", scripted(impl, "detect", "FS", journal), 3)],
                         memory=True)
    pick = scripted(impl, "pick", "RRS", journal)
    root = impl.Sequence("FetchTask", [
        impl.Condition("EStopReleased?", lambda: not estop["pressed"]),
        impl.Sequence("Fetch", [find, pick], memory=True),
    ], memory=False)
    statuses = []
    for _ in range(10):
        statuses.append(root.tick())
        if statuses[-1] is not impl.Status.RUNNING:
            break
    assert statuses[-1] is impl.Status.SUCCESS
    assert [e[1] for e in journal if e[0] == "tick"] == ["detect", "detect", "pick", "pick", "pick"]


def test_the_guard_halts_the_running_work(impl):
    journal = []
    estop = {"pressed": False}
    work = scripted(impl, "navigate", "RRRRR", journal)
    root = impl.Sequence("FetchTask", [
        impl.Condition("EStopReleased?", lambda: not estop["pressed"]),
        impl.Sequence("Fetch", [work], memory=True),
    ], memory=False)
    assert root.tick() is impl.Status.RUNNING
    estop["pressed"] = True
    assert root.tick() is impl.Status.FAILURE
    assert ("term", "navigate", "INVALID") in journal, "the e-stop must cancel the running goal"
