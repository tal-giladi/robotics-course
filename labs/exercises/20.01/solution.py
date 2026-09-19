"""20.01 — the integration review, as code (reference solution).

Two functions decide whether two parts of the final robot will actually talk to each other:

* ``qos_incompatibility`` — the DDS request/offered rules, which silently deliver **nothing**
  when a subscriber asks for more than a publisher promises;
* ``interface_rules`` — the rules an interface table has to obey before anybody wires it up.

Fill in every ``TODO(student)``; check with ``python course.py check 20.01``.
Standard library only. The objects you are given come from
``20-final-robot/code/system_contracts.py`` — read it, do not re-implement it.
"""

from __future__ import annotations

from typing import Any

#: The rule names ``interface_rules`` may return, in no particular order.
RULES = ("single-writer", "frame", "orphan", "qos", "rate", "timeout")


def qos_incompatibility(pub: Any, sub: Any) -> str | None:
    """Why this publisher and subscriber exchange no data, or ``None`` if they are compatible.

    ``pub`` and ``sub`` are ``Qos`` objects with ``.reliability`` (``"reliable"`` or
    ``"best_effort"``), ``.durability`` (``"volatile"`` or ``"transient_local"``), ``.depth`` and
    ``.deadline_s`` (``None`` = no deadline offered/requested).

    DDS matches on *request vs offered*: a subscriber may ask for the same or **less** than the
    publisher offers, never more. Return a short human sentence for the first violation you find,
    checked in this order:

    1. publisher ``best_effort``, subscriber ``reliable``
       -> ``"publisher offers best_effort, subscriber requests reliable"``
    2. publisher ``volatile``, subscriber ``transient_local``
       -> ``"publisher is volatile, subscriber requests transient_local"``
    3. the subscriber requests a deadline and the publisher offers none, or a longer one
       -> ``"offered deadline <x> is longer than the requested <y>s"``

    Only the substrings in quotes are checked, so the rest of the sentence is yours.

    >>> from system_contracts import Qos, SENSOR_DATA
    >>> qos_incompatibility(SENSOR_DATA, Qos()) is None
    False
    >>> qos_incompatibility(Qos(), SENSOR_DATA) is None
    True
    """
    if pub.reliability == "best_effort" and sub.reliability == "reliable":
        return "publisher offers best_effort, subscriber requests reliable - nothing is delivered"
    if pub.durability == "volatile" and sub.durability == "transient_local":
        return "publisher is volatile, subscriber requests transient_local - late joiners get nothing"
    if sub.deadline_s is not None and (pub.deadline_s is None or pub.deadline_s > sub.deadline_s):
        offered = "none" if pub.deadline_s is None else f"{pub.deadline_s:g}s"
        return f"offered deadline {offered} is longer than the requested {sub.deadline_s:g}s"
    return None


def interface_rules(contract: Any) -> list[str]:
    """Every rule ``contract`` breaks, as rule names from :data:`RULES` (no duplicates, sorted).

    ``contract`` is a ``system_contracts.Contract``: ``.topic``, ``.frame_id``, ``.publishers``
    and ``.subscribers`` (tuples of ``Endpoint`` with ``.node``, ``.qos``, ``.rate_hz``),
    ``.multi_writer_ok``, ``.on_timeout`` and ``.layer``. ``contract.rate_hz`` is the fastest
    publisher's rate.

    The rules:

    ==================  =========================================================================
    ``single-writer``   more than one publisher, and ``multi_writer_ok`` is False. Two writers on
                        an actuator topic is a race whose winner depends on scheduling.
    ``frame``           ``frame_id`` is not ``None`` and not in
                        ``system_contracts.KNOWN_FRAMES`` — a typo here costs an afternoon.
    ``orphan``          nothing subscribes. Either it is dead weight or a subscriber is missing.
    ``qos``             any publisher/subscriber pair for which ``qos_incompatibility`` is not
                        ``None``.
    ``rate``            a subscriber needs a higher ``rate_hz`` than the fastest publisher
                        provides. Ignore subscribers whose ``rate_hz`` is ``None`` or ``0``.
    ``timeout``         ``on_timeout`` is ``"—"`` (the em dash, meaning "not documented") and the
                        contract's ``layer`` is ``"act"`` or ``"safety"``: an actuation interface
                        must say what happens when the data stops.
    ==================  =========================================================================

    A healthy contract returns ``[]``.
    """
    import system_contracts

    broken: set[str] = set()
    if len(contract.publishers) > 1 and not contract.multi_writer_ok:
        broken.add("single-writer")
    if contract.frame_id is not None and contract.frame_id not in system_contracts.KNOWN_FRAMES:
        broken.add("frame")
    if not contract.subscribers:
        broken.add("orphan")
    for sub in contract.subscribers:
        for pub in contract.publishers:
            if qos_incompatibility(pub.qos, sub.qos) is not None:
                broken.add("qos")
        if sub.rate_hz and contract.rate_hz and sub.rate_hz > contract.rate_hz + 1e-9:
            broken.add("rate")
    if contract.on_timeout == "—" and contract.layer in ("act", "safety"):
        broken.add("timeout")
    return sorted(broken)
