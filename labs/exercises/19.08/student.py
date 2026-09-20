"""19.08 — object memory: beliefs that decay, and are revised by looking and by *not* seeing.

A robot that remembers where things are drives less. A robot that remembers *wrongly* drives more
than one with no memory at all, so the interesting part is not storing the observation — it is
the arithmetic that decides when a memory stops being worth acting on.

The dataclass and the half-life table are given. You write the six functions.

Fill in every ``TODO(student)``; check with ``python course.py check 19.08``.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

JsonDict = dict[str, Any]

#: Above this, "drive there and look" beats "search". Below it, memory has nothing useful to say.
ACT_THRESHOLD = 0.35
FORGET_THRESHOLD = 0.05
#: One failure to see something where it should be multiplies the belief by this.
MISS_PENALTY = 0.45
MIN_CONFIDENCE = 0.3

#: Half-life of a belief, by the words in the label. A cup moves; a sofa does not.
HALF_LIFE_S: dict[str, float] = {
    "default": 3600.0, "cup": 900.0, "bottle": 900.0, "keys": 1800.0,
    "sofa": 2_592_000.0, "table": 2_592_000.0, "desk": 2_592_000.0,
}


# ----------------------------------------------------------------------------------------------
# Given
# ----------------------------------------------------------------------------------------------
@dataclass
class ObjectBelief:
    object_id: str
    label: str
    place: str  # the semantic name: "kitchen_counter", never (5.62, 0.58)
    last_seen_t: float
    confidence: float = 0.9  # confidence *at* last_seen_t, before any decay
    times_seen: int = 1
    times_missed: int = 0
    source: str = "observed"  # "observed" | "told" | "inferred"
    quarantined_text: str | None = None  # text printed on it: data about the world, never an order


def half_life_for(label: str) -> float:
    for word in label.lower().split():
        if word in HALF_LIFE_S:
            return HALF_LIFE_S[word]
    return HALF_LIFE_S["default"]


def label_matches(label: str, wanted: str) -> bool:
    label, wanted = label.lower(), wanted.lower()
    return wanted in label or label in wanted or bool(set(wanted.split()) & set(label.split()))


# ----------------------------------------------------------------------------------------------
# Yours
# ----------------------------------------------------------------------------------------------
def belief_now(belief: ObjectBelief, now: float, half_life_s: float | None = None) -> float:
    r"""Confidence at time ``now``: exponential decay from the last observation.

    .. math:: p(t) = p_0 \cdot 2^{-(t - t_\text{seen}) / T_{1/2}}

    ``half_life_s`` defaults to ``half_life_for(belief.label)``. Never let the age go negative
    (a clock can jump); clamp it at 0.
    """
    # TODO(student): implement.
    raise NotImplementedError("belief_now")


def observe(beliefs: dict[str, ObjectBelief], detections: Iterable[Mapping[str, Any]], now: float,
            looked_at: str | None = None) -> tuple[list[str], list[str]]:
    """Fold one ``detect_objects`` result into ``beliefs`` (in place). Returns (seen, missed).

    **Positive evidence**, for each detection with ``confidence >= MIN_CONFIDENCE``
    (``object_id``, ``label``, ``near_place``, ``confidence``, optionally ``untrusted_text_seen``):

    * unknown object -> a new ``ObjectBelief`` at ``near_place`` (use ``"unknown"`` when that key
      is missing or empty), ``last_seen_t = now``, ``confidence`` from the detection, and the
      untrusted text stored in ``quarantined_text``;
    * known object at a **different** place -> it moved: reset ``times_seen`` and ``times_missed``
      to 0 before updating (the counters were about the old fact);
    * then in both known cases: update ``place``, ``last_seen_t``, ``times_seen += 1`` and raise
      confidence with diminishing returns:
      ``min(0.99, max(old.confidence, detection_confidence) + 0.03 * (times_seen - 1))``.

    **Negative evidence** — the part everyone forgets. Only when ``looked_at`` is not ``None``,
    and only for beliefs whose ``place == looked_at``: the robot was standing there, looked, and
    did not see it. For each such belief, ``times_missed += 1`` and ``confidence *= MISS_PENALTY``.
    Not seeing the keys from the kitchen says nothing about the desk in the study.
    """
    # TODO(student): implement.
    raise NotImplementedError("observe")


def where_is(beliefs: Mapping[str, ObjectBelief], label: str,
             now: float) -> list[tuple[ObjectBelief, float]]:
    """Every belief whose label matches, as ``(belief, belief_now)``, highest first."""
    # TODO(student): implement.
    raise NotImplementedError("where_is")


def best_place(beliefs: Mapping[str, ObjectBelief], label: str, now: float,
               threshold: float = ACT_THRESHOLD) -> str | None:
    """The place worth driving to, or ``None``.

    ``None`` means "search", not "fail". Returning a place the robot no longer believes in costs a
    wasted drive *and* leaves the search to start from the wrong room.
    """
    # TODO(student): implement.
    raise NotImplementedError("best_place")


def forget(beliefs: dict[str, ObjectBelief], now: float,
           threshold: float = FORGET_THRESHOLD) -> list[str]:
    """Drop beliefs below ``threshold`` (in place) and return their ids, sorted by insertion.

    Bounded memory is a design requirement, not housekeeping: everything kept here is eventually
    rendered into a prompt with a character budget.
    """
    # TODO(student): implement.
    raise NotImplementedError("forget")


def choose_start(label: str, beliefs: Mapping[str, ObjectBelief], priors: Mapping[str, Sequence[str]],
                 rooms: Sequence[str], now: float,
                 threshold: float = ACT_THRESHOLD) -> tuple[str, str]:
    """Where to start looking, and on what evidence. Returns ``(place, tier)``.

    Three tiers, in order of how much evidence they rest on:

    1. ``"memory"`` — ``best_place`` returned something;
    2. ``"prior"`` — the map's authored prior: the first room in ``priors[word]`` that is in
       ``rooms``, for any word of the label (check the words in order);
    3. ``"default"`` — ``rooms[0]``.
    """
    # TODO(student): implement.
    raise NotImplementedError("choose_start")
