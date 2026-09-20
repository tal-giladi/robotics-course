"""19.08 — reference solution: object beliefs that decay, and are revised by negative evidence."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

JsonDict = dict[str, Any]

ACT_THRESHOLD = 0.35
FORGET_THRESHOLD = 0.05
MISS_PENALTY = 0.45
MIN_CONFIDENCE = 0.3

HALF_LIFE_S: dict[str, float] = {
    "default": 3600.0, "cup": 900.0, "bottle": 900.0, "keys": 1800.0,
    "sofa": 2_592_000.0, "table": 2_592_000.0, "desk": 2_592_000.0,
}


@dataclass
class ObjectBelief:
    object_id: str
    label: str
    place: str
    last_seen_t: float
    confidence: float = 0.9
    times_seen: int = 1
    times_missed: int = 0
    source: str = "observed"
    quarantined_text: str | None = None


def half_life_for(label: str) -> float:
    for word in label.lower().split():
        if word in HALF_LIFE_S:
            return HALF_LIFE_S[word]
    return HALF_LIFE_S["default"]


def label_matches(label: str, wanted: str) -> bool:
    label, wanted = label.lower(), wanted.lower()
    return wanted in label or label in wanted or bool(set(wanted.split()) & set(label.split()))


def belief_now(belief: ObjectBelief, now: float, half_life_s: float | None = None) -> float:
    t_half = half_life_s if half_life_s is not None else half_life_for(belief.label)
    age = max(0.0, now - belief.last_seen_t)
    return float(belief.confidence * 2.0 ** (-age / t_half))


def observe(beliefs: dict[str, ObjectBelief], detections: Iterable[Mapping[str, Any]], now: float,
            looked_at: str | None = None) -> tuple[list[str], list[str]]:
    seen: list[str] = []
    for d in detections:
        confidence = float(d.get("confidence", 0.9))
        if confidence < MIN_CONFIDENCE:
            continue
        oid = str(d["object_id"])
        place = str(d.get("near_place") or "unknown")
        seen.append(oid)
        old = beliefs.get(oid)
        if old is None:
            beliefs[oid] = ObjectBelief(oid, str(d["label"]), place, now, confidence,
                                        quarantined_text=d.get("untrusted_text_seen"))
            continue
        if old.place != place:
            old.times_seen, old.times_missed = 0, 0
        old.place, old.last_seen_t = place, now
        old.times_seen += 1
        old.confidence = min(0.99, max(old.confidence, confidence) + 0.03 * (old.times_seen - 1))
        if d.get("untrusted_text_seen"):
            old.quarantined_text = d["untrusted_text_seen"]
    missed: list[str] = []
    if looked_at is not None:
        for belief in beliefs.values():
            if belief.place == looked_at and belief.object_id not in seen:
                belief.times_missed += 1
                belief.confidence *= MISS_PENALTY
                missed.append(belief.object_id)
    return seen, missed


def where_is(beliefs: Mapping[str, ObjectBelief], label: str,
             now: float) -> list[tuple[ObjectBelief, float]]:
    hits = [(b, belief_now(b, now)) for b in beliefs.values() if label_matches(b.label, label)]
    return sorted(hits, key=lambda pair: pair[1], reverse=True)


def best_place(beliefs: Mapping[str, ObjectBelief], label: str, now: float,
               threshold: float = ACT_THRESHOLD) -> str | None:
    hits = where_is(beliefs, label, now)
    return hits[0][0].place if hits and hits[0][1] >= threshold else None


def forget(beliefs: dict[str, ObjectBelief], now: float,
           threshold: float = FORGET_THRESHOLD) -> list[str]:
    dead = [k for k, b in beliefs.items() if belief_now(b, now) < threshold]
    for k in dead:
        del beliefs[k]
    return dead


def choose_start(label: str, beliefs: Mapping[str, ObjectBelief], priors: Mapping[str, Sequence[str]],
                 rooms: Sequence[str], now: float,
                 threshold: float = ACT_THRESHOLD) -> tuple[str, str]:
    place = best_place(beliefs, label, now, threshold)
    if place is not None:
        return place, "memory"
    for word in label.lower().split():
        for room in priors.get(word, ()):
            if room in rooms:
                return room, "prior"
    return rooms[0], "default"
