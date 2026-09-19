"""13.16 — the semantic object map: many noisy 3D detections, one list of objects in `map`.

A pure-Python upgrade of the `object_memory` node from 05.10's practical challenge: association
uses the *covariance* the geometry produced (13.16), not a fixed radius, and each object is fused
as a Kalman update rather than a running average. No ROS imports, so it is unit-tested on a laptop.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# chi-square with 3 degrees of freedom. A 95 % gate (7.815) rejects 5 % of the detections that
# really do belong to the object, and every rejection creates a twin in the map — so the default
# here is the 99 % gate, and `merge()` repairs the splits that still happen.
GATE_CHI2_3DOF_95 = 7.815
GATE_CHI2_3DOF_99 = 11.345


@dataclass
class TrackedObject:
    """One object the robot believes exists, in the `map` frame."""

    object_id: int
    class_name: str
    position: np.ndarray                  # (3,) metres in map
    covariance: np.ndarray                # (3, 3) metres^2
    hits: int = 1
    first_seen_s: float = 0.0
    last_seen_s: float = 0.0
    score_sum: float = 0.0

    @property
    def mean_score(self) -> float:
        return self.score_sum / max(self.hits, 1)

    @property
    def sigma_m(self) -> float:
        """One standard deviation of the position estimate, as a single number."""
        return float(np.sqrt(np.trace(self.covariance) / 3))

    def mahalanobis2(self, position: np.ndarray, covariance: np.ndarray) -> float:
        """Squared Mahalanobis distance between this object and a new detection (10.04's gate)."""
        d = np.asarray(position, float) - self.position
        s = self.covariance + np.asarray(covariance, float)
        return float(d @ np.linalg.solve(s, d))

    def update(self, position: np.ndarray, covariance: np.ndarray, score: float, t_s: float,
               min_sigma_m: float = 0.02) -> None:
        """Kalman update of a static object: the new estimate is the inverse-variance weighted mean.

        The covariance is floored at `min_sigma_m`. Without that floor, fusing 30 detections claims
        millimetre certainty, which is a lie: the errors share a common cause (camera extrinsics,
        a biased depth scale), so they do not average away. A too-confident object then rejects its
        own next detection and the map grows a twin.
        """
        p, r = self.covariance, np.asarray(covariance, float)
        k = p @ np.linalg.inv(p + r)                       # 3x3 Kalman gain
        self.position = self.position + k @ (np.asarray(position, float) - self.position)
        self.covariance = (np.eye(3) - k) @ p
        np.fill_diagonal(self.covariance, np.maximum(np.diag(self.covariance), min_sigma_m ** 2))
        self.hits += 1
        self.score_sum += score
        self.last_seen_s = t_s


@dataclass
class SemanticObjectMap:
    """Associate detections to objects, fuse them, and forget what stopped being there.

    `min_hits` is the confirmation rule of 16.01: an object is only reported to the rest of the
    robot once several independent detections agree, which removes most single-frame false alarms.
    """

    gate: float = GATE_CHI2_3DOF_99
    min_hits: int = 3
    min_sigma_m: float = 0.02             # the estimate never claims to be better than this
    forget_after_s: float = 60.0
    max_sigma_m: float = 0.60             # reject detections too vague to place (e.g. no depth)
    objects: list[TrackedObject] = field(default_factory=list)
    _next_id: int = 1

    def observe(self, class_name: str, position, covariance, score: float,
                t_s: float) -> TrackedObject | None:
        """Fold one 3D detection in. Returns the object it belongs to, or None if it was rejected."""
        position = np.asarray(position, dtype=float)
        covariance = np.asarray(covariance, dtype=float)
        if np.sqrt(np.trace(covariance) / 3) > self.max_sigma_m:
            return None
        candidates = [o for o in self.objects if o.class_name == class_name]
        best, best_d2 = None, self.gate
        for obj in candidates:
            d2 = obj.mahalanobis2(position, covariance)
            if d2 < best_d2:
                best, best_d2 = obj, d2
        if best is None:
            best = TrackedObject(self._next_id, class_name, position, covariance,
                                 hits=1, first_seen_s=t_s, last_seen_s=t_s, score_sum=score)
            self._next_id += 1
            self.objects.append(best)
        else:
            best.update(position, covariance, score, t_s, self.min_sigma_m)
            self.merge()
        return best

    def merge(self) -> int:
        """Fold objects that turned out to be the same thing. One outlier detection is enough to
        start a twin; once both have been updated a few times they agree, and nothing else in the
        system would ever notice. Returns how many objects disappeared."""
        removed = 0
        changed = True
        while changed:
            changed = False
            for i, a in enumerate(self.objects):
                for b in self.objects[i + 1:]:
                    if a.class_name != b.class_name or a.mahalanobis2(b.position, b.covariance) >= self.gate:
                        continue
                    keep, drop = (a, b) if a.hits >= b.hits else (b, a)
                    keep.update(drop.position, drop.covariance, drop.mean_score, max(a.last_seen_s, b.last_seen_s),
                                self.min_sigma_m)
                    keep.hits += drop.hits - 1                 # update() already counted one
                    self.objects.remove(drop)
                    removed += 1
                    changed = True
                    break
                if changed:
                    break
        return removed

    def prune(self, t_s: float) -> int:
        """Forget objects not seen for `forget_after_s`. Returns how many were dropped."""
        before = len(self.objects)
        self.objects = [o for o in self.objects if t_s - o.last_seen_s <= self.forget_after_s]
        return before - len(self.objects)

    def confirmed(self) -> list[TrackedObject]:
        return [o for o in self.objects if o.hits >= self.min_hits]

    def nearest(self, class_name: str, position) -> TrackedObject | None:
        """The confirmed object of this class closest to a point (e.g. the robot), or None."""
        position = np.asarray(position, dtype=float)
        matches = [o for o in self.confirmed() if o.class_name == class_name]
        if not matches:
            return None
        return min(matches, key=lambda o: float(np.linalg.norm(o.position - position)))


if __name__ == "__main__":
    rng = np.random.default_rng(0)
    truth = {"bottle": np.array([2.05, 1.70, 0.12]), "cup": np.array([1.20, 2.40, 0.09])}
    cov = np.diag([0.05, 0.05, 0.03]) ** 2
    world = SemanticObjectMap()
    for step in range(30):
        for name, p in truth.items():
            world.observe(name, p + rng.multivariate_normal(np.zeros(3), cov), cov, 0.9, step * 0.2)
    world.observe("bottle", np.array([4.0, 0.0, 0.1]), cov, 0.55, 6.0)      # a one-frame false alarm
    for o in world.objects:
        err = float(np.linalg.norm(o.position - truth[o.class_name])) if o.hits > 1 else float("nan")
        print(f"#{o.object_id} {o.class_name:7s} hits {o.hits:3d}  sigma {o.sigma_m * 100:5.1f} cm  "
              f"error {err * 100:5.1f} cm  {'confirmed' if o.hits >= world.min_hits else 'unconfirmed'}")
