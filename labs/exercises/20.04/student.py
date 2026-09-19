"""20.04 — the safety case: stopping distance, safe speed, and which hazards are still open.

Three functions, and the first two are the whole argument for every speed limit on the robot:

* ``stopping_distance_m`` — how far it travels between "something is wrong" and "it is stopped";
* ``max_safe_speed_m_s``  — the inverse: the fastest you may drive given a margin and a
  mechanism;
* ``open_hazards``        — the hazards whose controls do not actually hold up.

Fill in every ``TODO(student)``; check with ``python course.py check 20.04``.
Standard library only (you will want ``math.sqrt``).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any


def stopping_distance_m(speed_m_s: float, dead_time_s: float, decel_m_s2: float) -> float:
    """Distance covered from the hazard appearing to the robot standing still.

    Two parts, and people forget the first one:

    $$d = \\underbrace{v\\,t_\\text{dead}}_{\\text{still at full speed}} +
          \\underbrace{\\frac{v^2}{2a}}_{\\text{braking}}$$

    ``dead_time_s`` is detection plus actuation: the sensor period, the monitor's tick, the DDS
    hop, the serial line. ``decel_m_s2`` is what happens *after* the command changes — note that
    an e-stop, which removes power, gives you coasting friction and not braking.

    Return ``float("inf")`` for ``decel_m_s2 <= 0`` (nothing is slowing the robot down) and
    ``0.0`` for a speed of zero.

    >>> round(stopping_distance_m(0.3, 0.195, 1.5), 4)
    0.0885
    >>> stopping_distance_m(0.0, 1.0, 1.5)
    0.0
    """
    raise NotImplementedError  # TODO(student)


def max_safe_speed_m_s(margin_m: float, dead_time_s: float, decel_m_s2: float) -> float:
    """The fastest speed whose stopping distance still fits inside ``margin_m``.

    Invert the formula above. With $d = m$ fixed, $\\frac{v^2}{2a} + v\\,t - m = 0$, and the
    positive root is

    $$v_\\text{max} = -a t + \\sqrt{a^2 t^2 + 2 a m}$$

    Return ``0.0`` for a non-positive margin. For ``decel_m_s2 <= 0`` there is no braking at all,
    so the only thing that fits is $v = m / t$ (and ``float("inf")`` when ``dead_time_s`` is
    also zero — a robot that stops instantly, which does not exist).

    >>> round(max_safe_speed_m_s(0.365, 0.195, 1.5), 3)
    0.794
    >>> round(max_safe_speed_m_s(0.365, 4.0, 1.5), 3)
    0.091
    """
    raise NotImplementedError  # TODO(student)


def open_hazards(hazards: Sequence[Mapping[str, Any]]) -> list[str]:
    """The ids of the hazards whose controls do not hold, sorted.

    Each hazard is a mapping with ``"id"``, ``"severity"`` (1-4) and ``"controls"``: a list of
    mappings with ``"layer"`` (``"hardware"``, ``"firmware"``, ``"onboard-software"``,
    ``"offboard-software"`` or ``"procedure"``) and ``"tested_by"`` (a string; empty means nobody
    ever tested it).

    A hazard is **open** when any of these is true:

    * it has no controls at all;
    * any of its controls has an empty ``"tested_by"`` — an untested control is a wish;
    * ``severity >= 3`` and no control is at the ``"hardware"`` layer. Severity 3 means injury or
      expensive damage, and every software layer can be removed by one bad deploy;
    * ``severity >= 4`` and fewer than two ``"hardware"`` controls. Severe hazards get defence in
      depth, because a single control also has a single failure mode.

    >>> open_hazards([{"id": "H1", "severity": 2, "controls": [{"layer": "firmware", "tested_by": "x"}]}])
    []
    >>> open_hazards([{"id": "H2", "severity": 3, "controls": [{"layer": "firmware", "tested_by": "x"}]}])
    ['H2']
    """
    raise NotImplementedError  # TODO(student)
