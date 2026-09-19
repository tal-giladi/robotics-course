"""Read TF2 error messages like a debugger would (lesson 05.11). No rclpy import.

    explain('Lookup would require extrapolation into the future.  Requested time 1789.50 '
            'but the latest data is at time 1789.45, when looking up transform from frame '
            '[base_footprint] to frame [map]')
    -> Diagnosis(kind='future', gap_s=0.05, frames=('base_footprint', 'map'), hint='asked for ...')

The diagnosis rules are the decision procedure of 05.11's Troubleshooting section, as code.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

_FLOAT = r'([0-9]+(?:\.[0-9]*)?)'
_FUTURE = re.compile(r'extrapolation into the future.*?Requested time ' + _FLOAT
                     + r' but the latest data is at time ' + _FLOAT, re.S)
_PAST = re.compile(r'extrapolation into the past.*?Requested time ' + _FLOAT
                   + r' but the earliest data is at time ' + _FLOAT, re.S)
_FRAMES = re.compile(r'from frame \[([^\]]+)\] to frame \[([^\]]+)\]')
_MISSING = re.compile(r'"([^"]+)" passed to lookupTransform argument (\w+) does not exist')
_UNCONNECTED = re.compile(r"Could not find a connection between '([^']+)' and '([^']+)'")

#: Wall-clock stamps in 2026 are ~1.79e9 s. Sim time starts at 0. A ratio this big means two clocks.
CLOCK_MISMATCH_RATIO = 1000.0


@dataclass(frozen=True)
class Diagnosis:
    kind: str                     # future | past | missing-frame | unconnected | unknown
    gap_s: float | None           # |requested - available| for extrapolation errors
    frames: tuple[str, ...]
    hint: str


def explain(message: str, buffer_length_s: float = 10.0) -> Diagnosis:
    """Classify a TF2 exception text and suggest the most likely cause."""
    frames_match = _FRAMES.search(message)
    frames = frames_match.groups() if frames_match else ()

    for kind, pattern in (('future', _FUTURE), ('past', _PAST)):
        m = pattern.search(message)
        if not m:
            continue
        requested, available = float(m.group(1)), float(m.group(2))
        gap = abs(requested - available)
        small, big = sorted((requested, available))
        if big > 0 and (small == 0 or big / max(small, 1e-9) > CLOCK_MISMATCH_RATIO):
            hint = ('two different clocks: one side uses simulation time and the other wall time '
                    '(check use_sim_time on every node, and that nobody else publishes /clock)')
        elif kind == 'future' and gap < 0.2:
            hint = ('asked for a time newer than the newest transform (now() or a fresh message '
                    'stamp): use Time() for "latest", or wait/retry briefly for the data stamp')
        elif kind == 'future':
            hint = ('the transform is far behind the data: a publisher is slow, stopped or stamps '
                    'late; check the edge with tf2_monitor, or a device clock is ahead')
        elif gap > buffer_length_s / 2.0:
            hint = (f'the data is older than the buffer holds ({buffer_length_s:.0f} s by default): '
                    'stale stamps, a slow pipeline, or the node started after the data was taken')
        else:
            hint = ('the data predates the first transform this node received: normal for a '
                    'moment after startup, otherwise stale stamps')
        return Diagnosis(kind, round(gap, 6), frames, hint)

    m = _MISSING.search(message)
    if m:
        return Diagnosis('missing-frame', None, (m.group(1),),
                         f'nobody publishes "{m.group(1)}" (yet): typo, namespace prefix, '
                         'node not running, or queried before the first message arrived')
    m = _UNCONNECTED.search(message)
    if m:
        return Diagnosis('unconnected', None, m.groups(),
                         'two separate trees: a missing edge (localizer or odometry not running), '
                         'or one frame got a second parent and split the tree')
    return Diagnosis('unknown', None, frames, 'not a message this helper recognises')
