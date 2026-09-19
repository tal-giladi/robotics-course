"""15.07 — from a grasp to an executable reach: the tool pose, the waypoints, and the checks.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.07``.
Only the standard library and numpy are needed.

Convention for the whole of module 15 (and the SO-101's URDF in 14.08):

    base frame   x forward, y left, z up (REP-103)
    tool frame   z = the APPROACH axis (the way the gripper travels as it closes)
                 x = the CLOSING direction (the way the pads travel)
                 y = z x x

The reference implementation lives at ``15-manipulation/code/reach_pipeline.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given -----------------------------------------------------------------------------------
@dataclass(frozen=True)
class Waypoint:
    """One pose on the way to the object, with the reason it exists and how fast to go there."""

    name: str
    T_base_tool: Array
    why: str
    speed_scale: float = 1.0

    @property
    def position(self) -> Array:
        return np.asarray(self.T_base_tool, dtype=float)[:3, 3]


def se3(R: ArrayLike, t: ArrayLike) -> Array:
    out = np.eye(4)
    out[:3, :3] = np.asarray(R, dtype=float)
    out[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return out


def wrap_axis(a: float) -> float:
    """Wrap an *axis* angle to (-pi/2, pi/2]: a parallel jaw at theta and theta+pi is one grasp."""
    a = (a + math.pi / 2) % math.pi - math.pi / 2
    return a if a > -math.pi / 2 else a + math.pi


# --- implement these ---------------------------------------------------------------------------
def approach_axis(pitch_rad: float, azimuth_rad: float) -> Array:
    """Unit vector the gripper travels ALONG as it closes on the object.

    ``pitch_rad`` is measured **below the horizontal** (pi/2 = straight down, 0 = horizontal),
    matching ``arm_kinematics.approach_pitch``. ``azimuth_rad`` is the compass direction the
    gripper leans from.

        a = (cos(pitch) cos(azimuth),  cos(pitch) sin(azimuth),  -sin(pitch))

    Check yourself: pitch = pi/2 must give (0, 0, -1) for any azimuth.
    """
    raise NotImplementedError("approach_axis")  # TODO(student)


def grasp_tool_pose(position: ArrayLike, jaw_yaw: float, *, pitch_rad: float = math.pi / 2,
                    azimuth_rad: float | None = None) -> Array:
    """The 4x4 tool pose that puts the pads at ``position`` closing along ``jaw_yaw``.

    * ``z`` is ``approach_axis(pitch_rad, azimuth)``. When ``azimuth_rad`` is None, use
      ``atan2(y, x)`` of the position, so the arm reaches over its own shoulder.
    * ``x`` is the horizontal closing direction ``(cos jaw_yaw, sin jaw_yaw, 0)`` **projected
      perpendicular to z and renormalised** — the two are only independent when the approach is
      exactly vertical, and skipping the projection gives you a non-orthogonal "rotation matrix"
      that every downstream library will silently accept.
    * ``y = z x x``.

    Raise ``ValueError`` when the closing direction is parallel to the approach axis (the
    projection has near-zero length): there is no valid frame, and that is geometry, not an edge
    case to paper over.
    """
    raise NotImplementedError("grasp_tool_pose")  # TODO(student)


def jaw_yaw_of(T_base_tool: ArrayLike) -> float:
    """The jaws' closing direction, projected on the table, wrapped to (-pi/2, pi/2].

    Take the tool frame's x axis, ``atan2(x[1], x[0])``, and ``wrap_axis`` it.
    """
    raise NotImplementedError("jaw_yaw_of")  # TODO(student)


def approach_waypoints(T_grasp: ArrayLike, *, standoff_m: float = 0.080, lift_m: float = 0.060,
                       table_z: float = 0.0, clearance_m: float = 0.004) -> list[Waypoint]:
    """The four poses of a reach, in order, all sharing the grasp's orientation.

    | name | position | speed_scale |
    |---|---|---|
    | `pre_grasp` | grasp position − `standoff_m` × approach axis | 1.0 |
    | `approach`  | grasp position − 0.020 m × approach axis      | 0.25 |
    | `grasp`     | the grasp position itself                     | 0.25 |
    | `lift`      | grasp position + `lift_m` along **world +z**  | 0.5 |

    The lift is vertical in the *world*, not along the tool axis: a grasp that slipped must drop
    the object onto the table, not sideways into its neighbour.

    Raise ``ValueError`` if the grasp position is below ``table_z + clearance_m``.
    """
    raise NotImplementedError("approach_waypoints")  # TODO(student)


def check_plan(waypoints: list[Waypoint], *, table_z: float = 0.0, clearance_m: float = 0.004,
               reach_min_m: float = 0.12, reach_max_m: float = 0.32) -> list[str]:
    """Everything you can check without an IK solver. Returns a list of problems; empty = ok.

    Check, and report one string per problem:

    * the four waypoints are named and ordered ``pre_grasp, approach, grasp, lift`` (if not,
      report that and return immediately — the rest of the checks would be nonsense);
    * every waypoint is at least ``clearance_m`` above ``table_z``;
    * every waypoint's distance from the base origin is within ``[reach_min_m, reach_max_m]``;
    * the z of pre_grasp > approach > grasp (a strictly descending approach);
    * the lift goes up, and goes *straight* up (same x and y as the grasp).

    These are cheap and they catch the plans that would otherwise fail halfway through a motion.
    They do **not** replace an IK check — see the lesson's standoff grid for why.
    """
    raise NotImplementedError("check_plan")  # TODO(student)


def lateral_tolerance_m(stroke_m: float, object_width_m: float) -> float:
    """How far sideways (along the closing direction) the gripper may be before a pad hits first.

    The jaws come down fully open, so the free gap is ``stroke - width``, split between the two
    sides: ``(stroke - width) / 2``, never negative. This one number is what the whole error
    budget of the lesson has to fit inside.
    """
    raise NotImplementedError("lateral_tolerance_m")  # TODO(student)
