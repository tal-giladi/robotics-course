"""A 2D differential-drive simulator for the course labs.

>>> from robotlab.sim import World, DiffDriveSim, SimBase
>>> base = SimBase(DiffDriveSim(World.apartment(), pose=(1.0, 1.3, 0.0), seed=0))

Plotting helpers live in :mod:`robotlab.sim.viz` (imported separately; needs matplotlib).
"""

from robotlab.sim.base import SimBase
from robotlab.sim.occupancy import FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from robotlab.sim.robot import DiffDriveParams, DiffDriveSim, SensorParams, arc_update
from robotlab.sim.sensors import LandmarkObservation, LaserScan
from robotlab.sim.world import World, box_segments

__all__ = [
    "FREE",
    "OCCUPIED",
    "UNKNOWN",
    "DiffDriveParams",
    "DiffDriveSim",
    "LandmarkObservation",
    "LaserScan",
    "OccupancyGrid",
    "SensorParams",
    "SimBase",
    "World",
    "arc_update",
    "box_segments",
]
