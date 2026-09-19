"""Checker for <lesson-id> — <lesson title>.

Run: ``python course.py check <lesson-id>`` (``--solution`` runs the reference).
The ``impl`` fixture (labs/exercises/conftest.py) is student.py or solution.py.
"""

from __future__ import annotations

import pytest


@pytest.mark.parametrize(("value", "expected"), [(0.0, 0.0), (1.5, 3.0), (-2.0, -4.0)])
def test_example_function(impl, value, expected):
    assert impl.example_function(value) == pytest.approx(expected), "example_function should double its input"


# End-to-end pattern (see labs/exercises/09.04/test_exercise.py):
#
# from robotlab.config import load_config
# from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World
#
# def test_against_simulator(impl):
#     cfg = load_config()
#     base = SimBase(DiffDriveSim(World.apartment(), DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg),
#                                 pose=(1.0, 1.3, 0.0), seed=0))
#     for _ in range(250):                      # 5 s at 50 Hz; command every step (watchdog)
#         base.set_wheel_velocity(5.0, 5.0)
#         state = base.read()
#         ...                                   # feed `state` to the student's code
#     assert ... base.sim.pose ...              # compare with ground truth
