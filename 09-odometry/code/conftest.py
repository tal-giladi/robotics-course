"""Make the module-09 helpers importable from the tests (the repo runs pytest in importlib mode).

    py -m pytest 09-odometry/code
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for path in (HERE, HERE / "ros2" / "src" / "course_odometry", HERE.parent.parent / "labs" / "python"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
