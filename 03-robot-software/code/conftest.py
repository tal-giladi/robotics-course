"""Make the module-03 scripts importable from their tests (the repo runs pytest in importlib mode).

    py -m pytest 03-robot-software/code

Each lesson's scripts are prefixed with the lesson number (``l0304_jitter.py``) and tested by
``test_l03NN_*.py`` in this folder. Nothing here needs hardware, ROS or a network.
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for path in (HERE, HERE.parent.parent / "labs" / "python", HERE.parent.parent / "labs" / "robot"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
