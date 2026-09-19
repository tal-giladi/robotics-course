"""Make the module-05 helpers importable from the tests (the repo runs pytest in importlib mode).

    py -m pytest 05-frames-and-transforms/code
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for path in (HERE, HERE / "ros2" / "src" / "course_tf_examples"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
