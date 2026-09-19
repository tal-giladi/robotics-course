"""Make the module-01 helpers importable from the tests (the repo runs pytest in importlib mode).

    py -m pytest 01-first-robot/code
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
for path in (HERE, ROOT / "labs" / "python", ROOT / "labs" / "robot"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
