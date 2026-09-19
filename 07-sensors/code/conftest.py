"""Make the module-07 helpers importable from the tests (the repo runs pytest in importlib mode).

    py -m pytest 07-sensors/code
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
for path in (HERE, ROOT / "labs" / "python"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
