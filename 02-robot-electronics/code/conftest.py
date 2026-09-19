"""Make the module-02 helpers importable from the tests (the repo runs pytest in importlib mode).

    py -m pytest 02-robot-electronics/code

The MicroPython scripts in pico/ import `machine`; the tests use the course's CPython stand-ins
in labs/python/tests/mpshims so their pure logic runs on a PC.
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
for path in (HERE, HERE / "pico", ROOT / "labs" / "python" / "tests" / "mpshims"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
