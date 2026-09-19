"""Make the module-18 lab importable from the tests (the repo runs pytest in importlib mode).

    py -m pytest 18-embodied-ai/code
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
