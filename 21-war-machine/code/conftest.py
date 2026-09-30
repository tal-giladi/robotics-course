"""Make the module-21 code importable from the tests (the repo runs pytest in importlib mode).

    py -m pytest 21-war-machine/code
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
