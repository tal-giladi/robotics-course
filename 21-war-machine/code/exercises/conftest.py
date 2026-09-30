"""Make the exercise student module importable (repo runs pytest in importlib mode).

    py -m pytest 21-war-machine/code/exercises
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
