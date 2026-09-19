"""Make ``robot_agent`` (and ``robotlab``) importable from the tests (the repo runs pytest in importlib mode).

    py -m pytest 19-llm-robot-agents/code
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parents[1] / "labs" / "python"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
