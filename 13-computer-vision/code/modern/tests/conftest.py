"""Make the modules in 13-computer-vision/code/modern importable from the tests."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
