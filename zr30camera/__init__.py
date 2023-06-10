import sys, os
from pathlib import Path

_SRC_DIR = Path(__file__).parent
_SDK_DIR = _SRC_DIR.parent.parent / "siyi_sdk"

# sys.path.insert(0, _SDK_DIR)