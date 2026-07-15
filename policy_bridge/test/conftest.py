"""Make the source package importable without requiring installation."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_SOURCE_ROOT = Path(__file__).resolve().parents[1]

if str(PACKAGE_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE_ROOT))
