"""Where Flydeck and the demos keep small per-user files (settings, per-robot calibrations).

macOS: ~/Library/Application Support/flybrain. Windows: %APPDATA%\\flybrain. Elsewhere:
$XDG_DATA_HOME/flybrain (default ~/.local/share/flybrain). FLYDECK_HOME overrides the folder,
e.g. for tests.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def user_dir() -> Path:
    env = os.environ.get("FLYDECK_HOME")
    if env:
        return Path(env).expanduser()
    if sys.platform == "darwin":
        return Path.home() / "Library" / "Application Support" / "flybrain"
    if sys.platform == "win32" and os.environ.get("APPDATA"):
        return Path(os.environ["APPDATA"]) / "flybrain"
    base = os.environ.get("XDG_DATA_HOME") or Path.home() / ".local" / "share"
    return Path(base) / "flybrain"
