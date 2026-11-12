"""What Flydeck remembers between runs: the last robot and each demo's last settings.

Stored as JSON in ~/Library/Application Support/flybrain/flydeck.json (FLYDECK_HOME overrides
the folder, e.g. for tests). Nothing here leaves the Mac.
"""

from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any


def home() -> Path:
    env = os.environ.get("FLYDECK_HOME")
    if env:
        return Path(env).expanduser()
    return Path.home() / "Library" / "Application Support" / "flybrain"


class Settings:
    def __init__(self, path: Path | None = None):
        self.path = path or home() / "flydeck.json"
        self._lock = threading.Lock()
        self.data: dict[str, Any] = {}
        try:
            loaded = json.loads(self.path.read_text())
            if isinstance(loaded, dict):
                self.data = loaded
        except (OSError, ValueError):
            pass

    def get(self, key: str, default: Any = None) -> Any:
        return self.data.get(key, default)

    def set(self, key: str, value: Any) -> None:
        with self._lock:
            self.data[key] = value
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                tmp = self.path.with_suffix(".tmp")
                tmp.write_text(json.dumps(self.data, indent=2))
                tmp.replace(self.path)
            except OSError:
                pass  # remembering is a convenience; never fail a start because of it

    def demo_options(self, demo_id: str) -> dict[str, Any]:
        return dict(self.get("options", {}).get(demo_id, {}))

    def set_demo_options(self, demo_id: str, options: dict[str, Any]) -> None:
        allopts = dict(self.get("options", {}))
        allopts[demo_id] = options
        self.set("options", allopts)
