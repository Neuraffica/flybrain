"""What Flydeck remembers between runs: the last robot, each demo's last settings, and (in the
app) which flybrain folder's demos to show.

Stored as JSON in ~/Library/Application Support/flybrain/flydeck.json (FLYDECK_HOME overrides
the folder, e.g. for tests; see flybrain_demo_api.user_dir). Nothing here leaves the Mac.
"""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any

from flybrain_demo_api import user_dir as home


class Settings:
    def __init__(self, path: Path | None = None):
        self.path = path or home() / "flydeck.json"
        self._lock = threading.Lock()
        self.data: dict[str, Any] = self._read()

    def _read(self) -> dict[str, Any]:
        try:
            loaded = json.loads(self.path.read_text())
        except (OSError, ValueError):
            return {}
        return loaded if isinstance(loaded, dict) else {}

    def get(self, key: str, default: Any = None) -> Any:
        return self.data.get(key, default)

    def set(self, key: str, value: Any) -> None:
        with self._lock:
            # the Flydeck window and its hub are two processes sharing this file: keep the
            # other's keys by re-reading before writing
            self.data = {**self.data, **self._read(), key: value}
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
