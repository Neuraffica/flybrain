"""Optional offline benchmark shown on the dashboard.

Hook: ``find_benchmark(explicit)`` returns parsed JSON or None. The dashboard renders whatever
it gets (a dict of per-model dicts becomes a table; anything else is shown as JSON). Pass
``--benchmark PATH`` or edit ``PACKAGE_CANDIDATES`` to point at the file the model package
ships.
"""

from __future__ import annotations

import json
from importlib import resources
from pathlib import Path
from typing import Any

# (package, resource path) tried in order when no explicit path is given
PACKAGE_CANDIDATES: list[tuple[str, str]] = [
    # same file as flybrain_models.compass_benchmark.results_path()
    ("flybrain_models", "compass_benchmark.json"),
    ("flybrain_models", "benchmarks/compass.json"),
]


def find_benchmark(
    explicit: str | Path | None = None,
    candidates: list[tuple[str, str]] | None = None,
) -> dict[str, Any] | None:
    if explicit:
        p = Path(explicit)
        if p.exists():
            return _load(p.read_text(), str(p))
        return {"error": f"benchmark file not found: {p}"}
    for pkg, rel in PACKAGE_CANDIDATES if candidates is None else candidates:
        try:
            res = resources.files(pkg).joinpath(rel)
            if res.is_file():
                return _load(res.read_text(), f"{pkg}/{rel}")
        except (ModuleNotFoundError, FileNotFoundError, TypeError):
            continue
    return None


def _load(text: str, source: str) -> dict[str, Any] | None:
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        return {"error": f"{source}: {e}"}
    return {"source": source, "data": data}
