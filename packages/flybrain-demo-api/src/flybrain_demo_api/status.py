"""Machine-readable status lines a demo prints for Flydeck.

Flydeck starts a demo with the environment variable ``FLYDECK_STATUS=1`` and reads its stdout.
A status line looks like::

    FLYDECK {"event": "ready", "url": "http://127.0.0.1:8766/"}
    FLYDECK {"event": "state", "state": "calibrating", "detail": "keep hands away"}
    FLYDECK {"event": "failed", "reason": "could not connect: no Bittle found"}

When the variable is not set (a person running the demo in a terminal) nothing is printed.
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any

PREFIX = "FLYDECK "
ENV = "FLYDECK_STATUS"


def enabled() -> bool:
    return os.environ.get(ENV) == "1"


def emit(event: str, **fields: Any) -> None:
    if not enabled():
        return
    line = PREFIX + json.dumps({"event": event, **fields}, default=str)
    print(line, file=sys.stdout, flush=True)


def ready(url: str | None = None) -> None:
    """The demo is up: its dashboard (if any) answers at url, the robot is connected."""
    emit("ready", url=url)


def state(name: str, detail: str = "") -> None:
    """A named phase, e.g. "connecting", "calibrating", "running"."""
    emit("state", state=name, detail=detail)


def failed(reason: str) -> None:
    """The demo cannot go on; reason is shown to the user as is. Exit non-zero after it."""
    emit("failed", reason=reason)


def parse(line: str) -> dict[str, Any] | None:
    """The event in a status line, or None for any other line."""
    if not line.startswith(PREFIX):
        return None
    try:
        ev = json.loads(line[len(PREFIX) :])
    except json.JSONDecodeError:
        return None
    return ev if isinstance(ev, dict) and "event" in ev else None
