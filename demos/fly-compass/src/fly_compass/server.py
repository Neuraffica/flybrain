"""fly-compass dashboard server: the shared demo server (flybrain_demo_api.web) serving the
compass page by default.

    GET  /                 the dashboard (self-contained, works offline)
    GET  /api/stream       text/event-stream, one JSON frame per tick (~20 Hz)
    GET  /api/state        latest frame as JSON
    GET  /api/benchmark    offline benchmark JSON, or 204 when there is none
    POST /api/command      {"action": "rezero" | "reset_scores" | "mode" | "driver" |
                            "mock_turn_to" | "mock_nudge", ...}
"""

from __future__ import annotations

from importlib import resources

from flybrain_demo_api import web
from flybrain_demo_api.web import FrameHub

__all__ = ["DashboardServer", "FrameHub", "make_handler"]


def _page() -> bytes:
    return resources.files("fly_compass").joinpath("static/index.html").read_bytes()


def make_handler(hub, on_command, benchmark, page: bytes | None = None):
    return web.make_handler(hub, on_command, benchmark, _page() if page is None else page)


class DashboardServer(web.DashboardServer):
    """``page``: the HTML to serve at / (default: the compass dashboard)."""

    def __init__(self, hub, on_command, benchmark, host="127.0.0.1", port=8765, page=None):
        super().__init__(hub, on_command, benchmark, host, port, _page() if page is None else page)
