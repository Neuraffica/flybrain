"""The fly escape module that runs on the robot's own board (firmware/bittle/fly_escape).

Stock OpenCatEsp32 does not have it. With it, the robot reads its IR sensor, runs the escape
circuit and backs away by itself; the laptop can only switch it on or off and listen:

    on = OnboardEscape(bot)          # bot: a connected Bittle
    on.query()                       # "X?": is the module there, is it on?
    on.set_enabled(True)             # "XE" (the robot stands up and measures resting levels)
    on.set_telemetry(True)           # "XEP": 5 lines/s with the sensor and giant fibres
    for t, msg in on.drain(): ...    # protocol.OnboardEscapeMsg, as they arrive

Only "X?" is sent to find out; it exists in stock firmware and changes nothing there.
"""

from __future__ import annotations

import threading
from collections import deque

from . import protocol as P
from .robot import Bittle

X_TIMEOUT_S = 2.0
ENABLE_TIMEOUT_S = 5.0  # "XE" stands the robot up and calibrates before the echo


class OnboardEscape:
    def __init__(self, bot: Bittle):
        self.bot = bot
        self.present: bool | None = None  # None: could not tell (Bluetooth, no status line)
        self.reports = False  # the firmware prints "@FE" status lines (current module build)
        self.enabled: bool | None = None
        self.telemetry = False
        self.baseline: tuple[float, float] | None = None
        self._reader = P.OnboardEscapeReader()
        self._cv = threading.Condition()
        self._msgs: deque[tuple[float, P.OnboardEscapeMsg]] = deque(maxlen=2000)
        self._status_seen = 0
        bot.add_rx_listener(self._on_rx)

    def _on_rx(self, data: bytes, t: float) -> None:
        msgs = self._reader.feed(data.decode("latin-1"))
        if not msgs:
            return
        with self._cv:
            for m in msgs:
                if m.kind == "status":
                    self.present = self.reports = True
                    self.enabled, self.telemetry = m.enabled, bool(m.telemetry)
                    self._status_seen += 1
                elif m.kind == "base":
                    self.baseline = (m.left, m.right)
                self._msgs.append((t, m))
            self._cv.notify_all()

    def drain(self) -> list[tuple[float, P.OnboardEscapeMsg]]:
        """Messages received since the last call, oldest first, with host receive times."""
        with self._cv:
            out = list(self._msgs)
            self._msgs.clear()
        return out

    def query(self) -> bool | None:
        """Ask "X?" and update present / enabled / telemetry. Returns ``present``.

        The current module answers with an "@FE on/off" line on every port. Without one, the
        module list (USB only) still says whether an older build of the module is there."""
        with self._cv:
            seen = self._status_seen
        lines = self.bot.send(P.encode_extension("?"), echo=P.T_EXTENSION, timeout=X_TIMEOUT_S)
        with self._cv:
            if self._status_seen > seen:
                return self.present
        modules = P.parse_module_list(lines or [])
        if modules is not None:
            self.present = P.EXTENSION_FLY_ESCAPE in modules
            self.enabled = modules.get(P.EXTENSION_FLY_ESCAPE)
        return self.present

    def set_enabled(self, on: bool) -> bool:
        """ "XE" / "Xe". Enabling stands the robot up and measures the resting levels (keep
        hands away). Note: like every "X<letter>", "XE" also switches other sensor modules
        off. Returns True when the firmware echoed the command."""
        code = P.EXTENSION_FLY_ESCAPE if on else P.EXTENSION_FLY_ESCAPE.lower()
        timeout = ENABLE_TIMEOUT_S if on else X_TIMEOUT_S
        lines = self.bot.send(P.encode_extension(code), echo=P.T_EXTENSION, timeout=timeout)
        return lines is not None

    def set_telemetry(self, on: bool) -> bool:
        """ "XEP" / "XEp". Only sent while the module is on: "XE..." would switch it on, and
        the firmware turns telemetry off by itself whenever the module goes off."""
        if not self.enabled or on == self.telemetry:
            return False
        code = P.EXTENSION_FLY_ESCAPE + ("P" if on else "p")
        lines = self.bot.send(P.encode_extension(code), echo=P.T_EXTENSION, timeout=X_TIMEOUT_S)
        return lines is not None
