"""A Bluetooth link that comes up silent is dropped and opened again, a few times."""

import queue

from flybrain_edge.bittle import Bittle
from flybrain_edge.bittle import robot as R
from flybrain_edge.bittle.transport import Transport, TransportError


class SilentFirstLink(Transport):
    """Answers nothing on the first ``silent_opens`` connections, then like the firmware."""

    name = "ble"
    description = "fake BLE"

    def __init__(self, silent_opens: int, missing_opens: tuple[int, ...] = ()):
        self.silent_opens = silent_opens
        self.missing_opens = missing_opens  # connections on which the robot is not found
        self.busy_queries = 0  # first '?'s answered with the bare echo, as while a skill runs
        self.opens = 0
        self._open = False
        self._rx: queue.Queue[bytes] = queue.Queue()

    def open(self) -> None:
        self.opens += 1
        if self.opens in self.missing_opens:
            raise TransportError("No Petoi BLE device found")
        self._open = True
        self._rx = queue.Queue()
        if self.opens > self.silent_opens:
            self._rx.put(b"Petoi Bittle X\np\n")

    @property
    def is_open(self) -> bool:
        return self._open

    def write(self, data: bytes) -> None:
        if self.opens > self.silent_opens and data.startswith(b"?"):
            if self.busy_queries > 0:
                self.busy_queries -= 1
                self._rx.put(b"?\r\n")
            else:
                self._rx.put(b"Bittle X\r\nB10_251121\r\n?\r\n")

    def read(self, timeout: float = 0.05) -> bytes:
        try:
            return self._rx.get(timeout=timeout)
        except queue.Empty:
            return b""

    def close(self) -> None:
        self._open = False


def connect(silent_opens, monkeypatch, missing_opens=()):
    monkeypatch.setattr(R, "BLE_RECONNECT_PAUSE_S", 0.0)
    link = SilentFirstLink(silent_opens, missing_opens)
    bot = Bittle(link)
    orig = bot.send
    bot.send = lambda data, echo=None, timeout=2.0, **kw: orig(data, echo=echo, timeout=0.3, **kw)
    bot.connect(boot_timeout=0.3, quiet_s=0.1)
    return link, bot


def test_answering_link_is_not_reopened(monkeypatch):
    link, bot = connect(0, monkeypatch)
    try:
        assert link.opens == 1
        assert (bot.model, bot.version) == ("Bittle X", "B10_251121")
    finally:
        bot.close(rest=False)


def test_silent_link_is_reopened(monkeypatch, caplog):
    link, bot = connect(1, monkeypatch)
    try:
        assert link.opens == 2
        assert bot.version == "B10_251121"
        assert "reconnecting" in caplog.text
    finally:
        bot.close(rest=False)


def test_robot_missing_from_the_scan_right_after_is_tried_again(monkeypatch, caplog):
    # what the real robot did: silent link, then not found, then fine
    link, bot = connect(1, monkeypatch, missing_opens=(2,))
    try:
        assert link.opens == 3
        assert bot.version == "B10_251121"
        assert "reconnect 1 of 3" in caplog.text
    finally:
        bot.close(rest=False)


def test_robot_that_never_comes_back_is_an_error(monkeypatch):
    link = SilentFirstLink(1, missing_opens=(2, 3, 4))
    monkeypatch.setattr(R, "BLE_RECONNECT_PAUSE_S", 0.0)
    bot = Bittle(link)
    orig = bot.send
    bot.send = lambda data, echo=None, timeout=2.0, **kw: orig(data, echo=echo, timeout=0.3, **kw)
    try:
        bot.connect(boot_timeout=0.3, quiet_s=0.1)
    except TransportError as e:
        assert "No Petoi BLE device found" in str(e)
    else:
        raise AssertionError("expected TransportError")
    assert link.opens == 4


def test_link_that_stays_silent_gives_up(monkeypatch, caplog):
    link, bot = connect(9, monkeypatch)
    try:
        assert link.opens == 4
        assert bot.model is None
        assert "reconnecting" in caplog.text and "firmware may be busy" in caplog.text
    finally:
        bot.close(rest=False)


def test_trace_file(monkeypatch, tmp_path):
    path = tmp_path / "trace.txt"
    monkeypatch.setenv(R.TRACE_ENV, str(path))
    _, bot = connect(0, monkeypatch)
    bot.close(rest=False)
    text = path.read_text()
    assert " tx b'?\\n'" in text and "B10_251121" in text


def test_busy_robot_that_only_echoes_is_asked_again_not_reconnected(monkeypatch):
    # measured: the robot plays "hi" after the greeting and answers '?' late, echo only
    monkeypatch.setattr(R, "BLE_RECONNECT_PAUSE_S", 0.0)
    link = SilentFirstLink(0)
    link.busy_queries = 2
    bot = Bittle(link)
    bot.connect(boot_timeout=0.3, quiet_s=0.1)
    try:
        assert link.opens == 1
        assert (bot.model, bot.version) == ("Bittle X", "B10_251121")
    finally:
        bot.close(rest=False)
