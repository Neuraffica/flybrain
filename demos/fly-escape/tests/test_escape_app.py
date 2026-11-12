import json
import sys
import threading
import time
import urllib.request
from importlib import resources
from pathlib import Path

import escape_fakes
import pytest
from escape_fakes import fake_models
from fly_compass.server import DashboardServer, FrameHub
from fly_escape import cli
from fly_escape.app import EscapeApp
from fly_escape.escape import EscapeConfig
from fly_escape.loop import EscapeLoop
from fly_escape.scripts import SCRIPTS
from fly_escape.session import ReplaySource, SessionLog, load_samples
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport

FIXTURE = (
    Path(__file__).resolve().parents[3]
    / "packages/flybrain-models/tests/fixtures/ir_hand_approaches_usb.json"
)


def mock_app(tmp_path, link="usb", mode="escape", allow=True, unplugged=False, log=True):
    mock = MockBittle(ble_notifications=link == "ble")
    if unplugged:
        mock.ir_pins = {}
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    models, r2p = fake_models()
    loop = EscapeLoop(models, r2p)
    session = SessionLog(tmp_path / "runs") if log else None
    app = EscapeApp(
        loop,
        FrameHub(),
        bittle=bot,
        mock=mock,
        session=session,
        mode=mode,
        allow_moves=allow,
        escape_cfg=EscapeConfig(action_s=0.8, min_interval_s=2.0),
    )
    return mock, bot, app


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_trigger_backs_away_then_stands(tmp_path, link):
    mock, _bot, app = mock_app(tmp_path, link=link)
    try:
        app.start_robot()
        mock.hand.run_script(SCRIPTS["fast"], mock.clock() + 3.0)  # after calibration
        app.run(duration=7.0)
    finally:
        app.shutdown()
    assert app.loop.cal.ok
    assert len(app.escape.history) >= 1
    assert app.escape.history[0].stood is True
    ks = [c for c in mock.commands if c.startswith("k")]
    i = ks.index("kbkF")
    assert ks[i + 1] == "kup"
    assert mock.gaits_stopped_by_read == 0  # no analog read during the escape gait
    assert mock.dropped_bytes == 0 and mock.commands[-1] == "d"
    rows = load_samples(app.session.dir)
    assert len(rows) > 40
    trig = (app.session.dir / "triggers.csv").read_text()
    assert "connectome" in trig
    assert "bkF" in (app.session.dir / "escapes.csv").read_text()


def test_observe_mode_never_moves(tmp_path):
    mock, _bot, app = mock_app(tmp_path, mode="observe", allow=False, log=False)
    try:
        app.start_robot()
        mock.hand.run_script(SCRIPTS["fast"], mock.clock() + 3.0)
        app.run(duration=6.0)
        assert app.on_command({"action": "mode", "mode": "escape"})["ok"] is False
    finally:
        app.shutdown()
    assert app.loop.scores["connectome"].triggers >= 1
    assert not any(c.startswith("kbk") for c in mock.commands)


def test_unplugged_sensor_refuses_escape(tmp_path):
    _mock, _bot, app = mock_app(tmp_path, unplugged=True, log=False)
    events = []
    app.on_event = events.append
    try:
        app.start_robot()
        app.run(duration=3.0)
    finally:
        app.shutdown()
    assert app.loop.cal.done and not app.loop.cal.ok
    assert app.mode == "observe"
    assert any("ESCAPE MODE REFUSED" in e and "unplugged" in e for e in events)


def test_min_interval_holds_on_repeated_approaches(tmp_path):
    mock, _bot, app = mock_app(tmp_path, log=False)
    app.escape.cfg.min_interval_s = 30.0
    try:
        app.start_robot()

        def twice(t):
            return (80.0, 0.0) if t < 0 else SCRIPTS["fast"](t % 2.5)

        mock.hand.back_speed_cm_s = 0.0  # the hand follows the dog: it keeps looming
        mock.hand.run_script(twice, mock.clock() + 3.0)
        app.run(duration=10.0)
    finally:
        app.shutdown()
    assert len(app.escape.history) == 1
    assert app.escape.blocked >= 1


@pytest.mark.skipif(not FIXTURE.exists(), reason="model fixture not present")
def test_replay_of_the_real_recording():
    rows = load_samples(FIXTURE)
    assert len(rows) > 100
    models, r2p = fake_models()
    app = EscapeApp(EscapeLoop(models, r2p), replay=ReplaySource(rows), replay_speed=0)
    summary = app.run()
    assert app.finished and summary["samples"] == len(rows)
    assert summary["calibration"]["done"]
    assert app.mode == "observe" and not app.escape.history


def test_server_serves_the_escape_page(tmp_path):
    _mock, _bot, app = mock_app(tmp_path, log=False)
    page = resources.files("fly_escape").joinpath("static/index.html").read_bytes()
    srv = DashboardServer(app.hub, app.on_command, lambda: None, port=0, page=page).start()
    th = threading.Thread(target=app.run, kwargs={"duration": 4.0}, daemon=True)
    try:
        app.start_robot()
        th.start()
        time.sleep(2.6)
        with urllib.request.urlopen(srv.url, timeout=3) as r:
            body = r.read()
        assert b"<title>Fly escape</title>" in body and b"https://" not in body
        with urllib.request.urlopen(srv.url + "api/state", timeout=3) as r:
            st = json.loads(r.read())
        assert set(st["models"]) == {"connectome", "rewired", "random", "prox", "looming"}
        assert st["calibration"]["ok"] and st["status"]["mock"] is True
        assert "LPLC2_L" in st["models"]["connectome"]["activity"]

        def post(obj):
            req = urllib.request.Request(
                srv.url + "api/command",
                data=json.dumps(obj).encode(),
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(req, timeout=3) as r:
                return json.loads(r.read())

        assert post({"action": "hands_off"})["on"] is True
        assert post({"action": "driver", "model": "looming"})["ok"]
        assert post({"action": "mock_hand", "distance": 10, "side": 0.5})["ok"]
        assert post({"action": "mock_script", "name": "left"})["ok"]
        assert post({"action": "mock_script", "name": "nope"})["ok"] is False
        assert post({"action": "mode", "mode": "observe"})["ok"]
        th.join(timeout=5)
        assert app.loop.driver == "looming"
    finally:
        app.stop_event.set()
        app.shutdown()
        srv.stop()


def test_cli_fails_clearly_without_models(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "flybrain_models.escape", None)
    assert cli.main(["--mock", "--no-dashboard", "--no-log", "--duration", "0.1"]) == 3
    assert "escape models are not available" in capsys.readouterr().err


def test_cli_escape_needs_space(capsys):
    # a real robot needs --i-have-space for escape mode (the simulator does not)
    assert cli.main(["--port", "/dev/null", "--mode", "escape"]) == 2
    assert "--i-have-space" in capsys.readouterr().err


def test_cli_mock_run_with_fakes(monkeypatch, tmp_path, capsys):
    monkeypatch.setitem(sys.modules, "flybrain_models.escape", escape_fakes.install())
    code = cli.main(
        [
            "--mock",
            "--i-have-space",
            "--mock-script",
            "fast",
            "--duration",
            "6",
            "--no-dashboard",
            "--runs-dir",
            str(tmp_path),
        ]
    )
    out = capsys.readouterr().out
    assert code == 0, out
    assert "calibrated" in out and "ESCAPE: bkF" in out
