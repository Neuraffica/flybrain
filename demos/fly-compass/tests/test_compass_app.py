import json
import sys
import threading
import time
import urllib.request

import pytest
from compass_fakes import fake_models
from fly_compass import cli
from fly_compass.app import CompassApp
from fly_compass.loop import CompassLoop, LoopConfig
from fly_compass.server import DashboardServer, FrameHub
from fly_compass.session import ReplaySource, SessionLog, load_samples
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport


def _mock_app(tmp_path, mode="needle", traj=None, log=True, link="usb"):
    # link="ble": the simulator speaks like firmware B10_250330 over Bluetooth LE (one print
    # per notification, no newlines, IMU line and echo glued together)
    mock = MockBittle(imu_noise_deg=0.1, ble_notifications=link == "ble")
    if traj:
        mock.trajectory = traj
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=1.0, quiet_s=0.3)
    loop = CompassLoop(fake_models(), LoopConfig(rate_hz=20), mode=mode)
    session = SessionLog(tmp_path / "runs", meta={"test": True}) if log else None
    app = CompassApp(loop, FrameHub(), bittle=bot, mock=mock, session=session, source_label="mock")
    return mock, bot, app


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_control_loop_tracks_scripted_rotation_on_mock(tmp_path, link):
    # dog turns 90 deg left over 1.5 s, then holds
    mock, _bot, app = _mock_app(
        tmp_path, link=link, traj=lambda t: 37 + 60 * min(max(t - 1.0, 0), 1.5)
    )
    try:
        app.start_robot()
        summary = app.run(duration=4.5)
        fr = app.last_frame
    finally:
        app.shutdown()
    assert abs(fr["truth"] - 90) < 3, fr["truth"]
    assert abs(fr["models"]["connectome"]["err"]) < 5
    assert summary["connectome"]["mae_deg"] < 15
    # the head ends up pointing back at the start heading, inside the pan limit
    assert mock.max_head_cmd <= 80
    pan_cmds = [c for c in mock.commands if c.startswith("i0 ")]
    assert pan_cmds, "needle sent no head commands"
    assert int(pan_cmds[-1].split()[1]) == -80  # 90 deg turn saturates at the 80 deg limit
    # always rests on exit, and nothing was lost to merged writes
    assert mock.commands[-1] == "d"
    assert mock.dropped_bytes == 0
    # session log was written and replays
    rows = load_samples(app.session.dir)
    assert len(rows) >= 15


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_replay_reproduces_session(tmp_path, link):
    _mock, _bot, app = _mock_app(tmp_path, link=link, mode="observe", traj=lambda t: 37 + 40 * t)
    try:
        app.start_robot()
        app.run(duration=2.5)
    finally:
        app.shutdown()
    samples = load_samples(app.session.dir / "samples.csv")
    rep = CompassApp(
        CompassLoop(fake_models(), mode="observe"), replay=ReplaySource(samples), replay_speed=0
    )
    summary = rep.run()
    assert rep.finished
    assert summary["connectome"]["samples"] >= 10  # scored once per IMU sample (5 Hz)
    assert summary["connectome"]["mae_deg"] < 10


def _get(url, timeout=3):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.status, r.read()


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_server_serves_page_state_stream_and_commands(tmp_path, link):
    mock, _bot, app = _mock_app(tmp_path, log=False, link=link)
    srv = DashboardServer(
        app.hub, app.on_command, lambda: {"source": "x", "data": {"a": {"mae": 1}}}, port=0
    ).start()
    th = threading.Thread(target=app.run, kwargs={"duration": 3.0}, daemon=True)
    try:
        app.start_robot()
        th.start()
        time.sleep(0.6)
        code, page = _get(srv.url)
        assert code == 200 and b"<title>Fly compass</title>" in page
        assert b"https://" not in page and b"http://" not in page  # offline: no external assets
        code, body = _get(srv.url + "api/state")
        st = json.loads(body)
        assert st["status"]["mock"] is True and "connectome" in st["models"]
        # SSE: read the first data frame
        with urllib.request.urlopen(srv.url + "api/stream", timeout=3) as r:
            assert r.headers["Content-Type"].startswith("text/event-stream")
            line = b""
            while not line.startswith(b"data: "):
                line = r.readline()
            frame = json.loads(line[6:])
        assert set(frame["models"]) == {"connectome", "rewired", "gyro"}
        assert len(frame["models"]["connectome"]["bump"]) == 16

        def post(obj):
            req = urllib.request.Request(
                srv.url + "api/command",
                data=json.dumps(obj).encode(),
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(req, timeout=3) as r:
                return json.loads(r.read())

        assert post({"action": "driver", "model": "rewired"})["ok"]
        assert post({"action": "mode", "mode": "hold"})["ok"] is False  # no --i-have-space
        assert post({"action": "mock_turn_to", "deg": 45})["ok"]
        assert post({"action": "rezero"})["ok"]
        assert post({"action": "bogus"})["ok"] is False
        time.sleep(0.5)
        assert app.loop.driver == "rewired"
        code, body = _get(srv.url + "api/benchmark")
        assert json.loads(body)["data"]["a"]["mae"] == 1
    finally:
        app.stop_event.set()
        th.join(timeout=5)
        app.shutdown()
        srv.stop()
    assert mock.commands[-1] == "d"


def test_cli_fails_clearly_without_models(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "flybrain_models.compass", None)
    assert cli.main(["--mock", "--no-dashboard", "--no-log", "--duration", "0.1"]) == 3
    err = capsys.readouterr().err
    assert "compass models are not available" in err


def test_cli_hold_needs_flag(capsys):
    assert cli.main(["--mock", "--mode", "hold"]) == 2
    assert "--i-have-space" in capsys.readouterr().err


def test_scenario_on_mock_logs_and_replays_blindfold(tmp_path, monkeypatch):
    from compass_fakes import fly_like_models
    from fly_compass import scenarios
    from fly_compass.scenarios import Phase

    monkeypatch.setitem(
        scenarios.SCENARIOS,
        "quick",
        [Phase("cue", 1.0), Phase("dark", 2.0, turn_deg=45.0, blindfold=True), Phase("back", 1.0)],
    )
    mock = MockBittle(imu_noise_deg=0.1)
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=1.0, quiet_s=0.3)
    loop = CompassLoop(fly_like_models(), LoopConfig(rate_hz=20), mode="observe")
    session = SessionLog(tmp_path / "runs")
    app = CompassApp(
        loop, bittle=bot, mock=mock, session=session, scenario="quick", exit_after_scenario=True
    )
    done = []
    app.on_scenario_done = done.append
    try:
        app.start_robot()
        app.run(duration=10)
    finally:
        app.shutdown()
    assert done, "scenario did not finish"
    live = done[0]
    assert set(live) == {"quick: cue", "quick: dark", "quick: back"}
    # in the dark the 10 %-gain ring misses most of the 45 deg turn, the filter follows it
    assert abs(live["quick: dark"]["connectome"]["final_err_deg"]) > 25
    assert abs(live["quick: dark"]["gyro"]["final_err_deg"]) < 10
    samples = load_samples(session.dir)
    assert any(s.blindfold for s in samples) and not samples[0].blindfold
    assert {s.phase for s in samples} >= {"quick: cue", "quick: dark"}
    # replay reproduces the cue schedule and therefore the per-phase errors
    rep = CompassApp(
        CompassLoop(fly_like_models(), mode="observe"), replay=ReplaySource(samples), replay_speed=0
    )
    rep.run()
    again = rep.phase_summary()
    for ph in ("quick: dark", "quick: back"):
        for k in ("connectome", "gyro"):
            assert abs(again[ph][k]["final_err_deg"] - live[ph][k]["final_err_deg"]) < 8


@pytest.mark.parametrize("link", ["usb", "ble"])
def test_blindfold_command_and_flag(tmp_path, link):
    _mock, _bot, app = _mock_app(tmp_path, mode="observe", log=False, link=link)
    app.blindfold_after = 0.8
    try:
        app.start_robot()
        th = threading.Thread(target=app.run, kwargs={"duration": 2.5}, daemon=True)
        th.start()
        time.sleep(1.3)
        assert app.loop.blindfolded  # --blindfold-after
        assert app.on_command({"action": "blindfold", "on": False}) == {"ok": True, "on": False}
        assert app.on_command({"action": "blindfold"})["on"] is True  # toggle
        assert app.on_command({"action": "scenario", "name": "nope"})["ok"] is False
        th.join(timeout=5)
        fr = app.last_frame
        assert fr["blindfold"]["on"] and fr["blindfold"]["elapsed"] >= 0
        assert fr["models"]["connectome"]["mae_blind"] is not None
    finally:
        app.stop_event.set()
        app.shutdown()


def test_cli_script_needs_mock(capsys):
    assert cli.main(["--port", "/dev/null", "--mock-script", "memory"]) == 2
    assert "needs --mock" in capsys.readouterr().err
