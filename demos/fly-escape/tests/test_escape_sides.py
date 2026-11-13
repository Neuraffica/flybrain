"""The side check (which pin is the robot's left), its per-robot store, and the side setting."""

import json
import sys
import types

import escape_fakes
import pytest
from escape_fakes import fake_models
from fly_compass.server import FrameHub
from fly_escape import cli, sides
from fly_escape.app import EscapeApp
from fly_escape.escape import EscapeConfig
from fly_escape.loop import EscapeLoop
from fly_escape.session import ReplaySource
from fly_escape.sides import SideCheck, load_store, resolve, robot_identity, save_result
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport

REST = (3400.0, 3440.0)


def run(check, rows):
    """Feed (t, a, b) rows; returns the time the check finished, or None."""
    for t, a, b in rows:
        if check.add(t, a, b):
            return t
    return None


def hold(t0, seconds, a, b, rate=10.0):
    return [(t0 + i / rate, a, b) for i in range(int(seconds * rate))]


def started():
    c = SideCheck()
    c.start(0.0, REST)
    return c


# ---- the decision -------------------------------------------------------------------------
def test_classify_needs_a_deep_drop_and_a_clear_margin():
    assert SideCheck.classify(1.0, 0.0) == 0
    assert SideCheck.classify(0.1, 0.9) == 1
    assert SideCheck.classify(0.55, 0.2) == 0  # just deep enough, 0.35 apart
    assert SideCheck.classify(0.45, 0.0) is None  # not deep enough
    assert SideCheck.classify(1.0, 0.75) is None  # both down: no margin
    assert SideCheck.classify(0.0, 0.0) is None


@pytest.mark.parametrize(("a", "b", "left"), [(0, 3440, 0), (3400, 100, 1)])
def test_hand_on_one_side_decides_then_waits_for_the_hand_to_go(a, b, left):
    c = started()
    rows = hold(0.0, 1.0, *REST) + hold(1.0, 1.0, a, b) + hold(2.0, 2.0, *REST)
    t_end = run(c, rows)
    assert c.left == left and not c.active
    assert t_end is not None and t_end >= 2.5  # the hand was gone for RELEASE_S first


def test_a_brief_blip_is_not_enough():
    c = started()
    # two readings on pin A only, then nothing
    run(c, hold(0.0, 0.2, 0, 3440) + hold(0.2, sides.TIMEOUT_S + 6, *REST))
    assert c.left is None and "not clear enough" in c.message


def test_both_channels_together_time_out_as_ambiguous():
    c = started()
    t_end = run(c, hold(0.0, sides.TIMEOUT_S + 1, 50, 80) + hold(16.0, 2.0, *REST))
    assert c.left is None and t_end is not None
    assert "FAILED" in c.message and "both channels" in c.message


def test_nothing_seen_times_out():
    c = started()
    run(c, hold(0.0, sides.TIMEOUT_S + 2, *REST))
    assert c.left is None and "no hand seen" in c.message and not c.active


def test_hand_that_stays_still_ends_after_release_max():
    c = started()
    t_end = run(c, hold(0.0, 10.0, 0, 3440))
    assert c.left == 0
    assert t_end is not None and t_end < 0.5 + sides.RELEASE_MAX_S + 0.3


# ---- the store and the setting --------------------------------------------------------------
def test_store_round_trip_and_resolution(tmp_path):
    path = tmp_path / "sides.json"
    assert resolve("auto", "BittleA7", (36, 39), path) == (False, "default")
    assert save_result("BittleC2", 39, (36, 39), path) == ""
    assert save_result("BittleA7", 36, (36, 39), path) == ""
    assert resolve("auto", "BittleC2", (36, 39), path) == (True, "calibrated")
    assert resolve("auto", "BittleA7", (36, 39), path) == (False, "calibrated")
    assert resolve("auto", "BittleC2", (39, 36), path) == (False, "calibrated")  # pins reversed
    assert resolve("auto", "BittleC2", (34, 35), path) == (False, "default")  # other pins
    assert resolve("normal", "BittleC2", (36, 39), path) == (False, "flag")
    assert resolve("swapped", "BittleA7", (36, 39), path) == (True, "flag")
    assert set(json.loads(path.read_text())["robots"]) == {"BittleA7", "BittleC2"}


def test_store_tolerates_a_broken_file_and_env_override(tmp_path, monkeypatch):
    path = tmp_path / "x" / "s.json"
    monkeypatch.setenv("FLY_ESCAPE_SIDES", str(path))
    assert sides.store_path() == path
    path.parent.mkdir()
    path.write_text("{not json")
    assert load_store() == {}
    assert resolve("auto", "BittleC2", (36, 39)) == (False, "default")
    assert save_result("BittleC2", 39, (36, 39)) == ""
    assert resolve("auto", "BittleC2", (36, 39)) == (True, "calibrated")


def test_robot_identity_is_the_name_without_the_suffix():
    def ble(device_name, want=None):
        return types.SimpleNamespace(name="ble", device_name=device_name, want_name=want)

    # the same robot advertises either suffix (macOS name caching): one key for both
    assert robot_identity(ble("BittleA7_BLE")) == robot_identity(ble("BittleA7_SSP")) == "BittleA7"
    assert robot_identity(ble(None, "BittleC2_SSP")) == "BittleC2"
    assert robot_identity(ble(None)) == "ble:(scan)"
    usb = types.SimpleNamespace(name="usb", port="/dev/cu.usbmodem1101")
    banner = "* Start *\r\nBittle X+Arm\r\nIMU task created successfully\r\nBLE:\tBittleA7_BLE\r\n"
    assert robot_identity(usb, banner) == "BittleA7"
    assert robot_identity(usb, "no banner") == "usb:/dev/cu.usbmodem1101"


# ---- in the running demo ------------------------------------------------------------------
def mock_app(swapped=False, robot="BittleZZ"):
    mock = MockBittle()
    if swapped:
        mock.ir_pins = {36: "R", 39: "L"}
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    models, r2p = fake_models()
    loop = EscapeLoop(models, r2p, sides_source="default", robot=robot)
    app = EscapeApp(
        loop,
        FrameHub(),
        bittle=bot,
        mock=mock,
        mode="escape",
        allow_moves=True,
        escape_cfg=EscapeConfig(action_s=0.8, min_interval_s=2.0),
    )
    return mock, app


@pytest.mark.parametrize("swapped", [False, True])
def test_mock_side_check_finds_the_left_pin_and_stores_it(swapped):
    mock, app = mock_app(swapped=swapped)
    app.calibrate_sides = True
    events = []
    app.on_event = events.append
    try:
        app.start_robot()
        app.run(duration=9.0)
    finally:
        app.shutdown()
    lp = app.loop
    assert lp.cal.ok and not lp.side_check.active
    assert lp.swap_sides is swapped and lp.sides_source == "calibrated"
    assert any(e.startswith("sides calibrated") for e in events), events
    # the simulated hand came very close: nothing fired, nothing moved, nothing was scored
    assert not app.escape.history and not any(c.startswith("kbk") for c in mock.commands)
    assert all(sc.triggers == 0 for sc in lp.scores.values())
    assert resolve("auto", "BittleZZ", (36, 39)) == (swapped, "calibrated")
    if swapped:  # the resting levels follow the pins
        assert lp.cal.baseline[0] == pytest.approx(3440, abs=60)
    frame = lp.frame(0.0)
    assert frame["sides"]["source"] == "calibrated" and frame["sides"]["robot"] == "BittleZZ"


def test_dashboard_command_and_its_refusals():
    rows = [(i / 30, 3400, 3440) for i in range(90)]
    models, r2p = fake_models()
    rep = EscapeApp(EscapeLoop(models, r2p), replay=ReplaySource(rows), replay_speed=0)
    assert "replay" in rep._apply_command({"action": "calibrate_sides"}, 0.0)["error"]
    _mock, app = mock_app()
    try:
        r = app._apply_command({"action": "calibrate_sides"}, 0.0)
        assert not r["ok"] and "resting level" in r["error"]  # not measured yet
        app.start_robot()
        app.run(duration=3.0)
        assert app._apply_command({"action": "calibrate_sides"}, app.loop.last_t)["ok"]
        assert app.loop.side_check.phase == "hand"
        assert app.loop.frame(app.loop.last_t)["sides"]["check"]["phase"] == "hand"
        r = app._apply_command({"action": "recalibrate"}, app.loop.last_t)
        assert not r["ok"] and "side check" in r["error"]
    finally:
        app.shutdown()


def test_failed_check_keeps_the_setting(monkeypatch):
    monkeypatch.setattr(sides, "TIMEOUT_S", 2.0)
    _mock, app = mock_app()
    app.loop.swap_sides, app.loop.sides_source = True, "flag"
    try:
        app.start_robot()
        app.run(duration=3.0)
        assert app.start_side_check(app.loop.last_t) is None
        app.mock.hand.set(80.0, 0.0)  # the "person" never brings a hand
        app.run(duration=sides.TIMEOUT_S + 2.0)
    finally:
        app.shutdown()
    assert app.loop.swap_sides is True and app.loop.sides_source == "flag"
    assert "no hand seen" in app.loop.side_check.message
    assert load_store() == {}


# ---- the command line ---------------------------------------------------------------------
def test_cli_side_flags():
    p = cli.build_parser()
    assert p.parse_args([]).sides == "auto"
    assert p.parse_args(["--swap-sides"]).sides == "swapped"
    assert p.parse_args(["--no-swap-sides"]).sides == "normal"
    assert p.parse_args(["--sides", "swapped", "--calibrate-sides"]).calibrate_sides
    with pytest.raises(SystemExit):
        p.parse_args(["--swap-sides", "--no-swap-sides"])


def test_cli_refuses_side_check_on_a_replay(capsys):
    assert cli.main(["--replay", "x.json", "--calibrate-sides"]) == 2
    assert "--calibrate-sides" in capsys.readouterr().err


def test_cli_mock_calibrate_sides(monkeypatch, tmp_path, capsys):
    monkeypatch.setitem(sys.modules, "flybrain_models.escape", escape_fakes.install())
    args = ["--mock", "--no-dashboard", "--runs-dir", str(tmp_path), "--duration"]
    assert cli.main([*args, "9", "--calibrate-sides"]) == 0
    out = capsys.readouterr().out
    assert "side check: hold a hand" in out and "sides calibrated: pin 36" in out, out
    assert "saved for simulator" in out
    assert cli.main([*args, "0.5"]) == 0
    assert "left pin 36 (calibrated for this robot)" in capsys.readouterr().out
    assert cli.main([*args, "0.5", "--swap-sides"]) == 0
    assert "left pin 39 (set by flag)" in capsys.readouterr().out
