"""Our camera module in the robot's firmware (firmware/bittle/fly_pursuit, letter P): the "@FP"
line parser, the module reader, --camera auto / module, --onboard, and the exact start-up and
shut-down sequences, on a fake robot. No hardware is touched."""

import json
import math
import re
import time
from importlib import resources

import pytest
from fly_pursuit import cli, flydeck
from fly_pursuit import robot as R
from fly_pursuit.app import PursuitApp
from fly_pursuit.camera import (
    FP_ATTENTION,
    FP_ERROR_KINDS,
    FpStream,
    ModuleReader,
    parse_fp,
)
from fly_pursuit.sim import PursuitSim
from pursuit_fakes import FP_FACE_LEFT, FakeRobot, factory

MODULE_START = ["?", "X?~", "Xa~", "XP~", "ksit"]
MODULE_STOP = ["Xp~", "XA~", "d"]
ONBOARD_START = ["?", "X?~", "Xa~", "XP~", "XPB~"]
ONBOARD_STOP = ["XPb~", "Xp~", "XA~", "d"]
STOCK_START = ["?", "X?~", "Xa~", "XCr~", "XCP~"]
STOCK_STOP = ["Xc~", "XA~", "gU", "d"]
CAMERA_OFF_MODULE = ["?", "X?~", "Xps~", "Xc~", "Xa~", "XA~", "gU", "d"]
AUTOSTART_ON = ["?", "X?~", "XPS~", "Xp~", "d"]
AUTOSTART_OFF = ["?", "X?~", "Xps~", "d"]
FOLLOWING = ["@FP 7 -40 -16 69 76 80 b=10.0 t=30.0 p=5.0;\r\n"]  # an autostarted robot


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(cli, "START_DELAY_S", 0.0)
    monkeypatch.setattr(R, "JOINT_SETTLE_S", 0.05)
    monkeypatch.setattr(R, "X_TIMEOUT_S", 0.5)
    monkeypatch.setattr(R, "FP_STATUS_WAIT_S", 0.2)
    monkeypatch.setattr(R, "AUTOSTART_WAIT_S", 0.3)


# ---- the line parser ------------------------------------------------------------------------
def test_face_on_the_left_has_a_negative_x_and_a_positive_bearing():
    m = parse_fp("300 -30 5 69 76 80")
    assert m.kind == "result" and m.seq == 300 and m.box == (-30, 5, 69, 76, 80)
    assert not m.has_control and m.turn_deg_s is None and not m.hold
    r = ModuleReader(fov_deg=60)
    r.feed(b"@FP 300 -30 5 69 76 80;\r\n", 1.0)
    assert math.degrees(r.last.bearing) == pytest.approx(7.5)  # left
    assert r.last.score == 80 and r.input(1.05).state == "seen"


def test_face_on_the_right_and_no_face():
    m = parse_fp("2 31 -12 83 90 76")
    assert m.box == (31, -12, 83, 90, 76)
    r = ModuleReader(fov_deg=60)
    r.feed(b"@FP 2 31 -12 83 90 76;", 1.0)
    assert math.degrees(r.last.bearing) == pytest.approx(-31 / 240 * 60)  # right
    m = parse_fp("301 -")
    assert m.kind == "result" and m.seq == 301 and m.box is None and not m.has_control


@pytest.mark.parametrize(
    ("body", "box", "b", "t", "p", "hold"),
    [
        ("40 -64 -10 80 88 79 b=16.0 t=24.3 p=3.2", (-64, -10, 80, 88, 79), 16.0, 24.3, 3.2, False),
        ("41 64 -10 80 88 79 b=-16.0 t=-24.3 p=-3.2", (64, -10, 80, 88, 79), -16.0, -24.3, -3.2, 0),
        ("42 -64 -10 80 88 79 t=24.3 p=3.2", (-64, -10, 80, 88, 79), None, 24.3, 3.2, False),
        ("233 - t=0.0 p=21.5 hold", None, None, 0.0, 21.5, True),
        ("234 - t=-12.5 p=-21.5", None, None, -12.5, -21.5, False),
    ],
)
def test_results_with_on_board_control(body, box, b, t, p, hold):
    m = parse_fp(body)
    assert m is not None and m.kind == "result" and m.has_control
    assert (m.box, m.bearing_deg, m.turn_deg_s, m.pan_deg, m.hold) == (box, b, t, p, bool(hold))


def test_status_camera_and_every_error_kind():
    assert parse_fp("on l=1 b=0") == parse_fp(" on  l=1 b=0 ")
    m = parse_fp("on l=1 b=0")
    assert (m.kind, m.on, m.lines, m.control) == ("status", True, True, False)
    m = parse_fp("on l=0 b=1")
    assert (m.on, m.lines, m.control) == (True, False, True)
    m = parse_fp("off")
    assert (m.kind, m.on) == ("status", False)
    m = parse_fp("cam run 2")
    assert (m.kind, m.running, m.starts) == ("cam", True, 2)
    m = parse_fp("cam stop")
    assert (m.kind, m.running) == ("cam", False)
    for kind in FP_ERROR_KINDS:
        m = parse_fp(f"err {kind} 7")
        assert (m.kind, m.what, m.count) == ("err", kind, 7)
    assert parse_fp("boot bus sda=1 scl=1 now sda=1 scl=1").kind == "boot"


@pytest.mark.parametrize(
    "body",
    [
        "",
        "-",
        "1",
        "x 1 2 3 4 5",
        "5 1 2 3",  # too few box fields
        "5 1 2 3 4 101",  # score above 100
        "5 1 2 -3 4 50",  # negative width
        "5 1.5 2 3 4 50",
        "70000 -",  # beyond the 16-bit sequence number
        "-5 -",
        "5 - foo",
        "5 - t=1.0",  # t= without p=
        "5 - p=1.0",
        "5 - b=3.0 t=1.0 p=2.0",  # b= only with a face
        "5 - hold",
        "5 - t=nan p=1",
        "5 1 2 3 4 50 q=1 t=1 p=1",
        "on",
        "on l=2 b=0",
        "off now",
        "cam",
        "cam run",
        "cam run x",
        "err",
        "err silent",
        "err silent x",
        "err Silent 1",
    ],
)
def test_garbage_is_ignored(body):
    assert parse_fp(body) is None


def test_lines_split_across_chunks_and_several_per_chunk():
    text = (
        "sit\r\nX\r\n@FP on l=1 b=0;\r\n@FP cam run 1;\r\n"
        + FP_FACE_LEFT
        + "@FP 2 -;\r\n@FP 3 31 -12 83 90 76;\r\n@FP err i2c 1;\r\n"
    ).encode()
    for size in (1, 3, 7, len(text)):  # byte by byte ... all at once
        r = ModuleReader(fov_deg=60)
        for i in range(0, len(text), size):
            r.feed(text[i : i + size], 1.0)
        h = r.summary()
        assert r.results == 3 and r.faces == 2 and r.last.x == 31, size
        assert h["present"] and h["on"] and h["camera"] == "running" and h["starts"] == 1
        assert h["errors"] == {"i2c": 1} and h["missing"] == 0 and h["last_seq"] == 3


def test_newlines_lost_or_moved_and_garbage_between():
    s = FpStream()
    got = s.feed("@FP 1 -;@FP 2 -30 5 69 76 80;@F")  # no newlines (Bluetooth)
    got += s.feed("P 3 -;\r\n@FE on tel=0;\r\n@FP bogus line;\r\n@FP 4 -\r\n;")
    got += s.feed("noise @FP 5 1 2 3 4 50 b=1.0 t=2.0 p=3.0;")
    got += s.feed("@FP 6 -")  # unfinished: kept for the next read
    assert [m.seq for m in got] == [1, 2, 3, 4, 5]
    assert s.feed(";")[0].seq == 6
    assert s.feed("@FP 7 " + "1" * 300) == [] and s.feed(";") == []  # oversized: dropped


def test_no_face_is_lost_at_once_and_the_time_lost_counts_from_the_first():
    r = ModuleReader(fov_deg=60)
    r.feed(FP_FACE_LEFT.encode(), 1.0)
    assert r.input(1.01).state == "seen"
    r.feed(b"@FP 2 -;", 1.1)
    inp = r.input(1.12)
    assert inp.state == "lost" and inp.bearing is None
    for k in range(3, 50):  # "no face" every frame
        r.feed(f"@FP {k} -;".encode(), 1.1 + (k - 2) / 15)
    assert r.input(1.1 + 48 / 15).lost_s == pytest.approx(48 / 15, abs=0.01)


def test_no_line_at_all_for_a_second_is_lost():
    r = ModuleReader(fov_deg=60)
    r.feed(FP_FACE_LEFT.encode(), 1.0)
    assert r.input(1.9).state == "seen"
    assert r.input(1.0 + r.stale_s + 0.1).state == "lost"


def test_missing_sequence_numbers_are_counted():
    r = ModuleReader()
    for seq in (10, 11, 14, 15, 20):  # 2 + 4 missing
        r.feed(f"@FP {seq} -;".encode(), 1.0)
    assert r.missing == 6
    r = ModuleReader()
    for seq in (65534, 65535, 0, 1):  # the 16-bit counter wraps: nothing missing
        r.feed(f"@FP {seq} -;".encode(), 1.0)
    assert r.missing == 0
    r.feed(b"@FP 3 -;", 1.0)
    assert r.missing == 1
    r.feed(b"@FP 9000 -;", 1.0)  # a jump this large is a restarted board, not lost lines
    assert r.missing == 1 and r.summary()["last_seq"] == 9000


# ---- attention -----------------------------------------------------------------------------
class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _module_rig(onboard=False, **kw):
    clock = Clock()
    reader = ModuleReader(fov_deg=60)
    said = []
    cfg = R.HeadConfig(camera_source="module", onboard=onboard, **kw)
    rig = R.HeadRig(reader, cfg, clock=clock, on_attention=said.append)
    return rig, reader, clock, said


def _tick(rig, clock, dt=1 / 15):
    clock.t += dt
    rig.advance(dt)


@pytest.mark.parametrize(
    ("kind", "words"),
    [("silent", "switch the robot off and on"), ("busheld", "switch the robot off and on")],
)
def test_camera_to_power_cycle_raises_attention(kind, words):
    rig, reader, clock, said = _module_rig()
    reader.feed(f"@FP err {kind} 1;".encode(), clock.t)
    _tick(rig, clock)
    assert said == [FP_ATTENTION[kind]] and words in said[0] and "power-cycle" in said[0]
    f = rig.snapshot()
    assert f["attention"] == said[0] and f["camera_module"]["problem"] == kind
    reader.feed(f"@FP err {kind} 2;".encode(), clock.t)  # again: said once
    _tick(rig, clock)
    assert len(said) == 1
    reader.feed(b"@FP cam run 2;", clock.t)  # the camera answers again: cleared
    _tick(rig, clock)
    assert said == [FP_ATTENTION[kind], None] and rig.snapshot()["attention"] is None


def test_no_camera_on_the_socket_raises_attention():
    rig, reader, clock, said = _module_rig()
    reader.feed(b"@FP on l=1 b=0;\r\n@FP err nocam 1;\r\n", clock.t)
    _tick(rig, clock)
    assert said == [FP_ATTENTION["nocam"]] and "not plugged" in said[0]
    assert "SCL SDA +5V GND" in said[0]
    reader.feed(FP_FACE_LEFT.encode(), clock.t)  # a result: the camera works after all
    _tick(rig, clock)
    assert said[-1] is None


def test_errors_the_module_handles_itself_raise_nothing():
    rig, reader, clock, said = _module_rig()
    reader.feed(FP_FACE_LEFT.encode(), clock.t)
    for kind in ("i2c", "lock", "stall", "overflow", "cmd", "recover"):
        reader.feed(f"@FP err {kind} 3;".encode(), clock.t)
    _tick(rig, clock)
    assert said == [] and rig.snapshot()["camera_module"]["errors"]["recover"] == 3


def test_no_result_line_in_20_s_raises_attention():
    rig, _reader, clock, said = _module_rig()
    for _ in range(21 * 15):
        _tick(rig, clock)
    assert len(said) == 1 and "camera module" in said[0]


# ---- the head loop with the module ------------------------------------------------------------
def test_module_mode_the_laptop_turns_the_head_and_holds_after_3_s_of_no_face():
    rig, reader, clock, _ = _module_rig()
    for k in range(10):
        reader.feed(f"@FP {k} -72 -16 69 76 80;".encode(), clock.t)
        _tick(rig, clock)
    assert rig.pan > 0 and rig.input.state == "seen"
    seq = 10
    for _ in range(int(R.HOLD_AFTER_S * 15) + 3):  # "no face" every frame
        reader.feed(f"@FP {seq} -;".encode(), clock.t)
        seq += 1
        _tick(rig, clock)
    assert rig.input.state == "lost" and rig.holding
    held = rig.pan
    reader.feed(f"@FP {seq} -;".encode(), clock.t)
    _tick(rig, clock)
    assert rig.pan == held and rig.cmd == 0.0


def test_onboard_mode_shows_the_robots_numbers_and_the_shadow_model():
    rig, reader, clock, _ = _module_rig(onboard=True)
    assert rig.set_controller("bang-bang") == R.ONBOARD_LOCKED
    assert rig.set_arousal(0.5) == R.ONBOARD_LOCKED
    assert rig.controller == "connectome" and rig.arousal == 1.0
    lines = [
        "@FP 1 -72 -16 69 76 80 b=18.0 t=60.0 p=4.0;",
        "@FP 2 -60 -16 69 76 80 b=11.0 t=60.0 p=8.0;",
        "@FP 3 -40 -16 69 76 80 b=2.0 t=21.3 p=9.4;",
    ]
    for k, ln in enumerate(lines):
        reader.feed(ln.encode(), clock.t + k / 15)
    clock.t += 3 / 15
    rig.advance(1 / 15)
    f = rig.snapshot()
    ob = f["onboard"]
    assert ob["results"] == 3 and ob["turn_deg_s"] == 21.3 and ob["pan_deg"] == 9.4
    assert ob["bearing_deg"] == 2.0 and ob["hold"] is False
    assert f["head_deg"] == pytest.approx(9.4) and f["cmd_deg_s"] == pytest.approx(21.3)
    assert f["error_deg"] == pytest.approx(2.0)  # what the robot's circuit got
    assert f["raw_bearing_deg"] == pytest.approx(10.0)  # x = -40 measured: 10 deg left
    assert ob["shadow_turn_deg_s"] > 0 and 0 < ob["shadow_cmd_deg_s"] <= 60
    assert ob["difference_deg_s"] == pytest.approx(ob["shadow_cmd_deg_s"] - 21.3, abs=0.01)
    assert ob["mean_abs_difference_deg_s"] is not None
    assert f["controller"] == "connectome" and f["circuit"] is True
    reader.feed(b"@FP 4 - t=0.0 p=9.4 hold;", clock.t)
    _tick(rig, clock)
    f = rig.snapshot()
    assert f["hold"] and f["onboard"]["hold"] and f["onboard"]["shadow_cmd_deg_s"] == 0.0
    assert f["error_deg"] is None and f["onboard"]["bearing_deg"] is None


def test_onboard_commands_from_the_page_are_refused():
    rig, _reader, _clock, _ = _module_rig(onboard=True)
    app = PursuitApp(rig, frame_hz=15)
    for cmd in (
        {"action": "controller", "controller": "bang-bang"},
        {"action": "arousal", "value": 0.2},
        {"action": "target", "deg": 10},
    ):
        assert app.on_command(cmd)["ok"] is False
    assert app.on_command({"action": "reset"}) == {"ok": True}


def _page_fields():
    page = resources.files("fly_pursuit").joinpath("static/index.html").read_text()
    # frame.<name>, and f.<name> where a function takes the frame as f
    return set(re.findall(r"\b(?:frame|f)\.([a-z_0-9]+)", page))


@pytest.mark.parametrize("onboard", [False, True])
def test_frames_carry_every_field_the_page_reads(onboard):
    rig, reader, _clock, _ = _module_rig(onboard=onboard)
    app = PursuitApp(rig, frame_hz=15)
    line = "@FP 1 -72 -16 69 76 80" + (" b=18.0 t=60.0 p=4.0" if onboard else "") + ";"
    reader.feed(f"@FP on l=1 b={int(onboard)};@FP cam run 1;{line}".encode(), time.monotonic())
    f = json.loads(json.dumps(app.tick(1 / 15)))
    s = json.loads(json.dumps(PursuitApp(PursuitSim(seed=1)).tick(0.1)))
    assert set(f) == set(s)
    assert _page_fields() <= set(f)
    assert f["camera_source"] == "module" and f["camera_score"] == 80
    m = f["camera_module"]
    for k in ("present", "on", "camera", "starts", "errors", "results", "faces", "missing"):
        assert k in m
    assert m["camera"] == "running" and m["results"] == 1 and m["control_on"] is onboard
    assert s["camera_module"] is None and s["onboard"] is None
    if onboard:
        for k in (
            "results",
            "turn_deg_s",
            "pan_deg",
            "bearing_deg",
            "hold",
            "shadow_turn_deg_s",
            "shadow_cmd_deg_s",
            "difference_deg_s",
            "mean_abs_difference_deg_s",
            "max_abs_difference_deg_s",
        ):
            assert k in f["onboard"], k
        assert f["activity"] is not None  # the shadow model's activity
    else:
        assert f["onboard"] is None


def test_page_reads_the_new_fields():
    used = _page_fields()
    assert {"onboard", "camera_module", "camera_source", "attention"} <= used


# ---- start-up and shut-down through the real program ----------------------------------------
def _run(argv, *robots, port=True):
    link = ["--port", "/dev/fake"] if port else ["--ble"]
    return cli.main(
        [*link, "--no-open", "--http-port", "0", *argv], make_transport=factory(*robots)
    )


def _heads(cmds):
    return [c for c in cmds if c.startswith("i")]


def _events(out):
    return [json.loads(ln[8:]) for ln in out.splitlines() if ln.startswith("FLYDECK ")]


def test_auto_chooses_the_module_and_runs_it_exactly(capsys, monkeypatch):
    monkeypatch.setenv("FLYDECK_STATUS", "1")
    bot = FakeRobot(module="off", fp_lines=[FP_FACE_LEFT], serial_lines=True)
    assert _run(["--duration", "0.8"], bot) == 0
    cmds = bot.commands()
    assert bot.non_head() == MODULE_START + MODULE_STOP
    assert not any(c.startswith("XC") or c == "gU" for c in cmds)  # no stock camera, no gU
    assert bot._ser.log == [] and bot.boots == 0  # no board restart
    assert any(c.startswith("i0 ") and int(c.split()[1]) > 0 for c in cmds)  # turned left
    assert _heads(cmds[: cmds.index("Xp~")])[-1] == "i0 0"  # centred before the module goes off
    assert not _heads(cmds[: cmds.index("ksit")])  # no head command before it sits
    out, _ = capsys.readouterr()
    assert "using it (--camera module)" in out
    assert "camera module: 1 result lines, 0 missing" in out
    assert not any(e["event"] == "failed" for e in _events(out))


def test_auto_finds_the_status_line_after_the_echo_too():
    bot = FakeRobot(module="on", status_after_echo=True)
    assert _run(["--duration", "0.3"], bot) == 0
    assert bot.non_head() == MODULE_START + MODULE_STOP


def test_auto_falls_back_to_the_stock_module_over_bluetooth(capsys):
    bot = FakeRobot(name="ble")
    assert _run(["--duration", "0.3"], bot, port=False) == 0
    assert bot.non_head() == STOCK_START + STOCK_STOP
    out, _ = capsys.readouterr()
    assert "does not have our camera module" in out and "--camera robot" in out
    assert "once per power-on" in out


def test_auto_falls_back_over_usb_and_restarts_the_board_for_the_stock_module(capsys):
    first, second = FakeRobot(serial_lines=True), FakeRobot(serial_lines=True)
    assert _run(["--duration", "0.3"], first, second) == 0
    assert first.commands() == ["?", "X?~"] and first._ser.log == []
    assert second._ser.log == [("dtr", False), ("rts", True), ("rts", False)]
    assert second.non_head() == ["?", "Xa~", "XCr~", "XCP~", *STOCK_STOP]
    assert "restarting the robot's board" in capsys.readouterr().out


def test_module_mode_on_a_stock_firmware_is_refused(capsys):
    bot = FakeRobot()
    assert _run(["--camera", "module"], bot) == 4
    assert bot.commands() == ["?", "X?~"]  # nothing else sent, nothing moved
    assert "firmware/bittle/fly_pursuit" in capsys.readouterr().err


def test_module_mode_xp_off_on_every_exit_path(monkeypatch, capsys):
    def boom(self, seconds):
        raise RuntimeError("controller exploded")

    bot = FakeRobot(module="off")
    monkeypatch.setattr(R.HeadRig, "advance", boom)
    assert _run(["--camera", "module"], bot) == 1
    assert bot.non_head() == MODULE_START + MODULE_STOP
    assert "controller exploded" in capsys.readouterr().err


def test_module_mode_lost_link_reconnects_and_switches_the_module_off(capsys):
    first = FakeRobot(module="off", fp_lines=[FP_FACE_LEFT], lose_after_heads=2)
    second = FakeRobot(module="on")
    assert _run(["--camera", "module", "--duration", "10"], first, second) == 1
    assert first.non_head() == MODULE_START
    assert second.non_head() == ["?", *MODULE_STOP]
    assert _heads(second.commands()) == ["i0 0"]


def test_module_mode_without_a_reply_to_xp_says_the_saved_state_is_harmless(capsys):
    bot = FakeRobot(module="off", drop_echo=("Xp~",))
    assert _run(["--duration", "0.3", "--leave-voice-off"], bot) == 0
    assert bot.non_head() == [*MODULE_START, "Xp~", "d"]  # no XA~: voice left off
    out, _ = capsys.readouterr()
    assert "harmless" in out and "unplug the camera" not in out


def test_onboard_sequences_exactly_and_no_head_command_while_running(capsys, monkeypatch):
    monkeypatch.setenv("FLYDECK_STATUS", "1")
    lines = [
        "@FP 1 -72 -16 69 76 80 b=18.0 t=60.0 p=4.0;\r\n",
        "@FP 3 -40 -16 69 76 80 b=2.0 t=21.3 p=9.4;\r\n",  # seq 2 missing
    ]
    bot = FakeRobot(module="off", onboard_lines=lines, serial_lines=True)
    assert _run(["--onboard", "--duration", "0.8"], bot) == 0
    cmds = bot.commands()
    assert bot.non_head() == ONBOARD_START + ONBOARD_STOP
    assert "ksit" not in cmds and bot._ser.log == []  # the firmware sits; no restart
    assert not _heads(cmds[: cmds.index("XPb~")])  # no head command from the laptop running
    assert _heads(cmds) == ["i0 0"] and cmds.index("i0 0") < cmds.index("Xp~")
    assert cmds.index("XPb~") < cmds.index("Xp~")
    out, _ = capsys.readouterr()
    assert "1 missing between sequence numbers" in out
    assert "on-board: 2 results" in out
    ev = _events(out)
    assert any(e.get("detail", "").startswith("on the robot") for e in ev)


def test_onboard_without_the_module_is_refused(capsys):
    bot = FakeRobot(name="ble")
    assert _run(["--onboard"], bot, port=False) == 4
    assert bot.commands() == ["?", "X?~"]
    err = capsys.readouterr().err
    assert "firmware/bittle/fly_pursuit" in err and "does not have" in err


def test_onboard_ignores_the_laptop_controller_settings(capsys):
    bot = FakeRobot(module="on")
    argv = ["--onboard", "--controller", "bang-bang", "--gain", "0.5", "--duration", "0.3"]
    assert _run(argv, bot) == 0
    assert "ignored: --controller, --gain" in capsys.readouterr().out
    assert bot.non_head() == ONBOARD_START + ONBOARD_STOP


@pytest.mark.parametrize(
    "argv",
    [
        ["--port", "/dev/null-robot", "--onboard", "--camera", "robot"],
        ["--port", "/dev/null-robot", "--onboard", "--camera", "usb"],
        ["--mock", "--onboard"],
    ],
)
def test_onboard_bad_combinations_are_refused_before_connecting(argv):
    def no_link():
        raise AssertionError("a link was opened")

    assert cli.main(argv, make_transport=no_link) == 2


def test_camera_off_with_the_module_sends_xps_first(capsys):
    bot = FakeRobot(module="on", name="ble")  # an older build: Xps~ is taken as Xp~
    assert cli.main(["--ble", "--camera-off"], make_transport=factory(bot)) == 0
    assert bot.commands() == CAMERA_OFF_MODULE and bot.module == "off"
    out = capsys.readouterr().out
    assert "our camera module off" in out and "after power-on" not in out


def test_the_card_offers_the_new_choices():
    spec = flydeck.SPEC
    assert flydeck.CAMERAS == R.CAMERA_SOURCES
    cam = next(o for o in spec.options if o.name == "camera")
    assert cam.default == "auto" and set(cam.choices) == {"auto", "module", "robot", "usb"}
    onboard = next(o for o in spec.options if o.name == "onboard")
    assert onboard.type == "bool" and onboard.default is False
    assert any("firmware/bittle/fly_pursuit" in n for n in spec.needs)
    a = cli.build_parser().parse_args(spec.argv("ble", {"onboard": True}, 8123))
    assert a.onboard and a.camera == "auto"
    a = cli.build_parser().parse_args(spec.argv("usb", {"camera": "module"}, 8123, robot="/x"))
    assert a.camera == "module" and not a.onboard


# ---- the saved "follow a face after power-on" setting (newer builds of the module) ----------
@pytest.mark.parametrize(
    ("body", "autostart", "started"),
    [("auto 0", False, False), ("auto 1", True, False), ("auto start", None, True)],
)
def test_auto_lines(body, autostart, started):
    m = parse_fp(body)
    assert (m.kind, m.autostart, m.started) == ("auto", autostart, started)
    assert parse_fp(f" {body} ") == m


@pytest.mark.parametrize(
    "body", ["auto", "auto 2", "auto -1", "auto on", "auto 1 1", "auto start now", "auto Start"]
)
def test_malformed_auto_lines_are_ignored(body):
    assert parse_fp(body) is None


def test_status_lines_still_parse_beside_the_auto_line():
    got = FpStream().feed("@FP on l=1 b=0;\r\n@FP auto 1;\r\n@FP off;@FP auto 0;@FP 3 -;")
    assert [m.kind for m in got] == ["status", "auto", "status", "auto", "result"]
    assert (got[0].on, got[0].lines, got[0].control) == (True, True, False)
    assert got[2].on is False and got[1].autostart is True and got[3].autostart is False


def test_module_reader_keeps_the_setting_and_whether_it_fired():
    r = ModuleReader()
    h = r.summary()
    assert h["autostart"] is None and h["autostarted"] is False  # never reported (older build)
    r.feed(b"@FP on l=1 b=0;\r\n@FP auto 0;\r\n", 1.0)
    assert r.summary()["autostart"] is False and r.auto_count == 1 and r.status_count == 1
    r.feed(b"@FP auto 1;", 1.1)
    assert r.summary()["autostart"] is True and not r.summary()["autostarted"]
    r = ModuleReader()
    r.feed(b"@FP on l=1 b=1;@FP auto 1;@FP auto start;", 1.0)
    h = r.summary()
    assert h["autostart"] is True and h["autostarted"] is True and h["control_on"] is True
    assert r.auto_count == 1  # "auto start" is not a report of the setting


def test_the_frame_carries_the_setting():
    rig, reader, clock, _ = _module_rig()
    assert rig.snapshot()["camera_module"]["autostart"] is None
    reader.feed(b"@FP on l=1 b=1;@FP auto 1;@FP auto start;", clock.t)
    _tick(rig, clock)
    m = json.loads(json.dumps(rig.snapshot()))["camera_module"]
    assert m["autostart"] is True and m["autostarted"] is True
    page = resources.files("fly_pursuit").joinpath("static/index.html").read_text()
    assert "m.autostart === true" in page and "fly-pursuit --autostart off" in page


def _action(argv, *robots):
    return cli.main(["--port", "/dev/fake", *argv], make_transport=factory(*robots))


def test_autostart_on_saves_it_and_leaves_the_camera_off(capsys, monkeypatch):
    monkeypatch.setenv("FLYDECK_STATUS", "1")
    bot = FakeRobot(module="off", autostart=False)
    assert _action(["--autostart", "on"], bot) == 0
    assert bot.commands() == AUTOSTART_ON
    assert bot.autostart is True and bot.module == "off" and not bot.control
    out, _ = capsys.readouterr()
    final = [ln for ln in out.splitlines() if ln.startswith("fly-pursuit:")][-1]
    assert "is ON" in final and "beeps three times" in final and "floor" in final
    assert "away from edges" in final and "--autostart off" in final
    assert not any(e["event"] == "failed" for e in _events(out))


def test_autostart_off_saves_it(capsys):
    bot = FakeRobot(module="on", autostart=True, name="ble")
    assert cli.main(["--ble", "--autostart", "off"], make_transport=factory(bot)) == 0
    assert bot.commands() == AUTOSTART_OFF
    assert bot.autostart is False and bot.module == "off"
    assert "is OFF" in capsys.readouterr().out


def test_autostart_off_on_a_robot_following_by_itself_puts_voice_back():
    bot = FakeRobot(autostarted=True)
    assert _action(["--autostart", "off"], bot) == 0
    assert bot.commands() == [*AUTOSTART_OFF[:-1], "XA~", AUTOSTART_OFF[-1]]
    assert bot.autostart is False and bot.module == "off" and not bot.control


@pytest.mark.parametrize(("value", "sent"), [("on", AUTOSTART_ON), ("off", AUTOSTART_OFF)])
def test_autostart_on_an_older_module_is_reported_not_taken_as_done(value, sent, capsys):
    bot = FakeRobot(module="off")  # the build on the robot today: no S / s, no "@FP auto"
    assert _action(["--autostart", value], bot) == 4
    assert bot.commands() == sent and bot.module == "off"  # the camera is off again
    out, err = capsys.readouterr()
    assert "older" in err and "no autostart" in err and "firmware/bittle/fly_pursuit" in err
    assert "saved on the robot" not in out


def test_autostart_without_the_module_is_refused(capsys):
    bot = FakeRobot()
    assert _action(["--autostart", "on"], bot) == 4
    assert bot.commands() == ["?", "X?~"]  # nothing else sent, not even a rest
    assert "does not have the fly_pursuit camera module" in capsys.readouterr().err


def test_autostart_with_no_answer_is_not_taken_as_an_older_module(capsys):
    bot = FakeRobot(module="off", autostart=False, mute_from="XPS~")
    assert _action(["--autostart", "on"], bot) == 1
    assert bot.commands() == AUTOSTART_ON
    err = capsys.readouterr().err
    assert "did not answer XPS~" in err and "older" not in err


@pytest.mark.parametrize(
    "argv",
    [
        ["--mock", "--autostart", "on"],
        ["--port", "/dev/null-robot", "--autostart", "on", "--camera-off"],
        ["--port", "/dev/null-robot", "--autostart", "off", "--camera-off"],
    ],
)
def test_autostart_bad_combinations_are_refused_before_connecting(argv):
    def no_link():
        raise AssertionError("a link was opened")

    assert cli.main(argv, make_transport=no_link) == 2


def test_camera_off_clears_the_autostart_and_says_so(capsys):
    bot = FakeRobot(module="on", autostart=True)
    assert _action(["--camera-off"], bot) == 0
    assert bot.commands() == CAMERA_OFF_MODULE
    assert bot.autostart is False and bot.module == "off"
    assert "follow a face after power-on cleared" in capsys.readouterr().out


def test_an_autostarted_robot_laptop_mode_takes_the_head_over(capsys):
    bot = FakeRobot(
        autostarted=True, fp_lines=[FP_FACE_LEFT], following_lines=FOLLOWING, serial_lines=True
    )
    assert _run(["--duration", "0.8"], bot) == 0
    assert bot.non_head() == MODULE_START + MODULE_STOP
    heads = [(c, ctl) for c, ctl in bot.control_at if c.startswith("i")]
    assert heads and not any(ctl for _, ctl in heads)  # the robot's own control was off
    assert dict(bot.control_at)["XP~"] is False  # the bare XP~ switched it off
    assert bot.autostart is True  # a normal run leaves the saved setting alone
    out, _ = capsys.readouterr()
    assert "already turning its head by itself" in out and "leaves that setting" in out


def test_an_autostarted_robot_onboard_mode_runs_as_usual():
    lines = ["@FP 20 -40 -16 69 76 80 b=10.0 t=21.3 p=9.4;\r\n"]
    bot = FakeRobot(autostarted=True, following_lines=FOLLOWING, onboard_lines=lines)
    assert _run(["--onboard", "--duration", "0.5"], bot) == 0
    assert bot.non_head() == ONBOARD_START + ONBOARD_STOP
    assert _heads(bot.commands()) == ["i0 0"]
    assert bot.autostart is True and not bot.control and bot.module == "off"


def test_the_card_offers_the_autostart_choice():
    spec = flydeck.SPEC
    spec.validate()
    o = next(o for o in spec.options if o.name == "autostart")
    assert o.type == "choice" and o.default == "leave as it is"
    assert tuple(o.choices) == ("leave as it is", "on", "off")
    names = [o.name for o in spec.options]
    assert names.index("autostart") == names.index("camera_off") + 1
    argv = spec.argv("ble", {}, 8123)
    assert "--autostart" not in argv and cli.build_parser().parse_args(argv).autostart is None
    argv = spec.argv("ble", {"autostart": "leave as it is"}, 8123)
    assert "--autostart" not in argv
    for value in ("on", "off"):
        a = cli.build_parser().parse_args(spec.argv("usb", {"autostart": value}, 1, robot="/x"))
        assert a.autostart == value and a.port == "/x"
