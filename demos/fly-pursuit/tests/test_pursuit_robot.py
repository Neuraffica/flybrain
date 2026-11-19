"""Robot mode on a fake robot: the camera lines, the staleness rule, the head loop, and the
exact start-up and shut-down sequences. No hardware is touched."""

import itertools
import json
import math

import pytest
from fly_pursuit import cli
from fly_pursuit import robot as R
from fly_pursuit.camera import (
    CENTRE_DEG,
    STALE_S,
    CameraLine,
    CameraReader,
    bearing_rad,
    camera_input,
)
from pursuit_fakes import FACE_LEFT, FakeRobot, factory

STARTUP = ["?", "Xa~", "XCr~", "XCP~"]
SHUTDOWN = ["Xc~", "XA~", "gU", "d"]


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(cli, "START_DELAY_S", 0.0)
    monkeypatch.setattr(R, "JOINT_SETTLE_S", 0.05)
    monkeypatch.setattr(R, "X_TIMEOUT_S", 0.5)


# ---- camera lines ------------------------------------------------------------------------
def test_a_face_on_the_left_is_a_positive_bearing():
    r = CameraReader(fov_deg=60)
    r.feed(FACE_LEFT.encode(), 1.0)
    assert r.lines == 1 and r.last.x == -72 and r.last.width == 69 and r.last.height == 76
    assert math.degrees(r.last.bearing) == pytest.approx(72 / 240 * 60)  # 18 deg, left
    assert bearing_rad(120, 60) == pytest.approx(-math.radians(30))  # right edge
    assert bearing_rad(-60, 90) == pytest.approx(math.radians(22.5))


@pytest.mark.parametrize(
    "junk",
    [
        "Ready!",
        "=",
        "X",
        "size = 69\t76",
        "-72.00\tsize = 69\t76",
        "-72.00\t-16.00\tsize = ",
        "abc\tdef\tsize = 1\t2",
        "ICM:  -1.3  -0.8   9.9  130.8   -7.6   -4.5\t",
        "Camera task activated.",
    ],
)
def test_malformed_lines_are_ignored(junk):
    r = CameraReader()
    r.feed((junk + "\r\n").encode(), 1.0)
    assert r.lines == 0 and r.last is None


def test_lines_split_across_chunks():
    r = CameraReader()
    data = ("sit\r\nX\r\n" + FACE_LEFT + "53.00\t-44.00\tsize = 83\t90\t\r\n").encode()
    for i in range(0, len(data), 7):  # 7-byte pieces
        r.feed(data[i : i + 7], 2.0)
    assert r.lines == 2 and r.last.x == 53


def test_a_line_split_over_bluetooth_notifications_is_joined():
    r = CameraReader()
    for piece in ("-72.00\t", "-16.00\t", "size = 69\t76\t"):  # one notification each
        r.feed((piece + "\n").encode(), 1.0)
    assert r.lines == 1 and r.last.x == -72


def test_a_line_glued_to_an_echo():
    r = CameraReader()
    r.feed(b"X-72.00\t-16.00\tsize = 69\t76\t\n", 1.0)
    assert r.last.x == -72


def test_nothing_seen_means_lost():
    r = CameraReader()
    r.feed(FACE_LEFT.encode(), 1.0)
    r.feed(b"-121.00\t-121.00\tsize = 0\t0\r\n", 1.5)
    inp = r.input(1.6)
    assert inp.state == "lost" and inp.bearing is None


# ---- the staleness rule ------------------------------------------------------------------
def _line(t, deg):
    return CameraLine(t, 0.0, 0.0, 80, 90, math.radians(deg))


def test_efference_copy_arithmetic():
    from fly_pursuit.camera import corrected_bearing

    # measured 20 deg left with the head at 0; the head has since turned 15 deg left
    assert corrected_bearing(math.radians(20), 0.0, math.radians(15)) == pytest.approx(
        math.radians(5)
    )
    last = CameraLine(1.0, 0, 0, 80, 90, math.radians(20), pan=math.radians(10))
    inp = camera_input(1.5, last, pan_now=math.radians(25))
    assert inp.state == "seen" and math.degrees(inp.bearing) == pytest.approx(5.0)
    assert math.degrees(inp.raw) == pytest.approx(20.0)
    # the rules use the corrected bearing: raw 20 deg is off centre, corrected 5 deg is not
    inp = camera_input(1.0 + STALE_S + 1.0, last, pan_now=math.radians(25))
    assert inp.state == "centred" and math.degrees(inp.bearing) == pytest.approx(5.0)
    # and the other way: raw 3 deg (centred), but the head turned 20 deg away since
    last = CameraLine(1.0, 0, 0, 80, 90, math.radians(3), pan=0.0)
    inp = camera_input(1.0 + STALE_S + 1.0, last, pan_now=math.radians(-20))
    assert inp.state == "lost"


def test_positions_are_stored_with_the_pan_then():
    r = CameraReader(fov_deg=60)
    r.pan = math.radians(12)
    r.feed(FACE_LEFT.encode(), 1.0)
    assert r.last.pan == pytest.approx(math.radians(12))
    inp = r.input(1.1, pan_now=math.radians(30))
    assert math.degrees(inp.bearing) == pytest.approx(18 - 18)


def test_staleness_rule():
    assert camera_input(5.0, None).state == "waiting"
    # fresh: used whatever its size
    inp = camera_input(1.0 + STALE_S, _line(1.0, 25.0))
    assert inp.state == "seen" and math.degrees(inp.bearing) == pytest.approx(25.0)
    # stale and centred: a still face; kept
    inp = camera_input(1.0 + STALE_S + 2.0, _line(1.0, CENTRE_DEG - 0.5))
    assert inp.state == "centred" and math.degrees(inp.bearing) == pytest.approx(7.5)
    assert inp.age_s == pytest.approx(STALE_S + 2.0)
    # stale and off centre: lost, and for how long
    inp = camera_input(1.0 + STALE_S + 2.0, _line(1.0, -CENTRE_DEG - 0.5))
    assert inp.state == "lost" and inp.bearing is None
    assert inp.lost_s == pytest.approx(2.0)


# ---- the head loop -----------------------------------------------------------------------
class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _rig(**cfg):
    clock = Clock()
    reader = CameraReader(fov_deg=60)
    rig = R.HeadRig(reader, R.HeadConfig(**cfg), clock=clock)
    return rig, reader, clock


def _tick(rig, clock, dt=1 / 15):
    clock.t += dt
    rig.advance(dt)


def test_head_turns_left_no_faster_than_the_limit_and_stops_at_the_pan_limit():
    rig, reader, clock = _rig(head_rate_deg_s=60, pan_limit_deg=50)
    pans = [0.0]
    for _ in range(80):  # the face keeps 18 deg to the head's left (it walks away)
        reader.feed_line(FACE_LEFT, clock.t)
        _tick(rig, clock)
        pans.append(math.degrees(rig.pan))
    steps = [b - a for a, b in itertools.pairwise(pans)]
    assert pans[3] > 0  # left
    assert max(steps) <= 60 / 15 + 1e-9  # never faster than 60 deg/s
    assert max(steps) == pytest.approx(60 / 15)  # and the circuit asks for more
    assert pans[-1] == pytest.approx(50.0)  # stopped at the pan limit
    assert rig.snapshot()["head_deg"] == pytest.approx(50.0)


def test_the_head_loop_uses_the_corrected_bearing():
    rig, reader, clock = _rig(head_rate_deg_s=60)
    reader.feed_line(FACE_LEFT, clock.t)  # 18 deg left, once
    for _ in range(10):
        _tick(rig, clock)
    f = rig.snapshot()
    turned = f["head_deg"]
    assert turned > 5
    assert f["raw_bearing_deg"] == pytest.approx(18.0)
    # the frame's bearing is the one the controller got this tick (before this tick's turn)
    assert f["error_deg"] == pytest.approx(18.0 - turned, abs=60 / 15 + 0.1)
    assert turned == pytest.approx(18.0, abs=1.5)  # it settles on the face, no overshoot


def test_gain_scales_the_turn():
    a, ra, ca = _rig(gain=1.0)
    b, rb, cb = _rig(gain=0.5)
    for rig, reader, clock in ((a, ra, ca), (b, rb, cb)):
        reader.feed_line("-8.00\t0.00\tsize = 80\t90\t", clock.t)
        _tick(rig, clock)
    assert b.turn == pytest.approx(0.5 * a.turn) and a.turn > 0
    assert b.snapshot()["rig"]["gain"] == 0.5


def test_attention_when_no_position_arrives():
    said = []
    clock = Clock()
    rig = R.HeadRig(CameraReader(), R.HeadConfig(), clock=clock, on_attention=said.append)
    for _ in range(19 * 15):
        _tick(rig, clock)
    assert not said
    for _ in range(30):
        _tick(rig, clock)
    assert len(said) == 1 and "50 cm" in said[0] and "off and on" in said[0]
    assert rig.snapshot()["attention"] == said[0]


def test_no_attention_once_a_position_came():
    said = []
    clock = Clock()
    reader = CameraReader()
    rig = R.HeadRig(reader, R.HeadConfig(), clock=clock, on_attention=said.append)
    reader.feed_line(FACE_LEFT, clock.t)
    for _ in range(25 * 15):
        _tick(rig, clock)
    assert not said


def test_head_holds_when_the_target_is_lost():
    # the face is 18 deg left, but the head may turn only 5: it stays 13 deg off centre
    rig, reader, clock = _rig(pan_limit_deg=5)
    for _ in range(5):
        reader.feed_line(FACE_LEFT, clock.t)
        _tick(rig, clock)
    for _ in range(int((STALE_S + R.HOLD_AFTER_S) * 15) + 2):  # silence, face off centre
        _tick(rig, clock)
    assert rig.input.state == "lost" and rig.holding
    held = rig.pan
    for _ in range(30):
        _tick(rig, clock)
    assert rig.pan == held and rig.cmd == 0.0


def test_nothing_seen_lost_and_hold_while_turning():
    rig, reader, clock = _rig()
    for _ in range(3):
        reader.feed_line(FACE_LEFT, clock.t)
        _tick(rig, clock)
    reader.feed_line("-121.00\t-121.00\tsize = 0\t0", clock.t)
    for _ in range(int(R.HOLD_AFTER_S * 15) + 2):
        _tick(rig, clock)
    assert rig.input.state == "lost" and rig.holding
    held = rig.pan
    for _ in range(15):
        _tick(rig, clock)
    assert rig.pan == held and rig.cmd == 0.0
    f = rig.snapshot()
    assert f["camera"] == "lost" and f["hold"] and f["target"]["exists"] is False


def test_a_still_centred_face_keeps_the_head_where_it_is():
    rig, reader, clock = _rig()
    reader.feed_line("-8.00\t0.00\tsize = 80\t90\t", clock.t)  # 2 deg left, then silence
    for _ in range(60):
        _tick(rig, clock)
    assert rig.input.state == "centred" and not rig.holding
    f = rig.snapshot()
    assert f["camera"] == "centred" and f["camera_age_s"] > STALE_S


def test_settings_are_checked():
    assert R.HeadConfig().check() is None
    assert "pan-limit" in R.HeadConfig(pan_limit_deg=121).check()
    assert "gain" in R.HeadConfig(gain=3).check()
    assert "camera-fov" in R.HeadConfig(camera_fov_deg=5).check()
    assert "command-rate" in R.HeadConfig(command_hz=0).check()
    rig, _, _ = _rig()
    assert rig.set_target(10) and rig.set_motion("sweep") and rig.set_fov(60)  # refused


# ---- start-up and shut-down through the real program ----------------------------------------
def _events(out):
    return [json.loads(ln[8:]) for ln in out.splitlines() if ln.startswith("FLYDECK ")]


def _run(argv, *robots, camera="robot"):
    """The program on fake robots; these tests are about the stock camera module."""
    return cli.main(
        ["--port", "/dev/fake", "--no-open", "--http-port", "0", "--camera", camera, *argv],
        make_transport=factory(*robots),
    )


def _head_before(cmds, marker):
    heads = [c for c in cmds[: cmds.index(marker)] if c.startswith("i")]
    return heads[-1] if heads else None


def test_normal_run_start_and_shutdown_order(capsys, monkeypatch):
    monkeypatch.setenv("FLYDECK_STATUS", "1")
    bot = FakeRobot(face_on_start=FACE_LEFT)
    assert _run(["--duration", "0.8"], bot) == 0
    cmds = bot.commands()
    assert bot.non_head() == STARTUP + SHUTDOWN
    assert any(c.startswith("i0 ") and int(c.split()[1]) > 0 for c in cmds)  # turned left
    assert _head_before(cmds, "Xc~") == "i0 0"  # centred before the camera goes off
    out, _ = capsys.readouterr()
    ev = _events(out)
    assert "floor" in ev[0]["detail"]  # the first status line says where the robot must be
    assert any(e["event"] == "ready" for e in ev)
    assert not any(e["event"] == "failed" for e in ev)


def test_pan_sign_flips_the_command(monkeypatch):
    bot = FakeRobot(face_on_start=FACE_LEFT)
    assert _run(["--duration", "0.8", "--pan-sign", "-1"], bot) == 0
    assert any(c.startswith("i0 -") for c in bot.commands())


def test_leave_voice_off(monkeypatch):
    bot = FakeRobot()
    assert _run(["--duration", "0.3", "--leave-voice-off"], bot) == 0
    assert bot.non_head() == [*STARTUP, "Xc~", "gU", "d"]


def test_exception_in_the_loop_still_shuts_down(monkeypatch, capsys):
    def boom(self, seconds):
        raise RuntimeError("controller exploded")

    monkeypatch.setattr(R.HeadRig, "advance", boom)
    bot = FakeRobot()
    assert _run([], bot) == 1
    assert bot.non_head() == STARTUP + SHUTDOWN
    assert "controller exploded" in capsys.readouterr().err


def test_camera_that_fails_to_start(capsys, monkeypatch):
    monkeypatch.setenv("FLYDECK_STATUS", "1")
    bot = FakeRobot(camera="fail")
    assert _run([], bot) == 4
    cmds = bot.commands()
    assert bot.non_head() == ["?", "Xa~", "XCr~", *SHUTDOWN]
    assert _head_before(cmds, "Xc~") == "i0 0"
    out, err = capsys.readouterr()
    assert "SCL SDA +5V GND" in err
    assert _events(out)[-1]["event"] == "failed"


def test_missing_echoes_at_start_are_not_failures(capsys):
    bot = FakeRobot(camera="silent", drop_echo=("XCP~",), face_on_start=FACE_LEFT)
    assert _run(["--camera-timeout", "0.3", "--duration", "0.6"], bot) == 0
    assert bot.non_head() == STARTUP + SHUTDOWN
    out, _ = capsys.readouterr()
    assert "no reply to XCr~" in out and "no reply to XCP~" in out


def test_missing_echoes_at_shutdown_still_send_every_step(capsys):
    bot = FakeRobot(drop_echo=("Xc~", "XA~", "gU"))
    assert _run(["--duration", "0.3"], bot) == 0  # later replies ('d') show the robot answered
    assert bot.non_head() == STARTUP + SHUTDOWN
    out, _ = capsys.readouterr()
    assert "taken as off" in out and "unplug the camera" not in out


def test_no_reply_at_all_after_camera_off_gives_the_recovery(capsys):
    bot = FakeRobot(mute_from="Xc~")
    assert _run(["--duration", "0.3"], bot) == 1
    assert bot.non_head() == STARTUP + SHUTDOWN  # still sent every step
    assert "unplug the camera" in capsys.readouterr().out


def test_usb_restarts_the_board_first(capsys):
    bot = FakeRobot(serial_lines=True)
    assert _run(["--duration", "0.3"], bot) == 0
    assert bot._ser.log == [("dtr", False), ("rts", True), ("rts", False)] and bot.boots == 1
    assert bot.non_head() == STARTUP + SHUTDOWN
    assert "restarting the robot's board" in capsys.readouterr().out


def test_no_restart_when_asked_or_for_camera_off(capsys):
    bot = FakeRobot(serial_lines=True)
    assert _run(["--duration", "0.3", "--no-usb-restart"], bot) == 0
    assert bot._ser.log == []
    assert "once per power-on" in capsys.readouterr().out
    bot = FakeRobot(serial_lines=True)
    assert _run(["--camera-off"], bot) == 0
    assert bot._ser.log == []


def test_bluetooth_says_once_per_power_on(capsys, monkeypatch):
    monkeypatch.setenv("FLYDECK_STATUS", "1")
    bot = FakeRobot(name="ble")
    assert (
        cli.main(
            ["--ble", "--no-open", "--http-port", "0", "--duration", "0.3", "--camera", "robot"],
            make_transport=factory(bot),
        )
        == 0
    )
    out, _ = capsys.readouterr()
    assert "once per power-on" in out and "switch it off and on" in out
    assert bot.non_head() == STARTUP + SHUTDOWN


def test_lost_link_reconnects_once_to_shut_down(capsys):
    first, second = FakeRobot(face_on_start=FACE_LEFT, lose_after_heads=3), FakeRobot()
    assert _run(["--duration", "10"], first, second) == 1
    assert first.non_head() == STARTUP
    assert second.non_head() == ["?", *SHUTDOWN]
    assert _head_before(second.commands(), "Xc~") == "i0 0"
    out, err = capsys.readouterr()
    assert "lost" in err and "reconnecting" in out


def test_lost_link_without_a_reconnect_prints_the_recovery(capsys):
    first = FakeRobot(lose_after_heads=1)
    assert _run(["--duration", "10"], first) == 1
    out, _ = capsys.readouterr()
    assert "unplug the camera" in out and "--camera-off" in out


def test_no_robot_is_exit_4(capsys):
    assert _run([]) == 4
    assert "could not connect" in capsys.readouterr().err


def test_camera_off_maintenance():
    bot = FakeRobot()
    assert _run(["--camera-off"], bot) == 0
    assert bot.commands() == ["?", "X?~", "Xc~", "Xa~", "XA~", "gU", "d"]


def test_camera_off_also_switches_our_module_off():
    bot = FakeRobot(module="on")
    assert _run(["--camera-off"], bot, camera="auto") == 0
    assert bot.commands() == ["?", "X?~", "Xps~", "Xc~", "Xa~", "XA~", "gU", "d"]


def test_camera_off_over_bluetooth_and_refused_on_the_simulator():
    bot = FakeRobot(name="ble")
    assert cli.main(["--ble", "--camera-off"], make_transport=factory(bot)) == 0
    assert bot.non_head()[-5:] == ["Xc~", "Xa~", "XA~", "gU", "d"]
    assert cli.main(["--mock", "--camera-off"]) == 2


def test_error_statistic_uses_the_measured_bearing_not_the_corrected_one():
    """The corrected bearing tends to zero as the head turns; the statistic must not."""
    from fly_pursuit.camera import CameraReader
    from fly_pursuit.robot import HeadRig

    t = [100.0]
    reader = CameraReader()
    rig = HeadRig(reader, clock=lambda: t[0])
    reader.feed_line("-72.00\t-16.00\tsize = 69\t76\t", t[0])  # 18 degrees to the left
    for _ in range(30):  # 2 s: the head turns onto the target, the corrected bearing -> 0
        t[0] += 1 / 15
        rig.advance(1 / 15)
    st = rig.stats()
    assert st["mae_deg"] == pytest.approx(18.0, abs=0.01)
    assert st["within10"] == 0.0
