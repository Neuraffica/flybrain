"""The camera on its own USB cable (``--camera usb``), with a fake serial port and a fake robot.
No hardware is touched."""

import json
import math

import pytest
from fly_pursuit import cli, usbcam
from fly_pursuit import robot as R
from fly_pursuit.usbcam import SSCMAParser, UsbCameraReader, best_box, find_camera_port
from pursuit_fakes import FakeCamera, FakePorts, FakeRobot, factory, result


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(cli, "START_DELAY_S", 0.0)
    monkeypatch.setattr(R, "JOINT_SETTLE_S", 0.05)
    monkeypatch.setattr(R, "X_TIMEOUT_S", 0.5)


# ---- the parser ----------------------------------------------------------------------------
def test_results_split_across_reads_and_boot_text_ignored():
    p = SSCMAParser()
    data = b"\x00\xffboot rom\r\nSSCMA v2\r\n" + result([[41, 100, 80, 90, 77, 0]]) + result([])
    got = []
    for i in range(0, len(data), 5):
        got += p.feed(data[i : i + 5])
    assert [usbcam.boxes_of(r) for r in got] == [[[41, 100, 80, 90, 77, 0]], []]


def test_malformed_and_oversized_lines_are_dropped():
    p = SSCMAParser(max_line=200)
    junk = b'{"type": 1, "data": {"boxes": [[1, 2' + b"\r\n"  # cut short
    big = b'{"x": "' + b"a" * 500  # no end in sight
    got = p.feed(junk + big)
    got += p.feed(b'aaaa"}\r\n' + result([[150, 100, 80, 90, 88, 0]]))
    assert len(got) == 1 and usbcam.boxes_of(got[0])[0][0] == 150
    assert p.dropped == 2


def test_boxes_must_be_numbers():
    obj = {"data": {"boxes": [[1, 2, 3, 4, "x", 0], [150, 100, 80, 90, 60, 0], "nope"]}}
    assert usbcam.boxes_of(obj) == [[150, 100, 80, 90, 60, 0]]
    assert usbcam.boxes_of({"name": "INIT@STAT?", "data": {}}) is None


def test_highest_score_wins_and_bearing_sign():
    assert best_box([[41, 1, 1, 1, 70, 0], [150, 1, 1, 1, 90, 0]])[0] == 150
    r = UsbCameraReader(fov_deg=60)
    r.feed(result([[41, 100, 80, 90, 77, 0]]), 1.0)  # measured: a face on the dog's left
    assert math.degrees(r.last.bearing) == pytest.approx((120 - 41) / 240 * 60)
    assert r.last.score == 77
    r.feed(result([[41, 100, 80, 90, 50, 0], [150, 100, 80, 90, 85, 0]]), 1.1)
    assert math.degrees(r.last.bearing) == pytest.approx(-7.5) and r.last.score == 85


def test_empty_boxes_is_lost_at_once():
    r = UsbCameraReader()
    r.feed(result([[118, 100, 80, 90, 77, 0]]), 1.0)  # nearly centred
    r.feed(result([]), 1.07)
    inp = r.input(1.08)
    assert inp.state == "lost" and inp.bearing is None


def test_no_centre_band_guessing_on_a_steady_source():
    r = UsbCameraReader()
    r.feed(result([[118, 100, 80, 90, 77, 0]]), 1.0)  # centred, then the camera goes quiet
    assert r.input(1.5).state == "seen"
    assert r.input(1.0 + r.stale_s + 0.5).state == "lost"


# ---- finding the port ----------------------------------------------------------------------
def test_probing_finds_the_camera_and_skips_the_robot(monkeypatch):
    def not_a_camera(port):
        raise AssertionError("the robot's port was opened")

    ports = FakePorts(
        **{
            "/dev/cu.wchusbserial111": not_a_camera,  # the robot, under another name
            "/dev/cu.usbmodem222": lambda p: FakeCamera(p, name="Something else"),
            "/dev/cu.usbmodem333": FakeCamera,
        }
    ).install(monkeypatch)
    assert find_camera_port(skip=["/dev/cu.usbmodem111"]) == "/dev/cu.usbmodem333"
    assert ports.opened == ["/dev/cu.usbmodem222", "/dev/cu.usbmodem333"]
    assert all(c.closed for c in ports.cameras)
    assert ports.cameras[-1].commands() == ["AT+NAME?"]


def test_probing_finds_nothing(monkeypatch):
    FakePorts().install(monkeypatch)
    assert find_camera_port() is None


# ---- end to end, with the fake robot ------------------------------------------------------------
FACE_RIGHT = [150, 100, 80, 90, 80, 0]  # x = 150: right of centre (measured "about ahead")
OTHER = [41, 100, 80, 90, 55, 0]


def _events(out):
    return [json.loads(ln[8:]) for ln in out.splitlines() if ln.startswith("FLYDECK ")]


def test_usb_camera_end_to_end(monkeypatch, capsys):
    monkeypatch.setenv("FLYDECK_STATUS", "1")
    frames = [[OTHER, FACE_RIGHT]] * 12 + [[]]
    ports = FakePorts(cam=lambda p: FakeCamera(p, frames=frames)).install(monkeypatch)
    bot = FakeRobot(name="ble", serial_lines=True)
    argv = ["--ble", "--camera", "usb", "--no-open", "--http-port", "0", "--duration", "1.2"]
    assert cli.main(argv, make_transport=factory(bot)) == 0
    cmds = bot.non_head()
    assert not any(c.startswith("XC") or c == "Xc~" for c in cmds)  # no firmware camera
    assert cmds == ["?", "Xa~", "ksit", "XA~", "d"]  # no gU either
    assert bot._ser.log == []  # no restart for this source
    heads = [int(c.split()[1]) for c in bot.commands() if c.startswith("i0 ")]
    assert min(heads) < 0  # the best box (score 80) was on the right
    assert heads[-1] == 0  # centred at the end
    cam = ports.cameras[-1]
    assert cam.commands()[0] == "AT+INVOKE=-1,0,1" and cam.commands()[-1] == "AT+BREAK"
    assert cam.closed
    out, _ = capsys.readouterr()
    assert any(e["event"] == "ready" for e in _events(out))


def test_usb_camera_frame_has_the_score(monkeypatch):
    from fly_pursuit.app import PursuitApp

    reader = UsbCameraReader()
    head = PursuitApp(R.HeadRig(reader), frame_hz=15)
    import time

    reader.feed(result([OTHER, FACE_RIGHT]), time.monotonic())
    f = head.tick(1 / 15)
    assert f["camera_source"] == "usb" and f["camera_score"] == 80 and f["camera"] == "seen"


def test_usb_camera_not_found_is_exit_4(monkeypatch, capsys):
    FakePorts().install(monkeypatch)
    bot = FakeRobot(name="ble")
    argv = ["--ble", "--camera", "usb", "--no-open", "--http-port", "0"]
    assert cli.main(argv, make_transport=factory(bot)) == 4
    assert bot.writes == []  # the robot was not even connected
    assert "own USB cable" in capsys.readouterr().err


def test_usb_camera_with_robot_on_usb_needs_a_port():
    def no_link():
        raise AssertionError("a link was opened")

    assert cli.main(["--camera", "usb"], make_transport=no_link) == 2


def test_usb_camera_skips_the_robots_port(monkeypatch):
    ports = FakePorts(
        **{"/dev/cu.usbmodem111": FakeCamera, "/dev/cu.usbmodem333": FakeCamera}
    ).install(monkeypatch)
    bot = FakeRobot()
    argv = [
        "--port", "/dev/cu.wchusbserial111", "--camera", "usb",
        "--no-open", "--http-port", "0", "--duration", "0.3",
    ]  # fmt: skip
    assert cli.main(argv, make_transport=factory(bot)) == 0
    assert "/dev/cu.usbmodem111" not in ports.opened
