import json
import urllib.error
import urllib.request

import pytest
from flybrain_demo_api import DemoSpec, Option, SpecError, status
from flybrain_demo_api.web import DashboardServer, FrameHub


def spec(**kw):
    base = {"id": "my-demo", "title": "My demo", "summary": "does a thing", "module": "my_demo.cli"}
    base.update(kw)
    return DemoSpec(**base)


def test_option_coerce_types_and_limits():
    assert Option("n", "int", 3, minimum=1, maximum=5).coerce("4") == 4
    assert Option("x", "float", 1.0).coerce("2.5") == 2.5
    assert Option("b", "bool", False).coerce("true") is True
    assert Option("b", "bool", False).coerce(False) is False
    assert Option("m", "choice", "a", choices=("a", "b")).coerce("") == "a"
    with pytest.raises(SpecError, match="at most"):
        Option("n", "int", 3, maximum=5).coerce(9)
    with pytest.raises(SpecError, match="choose one of"):
        Option("m", "choice", "a", choices=("a", "b")).coerce("c")
    with pytest.raises(SpecError, match="not a int"):
        Option("n", "int", 3).coerce("three")


@pytest.mark.parametrize(
    ("kw", "msg"),
    [
        ({"id": "Bad Id"}, "lowercase"),
        ({"title": ""}, "required"),
        ({"module": "not a module"}, "module"),
        ({"transports": ("wifi",)}, "transports"),
        ({"transports": ("usb",)}, "supports_mock"),
        ({"options": (Option("a", "int", 1), Option("a", "int", 2))}, "repeat"),
        ({"options": (Option("m", "choice", "z", choices=("a",)),)}, "not a choice"),
        ({"ready": "smoke"}, "ready"),
    ],
)
def test_validate_rejects(kw, msg):
    with pytest.raises(SpecError, match=msg):
        spec(**kw).validate()


def test_default_argv_transports_options_and_space():
    s = spec(
        moves_robot=True,
        options=(
            Option("mode", "choice", "look", choices=("look", "walk")),
            Option("fast", "bool", False),
            Option("seed", "int", 0, mock_only=True),
        ),
        legs_move=lambda o: o["mode"] == "walk",
    )
    s.validate()
    assert s.argv("mock", {}, 9000) == [
        "--mock",
        "--http-port",
        "9000",
        "--no-open",
        "--mode",
        "look",
        "--seed",
        "0",
    ]
    assert s.argv("usb", {"fast": True}, 9001, "/dev/x") == [
        "--port",
        "/dev/x",
        "--http-port",
        "9001",
        "--no-open",
        "--mode",
        "look",
        "--fast",
    ]
    # legs move on a real robot: the space flag only after confirmation
    assert "--i-have-space" not in s.argv("ble", {"mode": "walk"}, 1, "BittleC2_BLE")
    assert s.argv("ble", {"mode": "walk"}, 1, "BittleC2_BLE", True)[:2] == ["--ble", "BittleC2_BLE"]
    assert "--i-have-space" in s.argv("ble", {"mode": "walk"}, 1, "B", True)
    assert "--i-have-space" in s.argv("mock", {"mode": "walk"}, 1)  # no real legs
    assert s.argv("ble", {}, 1)[:2] == ["--ble", "--http-port"]  # no name: first Bittle
    assert s.needs_space({"mode": "walk"}, "ble") and not s.needs_space({"mode": "walk"}, "mock")
    assert not s.needs_space({"mode": "look"}, "usb")
    with pytest.raises(SpecError, match="unknown option"):
        s.argv("mock", {"nope": 1}, 1)
    with pytest.raises(SpecError, match="does not support"):
        spec(transports=("mock",)).argv("ble", {}, 1)


def test_status_lines_only_when_enabled(monkeypatch, capsys):
    monkeypatch.delenv(status.ENV, raising=False)
    status.ready("http://x/")
    assert capsys.readouterr().out == ""
    monkeypatch.setenv(status.ENV, "1")
    status.ready("http://x/")
    status.state("calibrating", "hands away")
    status.failed("no robot")
    lines = capsys.readouterr().out.splitlines()
    evs = [status.parse(ln) for ln in lines]
    assert evs == [
        {"event": "ready", "url": "http://x/"},
        {"event": "state", "state": "calibrating", "detail": "hands away"},
        {"event": "failed", "reason": "no robot"},
    ]
    assert status.parse("hello") is None
    assert status.parse("FLYDECK {broken") is None


def test_web_server_page_state_and_command():
    hub = FrameHub()
    srv = DashboardServer(hub, lambda cmd: {"ok": True, "echo": cmd}, page=b"<p>hi</p>").start()
    try:
        assert urllib.request.urlopen(srv.url).read() == b"<p>hi</p>"
        hub.publish({"yaw": 1.5})
        state = json.loads(urllib.request.urlopen(srv.url + "api/state").read())
        assert state == {"yaw": 1.5}
        req = urllib.request.Request(srv.url + "api/command", data=b'{"action":"x"}', method="POST")
        assert json.loads(urllib.request.urlopen(req).read())["echo"] == {"action": "x"}
        assert urllib.request.urlopen(srv.url + "api/benchmark").status == 204
    finally:
        srv.stop()


def test_shared_look_is_whole_and_served_offline():
    from flybrain_demo_api import assets

    assert assets.missing() == []
    css, ctype = assets.asset("/assets/living-signal.css")
    assert ctype.startswith("text/css") and b"--primary: #0e6f66" in css
    assert b"http://" not in css and b"https://" not in css  # nothing fetched from the network
    for bad in ("/assets/../web.py", "/assets/fonts/../../web.py", "/assets/assets.py", "/x.css"):
        assert assets.asset(bad) is None
    inline = assets.inline_css()
    assert "url(fonts/" not in inline and inline.count("data:font/woff2;base64,") == 3
    assert assets.data_uri("flydeck.svg").startswith("data:image/svg+xml;base64,")

    srv = DashboardServer(FrameHub(), lambda cmd: {"ok": True}).start()
    try:
        for path, ctype in (
            ("assets/living-signal.css", "text/css"),
            ("assets/fonts/fraunces-latin-opsz-normal.woff2", "font/woff2"),
            ("assets/neuraffica.svg", "image/svg+xml"),
        ):
            r = urllib.request.urlopen(srv.url + path)
            assert r.headers["Content-Type"].startswith(ctype) and r.read()
        page = urllib.request.urlopen(srv.url).read()  # the default page uses the shared look
        assert b"/assets/living-signal.css" in page
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(srv.url + "assets/nope.css")
    finally:
        srv.stop()
