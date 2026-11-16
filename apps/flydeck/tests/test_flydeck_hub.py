import json
import os
import time
import urllib.error
import urllib.request

import pytest
from flydeck import registry
from flydeck.hub import Hub, HubServer
from flydeck.settings import Settings
from flydeck_fakes import fake_spec, write_fake


def post(url, action, body=None, headers=None):
    h = {"Content-Type": "application/json", **(headers or {})}
    req = urllib.request.Request(
        url + "api/" + action, data=json.dumps(body or {}).encode(), headers=h, method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def get_state(url):
    with urllib.request.urlopen(url + "api/state", timeout=10) as r:
        return json.loads(r.read())


def wait_for(cond, timeout=30.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.1)
    return False


@pytest.fixture
def fake_hub(tmp_path, monkeypatch):
    write_fake(tmp_path)
    monkeypatch.setenv("PYTHONPATH", str(tmp_path) + os.pathsep + os.environ.get("PYTHONPATH", ""))
    entry = registry.DemoEntry(id="fake-demo", spec=fake_spec(), source="test")
    hub = Hub(
        root=tmp_path,
        settings=Settings(tmp_path / "flydeck.json"),
        stop_timeout_s=3.0,
        entries=[entry],
    )
    server = HubServer(hub, port=0).start()
    yield hub, server.url
    hub.shutdown(timeout=10)
    server.stop()


def test_page_and_state(fake_hub):
    _hub, url = fake_hub
    html = urllib.request.urlopen(url).read().decode()
    assert "<title>Flydeck</title>" in html
    # the shared look (tokens, fonts, favicon) comes from the hub itself: nothing from the network
    assert 'href="/assets/living-signal.css"' in html and 'href="/assets/flydeck.svg"' in html
    assert "https://" not in html
    for path in ("living-signal.css", "flydeck.svg", "fonts/ibm-plex-mono-latin-600-normal.woff2"):
        assert urllib.request.urlopen(url + "assets/" + path).read()
    st = get_state(url)
    assert st["app"]["name"] == "Flydeck"
    assert [d["id"] for d in st["demos"]] == ["fake-demo"]
    opts = st["demos"][0]["options"]
    assert opts[0] == {
        "name": "behave",
        "type": "choice",
        "default": "ok",
        "label": "Behave",
        "help": "",
        "choices": ["ok", "crash", "hang", "noready", "fail4", "attention"],
        "min": None,
        "max": None,
        "mock_only": False,
    }


def test_start_refuses_without_a_robot_unless_simulator(fake_hub):
    _hub, url = fake_hub
    code, r = post(url, "start", {"demo": "fake-demo"})
    assert code == 200 and not r["ok"] and "No robot chosen" in r["error"]
    code, r = post(url, "start", {"demo": "fake-demo", "simulator": True})
    assert r["ok"], r
    code, r = post(url, "start", {"demo": "fake-demo", "simulator": True})
    assert not r["ok"] and "one demo at a time" in r["error"]
    assert wait_for(lambda: get_state(url)["run"]["state"] == "running")
    post(url, "stop")
    assert wait_for(lambda: get_state(url)["run"]["state"] == "stopped")


def test_safety_gate_and_robot_held_by_demo(fake_hub):
    hub, url = fake_hub
    assert post(url, "select", {"transport": "ble", "target": "BittleC2_BLE"})[1]["ok"]
    _, r = post(url, "preview", {"demo": "fake-demo", "options": {"walk": True}})
    assert r["needs_space"] and "--ble BittleC2_BLE" in r["command"]
    _, r = post(url, "start", {"demo": "fake-demo", "options": {"walk": True}})
    assert not r["ok"] and "has space" in r["error"]
    _, r = post(url, "start", {"demo": "fake-demo", "options": {"walk": True}, "space": True})
    assert r["ok"]
    st = get_state(url)
    assert st["robot"]["held_by"] == "Fake demo"
    _, r = post(url, "check")
    assert not r["ok"] and "connected to Fake demo" in r["error"]
    _, r = post(url, "scan")
    assert not r["ok"]
    # remembered for next time
    assert hub.settings.get("robot")["target"] == "BittleC2_BLE"
    assert hub.settings.demo_options("fake-demo")["walk"] is True


def test_cross_site_and_non_json_posts_are_refused(fake_hub):
    _, url = fake_hub
    code, _r = post(
        url,
        "start",
        {"demo": "fake-demo", "simulator": True},
        headers={"Origin": "http://evil.example"},
    )
    assert code == 403
    req = urllib.request.Request(
        url + "api/stop", data=b"x", method="POST", headers={"Content-Type": "text/plain"}
    )
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 415
    req = urllib.request.Request(url + "api/state", headers={"Host": "evil.example"})
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(req)
    assert e.value.code == 403


def test_quit_stops_the_running_demo(fake_hub):
    hub, url = fake_hub
    post(url, "start", {"demo": "fake-demo", "simulator": True})
    assert wait_for(lambda: hub.runner.current and hub.runner.current.ready)
    post(url, "quit")
    assert hub.quit_event.wait(15)
    assert hub.runner.current.state == "stopped"


def test_simulator_check_reads_identity_and_battery(tmp_path):
    hub = Hub(root=registry.find_root(), settings=Settings(tmp_path / "s.json"), entries=[])
    hub.check(background=False, mock=True)
    chk = hub.robot["check"]
    assert chk["ok"], chk
    assert chk["model"] and chk["version"]
    assert chk["voltage"] == pytest.approx(8.1)


def test_end_to_end_fly_escape_in_the_simulator(tmp_path):
    """hub -> start fly-escape on the simulator -> ready -> dashboard reachable -> stop."""
    root = registry.find_root()
    hub = Hub(root=root, settings=Settings(tmp_path / "s.json"))
    hub.runner.cwd = str(tmp_path)  # its runs/ log goes here, not into the checkout
    server = HubServer(hub, port=0).start()
    url = server.url
    try:
        _, r = post(
            url, "start", {"demo": "fly-escape", "simulator": True, "options": {"mode": "escape"}}
        )
        assert r["ok"], r
        assert "--mock" in r["run"]["command"] and "--no-open" in r["run"]["command"]
        assert wait_for(lambda: get_state(url)["run"]["ready"], 90), get_state(url)["run"]["log"]
        run = get_state(url)["run"]
        dash = urllib.request.urlopen(run["url"], timeout=10).read().decode()
        assert "<html" in dash.lower()
        assert wait_for(lambda: get_state(url)["run"]["state"] == "running", 30)
        post(url, "stop")
        assert wait_for(lambda: not get_state(url)["run"]["active"], 30)
        run = get_state(url)["run"]
        assert run["state"] == "stopped" and run["exit_code"] == 0, run
        with pytest.raises(OSError):
            urllib.request.urlopen(run["url"], timeout=2)
    finally:
        hub.shutdown(timeout=20)
        server.stop()
