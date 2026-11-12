import os
import plistlib
import shutil
import struct
import subprocess
import sys
import time
from pathlib import Path

import pytest
from flydeck import bundle, icon, registry


def run(*args, **kw):
    """subprocess.run that never raises on a non-zero exit: the tests check it."""
    return subprocess.run(*args, check=False, **kw)


MAC = sys.platform == "darwin" and shutil.which("osacompile") is not None


def test_icon_is_a_valid_icns_with_all_sizes():
    data = icon.icns_bytes()
    assert data[:4] == b"icns" and struct.unpack(">I", data[4:8])[0] == len(data)
    pos, types = 8, []
    while pos < len(data):
        typ, n = data[pos : pos + 4].decode(), struct.unpack(">I", data[pos + 4 : pos + 8])[0]
        assert data[pos + 8 : pos + 16] == b"\x89PNG\r\n\x1a\n"
        types.append(typ)
        pos += n
    assert pos == len(data)
    assert {"icp4", "ic07", "ic10"} <= set(types)


def test_launcher_records_checkout_and_python(tmp_path):
    text = bundle.launcher_text(
        Path("/Users/me/my flybrain"), "/Users/me/my flybrain/.venv/bin/python"
    )
    assert "CHECKOUT='/Users/me/my flybrain'" in text
    assert "PYTHON='/Users/me/my flybrain/.venv/bin/python'" in text
    assert "FLYBRAIN_BLE_BUNDLE=1" in text and "FLYDECK_APP=1" in text
    assert "has moved" in text  # the dialog text when the checkout is gone
    sh = tmp_path / "l.sh"
    sh.write_text(text)
    assert run(["bash", "-n", str(sh)]).returncode == 0


@pytest.mark.skipif(not MAC, reason="needs macOS osacompile")
def test_build_app(tmp_path):
    root = registry.find_root()
    app = bundle.build_app(root, python=sys.executable, directory=tmp_path)
    assert app == tmp_path / "Flydeck.app"
    with open(app / "Contents" / "Info.plist", "rb") as f:
        info = plistlib.load(f)
    assert info["CFBundleName"] == "Flydeck" and info["CFBundleDisplayName"] == "Flydeck"
    assert info["CFBundleIdentifier"] == bundle.BUNDLE_ID
    assert "Bluetooth" in info["NSBluetoothAlwaysUsageDescription"]
    assert not info.get("LSUIElement")  # a real Dock app
    assert info["OSAAppletStayOpen"] is True
    assert [k for k in info if k.endswith("UsageDescription")] == [
        "NSBluetoothAlwaysUsageDescription"
    ]
    res = app / "Contents" / "Resources"
    assert (res / "flydeck.icns").read_bytes()[:4] == b"icns"
    exe = app / "Contents" / "MacOS" / info["CFBundleExecutable"]
    assert os.access(exe, os.X_OK)
    launcher = res / bundle.LAUNCHER
    assert os.access(launcher, os.X_OK)
    assert str(root) in launcher.read_text() and sys.executable in launcher.read_text()
    v = run(["codesign", "--verify", "--deep", "--strict", str(app)], capture_output=True)
    assert v.returncode == 0, v.stderr


def test_launcher_starts_and_stops_the_hub(tmp_path):
    """The app's launcher script: start (hub up, URL opened), alive, stop (hub gone)."""
    if shutil.which("curl") is None:
        pytest.skip("needs curl")
    root = registry.find_root()
    sh = tmp_path / "flydeck-launch.sh"
    sh.write_text(bundle.launcher_text(root, sys.executable))
    sh.chmod(0o755)
    home = tmp_path / "home"
    opened = tmp_path / "opened.txt"
    fake_open = tmp_path / "fake-open"
    fake_open.write_text(f'#!/bin/bash\necho "$1" > {opened}\n')
    fake_open.chmod(0o755)
    env = dict(os.environ, HOME=str(home), FLYDECK_OPEN_CMD=str(fake_open))
    env.pop("FLYDECK_HOME", None)
    try:
        r = run(
            [str(sh), "start", str(os.getpid())],
            env=env,
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert r.returncode == 0, r.stderr
        url = opened.read_text().strip()
        assert url.startswith("http://127.0.0.1:")
        assert run([str(sh), "alive"], env=env).returncode == 0
        log = (home / "Library" / "Logs" / "Flydeck.log").read_text()
        assert "Flydeck at" in log
    finally:
        r = run([str(sh), "stop"], env=env, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0
    deadline = time.monotonic() + 10
    while run([str(sh), "alive"], env=env).returncode == 0:
        assert time.monotonic() < deadline, "hub still running after stop"
        time.sleep(0.2)


def test_launcher_explains_a_moved_checkout(tmp_path):
    sh = tmp_path / "flydeck-launch.sh"
    sh.write_text(bundle.launcher_text(tmp_path / "gone", str(tmp_path / "gone" / "python")))
    sh.chmod(0o755)
    r = run(
        [str(sh), "start", "1"],
        capture_output=True,
        text=True,
        env=dict(os.environ, HOME=str(tmp_path)),
    )
    assert r.returncode == 1
    assert "has moved" in r.stderr and "uv run flydeck install-app" in r.stderr
