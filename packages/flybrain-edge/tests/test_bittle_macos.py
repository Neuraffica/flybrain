"""The macOS Bluetooth helper bundle. Nothing here launches the bundle through `open` or
touches Bluetooth; the helper's shell script is run directly with bash to check how it passes
the command, working directory and exit code."""

import io
import os
import plistlib
import stat
import subprocess
import sys

import pytest
from flybrain_edge.bittle import macos
from flybrain_edge.bittle.probe import main as bittle_main


def test_bundle_layout_and_plist(tmp_path):
    app = macos.build_bundle(tmp_path)
    assert app == tmp_path / "FlybrainBLE.app"
    info = plistlib.loads((app / "Contents" / "Info.plist").read_bytes())
    assert info["CFBundleIdentifier"] == "com.neuraffica.flybrain.blerunner"
    assert info["CFBundleExecutable"] == "flybrain-ble-run"
    assert info["CFBundlePackageType"] == "APPL"
    assert info["LSUIElement"] is True
    usage = info["NSBluetoothAlwaysUsageDescription"]
    assert "Bluetooth" in usage and "Bittle" in usage
    exe = app / "Contents" / "MacOS" / "flybrain-ble-run"
    mode = exe.stat().st_mode
    assert mode & stat.S_IXUSR and mode & stat.S_IXGRP and mode & stat.S_IXOTH
    assert exe.read_text().startswith("#!/bin/bash")
    # rebuilding is idempotent
    assert macos.build_bundle(tmp_path) == app


@pytest.mark.skipif(sys.platform == "win32", reason="bash script")
def test_helper_script_runs_command_in_workdir_and_reports_exit(tmp_path):
    app = macos.build_bundle(tmp_path / "apps")
    exe = app / "Contents" / "MacOS" / "flybrain-ble-run"
    work = tmp_path / "work dir"
    work.mkdir()
    state = tmp_path / "state"
    code = (
        "import os,sys; print(os.getcwd()); print(os.environ['FLYBRAIN_BLE_BUNDLE']);"
        "print(os.environ['PYTHONUNBUFFERED']); print(sys.argv[1]); sys.exit(3)"
    )
    r = subprocess.run(
        [
            "bash",
            str(exe),
            str(work),
            str(state),
            os.environ["PATH"],
            sys.executable,
            "-c",
            code,
            "an arg with spaces",
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert r.returncode == 3, r.stderr
    out = r.stdout.splitlines()
    assert os.path.realpath(out[0]) == os.path.realpath(work)
    assert out[1:] == ["1", "1", "an arg with spaces"]
    assert (state / "exit").read_text().strip() == "3"
    assert int((state / "pid").read_text().strip()) > 0


def test_helper_script_rejects_bad_workdir(tmp_path):
    app = macos.build_bundle(tmp_path)
    exe = app / "Contents" / "MacOS" / "flybrain-ble-run"
    r = subprocess.run(
        ["bash", str(exe), str(tmp_path / "missing"), str(tmp_path / "s"), "/usr/bin:/bin", "true"],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert r.returncode == 66 and "cannot cd" in r.stderr


def test_open_command_passes_everything(tmp_path):
    app = tmp_path / "FlybrainBLE.app"
    cmd = macos.open_command(app, ["/x/uv", "run", "fly-compass", "--ble"], "/repo", "/st", "/p")
    assert cmd == [
        "open",
        "-W",
        "-n",
        "--stdout",
        "/st/stdout.log",
        "--stderr",
        "/st/stderr.log",
        str(app),
        "--args",
        "/repo",
        "/st",
        "/p",
        "/x/uv",
        "run",
        "fly-compass",
        "--ble",
    ]


def test_resolve_command_makes_program_absolute(tmp_path):
    prog = tmp_path / "toolx"
    prog.write_text("#!/bin/sh\n")
    prog.chmod(0o755)
    assert macos.resolve_command(["toolx", "-v"], str(tmp_path)) == [str(prog), "-v"]
    assert macos.resolve_command(["/abs/bin", "a"]) == ["/abs/bin", "a"]
    with pytest.raises(FileNotFoundError):
        macos.resolve_command(["no-such-tool-xyz"], str(tmp_path))


def test_hint_only_on_macos_outside_the_helper(monkeypatch):
    monkeypatch.delenv(macos.ENV_MARK, raising=False)
    monkeypatch.delenv(macos.ENV_NO_HINT, raising=False)
    monkeypatch.setattr(sys, "argv", ["/venv/bin/fly-compass", "--ble", "--mode", "observe"])
    monkeypatch.setattr(sys, "platform", "darwin")
    buf = io.StringIO()
    assert macos.warn_if_ble_will_be_killed(buf)
    text = buf.getvalue()
    assert "exit code 134" in text
    assert "uv run flybrain-bittle ble-run -- uv run fly-compass --ble --mode observe" in text
    monkeypatch.setenv(macos.ENV_MARK, "1")
    assert not macos.warn_if_ble_will_be_killed(io.StringIO())
    monkeypatch.delenv(macos.ENV_MARK)
    monkeypatch.setenv(macos.ENV_NO_HINT, "1")
    assert not macos.warn_if_ble_will_be_killed(io.StringIO())
    monkeypatch.delenv(macos.ENV_NO_HINT)
    monkeypatch.setattr(sys, "platform", "linux")
    assert not macos.warn_if_ble_will_be_killed(io.StringIO())


def test_macos_ble_app_subcommand_prints_open_line(tmp_path, capsys):
    assert bittle_main(["macos-ble-app", "--dir", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert str(tmp_path / "FlybrainBLE.app") in out
    assert "open -W -n --stdout" in out and "--args" in out
    assert "ble-run -- uv run flybrain-bittle probe --ble" in out
    assert (tmp_path / "FlybrainBLE.app" / "Contents" / "Info.plist").exists()


def test_ble_run_requires_a_command(tmp_path):
    with pytest.raises(SystemExit):
        bittle_main(["ble-run", "--dir", str(tmp_path)])
