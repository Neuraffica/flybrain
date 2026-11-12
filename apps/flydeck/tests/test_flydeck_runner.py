import os
import time

import pytest
from flydeck.runner import BLE_KILLED, ROBOT_MAY_STAND, Runner, StartRefused, pick_port
from flydeck_fakes import fake_spec, write_fake


@pytest.fixture
def runner(tmp_path):
    write_fake(tmp_path)
    env = dict(os.environ, PYTHONPATH=str(tmp_path) + os.pathsep + os.environ.get("PYTHONPATH", ""))
    r = Runner(cwd=tmp_path, env=env, stop_timeout_s=2.0)
    yield r
    r.stop(wait=True, timeout=10)


def wait_for(cond, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_start_ready_stop_clean(runner):
    run = runner.start(fake_spec(), "mock", {"behave": "ok"})
    assert wait_for(lambda: run.ready)
    assert run.state == "running"
    assert any("--mock" in ln for ln in run.log)
    runner.stop(wait=True)
    assert run.state == "stopped" and run.exit_code == 0
    assert "resting" in run.reason.lower()
    assert any("fake: resting the robot" in ln for ln in run.log)


def test_one_demo_at_a_time(runner):
    runner.start(fake_spec(), "mock", {"behave": "ok"})
    with pytest.raises(StartRefused, match="one demo at a time"):
        runner.start(fake_spec(), "mock", {"behave": "ok"})


def test_crash_reports_the_demos_reason(runner):
    run = runner.start(fake_spec(), "mock", {"behave": "crash"})
    assert runner.wait(10)
    assert run.state == "failed"
    assert run.reason == "Motor on fire"


def test_exit_code_without_reason_uses_contract_and_last_line(runner):
    run = runner.start(fake_spec(), "usb", {"behave": "fail4"}, robot="/dev/fake")
    assert runner.wait(10)
    assert run.state == "failed" and run.exit_code == 4
    assert run.reason.startswith("Could not connect to the robot")
    assert "cannot connect: nope" in run.reason


def test_hang_escalates_and_warns_robot_may_stand(runner):
    run = runner.start(fake_spec(), "mock", {"behave": "hang"})
    assert wait_for(lambda: run.ready)
    t0 = time.monotonic()
    runner.stop(wait=True, timeout=10)
    assert time.monotonic() - t0 < 8
    assert run.state == "failed" and run.killed
    assert ROBOT_MAY_STAND in run.reason


def test_no_ready_within_timeout_is_stopped(runner):
    run = runner.start(fake_spec(ready_timeout_s=1.0), "mock", {"behave": "noready"})
    assert runner.wait(10)
    assert run.state == "failed"
    assert "did not report ready" in run.reason


def test_attention_state_becomes_a_warning(runner):
    run = runner.start(fake_spec(), "mock", {"behave": "attention"})
    assert wait_for(lambda: run.state == "attention")
    assert run.warnings == ["too bright: move away from the window"]


def test_safety_gate_needs_confirmation_on_a_real_robot(runner):
    with pytest.raises(StartRefused, match="has space"):
        runner.start(fake_spec(), "ble", {"walk": True}, robot="BittleC2_BLE")
    assert not runner.busy
    run = runner.start(fake_spec(), "ble", {"walk": True}, robot="B", space_confirmed=True)
    assert run.argv[-1] == "--i-have-space"


def test_simulator_needs_no_confirmation(runner):
    run = runner.start(fake_spec(), "mock", {"walk": True})
    assert "--i-have-space" in run.argv


def test_bad_options_and_transport_refused(runner):
    with pytest.raises(StartRefused, match="choose one of"):
        runner.start(fake_spec(), "mock", {"behave": "dance"})
    with pytest.raises(StartRefused, match="cannot run over usb"):
        runner.start(fake_spec(transports=("mock",)), "usb", {})


def test_ble_kill_is_explained(runner, monkeypatch):
    monkeypatch.setattr("flydeck.runner.sys.platform", "darwin")
    run = runner.start(fake_spec(), "ble", {"behave": "ok"}, robot="B")
    assert wait_for(lambda: run.ready)
    os.kill(run.pid, 6)  # what macOS does to a process without Bluetooth permission
    assert runner.wait(10)
    assert run.reason == BLE_KILLED


def test_pick_port_falls_back_when_busy():
    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        s.listen()
        busy = s.getsockname()[1]
        assert pick_port(busy) != busy
    assert pick_port(0) > 0
