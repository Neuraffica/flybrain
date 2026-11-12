"""The visual simulator: seeded random approaches, ground-truth scoring, frame fields."""

import sys
import threading
import time
from collections import Counter

import escape_fakes
import pytest
from escape_fakes import fake_models
from fly_compass.server import FrameHub
from fly_escape import cli
from fly_escape.app import EscapeApp
from fly_escape.approaches import (
    KINDS,
    SAFE_KINDS,
    THREAT_KINDS,
    Approach,
    RandomApproaches,
    make_approach,
)
from fly_escape.escape import EscapeConfig
from fly_escape.groundtruth import GroundTruth, format_table
from fly_escape.loop import EscapeLoop
from fly_escape.sim import Simulator
from flybrain_edge.bittle import Bittle, MockBittle, MockTransport


def test_random_stream_is_seeded_and_mixed():
    a = RandomApproaches(7)
    b = RandomApproaches(7)
    seq_a = [(x.kind, x.side, round(x.move_s, 4)) for x in (a.draw() for _ in range(200))]
    seq_b = [(x.kind, x.side, round(x.move_s, 4)) for x in (b.draw() for _ in range(200))]
    assert seq_a == seq_b
    c = RandomApproaches(8)
    assert [(x.kind, x.side) for x in (c.draw() for _ in range(20))] != [s[:2] for s in seq_a[:20]]
    kinds = Counter(k for k, _, _ in seq_a)
    assert set(kinds) == set(KINDS)  # every kind shows up
    threats = sum(kinds[k] for k in THREAT_KINDS)
    assert 70 <= threats <= 130  # about half
    assert {s for _, s, _ in seq_a} == {-1, 0, 1}
    a.reset()
    assert [(x.kind, x.side) for x in (a.draw() for _ in range(5))] == [s[:2] for s in seq_a[:5]]


@pytest.mark.parametrize("kind", KINDS)
@pytest.mark.parametrize("side", [-1, 0, 1])
def test_approach_paths(kind, side):
    a = make_approach(kind, side)
    assert a.threat == (kind in THREAT_KINDS)
    assert a(-1.0)[0] == 80.0 and a(a.duration + 0.1)[0] == 80.0  # away before and after
    closest = min(a(t / 100)[0] for t in range(int(a.duration * 100)))
    nearer = closest - a.nearer_cm  # distance from the channel nearer the hand
    if a.threat:
        assert closest <= 3.5
        assert a.detect_t() is not None
    elif kind in ("hover", "stop-short", "drift"):
        assert 12.0 <= nearer <= 20.0  # inside the sensor's graded band, never close
    else:  # retreat: never comes into range
        assert nearer >= 20.0 and a.detect_t() is None
    assert a(0.0)[1] == pytest.approx(0.8 * side)


def test_speed_factor_scales_the_movement():
    slow = make_approach("fast", 0, speed=1.0)
    quick = make_approach("fast", 0, speed=2.0)
    assert quick.move_s < slow.move_s


def _approach(kind, side=1):
    a = make_approach(kind, side)
    a.id = 1
    return a


def test_ground_truth_scoring():
    gt = GroundTruth()
    t1 = gt.start(_approach("fast", 1), 10.0)
    gt.on_trigger("connectome", t1.t_detect + 0.05, 1, 12.0)
    gt.on_trigger("connectome", t1.t_detect + 0.4, -1, 5.0)  # only the first trigger counts
    gt.on_trigger("prox", t1.t_detect + 0.12, -1, 8.0)  # wrong side
    t2 = gt.start(_approach("hover", 0), 20.0)
    gt.on_trigger("looming", t2.t0 + 1.0, 0, 17.0)  # false escape on a non-threat
    gt.on_trigger("rewired", 15.5, 0, 80.0)  # between trials: spontaneous
    st = gt.stats()
    assert (st["trials"], st["threats"], st["non_threats"]) == (2, 1, 1)
    c = st["models"]["connectome"]
    assert (c["caught"], c["false"], c["rt_ms"], c["side_ok"], c["side_n"], c["dist_cm"]) == (
        1,
        0,
        50,
        1,
        1,
        12.0,
    )
    p = st["models"]["prox"]
    assert p["caught"] == 1 and p["side_ok"] == 0 and p["rt_ms"] == 120
    assert st["models"]["looming"]["false"] == 1 and st["models"]["looming"]["caught"] == 0
    assert st["models"]["rewired"]["spontaneous"] == 1
    assert st["models"]["random"]["caught"] == 0
    assert gt.stats(now=12.0)["trials"] == 0  # trials still running are not scored yet
    table = format_table(st, "connectome")
    assert "connectome *" in table and "1/1" in table


def test_simulator_frame_fields_and_random_launch():
    mock = MockBittle()
    sim = Simulator(mock, seed=3)
    now = mock.clock()
    sim.update(now)
    assert sim.random.next_at is not None and sim.current is None
    sim.update(sim.random.next_at + 0.01)
    assert sim.current is not None and len(sim.gt.trials) == 1
    fr = sim.frame(sim.random.next_at or now)
    assert set(fr) >= {
        "dog",
        "hand",
        "approach",
        "random",
        "speed",
        "paused",
        "score",
        "trials",
        "band",
    }
    assert set(fr["dog"]) == {"x_cm", "pose", "skill", "phase"}
    assert fr["random"]["seed"] == 3 and fr["band"] == {"near_cm": 12.0, "far_cm": 20.0}
    a = sim.launch("hover", -1)
    assert a.id == 2 and mock.hand.script is a
    sim.set_paused(True)
    d1 = mock.hand.state(mock.clock())[0]
    time.sleep(0.05)
    assert mock.hand.state(mock.clock())[0] == d1
    sim.set_paused(False)
    sim.reset()
    assert sim.gt.trials == [] and mock.dog_x_cm == 0.0
    with pytest.raises(ValueError):
        sim.launch("jump", 0)


def test_live_simulator_escape_moves_the_dog_and_scores():
    mock = MockBittle()
    bot = Bittle(MockTransport(mock))
    bot.connect(boot_timeout=3.0, quiet_s=0.3)
    models, r2p = fake_models()
    app = EscapeApp(
        EscapeLoop(models, r2p),
        FrameHub(),
        bittle=bot,
        mock=mock,
        mode="escape",
        allow_moves=True,
        escape_cfg=EscapeConfig(action_s=0.8),
    )
    app.sim = Simulator(mock)
    poses = []
    try:
        app.start_robot()
        t_end = time.monotonic() + 3.0
        th = threading.Thread(target=app.run, kwargs={"duration": 7.0}, daemon=True)
        th.start()
        while time.monotonic() < t_end:
            time.sleep(0.1)
        assert app.on_command({"action": "sim_launch", "kind": "fast", "side": 1})["ok"]
        while th.is_alive():
            if app.last_frame and "sim" in app.last_frame:
                poses.append(app.last_frame["sim"]["dog"]["pose"])
            time.sleep(0.05)
        th.join()
    finally:
        app.shutdown()
    fr = app.last_frame
    assert "back" in poses and poses[-1] in ("stand", "rest")
    assert fr["sim"]["dog"]["x_cm"] < -5.0  # it walked back
    gt = fr["sim"]["score"]
    assert gt["threats"] == 1 and gt["models"]["connectome"]["caught"] == 1
    assert fr["sim"]["trials"][0]["escaped"] is True
    assert "gf_left_n" in fr["models"]["connectome"]
    # the loop has stopped: apply commands directly
    assert app._apply_command({"action": "sim_speed", "value": 9}, 0.0)["speed"] == 4.0
    assert app._apply_command({"action": "sim_launch", "kind": "nope"}, 0.0)["ok"] is False


def test_cli_seeded_random_run_prints_ground_truth(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "flybrain_models.escape", escape_fakes.install())
    code = cli.main(
        ["--mock", "--mock-random", "5", "--duration", "14", "--no-dashboard", "--no-log"]
    )
    out = capsys.readouterr().out
    assert code == 0, out
    assert "random approaches, seed 5" in out
    assert "ground truth:" in out and "connectome *" in out
    for k in ("rewired", "random", "prox", "looming"):
        assert k in out.split("ground truth:")[1]


def test_threat_and_safe_kinds_partition():
    assert set(THREAT_KINDS) | set(SAFE_KINDS) == set(KINDS)
    assert isinstance(make_approach("fast", 0), Approach)
