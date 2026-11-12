import threading

import pytest
from escape_fakes import fake_models
from fly_escape.calibration import Calibration
from fly_escape.escape import EscapeConfig, EscapeController, action_skill
from fly_escape.loop import EscapeLoop


def approach_rows(t0, side=0.0, rest=(3400, 3440), dur=0.3, rate=30.0):
    """Hand approach: both channels fall from rest to 0 over `dur` (the side channel first)."""
    rows, n = [], int(1.0 * rate)
    for i in range(n):
        t = t0 + i / rate
        f = min(1.0, max(0.0, (t - t0) / dur))
        lag_l = 0.05 if side < 0 else 0.0
        lag_r = 0.05 if side > 0 else 0.0
        fl = min(1.0, max(0.0, (t - t0 - lag_l) / dur))
        fr = min(1.0, max(0.0, (t - t0 - lag_r) / dur))
        _ = f
        rows.append((t, rest[0] * (1 - fl), rest[1] * (1 - fr)))
    return rows


def rest_rows(t0, seconds, rate=30.0, rest=(3400, 3440)):
    return [
        (t0 + i / rate, rest[0] + (i % 3) * 10, rest[1] - (i % 2) * 10)
        for i in range(int(seconds * rate))
    ]


def feed(loop, rows):
    out = []
    for t, a, b in rows:
        out += loop.ingest(t, a, b)
    return out


def make_loop(**kw):
    models, r2p = fake_models()
    return EscapeLoop(models, r2p, **kw)


def test_calibration_then_triggers_events_and_latency():
    lp = make_loop()
    feed(lp, rest_rows(0.0, 2.2))
    assert lp.cal.done and lp.cal.ok
    assert lp.cal.baseline[0] == pytest.approx(3410, abs=15)
    fired = feed(lp, approach_rows(3.0))
    models = {tr.model for tr in fired}
    assert "connectome" in models
    tr = lp.take_driver_trigger()
    assert tr is not None and tr.model == "connectome"
    assert len(lp.events) == 1
    ev = lp.frame(4.0)["events"][0]
    assert ev["fired"]["connectome"]["dt_ms"] == 0
    stats = lp.latency_stats()
    assert stats["connectome"]["events"] == 1
    for k, s in stats.items():
        if k != "connectome" and s["events"]:
            assert s["mean_ms"] is not None


def test_side_and_swap():
    lp = make_loop()
    feed(lp, rest_rows(0.0, 2.2))
    fired = feed(lp, approach_rows(3.0, side=+1))  # left channel (pin A) falls first
    assert any(t.side == 1 for t in fired if t.model == "connectome")
    lp2 = make_loop(swap_sides=True)
    feed(lp2, rest_rows(0.0, 2.2))
    fired2 = feed(lp2, approach_rows(3.0, side=+1))
    assert any(t.side == -1 for t in fired2 if t.model == "connectome")


def test_hands_off_counts_false_alarms():
    lp = make_loop()
    feed(lp, rest_rows(0.0, 2.2))
    lp.set_hands_off(True, 2.5)
    feed(lp, approach_rows(3.0))
    lp.set_hands_off(False, 4.5)
    assert lp.scores["connectome"].false_alarms == 1
    fr = lp.frame(5.0)
    assert fr["events"][0]["hands_off"] is True
    assert fr["hands_off_spans"][0][1] is not None
    assert fr["models"]["connectome"]["missed"] == 0


@pytest.mark.parametrize(
    ("level", "noise", "ok", "word"),
    [(3400, 20, True, "calibrated"), (2, 2, False, "unplugged"), (3400, 900, False, "noisy")],
)
def test_calibration_checks(level, noise, ok, word):
    c = Calibration()
    c.start(0.0)
    i = 0
    while not c.add(i / 30.0, level + noise * (((i * 37) % 21) / 10.0 - 1.0), level):
        i += 1  # spread evenly over +-noise
    assert c.ok is ok and word in c.message


def test_calibration_needs_samples():
    c = Calibration()
    c.start(0.0)
    c.add(0.0, 3400, 3400)
    c.add(2.5, 3400, 3400)
    assert c.done and not c.ok and "readings" in c.message


class Job:
    def __init__(self, ok=True, done=True):
        self.done = threading.Event()
        if done:
            self.done.set()
        self.result = [] if ok else None


class FakeRobot:
    def __init__(self, stand_ok=True):
        self.stand_ok = stand_ok
        self.log = []
        self.paused = False

    def send_skill(self, name):
        self.log.append(name)
        return Job(ok=self.stand_ok or name != "up")

    def send_rest(self):
        self.log.append("d")
        return Job()

    def pause_sensing(self, paused):
        self.paused = paused
        self.log.append("pause" if paused else "resume")


def test_action_skills():
    assert action_skill("back", 1) == "bkF"
    assert action_skill("back-turn", 1) == "bkR"  # threat on the left: turn right
    assert action_skill("back-turn", -1) == "bkL"
    assert action_skill("back-turn", 1, turn_sign=-1) == "bkL"
    assert action_skill("back-turn", 0) == "bkF"
    with pytest.raises(ValueError):
        action_skill("jump", 1)


def test_escape_sequence_and_rate_limits():
    r = FakeRobot()
    c = EscapeController(r, EscapeConfig(action_s=1.0, min_interval_s=3.0, max_per_minute=3))
    assert c.trigger(10.0, 1)
    assert r.log == ["pause", "bkF"] and c.state == "acting"
    c.update(10.5)
    assert c.state == "acting"
    c.update(11.0)
    assert r.log[-1] == "up" and c.state == "standing"
    c.update(11.1)
    assert c.state == "idle" and r.log[-1] == "resume" and c.history[-1].stood is True
    assert not c.trigger(12.0, 1) and "less than 3 s" in c.last_block  # min interval
    t = 13.0
    for _ in range(2):
        assert c.trigger(t, 0)
        c.update(t + 1.0)
        c.update(t + 1.1)
        t += 3.5
    assert not c.trigger(t, 0) and "3 escapes" in c.last_block  # per-minute cap
    assert c.trigger(10.0 + 60.5, 0)  # the first one has left the window


def test_stand_not_confirmed_rests_and_drops_to_observe():
    r = FakeRobot(stand_ok=False)
    c = EscapeController(r, EscapeConfig(action_s=1.0, stand_retries=2))
    c.trigger(0.0, -1)
    c.update(1.0)
    out = [c.update(1.0 + 0.1 * i) for i in range(1, 6)]
    assert "observe" in out
    assert r.log.count("up") == 3 and r.log[-2:] == ["d", "resume"]
    assert c.state == "failed" and not c.trigger(30.0, 0)


def test_abort_mid_action_stands():
    r = FakeRobot()
    c = EscapeController(r, EscapeConfig(action_s=1.0))
    c.trigger(0.0, 0)
    c.abort(0.3)
    assert r.log[-1] == "up" and c.state == "standing"


def test_calibration_ignores_a_hand_late_in_the_window():
    """Like the recorded fixture: a hand arrives at 1.6 s of the 2 s window."""
    c = Calibration()
    c.start(0.0)
    i = 0
    while not c.add(i / 30.0, 3400 if i / 30.0 < 1.6 else 50, 3440 if i / 30.0 < 1.6 else 80):
        i += 1
    assert c.ok and abs(c.baseline[0] - 3400) < 1
