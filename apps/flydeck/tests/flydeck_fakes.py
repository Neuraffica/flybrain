"""A fake demo program for Flydeck tests: behaves as asked by --behave."""

import textwrap
from pathlib import Path

from flybrain_demo_api import DemoSpec, Option

FAKE_DEMO = textwrap.dedent(
    """
    import signal, sys, time
    from flybrain_demo_api import status

    args = sys.argv[1:]
    behave = args[args.index("--behave") + 1]
    print("fake: argv", " ".join(args), flush=True)
    stop = []
    if behave == "hang":
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    else:
        signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    if behave == "fail4":
        print("fake: cannot connect: nope", file=sys.stderr, flush=True)
        sys.exit(4)
    status.state("connecting")
    if behave != "noready":
        status.ready(None)
    if behave == "crash":
        time.sleep(0.2)
        status.failed("motor on fire")
        sys.exit(1)
    if behave == "attention":
        status.state("attention", "too bright: move away from the window")
    while not stop:
        time.sleep(0.05)
    print("fake: resting the robot", flush=True)
    sys.exit(0)
    """
)

BEHAVIOURS = ("ok", "crash", "hang", "noready", "fail4", "attention")


def fake_spec(**kw) -> DemoSpec:
    base = {
        "id": "fake-demo",
        "title": "Fake demo",
        "summary": "a test double",
        "module": "fakedemo",
        "moves_robot": True,
        "options": (
            Option("behave", "choice", "ok", choices=BEHAVIOURS),
            Option("walk", "bool", False),
        ),
        "legs_move": lambda o: bool(o.get("walk")),
        "ready_timeout_s": 5.0,
    }
    base.update(kw)
    return DemoSpec(**base)


def write_fake(dir_: Path) -> Path:
    (dir_ / "fakedemo.py").write_text(FAKE_DEMO)
    return dir_
