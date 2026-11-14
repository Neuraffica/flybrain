# Adding a demo

A demo is a small command-line program that drives the robot (or the simulator) and usually
shows a dashboard in the browser. Add one to `demos/` and it appears as a card in Flydeck,
the app that finds the robot and starts demos on it. This page is the 15-minute path,
then the rules every demo follows, then the details.

## The 15-minute path

```sh
uv run flydeck new-demo fly-groom      # copy the template, names filled in
uv sync                                # install it
uv run fly-groom --mock --open         # run it on the simulator, dashboard in the browser
```

You now have `demos/fly-groom/`, a working copy of `demos/hello-robot`: it connects, shows the
robot's identity and tilt, stands up, wags the head, and rests when you stop it. Make it yours:

1. **What the robot does**: `src/fly_groom/cli.py`. The main loop is the `while not
   stop.is_set():` block. Read sensors, run your model, send commands there.
2. **Its card in Flydeck**: `src/fly_groom/flydeck.py`. Title, one-line summary, the hardware it
   needs, and its settings (each becomes a field in Flydeck and a flag of your program).
3. **Its dashboard**: `src/fly_groom/static/index.html`. Whatever you `hub.publish({...})` in
   Python arrives in the page as JSON, about as often as you publish.
4. **Check it**: `uv run pytest demos/fly-groom` runs its smoke test on the simulator.

Show it in Flydeck: in the Flydeck app, press **Use my flybrain folder…** above the demo cards
and choose this folder (once; Flydeck remembers it, and **Reload list** picks up later
`uv sync`s). Your card is there, next to the other demos of this folder. Tick **Try in
simulator** and press Start. Started from the app, your demo runs with this folder's `.venv`
Python and the app's Bluetooth permission. Without the app (Windows, Linux, Intel Mac, or USB
only), `uv run flydeck` in this folder shows the same cards.

## Safety rules

The robot is a real machine on a table or floor, often next to people. Every demo follows these:

1. **Always rest on exit.** Whatever happens (Stop, Ctrl-C, an error, a lost link), the robot
   ends lying down. `Bittle.close()` does this; call it in a `finally:` block. The first
   SIGINT (Flydeck's Stop, or Ctrl-C) only asks your loop to finish; ignore later signals so
   they cannot interrupt the rest command. The template shows both. (The one exception is a
   demo that never moves the robot itself, such as `fly-escape-onboard`, where the robot's own
   firmware does the moving: it calls `close(rest=False)` and leaves the robot as it is.)
2. **Never write on top of an unconfirmed command.** The firmware merges two commands that
   arrive together and loses one (over USB the second, over Bluetooth the first), so a "stop"
   can vanish. Send everything through `Bittle` (`send`, `skill`, `set_joint`, `stand`), which
   waits for the firmware's echo or a guard time before the next write. Do not write to the
   transport yourself.
3. **Gate leg motion.** Anything that makes the legs walk needs `--i-have-space` on a real robot,
   and your spec says which settings move the legs (`legs_move=`). Flydeck then asks the user
   to confirm "the robot has space" on every start. The simulator needs no confirmation.
4. **Stop within a few seconds.** Flydeck waits 20 seconds after Stop, then kills the demo and
   warns that the robot may still be standing. Do not block for long in your loop; wait on the
   stop event (`stop.wait(0.1)`), not `time.sleep(10)`.
5. **Say what is wrong in plain words.** "Battery off: long-press the button" beats a
   traceback. Use the exit codes and `status.failed(...)` below.

## The robot link

Everything is in `packages/flybrain-edge/src/flybrain_edge/bittle/`:

| What | Where |
|---|---|
| `Bittle`: connect, `stand()`, `rest()`, `skill("wkF")`, `set_joint(0, 30)`, `poll_imu()`, `latest_sample`, `read_analog()`, `poll_analog()`, `read_voltage()`, `close()` | `robot.py` |
| `open_transport(port=..., ble=...)`, `MockTransport`, `find_serial_ports()` | `__init__.py`, `transport.py` |
| `MockBittle`: the simulator, same commands as the firmware | `mock.py` |
| Command tokens, skill names, joint numbers and limits | `protocol.py` |
| What the firmware really does (echoes, quirks, Bluetooth framing) | `packages/flybrain-edge/NOTES.md` |

`uv run flybrain-bittle probe --port ...` checks a robot from the terminal.

## The contract with Flydeck

Flydeck runs your demo as `python -m <module> <flags>` in its own process group, with the
environment variable `FLYDECK_STATUS=1`, and reads what it prints.

**Flags your program accepts** (argparse in the template):

| Flag | Meaning |
|---|---|
| `--mock` | use the simulator |
| `--port PATH` | USB serial device |
| `--ble [NAME]` | Bluetooth; NAME is the robot's advertised name, e.g. `BittleC2_BLE` |
| `--http-port N` | serve the dashboard on this port (Flydeck picks a free one) |
| `--no-open` | do not open a browser (Flydeck shows the dashboard itself) |
| `--i-have-space` | leg motion allowed (only after the user confirmed) |
| one flag per setting | `Option("sweep_angle", ...)` becomes `--sweep-angle VALUE`; a bool option is a bare flag |

**Status lines** (`from flybrain_demo_api import status`; they print nothing unless Flydeck
started you):

| Call | When |
|---|---|
| `status.state("connecting")` | before connecting |
| `status.ready(url)` | connected and the dashboard answers; Flydeck shows it now |
| `status.state("calibrating", "keep hands away")` | any named phase; shown under the title |
| `status.state("attention", "too bright: move away from the window")` | something the user should fix; shown as a warning |
| `status.failed("could not connect: no robot found")` | before exiting non-zero |

A status line is `FLYDECK {"event": "ready", "url": "http://127.0.0.1:8767/"}` on stdout, if
you want to emit it from another language.

**Exit codes**: 0 finished or stopped, 1 error, 2 bad options, 3 model or data file missing,
4 could not connect, 5 dashboard port in use. Flydeck turns them into a message when you did
not send `status.failed`.

**Registration**: the entry point in your `pyproject.toml` (the template has it):

```toml
[project.entry-points."flybrain.demos"]
fly-groom = "fly_groom.flydeck:SPEC"
```

`new-demo` also adds the demo to the root `pyproject.toml` (dependencies and
`[tool.uv.sources]`) so `uv sync` installs it. A demo package outside this repo works the same
way: depend on `flybrain-demo-api` and declare the entry point.

## The spec

`flydeck.py` holds one `DemoSpec` (from `packages/flybrain-demo-api`, which has no dependencies):

```python
SPEC = DemoSpec(
    id="fly-groom",  # lowercase, dashes
    title="Fly grooming",
    summary="One line on the card.",
    module="fly_groom.cli",  # what Flydeck runs with python -m
    needs=("Bittle X with arm",),  # hardware, in words
    moves_robot=True,
    supports_mock=True,  # --mock works
    transports=("mock", "usb", "ble"),
    options=(
        Option(
            "mode",
            "choice",
            "watch",
            choices=("watch", "groom"),
            label="Mode",
            help="groom: the arm moves",
        ),
        Option("speed", "float", 1.0, minimum=0.25, maximum=4, label="Speed"),
        Option("seed", "int", 0, mock_only=True),  # only offered in the simulator
    ),
    guide="README.md",  # shown by the card's "How it works" link
    legs_move=lambda o: o["mode"] == "groom",  # then Flydeck asks "has space?"
    http_port=8790,  # preferred dashboard port; 0 = any free one
)
```

Option types are `choice`, `bool`, `int`, `float` and `str`. If your flags do not map one to one,
pass `build_argv=` your own function (see `default_argv` in `flybrain_demo_api/spec.py`). Set
`ready="http"` if your program cannot print status lines: Flydeck then waits until the dashboard
answers.

`uv run flydeck list` shows every demo Flydeck sees, and the error for any spec it cannot use.
A broken demo shows its error on its card and does not affect the others.
