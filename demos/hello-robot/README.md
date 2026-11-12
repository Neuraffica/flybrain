# Hello robot

The smallest complete flybrain demo, and the template for your own.

It connects to the robot, shows its model, firmware, battery and tilt on a small dashboard,
stands it up, wags its head a few times and waits for you to press **Wag again**. Stop it and
the robot lies down to rest. With **Take a few steps forward** ticked it also walks for about
a second; that moves the legs, so Flydeck first asks you to confirm the robot has space.

```sh
uv run hello-robot --mock --open        # the simulator, no robot needed
uv run hello-robot --port /dev/cu.usbmodem1101
```

Or start it from Flydeck, which gives you the robot picker, the Stop button and the
Bluetooth permission.

## Make your own demo from it

```sh
uv run flydeck new-demo fly-groom
uv sync
uv run fly-groom --mock --open
```

Then change `src/fly_groom/cli.py` (what the robot does) and `src/fly_groom/flydeck.py` (its
card and settings in Flydeck). The rules every demo follows, and why, are in
[docs/adding-a-demo.md](../../docs/adding-a-demo.md).

## Files

| File | What it is |
|---|---|
| `src/hello_robot/cli.py` | the program: flags, connect, the main loop, always rest at the end |
| `src/hello_robot/flydeck.py` | the Flydeck card: title, settings, which settings move the legs |
| `src/hello_robot/static/index.html` | the dashboard page |
| `tests/test_hello_robot.py` | a smoke test on the simulator |
| `pyproject.toml` | the package, its command, and the `flybrain.demos` entry point |
