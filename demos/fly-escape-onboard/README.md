# Fly escape (on the robot)

The brain runs on the device. The fly looming-escape circuit from [fly-escape](../fly-escape/README.md)
is compiled into the Bittle's firmware ([firmware/bittle/fly_escape](../../firmware/bittle/fly_escape/README.md)):
the robot reads its IR sensor, steps the circuit every 35 ms and backs away from a looming hand
by itself. This demo only listens: it switches the module on or off and shows what the robot
reports, the sensor, the two giant fibres and every decision.

```sh
uv run fly-escape-onboard --mock --open                         # simulator, no robot needed
uv run fly-escape-onboard --port /dev/cu.usbmodem1101 --i-have-space
uv run flybrain-bittle ble-run -- uv run fly-escape-onboard --ble --i-have-space
```

Or start **Fly escape (on the robot)** in Flydeck.

What it does:

1. Connects and sends `X?`. Firmware without the module (stock Petoi firmware, or an older
   build of the module that prints to USB only) stops the demo with a pointer to the firmware
   README. Over Bluetooth the firmware's module list is not sent, so only the current module's
   `@FE` status line can say it is there.
2. If the module is off and **Switch the module on** is ticked (`--enable`, the default), sends
   `XE`: the robot stands and measures the resting level (keep hands away). From then on it
   walks backwards by itself, so on a real robot this needs `--i-have-space`.
3. Sends `XEP` (telemetry on, 5 lines a second) and draws it.
4. At Stop, Ctrl-C or a lost link: `XEp` (telemetry off) and the module keeps running, or with
   `--no-leave-running` `Xe` (module off). It never sends a motion command and does not rest the
   robot: it stays as the module left it.

Over Bluetooth the firmware itself lays the robot down when a client connects; the module keeps
listening and its next escape stands the robot up again.

The laptop demo [fly-escape](../fly-escape/README.md) switches an active on-board module off
while it runs (and back on at exit), so the two brains never fight over the legs.

## Files

| File | What it is |
|---|---|
| `src/fly_escape_onboard/cli.py` | flags, connect, find and switch the module, listen, publish |
| `src/fly_escape_onboard/flydeck.py` | the Flydeck card and its two settings |
| `src/fly_escape_onboard/static/index.html` | the dashboard |
| `tests/test_fly_escape_onboard.py` | end to end on the simulator, over USB and both Bluetooth framings |

The module's messages and the parser are in `flybrain_edge.bittle.protocol`
(`OnboardEscapeReader`), the commands in `flybrain_edge.bittle.onboard.OnboardEscape`, and the
simulated module in `MockBittle(fly_escape=True)`, which runs the same model as the board.
