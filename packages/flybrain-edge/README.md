# flybrain-edge

Edge-side tools for the starter kit. Implemented so far: **`flybrain_edge.bittle`**, a link to
the Petoi Bittle X (+ Arm) running OpenCatEsp32 firmware over USB serial or Bluetooth LE, with a
simulated robot that speaks the same protocol.

Still planned: exporting models to C for the XIAO ESP32-S3 and an on-device benchmark harness
(see [docs/hardware.md](../../docs/hardware.md)).

The protocol facts, each with its firmware file and line, are in [NOTES.md](NOTES.md).

## Robot link

```python
from flybrain_edge.bittle import Bittle, open_transport

bot = Bittle(open_transport(mock=True))  # or port="/dev/cu.usbmodem1101", or ble=""
bot.connect()  # waits for the boot banner, sends '?', reads model and version
bot.stand()  # 'kup', then 'gb' (balance reflex off, IMU still running)
bot.poll_imu(20)  # one 'gp' per reading (~18/s on firmware B10_250330); stream_imu() = 'gP'
bot.measure_yaw_drift(5)  # standing drift, subtracted from later samples
s = bot.read_yaw()  # ImuSample(t, yaw_deg (+ = CCW after yaw_sign), pitch_deg, roll_deg, source)
bot.set_joint(0, 30)  # head pan; clamped, latest value sent at most 15 times a second
bot.skill("vtL")  # step in place, turning left
bot.close()  # stops printing and always sends 'd' (rest)
```

| Module | What it does |
|---|---|
| `protocol` | tokens, joint map and limits, BLE UUIDs, encoders, the IMU line parser (works on fixed-width fields that run together and on BLE text without newlines), the experimental camera-line parser |
| `transport` | `SerialTransport` (macOS auto-detect: `/dev/cu.usbmodem*`, then `cu.wchusbserial*`, then `cu.usbserial*`), `BleTransport` (bleak, Nordic-UART service), `MockTransport` |
| `mock` | `MockBittle`: boot banner, token echoes, rate-limited IMU print, `g` options, head slewing, `kvtL`/`kvtR` turning (default 8°/s with a slow wobble, `turn_rate_deg_s`, `turn_unevenness`; `gait_stall_after_s` makes a gait stop silently until it is re-issued), rest. Its yaw can follow a script (`trajectory(t)`, with `t` measured from `clock_origin()`), the turn gaits, or `set_world_yaw()` / `nudge()`. The fly-compass demo's blindfold scenarios drive it through `trajectory` |
| `robot` | `Bittle`: reader thread (tolerates unsolicited prints), one-command-at-a-time writer that waits for each echo, non-blocking joint streamer, `close()` that always rests |
| `probe` | `flybrain-bittle probe`: read-only queries with raw bytes and timings |

## Check your robot first

```bash
uv run flybrain-bittle ports            # candidate serial ports (add --ble to scan BLE)
uv run flybrain-bittle probe            # USB, first likely port
uv run flybrain-bittle probe --port /dev/cu.usbmodem1101 --rotate-test
uv run flybrain-bittle probe --ble      # first device named Bittle*_BLE
uv run flybrain-bittle probe --mock     # what a good run looks like
```

`probe` sends only `?`, `g?`, `gp`, `v`, `j`, `gP` and `gp`; nothing moves. It prints every byte
with a timestamp, then a summary: model, version, which commands were acknowledged, IMU stream
rate, and with `--rotate-test` the sign of yaw when you turn the dog to its left. If the
verdict says CHECK, paste the whole output into an issue.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `No Bittle serial port found` | Cable is charge-only, battery is off, or the CH340 driver is missing. `ls /dev/cu.*` |
| `Cannot open ... Resource busy` | Another program (Petoi Desktop App, Arduino serial monitor) has the port |
| Probe waits ~10 s, then no `Ready!` | Opening the port did not reset the board (fine) or it is still booting; the probe continues anyway |
| `first-boot setup question (Y/n)` | New or reset board. Finish setup in the Petoi Desktop App |
| Model is `Bittle X`, not `Bittle X+Arm` | Firmware built without `ROBOT_ARM`: joint 1 is then a head tilt limited to ±85° |
| No IMU lines, `v` works | Old firmware; the link falls back to `v` for one-shot reads. Update the firmware for streaming |
| Yaw goes the wrong way | Use `--yaw-sign -1` in the demo, and report it |
| BLE: robot lies down when you connect | Expected: the firmware forces rest on every BLE connection |
| A beep with every head move | `i` commands beep in this firmware. `--joint-token I` avoids it (with caveats, see NOTES.md) or mute the robot with `b0` yourself (stored on the board) |

## Checked on hardware, and not

The link has run against a real Bittle X+Arm (firmware B10_250330, USB); the measurements are
in [NOTES.md](NOTES.md), section "Checked on a real Bittle X+Arm". In short: identification,
IMU polling (`gp`, about 18 readings/s; streaming with `gP` floods the link on that firmware),
drift measurement, `kup`, head pan with `i 0 <deg>`, `kvtL` / `kvtR`, and `d` work; the
firmware yaw is positive clockwise (`yaw_sign=-1`).

Confirmed by eye during a sweep run: the default head-pan sign is right (the arm holds its
direction while the body turns) and a turning gait keeps running while head commands stream.

Still unverified: the safe pan range with the arm attached beyond about 45 degrees, whether 15 `i` commands per second stay smooth,
BLE on macOS with this board, and the camera line format.

## Tests

```bash
uv run pytest packages/flybrain-edge
```
