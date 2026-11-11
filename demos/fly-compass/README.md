# Fly compass

A model of the fruit fly's head-direction circuit (the EPG ring of the central complex, wired
from the connectome) runs on a laptop and keeps a robot dog's head pointing one way, like a
compass needle, while someone turns the dog by hand, or while the dog turns itself (sweep).

* **needle** (default): the head pan joint, which carries the arm, counter-rotates so the arm
  keeps pointing at the heading captured at start. The connectome model drives it; the
  dashboard can hand the needle to the rewired control or the plain integrator to show the
  difference.
* **hold**: when the heading error passes 20°, the dog steps in place (`kvtL` / `kvtR`) until
  it is back within 6°, with a cooldown and a 6 s limit per turn. It moves the legs, so it needs
  `--i-have-space`.
* **sweep**: nobody touches the dog. It steps in place to the left until the measured heading
  is +35° from where it started, stands for 3 s, steps right to −35°, stands, and repeats. The
  needle keeps working the whole time, so the arm should stay on the start direction while the
  body swings under it. Moves the legs, so it needs `--i-have-space`. Details below.
* **observe**: no motion; models and dashboard only.

Three models get exactly the same inputs: the connectome ring, a rewired control ring, and a
gyro integrator that works as a complementary filter (it integrates angular velocity and is
pulled towards the cue when there is one).

## Inputs: angular velocity, a landmark cue, and the blindfold

* **Angular velocity**, every tick. It is derived from the IMU's fused yaw, because the OpenCat
  firmware prints yaw, pitch and roll but no raw gyro rate. The loop takes the rate over a
  window of at least 40 ms, plus a slow correction that keeps its integral on the measured yaw.
  How often yaw arrives depends on the firmware: the source read for this kit limits printing to
  5 Hz, while the real robot (firmware B10_250330) is polled with `gp` at about 18 readings a
  second (see `packages/flybrain-edge/NOTES.md`). On the real robot the drift measured during
  the first 5 s (`--drift-cal`) is subtracted.
* **A landmark cue**, on every tick that brings a fresh IMU sample: the same measured yaw, in
  the models' heading frame. It stands in for the visual landmark a fly uses to anchor its
  compass. On other ticks the cue is `None`.
* **Blindfold** (dashboard button, key **B**, or `--blindfold-after S`): the cue is withheld
  on every tick, so the models run on their recurrent wiring and the angular velocity alone.

"Truth" is the IMU's fused yaw, which is also the cue. With the cue on, the error therefore
mostly measures lag. The blindfolded numbers are the informative ones:

* **Dog held still, blindfolded:** does the bump stay where the cue left it? A ring whose
  wiring holds a bump should keep the heading; the rewired control is expected to lose it.
* **Dog turned while blindfolded:** the ring models only get the turn through angular velocity.
  The model package measured that the connectome ring does not integrate velocity well on its
  own, so expect the ring models to lose track here. The integrator keeps following the turn.
  The connectome ring is **not** a dead-reckoning compass in this demo.

Older model builds whose `step()` takes no `cue_rad` still run; the scoreboard says
"no cue input" for them.

## Run it

```bash
uv sync
uv run fly-compass --mock                  # simulated dog; open http://127.0.0.1:8765/
uv run fly-compass --mock --mode sweep --i-have-space          # the dog turns itself, +-35 deg
uv run fly-compass --mock --mode sweep --i-have-space --sweep-cycles 2 --no-dashboard
uv run fly-compass --mock --mode sweep --i-have-space --mock-gait-stall 3   # test the watchdog
uv run fly-compass --mock --mock-script sweep   # the simulated dog is turned as if by hand
uv run fly-compass --mock --mock-script memory  # turn 90, blindfold, hold still 20 s, cue back
uv run fly-compass --mock --mock-script dark-turn   # blindfold, turn 90 in the dark, cue back
uv run fly-compass --mock --mock-script memory --no-dashboard   # just print the phase table
uv run fly-compass --port ... --blindfold-after 30
uv run fly-compass                         # real dog, USB, first likely port
uv run fly-compass --port /dev/cu.usbmodem1101
uv run fly-compass --ble                   # first BLE device named Bittle*_BLE
uv run fly-compass --ble BittleA3          # by name
uv run fly-compass --port ... --mode hold --i-have-space
uv run fly-compass --port ... --mode sweep --i-have-space   # real dog turns itself, on the floor
uv run fly-compass --replay runs/20261111-101500             # play a session back (dashboard)
uv run fly-compass --replay runs/20261111-101500 --replay-speed 0   # as fast as possible, scores only
```

In mock mode, drag the dial, or press ← / → (Shift for 15°), to turn the simulated dog. The
scenarios can also be started from the dashboard (scenario menu, **Run**). Each scenario re-zeroes
first and keeps error statistics per phase; when it ends, the terminal prints a table of mean
|error|, error at the end of each phase, and mean bump confidence for every model
(`--exit-after-script` quits at that point; `--no-dashboard` implies it). Phases are defined in
`src/fly_compass/scenarios.py`.

Useful flags: `--rate 20` (loop Hz), `--http-port 8765`, `--driver connectome|rewired|gyro`,
`--duration S`, `--no-dashboard`, `--no-log`, `--open`, `--benchmark FILE.json`.
Hardware calibration, all with **unverified** defaults: `--yaw-sign`, `--pan-sign`,
`--pan-limit 80`, `--pan-rate 120`, `--joint-token i|I`, `--joint-rate 15`. Run
`uv run flybrain-bittle probe --rotate-test` once before the first real run.

`--mock-script sweep` moves the simulated dog as if a hand turned it; `--mode sweep` makes the
dog turn itself with its turning gaits. They cannot be combined.

### Sweep mode

A state machine on the measured heading (the IMU yaw relative to the zero taken at start, not a
model's heading):

1. turn left (`kvtL`) until the heading reaches +A (`--sweep-angle`, default 35°);
2. stand (`kup`) and pause P seconds (`--sweep-pause`, default 3);
3. turn right (`kvtR`) until −A, stand, pause, and start again. One left and one right turn is
   a cycle; `--sweep-cycles N` ends the run after N cycles, otherwise it runs until Ctrl-C or
   `--duration`.

Safeguards:

* Each turn has a time limit (`--sweep-timeout`, default 25 s). On timeout the dog stands, a
  warning is printed, and it turns the other way. After three timeouts in a row sweep gives
  up and switches to needle mode.
* Watchdog: on the real robot it is not certain that a turning gait keeps running while head
  commands stream. If a turn has moved the heading less than 3° in the commanded direction over
  the last 3 s, the turn skill is sent again, at most once every 3 s
  (`SWEEP_STALL_WINDOW_S`, `SWEEP_STALL_MIN_DEG`, `SWEEP_REISSUE_S` in `loop.py`). Re-sending
  a gait that is already running is harmless on the firmware.
* Switching mode from the dashboard in the middle of a turn sends `kup` on the next tick;
  Ctrl-C sends `kup` and then `d`.

The needle runs throughout, from the selected driver model, with the usual head limits and
command rate. The dashboard shows the sweep state (turning left / pausing / turning right,
target, time in state, cycle count, re-issues, timeouts) under the dial and marks ±A on it.

On carpet the real dog turns at only about 2 to 7°/s, so one leg from −35° to +35° can take
10 to 35 s; raise `--sweep-timeout` on slow floors. The simulator turns at 8°/s with some
unevenness (`--mock-turn-rate`), and `--mock-gait-stall S` makes its gait stop silently S
seconds after each (re)issue, to exercise the watchdog.

### With the real robot

1. Bittle X with the arm, OpenCatEsp32 firmware built as **Bittle X+Arm**, battery on.
2. Put it on a flat surface or its stand, with room for the arm to swing ±80°.
3. `uv run flybrain-bittle probe` and check the verdict says OK.
4. `uv run fly-compass --port ...`. It waits 3 s, stands the dog (`kup`), switches the balance
   reflex off (`gb`) so the legs stay still while it is handled, starts polling the IMU (`gp`),
   measures the standing drift for 5 s (do not touch it) and captures the current heading as
   the target.
5. Turn the dog slowly by hand. The arm should keep pointing the same way until it reaches the
   pan limit. Press **Re-zero** to take a new target.
6. Optional, on the floor with about a metre of clear space:
   `uv run fly-compass --port ... --mode sweep --i-have-space`. The dog turns itself between
   ±35° and the arm should keep pointing the same way.
7. Ctrl-C (or any error) stops motion and sends `d`: the dog rests and the servos switch off.

## Dashboard

One self-contained page served on 127.0.0.1 (no CDN, no web fonts, light and dark), updated
about 20 times a second over Server-Sent Events:

* the compass dial: dog heading (IMU), each model's heading, the target, and where the arm points;
* the EPG ring of the connectome and the rewired model side by side, one dot per neuron;
* heading error of all three models over the last 30 s;
* a cue bar across the top: "Cue on", or "Blindfolded" with the time since the cue was removed
  and how far the dog has turned since, plus the running scenario phase;
* a scoreboard: mean |error| with the cue on and while blindfolded (separate columns), current
  error, bump confidence (and its mean while blindfolded), ms per step, ring size; the radio
  buttons pick which model drives the needle (default: connectome);
* blindfolded stretches shaded in the error chart;
* mode buttons, Blindfold, scenario menu (mock), Re-zero, Reset scores, link status, IMU rate;
* the offline benchmark, if one is found (`--benchmark FILE`, or `compass_benchmark.json` in the
  model package; see `fly_compass/benchmark.py`). Whatever sections the file has are rendered
  generically: lists of per-model results become tables (transposed when there are many
  measures), nested per-trajectory tables fold out, plain-text sections are used as column
  descriptions.

API: `GET /api/state` (latest frame), `GET /api/stream` (SSE), `GET /api/benchmark`,
`POST /api/command` with `{"action": "rezero" | "reset_scores" | "mode" | "driver" |
"blindfold" (optional "on": true/false) | "scenario" ("name") | "mock_turn_to" | "mock_nudge",
...}`. Re-zero restarts the models at the current heading, which is itself a cue.

## Session logs

Each run (not replays) writes `runs/<stamp>/` (gitignored): `meta.json`, `samples.csv` (every IMU
sample as used: time, yaw after `--yaw-sign`, pitch, roll, `blindfold` 0/1, scenario `phase`) and
`frames.csv` (per tick: truth, angular velocity, whether a cue was given, blindfold, phase, each
model's heading, error and confidence, head command, mode, hold state, and `sweep_state`: the
sweep label such as "turning left" or "pausing", empty in other modes). `--replay` feeds `samples.csv`
through fresh models on a simulated clock and reproduces the cue schedule from the `blindfold`
column (older logs without it replay with the cue on).

## Safety

* Joint angles are clamped twice on the host (pan limit, then the firmware's joint table) and
  once more by the firmware; the head moves at most `--pan-rate` deg/s.
* Hold and sweep refuse to start without `--i-have-space`. Each hold turn stops after 6 s, and
  after three timeouts in a row hold disables itself; sweep's limits are described above.
* Ctrl-C, SIGTERM or any exception leads to `kup` (if a gait is running) and `d` (rest). If the
  link itself is lost nothing can be sent: the robot keeps its last posture.

## Troubleshooting

| Symptom | What to do |
|---|---|
| `compass models are not available` | `uv sync` from the repo root; the model lives in `packages/flybrain-models` |
| `dashboard port 8765 unavailable` | `--http-port 8766` |
| Status pill says "no IMU data" | Run `uv run flybrain-bittle probe`; old firmware without `gP` cannot stream |
| The arm turns the wrong way | `--pan-sign -1` (and report it) |
| The heading goes the wrong way | `--yaw-sign -1` (and report it) |
| Short beep every head move | Firmware beeps on `i`; try `--joint-token I`, or mute with `b0` in a serial monitor |
| Over BLE the dog lies down on connect | The firmware forces rest on each BLE connection; the demo stands it up again |

## Checked on hardware, and not

Run on a real Bittle X+Arm (firmware B10_250330, USB; see `packages/flybrain-edge/NOTES.md`):
reading the IMU by polling, drift measurement, observe mode, `kup`, head pan with `i 0 <deg>`,
`kvtL` / `kvtR` stepping in place, and `d`. The yaw sign was measured (the firmware counts
clockwise as positive, hence `--yaw-sign -1` on a real robot).

Sweep mode ran on the robot (2 cycles at +-25 degrees on carpet, 29 s, no timeouts or
re-issues), and the arm was seen to hold its direction, so the default pan sign (+1) is right
on this firmware and the turning gait keeps running while head commands stream.

Not yet checked on the robot: the safe pan range with the arm beyond about 45 degrees, hold
mode, sweep on a hard floor or at larger angles, and Bluetooth.

## Tests

```bash
uv run pytest demos/fly-compass
```

The tests use small fake models (`tests/compass_fakes.py`) so they do not depend on the
connectome model's state.
