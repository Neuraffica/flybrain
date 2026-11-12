# Fly escape

A model of the fruit fly's looming-escape circuit (looming-sensitive LPLC2 / LC4 cells feeding
the giant fibre, wired from the connectome) runs on a laptop. An infrared proximity sensor on a
Petoi Bittle X+Arm stands in for the fly's eye; when the circuit commands an escape, the dog walks
backwards for about a second and stands again. A guide for the room is in
[guide.html](guide.html).

Five models get the same input on every sensor reading: the connectome circuit, the same circuit
rewired, a random network of the same size, a proximity threshold and a looming-rate threshold
(all from `flybrain_models.escape`). Only the **driver** (default connectome, switchable on the
dashboard) moves the robot.

## Run it

```bash
uv sync
uv run fly-escape --mock --open                           # the visual simulator, escapes on
uv run fly-escape --mock --mock-random 7 --no-dashboard --duration 120   # scoreboard in the terminal
uv run fly-escape --mock --mock-random 7 --mode observe --no-dashboard --duration 120
uv run fly-escape --mock --mock-script cycle              # older fixed hand scripts
uv run fly-escape --replay packages/flybrain-models/tests/fixtures/ir_hand_approaches_usb.json
uv run fly-escape --port /dev/cu.usbmodem1101             # real dog, observe
uv run fly-escape --port /dev/cu.usbmodem1101 --i-have-space   # real dog, escapes on
uv run flybrain-bittle ble-run -- uv run fly-escape --ble  # Bluetooth on macOS
```

Dashboard: http://127.0.0.1:8766/ (`--http-port`).

Useful flags: `--driver connectome|rewired|random|prox|looming`, `--pins 36,39` (left first),
`--swap-sides`, `--escape-action back|back-turn`, `--action-seconds 1.0`, `--min-interval 3`,
`--max-per-minute 6`, `--imu` (also poll the IMU; off by default to keep the sensor rate up),
`--analog-rate 40`, `--duration S`, `--no-dashboard`, `--no-log`, `--benchmark FILE.json`.

## The simulator (`--mock`)

`uv run fly-escape --mock --open` opens the dashboard on a drawn top view of the Bittle X+Arm
(yellow-and-black body, four legs, head with the arm and claw, the bone-shaped sensor) with the
sensor's two beams, the graded band between 12 and 20 cm shaded, and a hand. Everything drawn is
the mock robot's own state: the hand is the `ProximityHand` the simulated sensor reads, the pose
comes from the skill the mock is running (stand, walking back, rest), the displacement from its
kinematics (15 cm/s while walking back). A small side view shows the step back against floor
marks. Frames are interpolated in the browser (about 60 fps on a 20 Hz stream). A beam lights as
its channel responds; when the driver model triggers, the dog and the giant-fibre gauges flash.
With `--mock` escape mode needs no `--i-have-space` (no real legs move).

Controls:

* **Approach now** (threats): slow or fast, from the left, head-on or from the right.
* **Not threats**: hover (stays in the graded band), stop short (lunges, stops at the band's
  edge), slow drift (creeps in over 5-7 s), retreat (starts outside the range, moves off).
* **Random**: keeps launching approaches, half threats and half not, random side and timing
  (3-6 s apart, after the dog has stood again), from a seeded generator; the seed is shown and
  can be changed. `--mock-random [SEED]` starts it from the command line.
* **Approach speed** slider (0.25x to 3x), **Pause**, **Hand away**, **Reset** (scores, hand,
  dog back on its start mark), and dragging the hand.
* **Driving the dog**: which model's escapes move the dog; the others run on the same input.

Ground-truth scoreboard (the simulator knows what each approach was; `fly_escape/groundtruth.py`):
threats caught, false escapes on non-threats, mean reaction time from the hand entering the
sensor's range, correct side over approaches from the left or right, the hand's distance at the
trigger, and triggers with no approach running. `--no-dashboard` prints it at the end.

**Read the escape-mode scores with care.** As on the robot, the sensor is not read while the dog
walks back, so after the driver escapes the other models see nothing more of that approach (and
the hand is further away when reading resumes). Models that would have fired later than the
driver are counted as misses. For a like-for-like comparison of all five models use
`--mode observe`, where the dog never moves. Approach definitions and distances are in
`fly_escape/approaches.py`; whether a lunge that stops at the band's edge "should" trigger is a
judgement (it is scored as a non-threat).

## Modes

* **observe** (default on a real robot unless `--i-have-space`): models, scores and dashboard;
  nothing moves.
* **escape** (default with `--i-have-space`, and always the default with `--mock`): the driver's
  trigger starts an escape.

Escape mode is refused, with the reason on the dashboard and in the terminal, until the start-up
calibration has passed.

## What happens on each reading

1. The two pins are read with one binary `R` command (`flybrain_edge.bittle.Bittle.read_analog`),
   about 28 times a second over USB with nothing else on the line.
2. Each raw reading becomes a proximity in [0, 1] with `raw_to_proximity(raw, baseline)`, where
   the baseline is that channel's resting level measured at start.
3. All five models step with the two proximities and the time since the last reading.
4. Triggers are grouped into **events** (triggers within 1 s of each other). For each event the
   dashboard lists which models fired and how much earlier or later than the driver.

### Calibration

For the first 2 s (and after **Recalibrate**) keep hands away. Each channel's mean and spread is
measured. Escape mode is refused if a channel rests below 1500 (sensor unplugged, wrong socket,
something in front of it; an unconnected pin reads about 0) or spreads by more than 300.

### The escape

Skills in the Bittle X+Arm instinct table that move away: `bkF` (walk back; the firmware maps
plain `bk` to it and, built with `ROBOT_ARM`, loads `bkArmF`), and `bkL` / `bkR` (walk back while
turning, `bkArmL`). Forward and turning gaits do not move away, and behaviours like `jmp` are
violent. So:

* `--escape-action back` (default): `kbkF` for `--action-seconds` (1 s), then `kup`.
* `--escape-action back-turn`: `kbkR` for a threat on the left, `kbkL` on the right, then `kup`.
  Which way `bkL` really rotates the body is not verified; `--turn-sign -1` swaps them.

Sequence: analog reads are paused (an `R` read stops a running gait: the firmware's read path
leaves the skill token unset), the gait is sent, after the action time `kup` is sent and its echo
awaited, then reading resumes. If the stand is not confirmed it is re-sent twice; then the robot
is put to rest and the demo drops to observe mode with a "STAND FAILED" message. Backing barely
changes the heading, so there is no heading-based stop check (unlike fly-compass).

On top of the model's own refractory period the app allows at most one escape every 3 s
(`--min-interval`) and six per minute (`--max-per-minute`); ignored triggers are counted and the
reason is shown.

## Scores (no ground truth)

* **Triggers**: since the last reset.
* **False**: triggers while **Hands off** is on (dashboard button or key **H**): mark the times
  when you know no hand is near.
* **Events / Missed**: events the model fired in, and events outside hands-off that others fired
  in and it did not.
* **Δt vs driver**: mean trigger time minus the driver's, over events where both fired.

## Dashboard

Same server and look as fly-compass: one self-contained page on 127.0.0.1, light and dark,
Server-Sent Events at 20 Hz. It shows the two proximity channels over the last 12 s with every
model's trigger moments in lanes below, a top view of the dog with the sensed side lit, the
connectome's LPLC2/LC4 strips and the giant-fibre drive against threshold, the scoreboard (radio
buttons pick the driver), the event list, mode buttons, Hands off, Recalibrate, and the offline
benchmark (`flybrain_models/escape_benchmark.json`) when it exists. With `--mock` the simulator
(above) comes first.

API: `GET /api/state`, `GET /api/stream`, `GET /api/benchmark`, `POST /api/command` with
`{"action": "mode" | "driver" | "hands_off" | "recalibrate" | "reset_scores" | "mock_hand" |
"mock_script", ...}`.

## Session logs and replay

Each run (not replays) writes `runs/escape-<stamp>/`: `meta.json` (pins, side swap, firmware,
escape settings), `samples.csv` (`t, raw_a, raw_b`: the two pins in `--pins` order),
`triggers.csv` (`t, model, side, hands_off`) and `escapes.csv` (`t, side, skill, stood`).
`--replay` accepts a run directory, a `samples.csv`, or a JSON list of `[t, raw_a, raw_b]` (the
format of the model package's fixture `ir_hand_approaches_usb.json`); `--replay-speed 0` runs as
fast as possible and prints the scores.

## Safety

* Escape mode needs `--i-have-space`; nothing that moves the legs runs without it.
* Commands are written one at a time and never on top of an unconfirmed one (see
  `packages/flybrain-edge/README.md`, "Safety and command timing"); the stand is echo-checked.
* Hard limits on top of the model: one escape per 3 s, six per minute.
* Ctrl-C or SIGTERM (only the first one interrupts) or any error: a running gait is stopped with
  `kup`, then `d` rests the robot. A "low power" message from the firmware is shown at start.
* No `X` module command is ever sent (switching modules on made the robot do skills by itself).

## Troubleshooting

| Symptom | What to do |
|---|---|
| `escape models are not available` | `uv sync`; the model lives in `packages/flybrain-models` |
| "calibration FAILED … rests at 0" | Sensor unplugged or in the wrong socket (must be 39/36), or a hand in front at start; check with `flybrain-bittle probe --analog 36,39` and press Recalibrate |
| Escapes on the wrong side | `--swap-sides`; check with `flybrain-bittle probe --analog 36,39 --side-test` |
| Escape mode button greyed out | Start with `--i-have-space`, and wait for calibration |
| Few sensor readings per second | Other polling on the line (`--imu`), or Bluetooth |

## Checked on hardware, and not

Checked on a Bittle X+Arm (firmware B10_250330, USB): the `R` analog reads of pins 36 and 39
(about 28 reads/s, the robot did not move), the sensor's resting level and response to a hand.
Not yet: escape mode on the robot, the escape action's distance and the stand after it, which pin
is the robot's left, `back-turn` direction, the sensor over Bluetooth, the arm-version mount.
<!-- hardware results: to be filled in after testing -->

## Tests

```bash
uv run pytest demos/fly-escape
```

Fake models in `tests/escape_fakes.py` keep the tests independent of the model package's state.
