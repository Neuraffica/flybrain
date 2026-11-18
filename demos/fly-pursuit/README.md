# Fly pursuit

A robot dog turns toward a target. The turn comes from a circuit of 265 neurons taken from the
fly connectome: the pathway a male fly uses to keep a moving female in front of him. It runs in
the simulator (a simulated dog, a simulated camera and one target) or on the robot, where only
the **head** turns, toward a face seen by the camera in the dog's mouth (see
[On the robot](#on-the-robot)). A guide for the room is in [guide.html](guide.html).

The camera reports where the target is (its bearing, left or right of the dog's nose). The
circuit turns that bearing into a turn command. The dog turns, the bearing changes, and the loop
closes. You move the target with the pointer, switch the controller while it runs, and watch
the signal go from the eye to the turn.

## Run it

```bash
uv sync
uv run fly-pursuit --mock --open
uv run fly-pursuit --mock --controller no-aotu019 --fov 30 --target jumps --open
uv run fly-pursuit --mock --no-open --http-port 8799    # serve only; open the page yourself
```

Dashboard: http://127.0.0.1:8769/ (`--http-port`). Stop with Ctrl-C; it prints the running
numbers. In Flydeck the card is "Fly pursuit"; tick **Try in simulator**.

Flags: `--controller` (see below), `--arousal 0..1`, `--duration S`, `--open` / `--no-open`.
Simulator only: `--fov 30|60|100`, `--target still|sweep|walk|jumps|hidden`, `--seed N` (camera
noise, dropouts, random walk). Without `--mock` it runs on the robot: see below.

## On the robot

The camera sits in the dog's mouth and turns with the head, so the head is the body the circuit
turns. The legs do not walk. Version 1: head only.

**Two ways to connect the camera** (`--camera robot|usb`, default `robot`)

| | `--camera robot`: through the robot | `--camera usb`: its own USB cable |
|---|---|---|
| Cable | Grove socket labelled **SCL SDA +5V GND** (the I2C one; not the one by TX2/RX2) | the camera's USB-C port into the laptop, a cable from the dog's mouth |
| Starts | **once per power-on** (see below) | any number of times |
| Positions | only when the face moves (about 4 a second with a moving face, none for a still one), no score, no "lost" message | every frame, about 15 a second, with a score; an empty result means no face |
| While it runs | the robot's IMU is off, and the infrared sensor cannot be used | the robot's sensors are untouched |

Choose `robot` for a demo without a cable; choose `usb` for steadier tracking, for repeated
runs, or when the robot's camera module misbehaves.

**Before you start**

- The robot **on the floor**, away from edges, battery on. With `--camera robot` the firmware
  makes it sit when the camera starts; with `--camera usb` the program tells it to sit.
- `--camera robot` **over Bluetooth**: switch the robot off and on before each run if its
  camera was used since it was switched on. The firmware can start its camera only once per
  power-on: a second start in the same boot still answers, but its camera task cannot be made,
  and it then reports only the last old position, which looks exactly like a still face.
  Nothing says so over Bluetooth. Over USB the program restarts the board itself first (an EN
  pulse, as when the reset button is pressed; `--no-usb-restart` skips it).
- `--camera usb`: the camera is found by opening each port with the camera's USB IDs and asking
  its name (`AT+NAME?`). The robot's own USB port has the same IDs, so when the robot is on USB
  too, give `--port` (the robot) or `--camera-port` (the camera); the robot's port is never
  probed. If the robot's USB cable is plugged in while you drive it over Bluetooth, give
  `--camera-port`, or the probe may open (and so restart) the robot's port.

**Run it**

```bash
# camera through the robot
uv run fly-pursuit --port /dev/cu.usbserial-XXXX --open   # USB (without --port: auto-detect)
uv run fly-pursuit --ble --open                           # Bluetooth (or --ble BittleC2)
# camera on its own USB cable, robot over Bluetooth
uv run fly-pursuit --ble --camera usb --open
uv run fly-pursuit --ble --camera usb --camera-port /dev/cu.usbmodemXXXX --open
```

On macOS, Bluetooth from a terminal needs the helper app: `uv run flybrain-bittle ble-run --
uv run fly-pursuit --ble --open` (see `packages/flybrain-edge/NOTES.md`, "macOS permission").
In Flydeck, pick the robot (USB or Bluetooth) instead of **Try in simulator** and choose the
camera connection on the card. Stop with Ctrl-C or Flydeck's Stop; a second Ctrl-C is ignored
so the shut-down cannot be cut short.

**What it does**

1. Start, `--camera robot`: [USB: restart the board, wait for `Ready!`], connect, `Xa~` (voice
   module off: sounds could make the robot move), `XCr~` (camera on, the camera's own head
   reaction off; the robot sits; waits up to 14 s for the robot's `X` reply, and goes on
   without it, saying so), `XCP~` (print the face's position whenever it changes). A missing
   reply is not a failure: over Bluetooth replies go missing while the camera runs although the
   command was carried out. "Fail to start", or a module list with the camera off, is.
   Start, `--camera usb`: find the camera's port, connect to the robot, `Xa~`, `ksit` (sit);
   no `XC` command at all; then `AT+INVOKE=-1,0,1` on the camera's port.
2. Bearing from a position, positive to the dog's left: through the robot
   `-x / 240 x field of view` (x from the image centre; negative x is the dog's left);
   on the USB cable `(120 - x) / 240 x field of view` (x from the image's left edge), taking the
   box with the highest score. The field of view is 60 deg across by default
   (`--camera-fov`); not properly measured: one head step gave about 57.
3. **The head's own turn is taken out.** The camera turns with the head, so every position is
   stored with the head angle at that moment, and the circuit gets
   `bearing measured - (head angle now - head angle then)`. Without this, an old position
   keeps pulling the head past the face (the first robot run swung between about +19 and -17
   deg). The dashboard shows the corrected bearing, and the measured one in small print.
4. No target: through the robot, positions come only when the face moves, so the last one is
   kept. If none came for more than 1 s and the corrected bearing is more than 8 deg off
   centre, the face counts as lost; within 8 deg it is kept (a still, centred face is silent).
   A position of x = -121 with size 0 (nothing found) is lost at once. On the USB cable an
   empty result is lost at once, and no result for 1 s (cable out) is lost too.
5. About 15 times a second the selected controller is stepped with the bearing (or no target).
   Its turn command, times `--gain` (default 1, for tuning on the robot), is clipped to the
   head's rate limit (`--head-rate`, default 60 deg/s, this program's choice), added up into a
   head angle, kept within the pan limit (`--pan-limit`, default +-50 deg; never beyond the
   firmware's 120) and sent as a head joint command, at most `--command-rate` times a second
   (default 8: head commands compete with the camera on the robot's board).
6. When the face has been lost for 3 s, the head **holds where it is**. There is no search.
7. If no camera position at all arrives in the first 20 s, the dashboard and Flydeck show a
   warning: show a face about 50 cm in front of the camera; if the head still does not react,
   switch the robot off and on and start again.

Controllers, arousal and the steering curve work as in the simulator and can be switched
while it runs (the curve is drawn times the gain). The target presets, dragging the target and
the field-of-view buttons do not apply on the robot.

**Shut-down, always** (Stop, Ctrl-C, an error, a camera that does not start, a lost link).
Every step is sent even when a reply is missing.

- `--camera robot`: head to centre, `Xc~` (camera off; the robot also saves its module list),
  `XA~` (voice back on; `--leave-voice-off` skips it), `gU` (IMU updates back on), rest (`d`).
  `Xc~` counts as not done only if neither its reply nor any later reply arrives.
- `--camera usb`: `AT+BREAK` on the camera's port, head to centre, `XA~`, rest. No `Xc~`, no
  `gU`.

After a lost link it reconnects once to do this (without restarting the board).

**If the camera does not start**, it says what to check (the socket, the robot switched on, or
the camera's cable), shuts down as above and exits with code 4 ("could not connect").

**Recovery** (`--camera robot`). The robot saves which modules are on and starts them at every
power-on, the camera with its own head reaction on. If the program says the camera could not
be switched off, or the head moves by itself after a camera run:

1. switch the robot off,
2. unplug the camera,
3. switch it on,
4. run `uv run fly-pursuit --camera-off --port /dev/cu.usbserial-XXXX` (or `--ble`).

`--camera-off` sends `Xc~`, then `Xa~` and `XA~` (switching voice off and on forces the robot to
save the module list even when the camera flag was only in its saved settings), `gU`, rest, and
exits 0. It works when the camera was never started, and does not restart the board.

| Robot option | Default | |
|---|---|---|
| `--camera robot\|usb` | robot | how the camera is connected (above) |
| `--camera-port PATH` | found by probing | `--camera usb`: the camera's serial device |
| `--gain G` | 1.0 | scales the turn command on the robot, 0.1 to 2 |
| `--camera-fov DEG` | 60 | field of view across the image; not properly measured (about 57 from one step) |
| `--head-rate DEG_S` | 60 | fastest head turn; this program's choice |
| `--pan-limit DEG` | 50 | furthest the head turns each way; at most 120 (firmware) |
| `--command-rate HZ` | 8 | head commands per second |
| `--pan-sign 1\|-1` | 1 | 1 if a positive head angle looks left (two readings with the camera robot: positive looks left) |
| `--no-usb-restart` | off | `--camera robot` over USB: do not restart the board first |
| `--leave-voice-off` | off | leave the voice module off at the end |
| `--camera-off` | | only switch the robot's camera off and save that, rest, exit |

Exit codes: 0 stopped, 1 error or lost link, 2 bad options, 3 model data missing, 4 could not
connect or the camera did not start, 5 dashboard port in use.

## What the dashboard shows

- **Arena**, seen from above. The dog in the middle turns with its heading. The target sits on
  a ring around it; drag it, or click the arena and press the arrow keys (Shift: 15 deg). The
  shaded wedge is the camera's field of view. Below: the bearing error (+ = target to the left)
  and whether the camera sees the target in its latest frame.
- **The circuit, from eye to turn.** Two strips of LC10a neurons, one per eye, each neuron
  placed at the spot of the visual field it watches. Below them AOTU019 and AOTU025 of each
  side, then the two DNa02, then the turn command with the robot's 90 deg/s limit marked.
  Brightness is activity as a share of the connectome circuit's largest steady response, so a
  weaker or stronger variant shows as such. For the two baselines the panel says there are no
  neurons and shows only the turn.
- **Steering curve**: the turn command after a target is held at one bearing for 1 s, from
  rest, with no feedback. Recomputed when the controller or arousal changes. The dot is the
  bearing now.
- **Last 20 seconds**: bearing error and turn command, with the time the camera did not see the
  target shaded. Running numbers since the last reset: mean |error|, share of time within
  10 deg, share of time in view. Below them, the offline benchmark's numbers for the same
  controller and camera.
- A note that this is a simulation with assumed camera and robot numbers (on the robot: that
  it runs on the robot, that the field of view is assumed and that the camera reports a
  position only when it changes).

Controls: the controller, arousal (0..1, circuit controllers only), the field of view (+-30,
+-60, +-100 deg) and the target motion (still, slow sweep of +-40 deg every 10 s, random walk,
jumps every 4 s to 35, 65 or 90 % of the half field of view on alternating sides, hidden).
Switching the controller restarts its state; the dog, the target and the camera carry on.

## The fly circuit

LC10a are small-object detectors in the fly's visual system, about 115 per side. Each watches
a small patch of the visual field. They feed two neurons on their own side:

| Stage | What it does |
|---|---|
| LC10a | small-object detectors; each watches one spot of the visual field |
| AOTU019 | inhibits the opposite side's steering neuron; driven by frontal targets |
| AOTU025 | excites the same side's steering neuron; driven by lateral targets |
| DNa02 | steering neurons; the left-right difference sets the turn |

So a target on the left raises left DNa02 (through AOTU025 left) and lowers right DNa02
(through AOTU019 left), and the dog turns left. The circuit also has the other cell types that
connect LC10a to DNa02 (AOTU015a, CB3127, CB2070, LAL027), TuTuAa and AOTU041: 265 neurons in
all.

Where it comes from:

- Wiring: FlyWire, FAFB v783 (Dorkenwald et al. 2024; Schlegel et al. 2024), CC BY 4.0. The
  `pursuit` circuit pack in `packages/flybrain-data` (see `docs/data.md`).
- The pathway: LC10a in pursuit during courtship (Ribeiro et al. 2018, Cell); LC10a receptive
  fields of about 7.5-10.5 deg (Hindmarsh Sten et al. 2021, Nature); AOTU019 for frontal and
  AOTU025 for lateral targets, and arousal recruiting AOTU019 (Collie et al. 2026, Neuron,
  doi:10.1016/j.neuron.2026.01.001); turning follows the left-right DNa02 difference
  (Rayshubskiy et al. 2025, eLife).
- The model: `packages/flybrain-models/src/flybrain_models/pursuit.py`; the benchmark and the
  rig: `pursuit_benchmark.py`, `pursuit_tasks.py`; a summary in `docs/models.md`.

### Measured, assumed, fitted

- **From the connectome**: which neurons connect, the synapse counts, the signs (from the
  predicted transmitter), and each neuron's total number of input synapses in the whole brain.
  Each weight is sign x synapses / the receiving neuron's total input.
- **Assumed**: each LC10a's spot in the visual field (estimated from the eye columns of its
  inputs, with an assumed linear map from column to angle); its tuning width (10 deg); the time
  constant (20 ms); a resting drive of 1 for DNa02, so that inhibition can lower it; how
  arousal acts (at 0 it silences AOTU019, a modelling choice after Collie et al. 2026). The
  camera and robot numbers are also assumed: 15 frames a second, 100 ms delay, 2 deg noise, 5 %
  dropped frames, 90 deg/s turn limit, 150 ms lag.
- **Fitted**: one number, the output gain (turn per unit of DNa02 difference). Each baseline
  also has one fitted number.

**Mirror averaging.** This is one fly's brain and its two halves are not equal (for example
AOTU019 to DNa02: 216 synapses left to right, 121 right to left). Run as it is, the circuit
holds the target about 17 deg to the right of centre. The default controller runs the circuit
twice, once with the bearing and once with its mirror image, and averages the two turns with
opposite sign. That cancels the imbalance without fitting anything. "Connectome, single brain"
is the circuit without this step.

## Benchmark

114 simulated closed-loop runs (steps, sweeps, random walks, fast passes; at +-100 also targets
appearing at the side) with the three field-of-view settings. Mean |bearing error| in degrees,
from `packages/flybrain-models/src/flybrain_models/pursuit_benchmark.json` (2026-11-16). Every
circuit variant uses the connectome's fitted gain; none is refitted.

| Controller | All | +-30 | +-60 | +-100 | Within 10 deg | Steps settled |
|---|---|---|---|---|---|---|
| Proportional (baseline) | 13.80 | 12.53 | 9.34 | 19.06 | 77 % | 100 % |
| Connectome (mirror-averaged) | 14.32 | 13.11 | 9.42 | 19.93 | 72 % | 97 % |
| Without AOTU025 | 14.50 | 13.20 | 9.46 | 20.32 | 72 % | 97 % |
| Bang-bang (baseline) | 14.53 | 13.02 | 10.31 | 19.80 | 72 % | 95 % |
| Connectome, single brain | 31.09 | 45.20 | 20.62 | 27.74 | 23 % | 10 % |
| Without AOTU019 | 36.10 | 52.69 | 25.31 | 30.78 | 20 % | 0 % |
| Connectome at arousal 0 | 36.10 | 52.69 | 25.31 | 30.78 | 20 % | 0 % |
| AOTU019 excitatory | 52.93 | 71.43 | 41.87 | 46.10 | 10 % | 0 % |
| Rewired (mean of 5) | 53.47 | 48.77 | 52.64 | 58.53 | 22 % | 18 % |
| Random (mean of 3) | 57.77 | 59.43 | 54.00 | 59.72 | 16 % | 1 % |

What it shows:

- With one fitted number, the circuit tracks about as well as a tuned proportional controller,
  and slightly worse (14.32 against 13.80 deg). It does not beat the baselines.
- The wiring matters: rewired and random networks do not pursue.
- AOTU019 does the fine frontal steering. Without it, or at arousal 0, a target within about
  20 deg is turned away from, because other cell types (CB3127, CB2070) excite the opposite
  DNa02. An excitatory AOTU019 does the same.
- AOTU025 carries the turn toward lateral targets. Silencing it cuts the open-loop turn at
  60-90 deg to a third or two fifths, but costs little in closed loop, because what is left is
  still above the robot's 90 deg/s limit.

The rewired and random rows are means over 5 and 3 networks; the dashboard runs one of each
(seed 0).

## Things to try

- **Switch off AOTU019** ("Without AOTU019"), field of view +-30, target **Jumps**. Frontal
  targets are now pushed away: the dog turns until the target leaves the field of view. The
  steering curve shows why: near 0 deg it points the wrong way.
- **Connectome, single brain**, target **Still** in front. The dog settles with the target about
  17 deg to the right. Switch back to the mirror-averaged connectome and it centres it.
- **Arousal 0** with the connectome: the same as removing AOTU019. Raise it slowly and watch
  the AOTU019 nodes and the frontal part of the steering curve come back.
- **Without AOTU025**, field of view +-100, drag the target to the side: the steering curve is
  much lower at 60-90 deg, but still above the turn limit, so the dog turns as fast.
- **Rewired** and **Random**: the same neurons or the same number of synapses, different wiring.
  The steering curve loses its shape.
- **Proportional** and **Bang-bang**: no neurons. Compare their running numbers with the
  connectome's on the same target motion.

## Not yet

- **On the robot, only the head turns.** The legs do not walk, and a lost face is not searched
  for.
- **Simulator camera numbers are assumed**: 15 frames a second, 100 ms delay, 2 deg noise, 5 %
  dropped frames, and the field of view. The 90 deg/s turn limit and 150 ms lag are assumed
  too. On the robot the camera's field of view is still assumed (60 deg across), and the
  delay between a face moving and the head command is not measured.
- **Robot mode has had one run on the robot** (Bluetooth, camera through the robot, before the
  head-turn correction). The camera on its own USB cable has not been run on the robot by this
  program; its tests use fakes. Which command rate leaves the robot's camera enough time is
  being measured.
- **+-100 deg** stands for a camera on a panning head. The head and its own control loop are
  not simulated; the camera simply reports the bearing over +-100 deg.
- **No direction selectivity.** Real LC10a respond to small moving objects; here any visible
  target drives them, moving or not.
- **No forward walking.** The dog only turns in place; in the fly, pursuit gain depends on
  walking speed.
- Not modelled either: elevation, object size and contrast, arousal that builds up over time.

## Files

| File | What it is |
|---|---|
| `src/fly_pursuit/sim.py` | the world: heading, target, camera, turn limit and lag (plain class, no threads) |
| `src/fly_pursuit/controllers.py` | the nine controllers, the steering curve, the benchmark numbers for the page |
| `src/fly_pursuit/camera.py` | camera lines from the robot's output, the bearing, the rule for "no target" (pure) |
| `src/fly_pursuit/usbcam.py` | the camera on its own USB cable: SSCMA parser, port probing, reader thread |
| `src/fly_pursuit/robot.py` | the robot: board restart, start-up, the head loop, shut-down, `--camera-off` |
| `src/fly_pursuit/app.py` | runs the rig (simulation or robot head) in real time, takes the dashboard's commands, publishes frames |
| `src/fly_pursuit/cli.py` | the command and its flags |
| `src/fly_pursuit/flydeck.py` | the Flydeck card |
| `src/fly_pursuit/static/index.html` | the dashboard |
| `tests/` | simulation, commands, frame, spec, CLI, robot mode on a fake robot and a fake camera port (`pursuit_fakes.py`), one end-to-end run (`uv run pytest demos/fly-pursuit`) |

### Dashboard protocol

The page gets one JSON frame about 20 times a second from `/api/stream` (the latest also at
`/api/state`) and sends commands as JSON to `POST /api/command`:

```text
{"action": "target", "deg": 35}           put the target at this world bearing (it then holds still)
{"action": "nudge", "deg": -5}            move the target by this much
{"action": "motion", "motion": "sweep"}   still | sweep | walk | jumps | hidden
{"action": "controller", "controller": "no-aotu019"}
{"action": "arousal", "value": 0.5}       0..1
{"action": "fov", "deg": 60}              30 | 60 | 100
{"action": "reset"}                       restart the running numbers
```

Each reply is `{"ok": true}` or `{"ok": false, "error": "..."}`. Angles are in degrees,
positive to the left. On the robot, `target`, `nudge`, `motion` and `fov` are refused, and the
frame's `mode` is `"robot"`, with the head angle (`head_deg`), the camera's age and state, and
the link's numbers. `/api/benchmark` gives the offline benchmark's numbers per controller.
