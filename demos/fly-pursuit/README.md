# Fly pursuit

A robot dog turns toward a target. The turn comes from a circuit of 265 neurons taken from the
fly connectome: the pathway a male fly uses to keep a moving female in front of him. It runs in
the simulator (a simulated dog, a simulated camera and one target) or on the robot, where only
the **head** turns, toward a face seen by the camera in the dog's mouth (see
[On the robot](#on-the-robot)); with our firmware module the circuit can also run on the
robot's own board. A guide for the room is in [guide.html](guide.html).

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

**Four ways to run it**, in the order to try them. The first two need our camera module in the
robot's firmware (`firmware/bittle/fly_pursuit`, extension letter `P`; flashing it is described
in its README); the last two work with the stock firmware.

| | 1. On-board (`--onboard`) | 2. Module, laptop computes (`--camera module`) | 3. Stock module (`--camera robot`) | 4. Own USB cable (`--camera usb`) |
|---|---|---|---|---|
| Who computes | the robot's board: the circuit (264 of its 265 neurons; the one left out cannot change the turn) runs in the firmware and moves the head | the laptop, any controller, arousal and gain switchable live | the laptop | the laptop |
| Camera read by | our module | our module | the stock camera module `C` | the laptop, over the camera's USB-C port |
| Needs | our firmware module | our firmware module | stock firmware | a cable from the dog's mouth to the laptop |
| Camera results | every frame (about 15 a second), with a score and a real "no face" | the same | only when the face moves, no score, no "no face" | every frame, with a score and "no face" |
| Starts | any number of times | any number of times | **once per power-on** | any number of times |
| While it runs | the IMU keeps working | the IMU keeps working | the IMU is off, the infrared sensor cannot be used | the robot's sensors are untouched |

`--camera auto` (the default) asks the robot after connecting (`X?~`): if an `@FP` status line
comes back, the firmware has our module and it uses `module`; if not, it uses `robot` and says
so. `--onboard` needs the module; without it the program stops with a message naming
`firmware/bittle/fly_pursuit`.

**Measured on the robot** (firmware module README, "Run on the robot", 2026-11-18, a Bittle X
with firmware B10_251121). These runs used the module's own commands; **this program's
`--onboard` and `--camera module` have so far been tested only with a fake robot**, and no
session has been longer than a minute.

- USB, passive (`XP~`): `@FP cam run 1;` 0.15 s after the command; 196 results in 12.7 s (15.4
  a second), none skipped, median gap 54 ms, longest 112 ms, no error line. `Xp~` then `XP~`
  again in the same boot: `@FP cam stop;`, then `@FP cam run 2;` and 15.5 results a second.
- USB, on-board (`XPB~`), a person's face about 50 cm away moving side to side: 481 results in
  31.2 s (15.4 a second), none skipped, a face in 385 of them (80 %). Head pan between -36 and
  +41 degrees for a face between -42 and +43 degrees from straight ahead. Face offset from the
  image centre while seen: median 3.0, mean 4.0, 90th percentile 8.0, largest 19.8 degrees.
- Bluetooth: echoes `XP~` 0.08 s, `XPB~` 0.85 s (it sits first), `XPb~` 0.04 s, `Xp~` 0.06 s,
  none missing. Passive, 6 s: 89 results (15.5 a second), none missing. On-board, 30 s, a face
  moving side to side: 463 results (15.4 a second), none missing, a face in 378 (82 %); offset
  from the image centre median 1.2, 90th percentile 9.8 degrees; head pan between -25 and +50
  degrees. No error line.
- Not measured: how long one on-board control step takes, the IMU's read rate while the camera
  runs, a session longer than a minute, and what happens when the camera's own side stalls.

The line format (`@FP <seq> <x> <y> <w> <h> <score> [b=] t= p= [hold];`, status, `cam run`,
`cam stop`, `err <kind> <count>`) is in the module's README, "Host lines"; this program follows
it (`camera.py`, `parse_fp`).

**Before you start**

- The robot **on the floor**, away from edges, battery on. It sits when the demo starts (with
  `--onboard` and `--camera robot` the firmware does it; otherwise the program sends `ksit`).
- The camera in the Grove socket labelled **SCL SDA +5V GND** (the I2C one; not the one by
  TX2/RX2), except with `--camera usb`.
- `--camera robot` **over Bluetooth**: switch the robot off and on before each run if its
  camera was used since it was switched on. The stock firmware can start its camera only once
  per power-on: a second start still answers, but it then reports only the last old position,
  which looks exactly like a still face. Over USB the program restarts the board itself first
  (an EN pulse, as when the reset button is pressed; `--no-usb-restart` skips it). Our module
  needs none of this. Do not use the stock camera module and ours in the same power-on.
- `--camera usb`: the camera is found by opening each port with the camera's USB IDs and asking
  its name (`AT+NAME?`). The robot's own USB port has the same IDs, so when the robot is on USB
  too, give `--port` (the robot) or `--camera-port` (the camera); the robot's port is never
  probed. If the robot's USB cable is plugged in while you drive it over Bluetooth, give
  `--camera-port`, or the probe may open (and so restart) the robot's port.

**Run it**

```bash
# 1. on-board: the robot computes
uv run fly-pursuit --ble --onboard --open                          # Bluetooth (or --ble BittleC2)
uv run fly-pursuit --port /dev/cu.usbserial-XXXX --onboard --open  # USB
# 2. our module, the laptop computes (--camera auto picks it by itself when the module is there)
uv run fly-pursuit --ble --camera module --open
uv run fly-pursuit --port /dev/cu.usbserial-XXXX --camera module --open
# 3. the stock camera module
uv run fly-pursuit --ble --camera robot --open
# 4. the camera on its own USB cable, robot over Bluetooth
uv run fly-pursuit --ble --camera usb --open
uv run fly-pursuit --ble --camera usb --camera-port /dev/cu.usbmodemXXXX --open
```

Without `--port` or `--ble` the robot's USB port is found by itself. On macOS, Bluetooth from a
terminal needs the helper app: `uv run flybrain-bittle ble-run -- uv run fly-pursuit --ble
--onboard --open` (see `packages/flybrain-edge/NOTES.md`, "macOS permission"). In Flydeck,
pick the robot (USB or Bluetooth) instead of **Try in simulator**, then tick **the robot
computes (on-board)** or choose the camera connection on the card. Stop with Ctrl-C or
Flydeck's Stop; a second Ctrl-C is ignored so the shut-down cannot be cut short.

**What it does**

1. Start, every robot run: connect (the robot answers `?`). With `--camera auto`, `module` or
   `--onboard`: `X?~`, to see whether the firmware has our module.
   - `--onboard`: `Xa~` (voice module off: sounds could make the robot move), `XP~` (module on,
     passive), `XPB~` (on-board control on; the firmware makes the robot sit). From then on the
     laptop sends **no head command**: it reads each result line's `t=` (the robot's head turn
     command, deg/s), `p=` (its pan target), `b=` (the bearing its circuit got) and `hold`.
   - `--camera module`: `Xa~`, `XP~` (passive: the module does not move the robot), `ksit`. No
     `XC`, no `gU`, no board restart.
   - `--camera robot`: [USB: restart the board, wait for `Ready!`], `Xa~`, `XCr~` (camera on,
     the camera's own head reaction off; the robot sits; waits up to 14 s for the robot's `X`
     reply, and goes on without it, saying so), `XCP~` (print the face's position whenever it
     changes). A missing reply is not a failure: over Bluetooth replies go missing while the
     stock camera runs although the command was carried out. "Fail to start", or a module list
     with the camera off, is.
   - `--camera usb`: find the camera's port, connect to the robot, `Xa~`, `ksit`; no `XC`
     command at all; then `AT+INVOKE=-1,0,1` on the camera's port.
2. Bearing from a position, positive to the dog's left: through the robot (both modules)
   `-x / 240 x field of view` (x from the image centre; negative x is the dog's left); on the
   USB cable `(120 - x) / 240 x field of view` (x from the image's left edge), taking the box
   with the highest score. The field of view is 60 deg across by default (`--camera-fov`); not
   properly measured: one head step gave about 57. The firmware module assumes the same 60.
3. **The head's own turn is taken out.** The camera turns with the head, so every position is
   stored with the head angle at that moment, and the circuit gets
   `bearing measured - (head angle now - head angle then)`. Without this, an old position
   keeps pulling the head past the face (the first robot run swung between about +19 and -17
   deg). The dashboard shows the corrected bearing, and the measured one in small print.
   On-board, the firmware does the same correction itself.
4. No target: with our module and on the USB cable a result comes every frame, so "no face"
   is lost at once, and no result line at all for 1 s is lost too. Through the stock module,
   positions come only when the face moves, so the last one is kept: if none came for more than
   1 s and the corrected bearing is more than 8 deg off centre, the face counts as lost; within
   8 deg it is kept (a still, centred face is silent). A position of x = -121 with size 0
   (nothing found) is lost at once.
5. Laptop computing: about 15 times a second the selected controller is stepped with the
   bearing (or no target). Its turn command, times `--gain` (default 1, for tuning on the
   robot), is clipped to the head's rate limit (`--head-rate`, default 60 deg/s, this
   program's choice), added up into a head angle, kept within the pan limit (`--pan-limit`,
   default +-50 deg; never beyond the firmware's 120) and sent as a head joint command, at most
   `--command-rate` times a second (default 8). On-board, the firmware does this once per
   camera result with the same numbers (60 deg/s, +-50 deg, hold after 3 s), and the circuit is
   fixed: connectome, mirror-averaged, arousal 1, gain 1.
6. When the face has been lost for 3 s, the head **holds where it is**. There is no search.
7. Warnings on the dashboard and in Flydeck: no camera result at all in the first 20 s (show a
   face about 50 cm in front of the camera; then a power cycle). With our module also:
   `err silent` or `err busheld` ("power-cycle the camera: switch the robot off and on"), and
   `err nocam` ("the camera is not plugged into the I2C socket"). They clear when the camera
   answers again.

Controllers, arousal and the steering curve work as in the simulator and can be switched while
the laptop computes (the curve is drawn times the gain). On-board they cannot: the dashboard
says so and disables them. The target presets, dragging the target and the field-of-view
buttons do not apply on the robot.

**On-board, the circuit panel is a shadow.** The robot does not send its neurons' activity. The
panel runs the same Python circuit on the laptop, once per result line, with the bearing the
robot reports (`b=`, or "no target" for a frame without a face), timed by when the lines arrive.
It is labelled as recomputed on the laptop, for display. The robot's own turn command is the one
its firmware prints (`t=`); the dashboard shows it next to the recomputed one and their
difference, so a mismatch is visible.

**Camera module health** (with our module): a line on the dashboard with whether the camera is
running (`cam run <n>` / `cam stop`), how many result lines came and how many had a face, how
many are missing between consecutive sequence numbers (lost on the link, or skipped by the robot
while busy), and the latest count of each `err` kind. The same numbers are printed at the end.

**Shut-down, always** (Stop, Ctrl-C, an error, a camera that does not start, a lost link).
Every step is sent even when a reply is missing.

- `--onboard`: `XPb~` (on-board control off), head to centre, `Xp~` (module off), `XA~` (voice
  back on; `--leave-voice-off` skips it), rest (`d`).
- `--camera module`: head to centre, `Xp~`, `XA~`, rest.
- `--camera robot`: head to centre, `Xc~` (camera off; the robot also saves its module list),
  `XA~`, `gU` (IMU updates back on), rest. `Xc~` counts as not done only if neither its reply
  nor any later reply arrives.
- `--camera usb`: `AT+BREAK` on the camera's port, head to centre, `XA~`, rest. No `Xc~`, no
  `gU`.

After a lost link it reconnects once to do this (without restarting the board).

**If the camera does not start**, it says what to check (the socket, the robot switched on, or
the camera's cable), shuts down as above and exits with code 4 ("could not connect").

**Recovery.** With our module there is nothing to recover: the robot saves which modules are
on, but a start of our module at power-on is passive (result lines only, the robot does not
move), so if `Xp~` was not confirmed the saved state is harmless. The one exception is the
"follow a face after power-on" setting, which you switch on yourself (see "The robot by
itself" below); `--camera-off` switches it off too. With the stock module
(`--camera robot`) it is not: that module starts at every power-on with its own head reaction.
If the program says the camera could not be switched off, or the head moves by itself after a
camera run:

1. switch the robot off,
2. unplug the camera,
3. switch it on,
4. run `uv run fly-pursuit --camera-off --port /dev/cu.usbserial-XXXX` (or `--ble`).

`--camera-off` asks `X?~`; with our module it sends `Xps~` (module off and the follow after
power-on setting off; an older build of the module takes it as `Xp~`); then `Xc~`, `Xa~` and
`XA~` (switching voice off and on forces the robot to save the module list even when the camera
flag was only in its saved settings), `gU`, rest, and exits 0. It says the setting was cleared
only when the robot confirms it with `@FP auto 0;`. It works when the camera was never
started, and does not restart the board.

**Limits.** Short runs only so far (no session longer than a minute, on-board or not). The
stock camera module has measured problems on this robot (`packages/flybrain-edge/NOTES.md`,
"Petoi AI Vision camera"): it starts once per power-on, reports a position only when it
changes, stops the IMU, loses replies over Bluetooth, and once left the I2C bus stuck so that
the robot did not boot until its power was cycled. Our module is built to avoid each of these
(its README, "The six failures").

| Robot option | Default | |
|---|---|---|
| `--onboard` | off | the robot computes (needs our module; implies `--camera module`) |
| `--camera auto\|module\|robot\|usb` | auto | how the camera is read (above) |
| `--camera-port PATH` | found by probing | `--camera usb`: the camera's serial device |
| `--gain G` | 1.0 | scales the turn command on the robot, 0.1 to 2 (not on-board) |
| `--camera-fov DEG` | 60 | field of view across the image; not properly measured (about 57 from one step) |
| `--head-rate DEG_S` | 60 | fastest head turn; this program's choice (on-board: the firmware's 60) |
| `--pan-limit DEG` | 50 | furthest the head turns each way; at most 120 (on-board: the firmware's 50) |
| `--command-rate HZ` | 8 | head commands per second (not on-board: the laptop sends none) |
| `--pan-sign 1\|-1` | 1 | 1 if a positive head angle looks left (two readings with the camera robot: positive looks left) |
| `--no-usb-restart` | off | `--camera robot` over USB: do not restart the board first |
| `--leave-voice-off` | off | leave the voice module off at the end |
| `--camera-off` | | only switch the robot's camera off (and our module and its follow after power-on setting, if there) and save that, rest, exit |
| `--autostart on\|off` | | only save our module's follow a face after power-on setting, rest, exit (below) |

With `--onboard`, `--controller`, `--arousal`, `--gain`, `--head-rate`, `--pan-limit`,
`--camera-fov` and `--pan-sign` are ignored (the program says which).

Exit codes: 0 stopped, 1 error or lost link, 2 bad options, 3 model data missing, 4 could not
connect, the camera did not start, or `--onboard` / `--camera module` / `--autostart` without
our module in the firmware (for `--autostart`: without its newer build), 5 dashboard port in use.

## The robot by itself

The newer build of our firmware module (`firmware/bittle/fly_pursuit`) can save a "follow a
face after power-on" setting. With it on, a few seconds after every power-on, and with no
computer connected, the robot beeps three times, sits and turns its head toward a face by
itself: the on-board circuit, the same as `--onboard`, with nobody pressing Stop.

**Safety.** Once this is on, the robot moves every time it is switched on. Switch it on only on
the floor, away from edges, and expect the three beeps, then the sit.

Switch it on or off from the command line:

```bash
uv run fly-pursuit --ble --autostart on      # or --port /dev/cu.usbserial-XXXX
uv run fly-pursuit --ble --autostart off
```

or in Flydeck: pick the robot, set "Robot: follow a face after power-on" to on or off, Start.
"leave as it is" (the default) is a normal run. Either way the program connects, saves the
setting, rests the robot and exits; it does not open the dashboard. The last line says what is
now saved and what the robot will do at the next power-on.

- `on` sends `X?~`, `XPS~` (module on, setting saved on), waits for `@FP auto 1;`, then `Xp~`
  so that the camera is off again and the robot stays still now. The setting acts at the next
  power-on.
- `off` sends `X?~`, `Xps~` (module off, setting saved off) and waits for `@FP auto 0;`.
- `--camera-off` switches the setting off too.

It needs the newer build. The build of the module flashed on 2026-11-18 does not know this
setting: it ignores the command and never prints `@FP auto`, and the program then says "this
robot's module is older and has no autostart", tells you to flash the newer build, and exits 4.
Without our module at all it exits 4 too, having sent nothing but `?` and `X?~`.

A normal run of the demo (`--onboard`, `--camera module`, `--camera auto`) does not change the
saved setting. If the robot has already started following a face by itself when the demo
connects, the demo says so and takes over: a bare `XP~` switches the robot's own control off on
the newer build, so the laptop can turn the head, and `--onboard` switches it on again with
`XPB~`. At the end it switches the module off with `Xp~`, which leaves the setting as it was.
The dashboard's camera module line says whether the setting is on.

Run on a Bittle X on 2026-11-18 with the newer build flashed: `--autostart off` and `on` over
Bluetooth were both confirmed by the robot, and the demo connected over Bluetooth to the robot
while it was following by itself and took the head over (no face was in view in that run, so
the head did not turn). The robot's own start was run after a restart over USB, not after the
power switch; the firmware folder's README has the numbers and what was not tried.

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
  it runs on the robot, who computes, that the field of view is assumed, and how the camera in
  use reports "no face").
- On the robot with our module: the camera module line (above), and on-board the robot's turn
  command, pan target and bearing next to the recomputed circuit.

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
- **This program on the robot**: with the stock camera module, runs over USB and Bluetooth
  (`packages/flybrain-edge/NOTES.md`). `--onboard` and `--camera module` have been tested only
  with a fake robot; the firmware module itself has been run on the robot with its own commands
  (above), never longer than a minute. The camera on its own USB cable has not been run on the
  robot by this program either. Which head command rate leaves the camera enough time is not
  measured.
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
| `src/fly_pursuit/camera.py` | camera lines from the robot's output (the stock module's, and our module's `@FP` lines), the bearing, the rule for "no target" (pure) |
| `src/fly_pursuit/usbcam.py` | the camera on its own USB cable: SSCMA parser, port probing, reader thread |
| `src/fly_pursuit/robot.py` | the robot: module detection, board restart, start-up, the head loop (or, on-board, reading the robot's own and the shadow model), shut-down, `--camera-off`, `--autostart` |
| `src/fly_pursuit/app.py` | runs the rig (simulation or robot head) in real time, takes the dashboard's commands, publishes frames |
| `src/fly_pursuit/cli.py` | the command and its flags |
| `src/fly_pursuit/flydeck.py` | the Flydeck card |
| `src/fly_pursuit/static/index.html` | the dashboard |
| `tests/` | simulation, commands, frame, spec, CLI, robot mode on a fake robot (with or without our module) and a fake camera port (`pursuit_fakes.py`), one end-to-end run (`uv run pytest demos/fly-pursuit`) |

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
positive to the left. On the robot, `target`, `nudge`, `motion` and `fov` are refused (on-board
also `controller` and `arousal`), and the frame's `mode` is `"robot"`, with the head angle
(`head_deg`), the camera's age and state, and the link's numbers. With our module the frame also
has `camera_module` (`present`, `on`, `lines_on`, `control_on`, `camera` running or stopped,
`starts`, `errors` per kind, `results`, `faces`, `missing`, `last_seq`, `problem`), and
on-board `onboard` (`turn_deg_s`, `pan_deg`, `bearing_deg` and `hold` as the robot printed
them, `shadow_turn_deg_s`, `shadow_cmd_deg_s`, `difference_deg_s`, the mean and largest
difference, `results`, `age_s`); both are `null` otherwise. `/api/benchmark` gives the offline
benchmark's numbers per controller.
