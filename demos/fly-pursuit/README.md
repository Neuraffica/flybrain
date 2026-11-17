# Fly pursuit

A robot dog turns toward a target. The turn comes from a circuit of 265 neurons taken from the
fly connectome: the pathway a male fly uses to keep a moving female in front of him. This first
version runs **in the simulator only**: a simulated dog, a simulated camera and one target. A
guide for the room is in [guide.html](guide.html).

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

Flags: `--controller` (see below), `--fov 30|60|100`, `--target still|sweep|walk|jumps|hidden`,
`--arousal 0..1`, `--seed N` (camera noise, dropouts, random walk), `--duration S`,
`--open` / `--no-open`. Without `--mock`, or with `--port` / `--ble`, it says that this version
runs only in the simulator and exits with code 2 (bad options).

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
- A note that this is a simulation with assumed camera and robot numbers.

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

- **No robot link.** The dog, the camera and the target are all simulated. `--port` and `--ble`
  are refused.
- **Camera numbers are assumed**: 15 frames a second, 100 ms delay, 2 deg noise, 5 % dropped
  frames, and the field of view. The robot version needs them measured on the real camera
  first. The 90 deg/s turn limit and 150 ms lag are assumed too.
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
| `src/fly_pursuit/app.py` | runs the simulation in real time, takes the dashboard's commands, publishes frames |
| `src/fly_pursuit/cli.py` | the command and its flags |
| `src/fly_pursuit/flydeck.py` | the Flydeck card |
| `src/fly_pursuit/static/index.html` | the dashboard |
| `tests/` | simulation, commands, frame, spec, CLI and one end-to-end run (`uv run pytest demos/fly-pursuit`) |

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
positive to the left. `/api/benchmark` gives the offline benchmark's numbers per controller.
