# flybrain-models

Models built on `flybrain-data` circuit packs:

- the **fly compass** (`compass.py`): a head-direction ring attractor on the FlyWire
  central-complex connectome, plus an engineering baseline;
- **fly escape** (`escape.py`, [below](#fly-escape)): the looming-escape pathway (LC4,
  LPLC2 -> giant fiber) driven by a two-channel infrared range sensor, plus two baselines.

```python
import math
from flybrain_models.compass import RingAttractor, GyroIntegrator

fly = RingAttractor.from_pack("central_complex")  # connectome
ctrl = RingAttractor.from_pack("central_complex", "rewired", seed=1)
base = GyroIntegrator()  # complementary filter
for m in (fly, ctrl, base):
    m.reset(0.0)
h = fly.step(math.radians(90), 0.02, cue_rad=0.4)  # gyro rad/s (CCW +), dt s, landmark or None
h = fly.step(math.radians(90), 0.02)  # no cue sample this tick
fly.bump, fly.bump_angles, fly.confidence  # EPG activity, angles (heading frame), [0, 1]
```

`step` is pure NumPy, about 20-40 us per 20 ms step on an Apple-silicon laptop.

## Model

Firing-rate network on the pack's 151 neurons (EPG, PEN_a, PEN_b, PEG, Delta7):
`tau dr/dt = -r + tanh(max(0, g(w) * W r + b + cue))`. `W` = signed synapse counts
(acetylcholine +, GABA/glutamate -) times a gain per presynaptic cell type; PEN inputs in the
ellipsoid body are left out (PEN arbors there are 70% presynaptic; config `pen_dendrites`).
Angular velocity `w` multiplicatively modulates PEN input with opposite sign on the two bridge
sides. Heading = population-vector angle of EPG activity on the EPG ring angles stored in the
pack, which were **inferred from connectivity** (FlyWire has no glomerulus labels).

**Landmark cue.** `step(w, dt, cue_rad)` injects a von Mises current onto the EPGs centred at
`cue_rad` (heading frame = ring frame = `bump_angles`). Each cue sample keeps driving for
0.25 s (`CUE_HOLD_S`), so a 5 Hz IMU stream acts as a continuous cue; once no sample has
arrived for 0.25 s the network is in darkness and runs on recurrence (and velocity input)
alone. `reset(h)` shows the cue at `h` for a moment and then releases it; it no longer
re-anchors the readout, so a network that cannot hold a cue position shows it immediately.

**GyroIntegrator** is the standard baseline: a complementary filter that integrates the gyro and
moves 30% of the way to each cue sample; with no cue it is pure integration.

**Fitted: 15 global scalars**, no per-synapse weights: 5 type gains, 5 type biases, time
constant, 2 velocity-input scalars, cue gain and cue width. Shipped values (`compass_params.json`,
key `connectome`): gains EPG 2.83, PEN_a 0.371, PEN_b 1.60, PEG 0.972, Delta7 185.8;
biases EPG -1.93, PEN_a 3.83, PEN_b 0.537, PEG 3.47, Delta7 -0.323; tau 41.9 ms;
velocity k 1.81, a 6.46; **cue gain 5.14, cue kappa 4.92**. Procedure in `compass_fit.py`:
a velocity-response stage (tau <= 0.2 s) from two jittered starts, then all 15 scalars jointly
on a cue + darkness objective (8 training headings, 10 s darkness, plus tracking a noisy cue);
the start with the lower training loss is kept. Refitting jointly was clearly better than
fitting only the cue on top of the earlier velocity-only scalars (with those, no bump survived
darkness: loss 99-101 vs 57). The rewired and random controls get the connectome's 15 scalars
unchanged; `rewired_seed0` / `random_seed0` are the identical pipeline re-run on a control.

## Results (`compass_benchmark.json`, `uv run python -m flybrain_models.compass_benchmark`)

Controls: rewired = mean of 5 seeds, random = mean of 3 seeds, both with the connectome's
scalars. no_recurrence = ablation (same neurons, leak, biases and cue; W = 0). Chance level
for "held" by a bump sitting at an unrelated position is about 0.17.

**C. Heading memory (key test):** 24 headings, bump started 90 deg away, cue 3 s at 5 Hz,
then darkness at rest. held = a bump (confidence > 0.3) within 30 deg of the last cue.
err = mean abs error, trials without a bump counted as 90 deg.

| model | held at cue end | held 5 s | held 30 s | held 60 s | err 60 s | bump left at 60 s |
|---|---|---|---|---|---|---|
| connectome | 0.88 | 0.58 | 0.58 | 0.58 | 42.8 | 100% |
| rewired x5 | 0.98 | 0.04 | 0.04 | 0.05 | 89.1 | 27% |
| random x3 | 0.97 | 0.00 | 0.00 | 0.00 | 90.0 | 0% |
| no recurrence (W = 0) | 1.00 | 0.00 | 0.00 | 0.00 | 90.0 | 0% |
| random seed 0, refit | 1.00 | 0.17 | 0.17 | 0.17 | 70.2 | 50% |
| rewired seed 0, refit | 0.00 | 0.00 | 0.00 | 0.00 | 90.0 | 0% |
| complementary filter | 1.00 | 1.00 | 1.00 | 1.00 | 0.4 | n/a |

**B. Cue tracking**, six benchmark trajectories with a 5 Hz cue (MAE deg / lag ms):
connectome 14.3 / 143; rewired 9.7 / 114; random 10.4 / 107; no recurrence 6.0 / 120;
complementary filter 0.0 / 0. With 5 deg cue noise: 14.0, 10.0, 10.9, 7.5, 1.7.

**A. Dead reckoning, no cue** (gyro only; MAE deg): connectome 85.3; rewired 86.0; random
81.4; no recurrence 81.8; complementary filter (pure integration) 3.9; static null 81.8.
The earlier velocity-only fit scores 98.2 here. (The 82.3 reported earlier for it was
flattered: `reset` used to re-anchor the readout to the requested heading, which hid that
that network has a single fixed bump position.)

**D. Darkness while turning** (cue 3 s, then 4 s at 90 deg/s in the dark): held connectome
0.21, rewired 0.13, random 0, no recurrence 0, filter 1.00. This is dead reckoning and it fails.

## What the numbers support

With the same 15 fitted global scalars, the FlyWire wiring keeps a heading bump alive in
darkness and near the last landmark: 58% of 24 headings stay within 30 deg for a full
minute, with a bump present in every trial. Degree-preserving rewired wiring (5% held), random
wiring (0%) and the network with its wiring removed (0%) lose the bump within seconds. So the
memory comes from the connectome's wiring, not from the cue input or the neurons' leak. It is
a coarse memory: errors are about 43 deg on average because the bump settles into a small
number of preferred positions rather than anywhere on the ring. It does not integrate turns:
with the gyro alone or while turning in the dark it is no better than not moving at all. A
simple complementary filter beats it on every accuracy number (0.4 deg memory error, 4 deg dead
reckoning). The claim is that fly wiring sustains a persistent, landmark-anchored heading
state; it is not a better compass.

## Rebuild

```bash
# stage 1 (two starts) and stage 2 for the connectome; repeat with --variant rewired|random
# --seed 0 for the control refits; keep the start with the lower stage-2 loss
for j in 1 2; do
  uv run python -m flybrain_models.compass_fit --config pen_dendrites --tau-max 0.2 \
      --jitter-seed $j --probe-iters 100 --traj-iters 0 --key base_j$j
  uv run python -m flybrain_models.compass_fit --memory-from base_j$j --iters 50 --key mem_j$j
done
uv run python -m flybrain_models.compass_benchmark
```

The shipped file was assembled from those runs and renamed to `connectome`, `connectome_base`,
`rewired_seed0(_base)`, `random_seed0(_base)`; `connectome_dead_reckoning` is the earlier
velocity-only fit (`--config point`).

Connectome data: FlyWire v783 (Dorkenwald et al. 2024; Schlegel et al. 2024), CC BY 4.0.

# Fly escape

The robot jumps back from an approaching hand. The signal path is the fly's looming-escape
pathway, built from the FlyWire `escape` pack (LC4, LPLC2, the giant fiber DNp01 of each
side, interneurons between them). **The robot has no camera here:** a two-channel infrared
proximity sensor stands in for vision. The model does not see; it reads distance.

```python
from flybrain_models.escape import EscapeCircuit, ProximityThreshold, LoomingRateThreshold
from flybrain_models.escape import raw_to_proximity

fly = EscapeCircuit.from_pack("escape")  # or variant="rewired" / "random", seed=k
out = fly.step(raw_to_proximity(raw_pin36), raw_to_proximity(raw_pin39), dt)  # dt varies
out.trigger, out.side, out.gf_left, out.gf_right, out.drive, out.threshold
fly.activity["GF"], fly.activity["LC4_L"], fly.activity["LPLC2_R"]
```

**Sensor mapping** (`raw_to_proximity`): linear, 0 at or above `baseline - 150` (3400 measured
with nothing in range; 150 is about 3 noise sd), 1 at raw 0. **Inputs**: each side's proximity
drives that side's LPLC2 neurons (looming *size* in the fly) and the positive rate of change
of proximity drives that side's LC4 neurons (looming *speed*); every neuron of a population
gets the same input, because the sensor has no spatial resolution. **Network**: rate units on
the pack's signed synapse counts, `tau dr/dt = -r + tanh(max(0, W r + b + input))`,
exponential-Euler sub-steps of at most 10 ms, so any `dt` works (35-90 ms on the robot).
**Trigger**: each giant fiber has its own threshold; `drive = max(GF_L / th_L, GF_R / th_R)`
crossing 1 (rising edge) triggers, then 1 s refractory. **Side**: from the left vs right
LC4+LPLC2 activity. **Fitted**: 11 global scalars, never per-synapse weights: gains for
LC4+LPLC2 (shared, so the LC4:LPLC2 balance at the GF comes from synapse counts), for
excitatory and inhibitory interneurons and for the GF; 3 biases; tau; input gain; the two GF
thresholds (`escape_fit.py`, CMA-ES, `escape_params.json`). The baselines fit one threshold each.

Shipped values: gains LC4+LPLC2 0.708, excitatory interneurons 0.103, inhibitory 1.097, GF
0.201; biases LC4+LPLC2 -1.036, interneurons -0.730, GF -1.231; tau 13.5 ms; input gain 2.98;
**GF thresholds left 0.198, right 0.834**.

## The robot finding: one side never triggered (fixed)

On the robot, the first shipped fit triggered only when the pin 39 (right) channel dropped;
approaches seen only by pin 36 (left) never triggered. It was not a mapping bug: pin 36 drives
the left LC4/LPLC2, which drive the left GF, which was read out. Two causes together:

1. **A property of the connectome**: the two giant fibers are unequally wired in FlyWire. After
   the threshold, the left GF gets 634 synapses from LC4+LPLC2 (329 + 305) and the right GF
   967 (428 + 539). Under the shared fitted scalars, left-only input drove the left GF to a
   peak of 0.296 against a single threshold of 0.305, while right-only input reached 0.753.
2. **A gap in the training data**: every simulated lateral approach reached both channels, so
   a single threshold that only the right GF could cross on its own was never penalised.

Fix: one threshold per GF, fitted on training data that now includes approaches seen by one
channel only. With the refit the left threshold is 0.198 and the right 0.834; both one-sided
cases now trigger (simulated and on the robot recording, below). The per-side thresholds
compensate for the anatomical asymmetry; they do not remove it.

## Results (`escape_benchmark.json`, `uv run python -m flybrain_models.escape_benchmark`)

Simulated: 384 episodes (approaches at 0.15-1.5 m/s from the left, right, head-on and seen by
one channel only; fast hands stopping short inside the sensor band; slow drift; hovering;
moving away; empty), at 12 and 28 Hz with +-25% jitter, with normal and high noise. A threat is
an approach >= 0.3 m/s to within 5 cm; latency is measured from the hand entering the sensor's
range (12 cm). Real: the 25 s desk recording (5 approaches) and the 45 s robot run (9
approaches), labelled by an offline rule: onset = first sample < 3100 after 0.5 s quiet;
approach if < 500 within 0.5 s; pin 36 taken as left. rewired = 5 seeds, random = 3 seeds, both
with the connectome's scalars; *refit* rows have their own fitted scalars.

| model | hit | FA/min | latency ms | side | left-only hit | right-only hit | desk (of 5) | robot (of 9) |
|---|---|---|---|---|---|---|---|---|
| connectome | 1.00 | 1.87 | 78 | 0.90 | 1.00 | 1.00 | 5 | 8 |
| rewired x5 | 0.71 | 1.48 | 90 | 0.77 | 0.00 | 0.56 | 5 | 6.2 |
| random x3 | 0.00 | 0.00 | - | - | 0.00 | 0.00 | 0 | 0 |
| no interneurons | 1.00 | 5.99 | 65 | 0.92 | 1.00 | 1.00 | 5 | 9 |
| no interneurons, refit | 1.00 | 2.10 | 68 | 0.95 | 1.00 | 1.00 | 5 | 9 |
| rewired seed 0, refit | 1.00 | 2.15 | 66 | 0.95 | 1.00 | 1.00 | 5 | 9 |
| random seed 0, refit | 0.76 | 0.28 | 102 | 0.38 | 0.82 | 0.00 | 5 | 4 |
| proximity threshold | 1.00 | 1.45 | 125 | 0.89 | 1.00 | 1.00 | 5 | 9 |
| looming-rate threshold | 1.00 | 1.17 | 62 | 0.96 | 1.00 | 1.00 | 5 | 8 |

False alarms per minute by kind (stop short / hover / slow drift): connectome 6.7 / 7.4 / 1.7;
no interneurons, refit 9.0 / 1.7 / 8.0; rewired refit 9.0 / 2.0 / 8.0; proximity 0.0 / 2.3 / 8.0;
looming rate 9.0 / 3.0 / 0.0. Hand distance at trigger, 0.8-1.6 m/s vs 0.3-0.5 m/s: connectome
5.5 / 8.3 cm, looming rate 6.7 / 9.0 cm, proximity 3.6 / 5.4 cm. The connectome's one miss on
the robot run is a re-approach 0.5 s after the previous event (the GF had not dropped below
threshold); the looming-rate baseline misses the same one.

What the benchmark can and cannot show. The pathway is nearly feed-forward, two populations
converging on one neuron per side, and the inputs are spatially uniform, so which LC4 contacts
which GF matters little once gains and thresholds are fitted. It can show three things: (1)
side wiring: all of each GF's LC4/LPLC2 input is ipsilateral in FlyWire, about 54% after
rewiring; (2) the LC4:LPLC2 balance at the GF set by synapse counts (0.90 in FlyWire; random
0.34); (3) what the interneurons do with fixed scalars. It cannot test the fly's spatial
looming computation, because the sensor has none.

**Plain-English summary.** On this task the fly's wiring does not beat simple engineering. A
threshold on how fast the IR reading is rising (the looming-rate baseline) catches every
simulated approach with fewer false alarms (1.2 vs 1.9 per minute) and earlier (62 vs 78 ms),
and on the real recordings it does as well (5 of 5 desk approaches, 8 of 9 on the robot, same
as the connectome model). The connectome model works: it catches every simulated approach
from either side, all 5 desk approaches and 8 of 9 on the robot, with no false alarms on
either recording. What the wiring adds is visible only when its fitted settings are kept
fixed: shuffle the connections and the model misses approaches on one side (left-only hit rate
0) because each giant fiber no longer listens mostly to its own side; randomise them and it
never triggers; remove the interneurons and false alarms from a hovering hand rise from 7 to 29
per minute. Re-fit the settings for the shuffled or interneuron-free network and those gaps
close (2.1-2.2 false alarms per minute, every approach caught). The robot also exposed a real
asymmetry in the fly data: the right giant fiber gets about 1.5 times as many looming-neuron
synapses as the left, so each side needs its own trigger threshold. The sensor measures
distance, not vision; the model does not see.

## Rebuild

```bash
uv run flybrain-data build-pack escape
uv run python -m flybrain_models.escape_fit --baselines
uv run python -m flybrain_models.escape_fit --iters 25 --popsize 12
uv run python -m flybrain_models.escape_fit --ablate-interneurons --iters 25 --popsize 12
uv run python -m flybrain_models.escape_fit --variant rewired --seed 0 --iters 25 --popsize 12
uv run python -m flybrain_models.escape_fit --variant random --seed 0 --iters 25 --popsize 12
uv run python -m flybrain_models.escape_benchmark
```
