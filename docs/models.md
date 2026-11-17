# Models

Which model families fit the hackathon rules, what each costs to train and run, and what
to pretrain before the event.

The rules that shape this: fly connectivity must shape the model, it must be compared to a
standard baseline with numbers, and it must run on a device.

## Model families

| # | Family | How the connectome enters | Training | Runs on ESP32-S3? | Fit for 5 hours |
|---|---|---|---|---|---|
| 1 | **Connectome reservoir** | A fly circuit is the fixed recurrent weight matrix. Only a linear readout is trained | Ridge regression, seconds on a laptop | Yes, up to a few thousand neurons | Best |
| 2 | **Hand-wired circuit models** | Known circuit motifs implemented directly: motion detector, looming detector, ring-attractor compass, mushroom-body memory | None, or one-shot Hebbian learning on device | Yes, very small | Best |
| 3 | **Connectome-masked trained net** | The connectome sets which weights exist and their sign; gradient descent sets magnitudes (the flyvis approach) | Backprop, minutes to hours | Small ones, as a sparse int8 net | Feasible if pretrained |
| 4 | **Connectome-coupled gait generator** | Leg oscillators coupled with weights taken from nerve-cord premotor circuits | Tune 5 to 10 parameters by hand, CMA-ES or RL in sim | Yes, tiny | Feasible |
| 5 | **Hybrid: fly front end + learned head** | A frozen fly circuit (1 or 2) produces features; a small MLP, GRU or transformer on top is trained | Backprop on the head only | Head must be small; a transformer head stays on the laptop | Feasible |
| 6 | **Whole-brain spiking model on a server** | The full 139k-neuron brain. Stimulate sensory neurons, read descending neurons, send commands to the robot | None | No. Laptop or GPU, robot over USB/BLE | Impressive, but weak on the "runs on a device" rule |

### 1. Connectome reservoir (recommended default)

State update `x[t+1] = (1-a)·x[t] + a·tanh(W·x[t] + W_in·u[t])`, output `y = W_out·x`.
`W` is a signed circuit pack scaled to a chosen spectral radius. Only `W_out` is learned.

Why it suits the day: training is a single linear solve, so a team can iterate dozens of
times; the brain is the model, not decoration; inference is one sparse matrix-vector
product per step, which an MCU handles.

Honest caveat: published evidence that a fly connectome beats a random reservoir is mixed.
One 2025 paper reports a win on time-series prediction; two conference posters report that
random reservoirs do as well or better. So the comparison is the experiment, and the
controls below are required rather than optional.

### 2. Hand-wired circuits

- **Motion detector** (T4/T5): correlate each pixel with its delayed neighbour in four
  directions. Gives optic flow from a 32x32 grayscale image for almost no compute.
- **Looming detector** (LPLC2): fires when the four motion directions all point outward
  from a centre. Standard baseline: frame differencing or a small CNN.
- **Compass and path integrator** (central complex): a ring of 16 to 32 neurons holds
  heading, driven by gyro yaw rate; a second layer sums steps into a home vector.
  Baseline: plain yaw integration, or a GRU.
- **Mushroom body memory**: project the input to many sparsely active units (random sparse
  expansion, keep the top 5%), learn by changing one layer of output weights after a single
  exposure. Works for one-shot image classification and for "have I seen this view before",
  which is enough to retrace a taught route. Baseline: a CNN trained with backprop, or
  nearest neighbour on raw pixels.

### 3. Connectome-masked nets

For teams with ML depth. Take a circuit pack as the sparsity mask and sign constraint of a
recurrent or convolutional net and train it on the task. flyvis is the reference. Too slow
to train from scratch in 5 hours for anything large, which is why we pretrain.

### 4. Gait generator

The fly has six legs and the dog has four, so this is transfer of a coordination rule, not
a copy. Plan: one oscillator per leg (drop the middle pair or fold it into the others),
coupling weights from MANC premotor connectivity between leg neuropils, output mapped to
shoulder and knee angles. Baseline: Petoi's built-in walk and trot.

### 5 and 6. Hybrids and servers

A transformer does not fit on the ESP32-S3 in any useful size. The workable split is the
one the fly uses: fast reflexes on the body, slow deliberation elsewhere.

- On device at 20 to 50 Hz: looming stop, steering, compass, gait.
- Off device at 1 to 2 Hz: a planner (a small transformer, or a hosted vision-language
  model) that sets goals such as "turn left at the next opening".
- Variant: the whole-brain spiking model on a laptop replaces the planner. Camera or
  ultrasonic readings drive sensory neurons; rates of DNa01/DNa02, DNp09 and MDN are mapped
  to turn, walk and back up.

## Required controls ("does the brain matter?")

Every model in the zoo ships with these, and teams should report at least the first two:

1. **Degree-preserving rewire**: same neurons, same in/out degrees, wiring shuffled.
2. **Random sparse**: same size, density and sign ratio.
3. **Standard baseline at equal parameter count or equal latency**: MLP, small CNN or GRU.
4. **Sign shuffle** (optional): same wiring, excitatory/inhibitory labels shuffled.

Metrics to report: task score, parameters, flash and RAM in KB, latency in ms on the
device, and energy per inference if a USB power meter is available.

## What organizers should pretrain and package

| Artifact | Why |
|---|---|
| **Circuit packs** for the 7 circuits in `docs/data.md`, plus rewired and random controls of each | Removes the 850 MB download and an hour of data wrangling from every team |
| **Reservoir readouts** for 2 or 3 reference tasks (IMU gesture or terrain classification, sequence prediction, audio keyword) with baselines | A working end-to-end example with numbers to beat |
| **Distilled looming / motion detector** for the XIAO camera, int8, with a flyvis teacher | The vision path is the hardest to build in 5 hours |
| **Mushroom-body classifier** firmware with on-device one-shot learning | A demo of learning on a microcontroller that teams can extend |
| **Compass / path integrator** firmware using the Bittle IMU | Needed for any maze or homing project |
| **Bittle sim environment** and a pretrained walking policy | Lets teams without a robot in hand keep working |
| **Baselines** (MLP, CNN, GRU) trained on the same tasks, already exported for the device | Makes rule 2 cheap to satisfy |
| **Benchmark harness**: one command that flashes a model and reports accuracy, latency and memory | Makes the judging comparable |
| **Datasets recorded on the actual hardware**: XIAO camera clips of looming and non-looming objects, Bittle IMU traces on different floors, maze runs | Models pretrained on other cameras will not transfer |

## Fly pursuit (`flybrain_models.pursuit`)

Steers toward a target from its bearing (a camera's box centre). `PursuitCircuit` is a
rectified-linear rate network on the `pursuit` pack (LC10a, AOTU019, AOTU025, DNa02, the
types bridging LC10a to DNa02, TuTuAa, AOTU041). Each weight is a fractional input:
sign x synapse count / the postsynaptic neuron's total input synapses in the whole brain
(stored in the pack). Fixed choices, not fitted: tau 20 ms; LC10a input a Gaussian of the
bearing around each neuron's estimated receptive field (sd 10 deg, amplitude 1); a tonic
DNa02 bias of 1 so that AOTU019 inhibition can lower it; all other biases 0. Turn =
k_out x (DNa02 left - DNa02 right), and `k_out` is the only fitted number (a 1-D search, 17 s),
the same count as each baseline.

The network is run twice, once with the bearing and once with the mirrored bearing, and the
two turn commands are averaged with opposite sign (`mirror_average=True`). This is one fly's
brain and its halves are unequal (AOTU019 -> DNa02: 216 vs 121 synapses); run as a single
brain it holds the target about 16-17 deg to the right. `arousal` 0 adds an inhibitory input
of 0.05 to AOTU019, enough to silence it (a modelling choice after Collie et al. 2026).
Not modelled: direction selectivity, forward-speed gain, elevation, arousal dynamics.

`pursuit_benchmark` runs 114 held-out closed-loop episodes with three ASSUMED field-of-view
settings: +-30 and +-60 deg (a fixed camera) and +-100 deg (a +-30 deg camera on a neck
panning about +-70 deg; the head loop is not simulated). The rest of the rig is also
ASSUMED: 15 Hz camera, 100 ms delay, 2 deg noise, 5 % dropouts, 90 deg/s turn limit, 150 ms
lag. Mean absolute bearing error in degrees (`pursuit_benchmark.json`, 2026-11-16), every
model with one fitted number, the circuit variants all with the connectome's k_out:

| Model | All | +-30 | +-60 | +-100 | steps settled |
|---|---|---|---|---|---|
| Proportional, k fitted (baseline) | 13.80 | 12.53 | 9.34 | 19.06 | 1.00 |
| Connectome circuit (mirror-averaged) | 14.32 | 13.11 | 9.42 | 19.93 | 0.97 |
| Bang-bang, threshold fitted (baseline) | 14.53 | 13.02 | 10.31 | 19.80 | 0.95 |
| AOTU025 silenced | 14.50 | 13.20 | 9.46 | 20.32 | 0.97 |
| Connectome circuit, single brain | 31.09 | 45.20 | 20.62 | 27.74 | 0.10 |
| AOTU019 silenced (= arousal 0) | 36.10 | 52.69 | 25.31 | 30.78 | 0 |
| AOTU019 made excitatory | 52.93 | 71.43 | 41.87 | 46.10 | 0 |
| Rewired, 5 seeds | 53.47 | 48.77 | 52.64 | 58.53 | 0.18 |
| Random, 3 seeds | 57.78 | 59.43 | 54.00 | 59.72 | 0.01 |

What this shows:

- With one fitted number each, the circuit tracks about as well as a tuned proportional
  controller, and slightly worse (14.3 vs 13.8 deg). It settles after almost every step.
- The wiring matters: rewired and random graphs do not pursue.
- AOTU019 makes the fine frontal steering: without it (or at arousal 0, which silences it)
  a target within about 20 deg is turned away from, because the excitatory bridging types
  CB3127 and CB2070 cross to the opposite DNa02. An excitatory AOTU019 does the same.
- AOTU025 carries lateral steering: silencing it cuts the open-loop turn for targets at
  60-90 deg to a third to two fifths, but costs little error in closed loop (0.4 deg at
  +-100, under 0.1 at +-30 and +-60), because what is left (120-170 deg/s) is still above
  the robot's 90 deg/s turn limit.

An earlier version fitted 13 global scalars (per-class gains, biases, tau, gains). It reached
a similar error only by becoming a saturated bang-bang controller in which the lateral
AOTU025 pathway did nothing, and one of its fits learned to spin toward one side whenever it
saw no target.

## Sources

- flyvis: https://github.com/TuragaLab/flyvis
- Whole-brain model: https://github.com/philshiu/Drosophila_brain_model
- Connectome reservoir, positive result: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12109256/
- Connectome reservoir, negative results: https://pure.ulster.ac.uk/en/publications/random-and-biological-network-connectivity-for-reservoir-computin/ and https://world-wide.org/conf/cosyne-25/sparse-neural-engagement-connectome-h7r1dty6
- FlyGM: https://arxiv.org/abs/2602.17997
- conn2res: https://github.com/netneurolab/conn2res
