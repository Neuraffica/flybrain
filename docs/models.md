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

## Sources

- flyvis: https://github.com/TuragaLab/flyvis
- Whole-brain model: https://github.com/philshiu/Drosophila_brain_model
- Connectome reservoir, positive result: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12109256/
- Connectome reservoir, negative results: https://pure.ulster.ac.uk/en/publications/random-and-biological-network-connectivity-for-reservoir-computin/ and https://world-wide.org/conf/cosyne-25/sparse-neural-engagement-connectome-h7r1dty6
- FlyGM: https://arxiv.org/abs/2602.17997
- conn2res: https://github.com/netneurolab/conn2res
