# flybrain-models

Models built on `flybrain-data` circuit packs. So far: the **fly compass**, a head-direction
ring attractor on the FlyWire central-complex connectome, plus an engineering baseline.

```python
import math
from flybrain_models.compass import RingAttractor, GyroIntegrator

fly = RingAttractor.from_pack("central_complex")                  # connectome
ctrl = RingAttractor.from_pack("central_complex", "rewired", seed=1)
base = GyroIntegrator()                                           # complementary filter
for m in (fly, ctrl, base):
    m.reset(0.0)
h = fly.step(math.radians(90), 0.02, cue_rad=0.4)   # gyro rad/s (CCW +), dt s, landmark or None
h = fly.step(math.radians(90), 0.02)                # no cue sample this tick
fly.bump, fly.bump_angles, fly.confidence           # EPG activity, angles (heading frame), [0, 1]
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
