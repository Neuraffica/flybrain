# flybrain-data

FlyWire downloads, circuit packs and null-model controls. A circuit pack is a small signed
weight matrix plus a neuron table, shipped inside this package as one `.npz` so nothing has
to be downloaded to use it.

```python
from flybrain_data import load_pack

pack = load_pack("central_complex")                      # FlyWire v783 connectome
pack.W            # (151, 151) float32 signed weights, W[post, pre]
pack.table()      # root_id, cell_type, side, nt, sign, ring_angle, degrees (pandas)
rew = load_pack("central_complex", "rewired", seed=1)    # control
rnd = load_pack("central_complex", "random", seed=1)     # control
```

```bash
uv run flybrain-data info central_complex          # summary + attribution
uv run flybrain-data info central_complex --json   # full metadata
```

## The `central_complex` pack

Head-direction ring of the fly central complex, from FlyWire FAFB proofread connections v783.

| cell_type (exact string) | neurons | left | right | sign |
|---|---|---|---|---|
| `EPG` | 47 | 24 | 23 | + (acetylcholine) |
| `PEN_a(PEN1)` | 20 | 10 | 10 | + |
| `PEN_b(PEN2)` | 22 | 11 | 11 | + (one neuron predicted serotonin, treated as +) |
| `PEG` | 20 | 9 | 11 | + |
| `Delta7` | 42 | 21 | 21 | - (glutamate; one neuron predicted acetylcholine, kept +) |

151 neurons, 2,665 connected pairs, 37,471 synapses after the threshold.

- **Weight** = synapse count summed over all neuropils for each (pre, post) pair, times the
  sign of the presynaptic neuron's predicted transmitter (`top_nt`): acetylcholine +1, GABA
  and glutamate -1, anything else +1 (Shiu et al. 2024 convention).
- **Threshold**: pairs with fewer than 5 synapses in total are dropped (6,372 pairs and
  44,887 synapses before, 2,665 and 37,471 after).
- **Per-neuropil split**: `pack.by_neuropil` (EB, PB, NO, other) sums exactly to `counts`.
  The metadata also holds, per cell type and neuropil, the fraction of the type's synapses
  that are inputs (from the FlyWire per-neuron neuropil tables), e.g. PEN: 0.98 in the
  protocerebral bridge, 0.3 in the ellipsoid body.
- **Side** is the soma side from the annotation table, which for these types is the
  protocerebral-bridge side. PEN left/right is what the compass model uses to give the two
  PEN populations opposite-sign turn input.

### Ring angles are inferred, not labelled

FlyWire v783 annotations (`cell_type`, `hemibrain_type`, `synonyms`, ...) carry no
protocerebral-bridge glomerulus or ellipsoid-body wedge for EPG neurons; the hemibrain and
male-CNS instance names that do (e.g. `EPG(PB08)_L3`) belong to other animals' body IDs and
cannot be mapped to FlyWire root IDs without a matching step. So each EPG's ring angle is
**inferred from connectivity**: spectral circular seriation of the EPG partner profiles
(cosine similarity of sqrt synapse counts to all pack neurons, normalised Laplacian,
eigenvectors 2 and 3), with angles set to evenly spaced ranks. Angles are defined up to a
rotation and a reflection. Checks stored in `metadata["ring_angle"]`:

- the ring recovered from ellipsoid-body synapses alone and from bridge synapses alone agree
  (circular correlation 0.98), and each agrees with the full ring (0.996, 0.986);
- left-bridge and right-bridge EPGs each cover the whole circle (resultant length 0.07,
  largest gap 38 deg), as expected because EB wedges alternate between the two sides;
- measured on the inferred ring, left PENs send their ellipsoid-body output about +55-60 deg
  away from the EPGs that feed them in the bridge, and right PENs about -55-60 deg: the
  known PEN offset that lets PENs rotate the bump (a consistency check, not an independent
  label, since the seriation also used these synapses).

## Controls

Both are seeded and deterministic (`load_pack(name, variant, seed)`).

- `rewired`: degree-preserving double-edge swaps (20 attempted per edge) separately inside
  every pre-type -> post-type block. Each neuron keeps its in- and out-degree in every block,
  each presynaptic neuron keeps its own synapse counts (so its sign), and every block keeps
  its edge and synapse totals. About 95% of edges move.
- `random`: same number of neurons, edges from excitatory and from inhibitory neurons (so
  density and edge sign ratio), the same synapse-count values, neuron signs unchanged; edges
  placed uniformly at random without self-loops.

## Rebuilding the pack

```bash
uv run flybrain-data download            # ~1.1 GB into ./data (resumable, md5-checked)
uv run flybrain-data build-pack central_complex
```

`download` fetches `proofread_connections_783.feather` (852 MB) from
https://zenodo.org/records/10676866, `Supplemental_file1_neuron_annotations.tsv` from
https://github.com/flyconnectome/flywire_annotations, and two optional per-neuron neuropil
count tables (250 MB, `--required-only` skips them). It never fetches the 9.5 GB synapse
table. Set `FLYBRAIN_DATA` to use another directory.

## Credit

FlyWire connectome v783, CC BY 4.0. Dorkenwald et al. 2024, *Neuronal wiring diagram of an
adult brain*, Nature 634:124-138. Schlegel et al. 2024, *Whole-brain annotation and
multi-connectome cell typing of Drosophila*, Nature 634:139-152. Neurotransmitter predictions:
Eckstein et al. 2024, Cell 187:2574-2594.
