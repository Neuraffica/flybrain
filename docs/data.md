# Fly connectome data

Open datasets, where to get them, and which circuits are worth extracting.
Researched 2026-11-11. **(verify)** marks details not confirmed on a primary page.

## Datasets

"Connections" below are neuron-pair edges, not synapses. Counts are from the FlyWire Codex
download page.

| Dataset | Covers | Neurons | Connections | Get it | License |
|---|---|---|---|---|---|
| **FlyWire FAFB v783** | Whole female brain | 139,255 | 3.7 M | Zenodo, no login | CC BY 4.0 |
| **MANC v1.2.1** | Male nerve cord (leg and wing motor circuits) | 23,665 | 5.3 M | Janelia public bucket, neuPrint | CC BY |
| **Male CNS v1.0** | Male brain + nerve cord | 166,700 | 6.2 M | male-cns.janelia.org, no login | CC BY 4.0 |
| **BANC v888** | Female brain + nerve cord | 158,262 | 3.0 M | Harvard Dataverse, Codex | CC BY 4.0 |
| **Optic lobe v1.1** | Male right optic lobe | 52,445 | 6.5 M | Janelia public bucket, neuPrint | CC BY |
| Hemibrain v1.2.1 | Part of central brain | ~25,000 | | neuPrint (account needed) | CC BY |
| Larva (Winding 2023) | Whole larval brain | ~3,000 | | **(verify)** | **(verify)** |

Codex (codex.flywire.ai) hosts most of these in one place but needs a Google sign-in.

### What the starter kit will ship

The starter kit default is **FlyWire FAFB v783**, because it is the best annotated:

- `proofread_connections_783.feather`, 852 MB, the edge list with synapse counts per
  neuropil and predicted neurotransmitter. From https://zenodo.org/records/10676866
- `Supplemental_file1_neuron_annotations.tsv`, cell type, class, side and neurotransmitter
  for every neuron. From https://github.com/flyconnectome/flywire_annotations

Optional extras, fetched by the same `uv run flybrain-data download` (or `--only <name>`):

- `per_neuron_neuropil_count_{pre,post}_783.feather`, 250 MB, synapse counts per neuron and
  neuropil (Zenodo record above). Used only for a metadata table in the compass pack.
- `extra/column_assignment.csv.gz`, 0.5 MB, the eye column (`x`, `y`, `p`, `q`) of about
  45,500 columnar optic-lobe neurons (Tm20, Tm4, T2a, Tm3, ...). From the FlyWire Codex public
  bucket,
  https://storage.googleapis.com/flywire-data/codex/data/fafb/783/column_assignment.csv.gz
  (listed on https://codex.flywire.ai/api/download). Matsliah et al. 2024, Nature 634:166-180,
  CC BY 4.0. `x` runs -9..8 in each eye, +x anterior. Needed only to build the `pursuit` pack.

Skip `flywire_synapses_783.feather` (9.5 GB). Nobody needs per-synapse coordinates in 5 hours.

For gait work add **MANC**. For anything that needs brain and cord in one graph (sensor to
motor neuron), use **Male CNS** connection weights (1.1 GB Feather).

Plan: `flybrain-data` downloads these once, caches under `data/` (gitignored), and exposes
circuits as small sparse matrices. Organizers should also bring the files on USB sticks; a
room of people pulling 850 MB at 11:00 will not go well.

Cite Dorkenwald et al. 2024 and Schlegel et al. 2024 (Nature) when using FlyWire data.

## Circuits worth extracting

Filter the annotation table by `cell_type` to get these. Each becomes a "circuit pack": a
signed sparse weight matrix plus a table of neuron ids, types and input/output roles.

| Circuit | Cell types | What it computes | Use on a robot |
|---|---|---|---|
| Motion detection | T4a-d, T5a-d, Mi1, Tm3, Mi4, Mi9, Tm1, Tm2, Tm4, Tm9, LPTCs (HS, VS) | Local direction of motion in 4 cardinal directions; wide-field optic flow | Optic-flow steering, corridor centring |
| Looming and escape | LPLC2, LC4, DNp01 (giant fiber) | Detects an expanding object on a collision course | Dodge, brake, obstacle stop |
| Steering | DNa01, DNa02 | Turn commands from brain to legs | Map to left/right turn skills |
| Walking control | DNp09 (forward), MDN (backward), BPN | Start, stop and reverse walking | Map to walk/back/stop |
| Heading and path integration | EPG, PEN, PFN, hDelta, FC (central complex) | Ring attractor compass; integrates steps into a home vector | Dead reckoning, maze homing |
| Learning | Kenyon cells, MBON, DAN, APL (mushroom body) | Sparse random expansion plus one-shot associative learning | On-device learning, place and route memory |
| Leg premotor | MANC leg motor neurons and premotor interneurons | Rhythm and inter-leg coordination | Gait pattern generator |

### Shipped pack: `pursuit` (small-object pursuit)

`uv run flybrain-data build-pack pursuit` writes `packs/pursuit.npz` (18 KB): 265 neurons,
1842 edges, 34,984 synapses (pairs with >= 5 synapses). Cell types: LC10a (115 left, 119
right), AOTU019 and AOTU025 (one per side), DNa02 (one per side), the types bridging LC10a to
DNa02 by the rule ">= 500 synapses from LC10a and >= 150 onto DNa02, whole brain, edges >= 5"
(AOTU015a, CB3127, CB2070, LAL027; AOTU019 and AOTU025 also pass), and TuTuAa and AOTU041
(local inhibition). The build checks, and stores in the metadata:

- LC10a -> AOTU025 and LC10a -> AOTU019 are same-side only (1427/1345 and 1302/1556
  synapses, left/right); AOTU025 -> DNa02 is same-side (167/154), AOTU019 -> DNa02 is
  opposite-side (216 left->right, 121 right->left).
- Each LC10a gets a receptive-field azimuth (`pack.azimuth`, radians, + = left), the
  synapse-weighted mean column `x` of its column-assigned inputs mapped linearly to azimuth.
  The map is an assumption: x = 8 -> 15 deg across the midline, x = -9 -> 160 deg to that
  eye's side. 232 of 234 LC10a have an estimate (median 23 column-assigned input synapses).
- Each neuron's total input synapses in the whole brain (`pack.input_total`, all partners,
  no threshold), so that a weight can be expressed as a fraction of the neuron's input.
- The LC10a input to AOTU019 is frontal (synapse-weighted mean 33.7 deg left, -34.0 deg
  right) and to AOTU025 lateral (75.8 deg, -74.0 deg), in line with the receptive fields
  reported by Collie et al. 2026 (peaks about 26 and 72 deg).

Sign convention used by published whole-brain models (Shiu et al. 2024): acetylcholine is
excitatory; GABA and glutamate are inhibitory. Weight = sign x synapse count.

## Sources

- https://codex.flywire.ai/api/download
- https://zenodo.org/records/10676866
- https://github.com/flyconnectome/flywire_annotations
- https://www.janelia.org/manc-connectome
- https://male-cns.janelia.org/download/
- https://github.com/natverse/bancr
- https://doi.org/10.1038/s41586-025-08746-0 (optic lobe, Nern et al. 2025)
- https://join.flywire.ai/guidelines
