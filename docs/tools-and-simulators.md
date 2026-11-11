# Tools and simulators

Existing open software the starter kit builds on or points to.
Researched 2026-11-11. **(verify)** marks details not confirmed on a primary page.

## Connectome access

| Tool | Use |
|---|---|
| pandas + scipy.sparse | Enough for the Zenodo Feather edge list. This is what `flybrain-data` uses |
| [fafbseg](https://github.com/navis-org/fafbseg-py) | FlyWire helpers, can fetch release files |
| [navis](https://github.com/navis-org/navis) | Neuron morphology, plotting |
| [neuprint-python](https://github.com/connectome-neuprint/neuprint-python) | Query MANC, optic lobe, male CNS (token needed) |
| [Codex](https://codex.flywire.ai) | Browse circuits in the browser; good for finding cell types |

No ready-made adjacency-matrix package was found on Hugging Face or Kaggle, so shipping
circuit packs is real added value.

## Models built from the connectome

| Project | What it is | Stack | Notes |
|---|---|---|---|
| [flyvis](https://github.com/TuragaLab/flyvis) | Connectome-constrained model of the fly visual motion pathway, 64 cell types (Lappalainen et al. 2024) | PyTorch, MIT, `pip install flyvis` | Ships pretrained models. Too big for an MCU as is; good teacher for distillation |
| [Drosophila_brain_model](https://github.com/philshiu/Drosophila_brain_model) | Whole-brain spiking model, ~130k neurons (Shiu et al. 2024) | Brian2 | Activate sensory neurons, read out descending neurons |
| [eonsystemspbc/fly-brain](https://github.com/eonsystemspbc/fly-brain) | Same model with Brian2CUDA, PyTorch, NEST GPU and GeNN backends | CUDA recommended | From Eon Systems, who drove a simulated fly body with it in 2026 |
| [flycns](https://pypi.org/project/flycns/) | Standalone PyTorch/NumPy port of the whole-brain model | MIT | Reported to run on an Apple-silicon laptop **(verify)** |
| [conn2res](https://github.com/netneurolab/conn2res) | Reservoir computing on any connectome matrix | Python, BSD-3 | Alpha quality. Useful reference for tasks and metrics |
| FlyGM (arXiv 2602.17997) | Graph network over the whole FlyWire graph, trained with RL for locomotion | | Public code not confirmed **(verify)** |

## Body and robot simulators

| Simulator | Simulates | Stack | Notes |
|---|---|---|---|
| [flygym / NeuroMechFly v2](https://github.com/NeLy-EPFL/flygym) | The fly body with vision and olfaction | MuJoCo, `pip install flygym` | v2 dropped the Gymnasium API; `flygym-gymnasium` keeps the old one |
| [flybody](https://github.com/TuragaLab/flybody) | Fly body for walking and flight | MuJoCo, Apache-2.0 | Janelia + DeepMind |
| [opencat-gym](https://github.com/ger01d/opencat-gym) | Bittle | PyBullet + Stable-Baselines3 PPO | Has a [sim-to-real fork](https://github.com/ger01d/opencat-gym-sim2real) that streams joint angles to a BiBoard |
| [Bittle_URDF](https://github.com/AIWintermuteAI/Bittle_URDF) | Bittle URDF and meshes | Isaac Gym | Older; needs tuning |
| Petoi online simulator | Bittle X skills in the browser | | Linked from the OpenCat repo |
| [Wokwi](https://docs.wokwi.com/guides/esp32) | XIAO ESP32-S3 firmware | Browser / VS Code | No Bluetooth, no camera, slow CPU. Good for testing C inference code only |

No MuJoCo model of Bittle was found. The starter kit plan is to wrap the PyBullet Bittle
in a small Gymnasium environment (`sim/`) with the same command interface as the real dog,
so a controller can be swapped between sim and hardware with one flag.

## Edge deployment

| Tool | Use |
|---|---|
| TFLite Micro + ESP-NN | Int8 CNNs and dense nets on the ESP32-S3 |
| Edge Impulse | No-code path from data to firmware; Seeed supports the XIAO directly |
| Plain C export (planned, `flybrain-edge`) | Sparse recurrent nets as a header file with CSR int8 weights. TFLite Micro handles sparse recurrent models poorly, so this is the main path for reservoirs |
| PlatformIO / Arduino | Firmware build for the XIAO |
| OpenCat `ardSerial.py`, `PetoiRobot` | Laptop control of Bittle over USB serial |
