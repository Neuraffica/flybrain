# flybrain

Starter kit for **Fly Brain & Tiny Machines**, a one-day BioCompute hackathon by [Neuraffica](https://neuraffica.com) in Berkeley, CA (Dec 5, 2026). Event page: https://luma.com/berkeley-1yw9

The challenge: use the *Drosophila* (fruit fly) connectome to make a machine see, move or decide better at the edge, and measure it.

The rules:

1. **The brain has to matter.** Fly connectivity must shape your model.
2. **Compare against a standard baseline** and show the numbers.
3. **Run it on a device**, ours or yours.

> Status: repository scaffold and planning docs. The packages, firmware and tutorials below are outlined but not implemented yet.

## Start here

| Read | For |
|---|---|
| [docs/hardware.md](docs/hardware.md) | What the XIAO ESP32-S3 and Petoi Bittle X can do, how to control them, and where the model should run |
| [docs/data.md](docs/data.md) | Which connectome datasets to use and which circuits to extract |
| [docs/tools-and-simulators.md](docs/tools-and-simulators.md) | Existing models, simulators and deployment tools |
| [docs/models.md](docs/models.md) | Model families that fit the rules, required controls, and what gets pretrained |
| [docs/project-ideas.md](docs/project-ideas.md) | Projects sized for 5 hours, and the maze |
| [docs/demo-plan.md](docs/demo-plan.md) | Plan for the reference demo: a robot dog walking a maze on fly circuits |
| [tutorials/](tutorials/README.md) | Tutorial outline |

## Layout

| Path | Contents |
|---|---|
| `packages/flybrain-data` | Download connectome data, extract circuits as small signed sparse matrices ("circuit packs"), build rewired and random controls |
| `packages/flybrain-models` | Connectome reservoirs, hand-wired fly circuits (motion, looming, compass, mushroom body), gait generator, baselines |
| `packages/flybrain-edge` | Export models to C for the ESP32-S3, robot link (USB / BLE / UART), on-device benchmark harness |
| `firmware/xiao-esp32s3` | Firmware for the XIAO ESP32-S3 Sense: camera, inference runtime, link to the robot |
| `firmware/bittle` | Notes and patches for Petoi OpenCat firmware on the Bittle X |
| `sim/` | Bittle simulation environment with the same command interface as the real robot |
| `models/` | Pretrained model zoo (weights are released as downloads, not committed) |
| `demos/` | Reference demos |
| `tutorials/` | Step-by-step notebooks and guides |
| `tools/` | Scripts for organizers: dataset recording, device flashing |
| `data/` | Local cache for downloaded connectome files (gitignored) |

## Setup

Requires [uv](https://docs.astral.sh/uv/) and Python 3.11 or newer.

```bash
uv sync
```

## Data credit

Connectome data comes from the FlyWire consortium (Dorkenwald et al. 2024; Schlegel et al. 2024) and Janelia FlyEM, under CC BY 4.0. See [docs/data.md](docs/data.md).

## License

Copyright (c) 2026 Neuraffica, Inc.

Licensed under the [PolyForm Noncommercial License 1.0.0](LICENSE.md). You may use, modify and share this code for noncommercial purposes, including research and hackathon projects. For commercial use, contact Neuraffica.
