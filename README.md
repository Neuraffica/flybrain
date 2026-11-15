# flybrain

Starter kit for **Fly Brain & Tiny Machines**, a one-day BioCompute hackathon by [Neuraffica](https://neuraffica.com) in Berkeley, CA (Dec 5, 2026). Event page: https://luma.com/berkeley-1yw9

The challenge: use the *Drosophila* (fruit fly) connectome to make a machine see, move or decide better at the edge, and measure it.

The rules:

1. **The brain has to matter.** Fly connectivity must shape your model.
2. **Compare against a standard baseline** and show the numbers.
3. **Run it on a device**, ours or yours.

Using this kit is optional: you can enter the hackathon with your own code and hardware.

> Status: four demos, all of which run in the simulator: [Fly compass](demos/fly-compass/README.md), [Fly escape](demos/fly-escape/README.md), [Fly escape (on the robot)](demos/fly-escape-onboard/README.md) and [Hello robot](demos/hello-robot/README.md). Each README has a "checked on hardware, and not" section; parts of them have run on one Bittle X+Arm over USB and Bluetooth, not every mode. The robot's own escape module is in [firmware/bittle/fly_escape](firmware/bittle/fly_escape/README.md). The simulator package (`sim/`), the XIAO firmware and the tutorials are outlined but not written yet.

## Run the demos: Flydeck for Mac

**Flydeck** is a Mac app: pick a robot (Bluetooth or USB cable) or the simulator, pick a fly-brain demo, press Start. It needs a Mac with Apple Silicon (M1 or newer) and macOS 12 or newer; it has been tried on one Mac only so far.

1. Download `Flydeck-<version>-macos-arm64.dmg` from the [latest release](https://github.com/Neuraffica/flybrain/releases/latest), open it and drag **Flydeck** to **Applications**.
2. Open Flydeck. The first time, macOS refuses: the app is not notarized by Apple, so macOS cannot verify it. Click **Done** (not Move to Trash), open **System Settings › Privacy & Security**, scroll down to the line about Flydeck, click **Open Anyway** and confirm. If macOS instead says the app "is damaged", or you prefer Terminal, run this once and open it again:

   ```bash
   xattr -dr com.apple.quarantine /Applications/Flydeck.app
   ```

3. Flydeck opens in its own window. Switch the robot on, press **Scan**, click **Allow** when macOS asks whether Flydeck may use Bluetooth, and press **Use** next to your robot (or plug in the USB cable and press **Use** under *USB cable*). No robot? Tick **Try in simulator** on any demo.
4. Click a demo card, choose its settings, press **Start**. Its dashboard appears in the window; **Stop** (or quitting Flydeck) lays the robot down to rest.

The [Flydeck guide](apps/flydeck/guide.html) (also under **Guide** in the app) covers each step and troubleshooting.

## Build your own demo

You need [uv](https://docs.astral.sh/uv/) and git. In Terminal:

```bash
git clone https://github.com/Neuraffica/flybrain.git
cd flybrain
uv sync
uv run flydeck new-demo my-idea     # copies the hello-robot template to demos/my-idea
uv sync                             # installs it
```

Then, in Flydeck, press **Use my flybrain folder…** above the demo cards and choose the `flybrain` folder. Your demo appears as a card next to the others and runs with your folder's Python, with Flydeck's Bluetooth permission. After `uv sync` press **Reload list**; **Use the built-in demos** switches back. The rules and the robot API are in [docs/adding-a-demo.md](docs/adding-a-demo.md).

### Share your demo

Send your demo back as a pull request, so the next team can start from it. This is optional.

1. Fork this repository on GitHub and push your branch to the fork.
2. Open a pull request against `main`. Keep it to your own folder under `demos/` (and the one line `uv sync` adds to `uv.lock` for it).
3. In the description, say which device it runs on, what the fly wiring does in your model, and your numbers against the baseline.
4. Put the pull request number in the hackathon submission form.

## No Mac, or an Intel Mac

There is no download for Windows, Linux or Intel Macs. Clone the repository and run Flydeck from the folder:

```bash
uv sync
uv run flydeck            # opens in a window on a Mac, in the web browser elsewhere
```

USB and the simulator work this way. On macOS, Bluetooth needs the app: macOS stops a program started from Terminal when it touches Bluetooth. Each demo also runs on its own, e.g. `uv run fly-compass --mock --open`.

To build the app yourself on an Apple Silicon Mac: `uv run flydeck build-app` writes `dist/Flydeck.app` and `dist/Flydeck-<version>-macos-arm64.dmg` (`install-app` builds it and puts it in `~/Applications`).

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
| `demos/` | Reference demos, and `hello-robot`, the template for new ones |
| `apps/flydeck` | Flydeck, the app that finds the robot and starts demos (`uv run flydeck build-app` builds the Mac app) |
| `packages/flybrain-demo-api` | The small API a demo uses to appear in Flydeck (no dependencies) |
| `tutorials/` | Step-by-step notebooks and guides |
| `tools/` | Scripts for organizers: dataset recording, device flashing |
| `data/` | Local cache for downloaded connectome files (gitignored) |

## Setup (command line)

Requires [uv](https://docs.astral.sh/uv/) and Python 3.11 or newer.

```bash
uv sync
uv run fly-compass --mock   # the demo on a simulated dog; dashboard at http://127.0.0.1:8765
```

## Data credit

Connectome data comes from the FlyWire consortium (Dorkenwald et al. 2024; Schlegel et al. 2024) and Janelia FlyEM, under CC BY 4.0. See [docs/data.md](docs/data.md).

## License

Copyright (c) 2026 Neuraffica, Inc.

Licensed under the [PolyForm Noncommercial License 1.0.0](LICENSE.md). You may use, modify and share this code for noncommercial purposes, including research and hackathon projects. For commercial use, contact Neuraffica.
