# fly_escape: the looming-escape circuit on the Bittle's own board (draft)

> **Status: draft. Not compiled for the ESP32, not flashed, not run on a robot.** There was no
> ESP32 toolchain (arduino-cli / PlatformIO / Arduino IDE core) on the machine this was written
> on. What *has* been checked on a laptop:
>
> - the generated model (`src/fly_escape_all.h`) compiles warning-free as C99 and C++11 with
>   the host compiler;
> - compiled C matches the Python model tick by tick: same trigger ticks and sides on both real
>   recordings and on all 384 simulated benchmark episodes (42,796 steps, 0 mismatches,
>   giant-fiber activity within 2e-6);
> - `src/flyEscape.h` compiles against host stand-ins for the Arduino/OpenCat symbols and queues
>   exactly one escape for a simulated hand on the left channel
>   (`packages/flybrain-edge/tests/test_export_escape.py`).

What it does: an OpenCatEsp32 extension module (letter **E**). When enabled it reads the Petoi
double IR distance sensor on GPIO 36 (`ANALOG3`, treated as left) and GPIO 39 (`ANALOG4`, right)
every time the firmware polls modules, averages the readings, and every 35 ms runs one step of
the FlyWire looming-escape circuit (LC4, LPLC2 -> giant fibers; 351 neurons, 2,212 synapse
weights). On a trigger it queues `bk` (walk backward, 1 s) and `up` (stand) through the
firmware's task queue, at most once every 3 s. It calibrates the empty-scene level at
activation, so keep hands away from the sensor for the first ~0.15 s after enabling.

Like every OpenCat sensor module it only runs while the robot is idle (the firmware polls
modules when no command is executing), and it does not go through the serial "read analog
pin" path that stops a running gait when the laptop demo polls the sensor.

## Files

| File | What |
|---|---|
| `src/fly_escape_all.h` | Generated model (single header, all functions `static inline`, data `static const` in flash). Regenerate: `uv run flybrain-export escape --single-header --out firmware/bittle/fly_escape/src` |
| `src/flyEscape.h` | The OpenCat module: setup/calibration, polling, stepping, queueing the escape |

To use the looming-rate baseline instead of the connectome model, add
`#define FLY_ESCAPE_MODEL FE_MODEL_LOOMING` before `#include "flyEscape.h"`.

## Adding it to OpenCatEsp32

Upstream: https://github.com/PetoiCamp/OpenCatEsp32 (checked against `main` on 2026-11-11;
line positions may move).

1. Copy `src/flyEscape.h` and `src/fly_escape_all.h` into `OpenCatEsp32/src/`.
2. `src/OpenCat.h`, next to the other `EXTENSION_*` defines (`E` is not used by any of
   `S A T L D I B U G C Q`):

   ```c
   #define EXTENSION_FLY_ESCAPE 'E'  // fly looming escape, connect IR sensor to ANALOG3, ANALOG4
   ```

   and append one entry to each of the parallel module arrays (they must stay the same length):

   ```c
   int8_t moduleList[] = { ..., EXTENSION_QUICK_DEMO, EXTENSION_FLY_ESCAPE, };
   String moduleNames[] = { ..., "Quick_Demo", "Fly_Escape" };
   bool moduleActivatedQ[] = { ..., 0 };   // every board variant's list: add one trailing 0
   ```

3. `OpenCatEsp32.ino` (or wherever the other module switches such as `DOUBLE_INFRARED_DISTANCE`
   are defined): `#define FLY_ESCAPE`. Do not enable `DOUBLE_INFRARED_DISTANCE`'s module at the
   same time: both use pins 36/39.
4. `src/moduleManager.h`:

   ```c
   #ifdef FLY_ESCAPE                       // with the other module #includes at the top
   #include "flyEscape.h"
   #endif
   ```

   in `initModule()`:

   ```c
   #ifdef FLY_ESCAPE
       case EXTENSION_FLY_ESCAPE:
         {
           loadBySkillName("up");
           flyEscapeSetup();
           break;
         }
   #endif
   ```

   in `stopModule()`:

   ```c
   #ifdef FLY_ESCAPE
       case EXTENSION_FLY_ESCAPE:
         {
           flyEscapeStop();
           break;
         }
   #endif
   ```

   in `read_serial()` next to the other module polls (`read_doubleTouch()` etc.):

   ```c
   #ifdef FLY_ESCAPE
       if (moduleActivatedQ[indexOfModule(EXTENSION_FLY_ESCAPE)])
         read_flyEscape();
   #endif
   ```

   Optionally add `|| moduleActivatedQfunction(EXTENSION_FLY_ESCAPE)` to the `moduleDemoQ`
   expression, as the other sensor demos do.

5. Enable on the robot with the serial command `XE`; `Xe` disables it; `X?` shows module status.
   The module state is saved, so it will be active again after a reboot until disabled.

## Build and flash (Petoi's documented route; untested here)

Follow Petoi's guide for uploading OpenCatEsp32 to a BiBoard (docs.petoi.com, "Upload sketch
for BiBoard"): Arduino IDE with Espressif's **esp32** board package at the version that guide
specifies, board **ESP32 Dev Module**, and the partition scheme and other board settings that
guide lists **(verify on the page; not checked here)**. In `OpenCatEsp32.ino` select
`#define BITTLE` and `#define BiBoard_V1_0`. Compile first (Verify) and note the sketch size
reported with and without `#define FLY_ESCAPE`; then Upload over USB.

With arduino-cli the equivalent is roughly (untested):

```bash
arduino-cli core install esp32:esp32           # at the version Petoi's guide names
arduino-cli compile --fqbn esp32:esp32:esp32 OpenCatEsp32      # compile only
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbmodemXXXX OpenCatEsp32
```

## Restoring the stock firmware

Use the Petoi Desktop App's **Firmware Uploader** (choose the Bittle and BiBoard V1 and the
standard mode), or rebuild an unmodified OpenCatEsp32 checkout with the same settings. Module
settings live in the board's saved config; after reflashing stock firmware run `X` to close all
modules if the old state is still listed.

## Expected cost on the BiBoard (estimate, not measured)

16 KB of const model data in flash, about 1.5 KB of RAM for the model state plus 2.8 KB of
static scratch. Per 35 ms step: about 8,800 multiply-adds, 1 `expf`, and on average about 130
`tanhf` calls (up to about 1,400 in the busiest steps). Estimated 0.5 ms per step on average and
about 2 ms in the worst case at 240 MHz, i.e. roughly 1.5-6% of one core at 28 Hz. See
`packages/flybrain-edge/src/flybrain_edge/export/` and the report for the derivation.
