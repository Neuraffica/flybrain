# fly_escape: the looming-escape circuit on the Bittle's own board

> **Status: compiles for the BiBoard V1 (compile-only, 2026-11-11). Not flashed, not run on a
> robot.** Built with arduino-cli 1.6.0 and Espressif's esp32 core **2.0.12** (the version
> Petoi's BiBoard guide requires), board `ESP32 Dev Module`, partition scheme *Minimal SPIFFS
> (1.9MB APP with OTA)*, Bittle + `ROBOT_ARM` + `BiBoard_V1_0`, against two OpenCatEsp32
> revisions, each first unmodified and then with this module applied exactly as described below:
>
> | OpenCatEsp32 revision | | program flash | static RAM |
> |---|---|---|---|
> | `c481e47` (2025-04-08, `DATE "250330"`, model "Bittle X+Arm", what the robot runs) | stock | 1,299,929 B (66%) | 50,484 B (15%) |
> | | with fly_escape | 1,320,173 B (67%) | 54,772 B (16%) |
> | | **module cost** | **+20,244 B** | **+4,288 B** |
> | `main` `f5aba6b` (2026-10-24) | stock | 1,766,401 B (89%) | 70,876 B (21%) |
> | | with fly_escape | 1,786,581 B (90%) | 75,164 B (22%) |
> | | **module cost** | **+20,180 B** | **+4,288 B** |
>
> App partition 1,966,080 B: headroom with the module is 645,907 B (c481e47) and 179,499 B
> (main). No compiler warnings from this module's files (the only warnings in both builds come
> from the stock `Seeed_Arduino_SSCMA` library). The model tables land in flash-mapped rodata
> (`FE_VAL` 8,848 B, `FE_COL_IDX` 4,424 B at 0x3F40xxxx); `read_flyEscape` (with `fe_step`
> inlined) is 1,532 B of code. Also checked on a laptop: the generated C matches the Python
> model tick by tick on both real recordings and 384 simulated episodes (0 trigger or side
> mismatches, giant-fiber activity within 2e-6), and `src/flyEscape.h` queues exactly one
> escape for a simulated left-side hand (`packages/flybrain-edge/tests/test_export_escape.py`).

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

## Build (as verified, compile only)

Petoi's guide for BiBoard (docs.petoi.com, "Upload sketch for BiBoard") asks for the esp32 core
**2.0.12** ("2.0.13 and above may cause the mainboard to fail to start up"), the extra line
`#define CONFIG_DISABLE_HAL_LOCKS 1` at the end of
`<core>/tools/sdk/esp32/qio_qspi/include/sdkconfig.h`, board **ESP32 Dev Module**, and
partition scheme **Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)**; libraries ArduinoJson,
WebSockets, WiFiManager (Arduino library manager) and MuVisionSensor3
(https://github.com/mu-opensource/MuVisionSensor3). Verified with ArduinoJson 7.4.3, WebSockets
2.7.2, WiFiManager 2.0.17, MU Vision Sensor 3 1.2.2.

In `OpenCatEsp32.ino`: `#define BITTLE`, `#define BiBoard_V1_0` (other board lines commented),
and `#define ROBOT_ARM` for a Bittle X+Arm. Apply the module (section above). Then:

```bash
arduino-cli core install esp32:esp32@2.0.12 \
  --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli lib install ArduinoJson WebSockets WiFiManager
arduino-cli config set library.enable_unsafe_install true   # needed for the next line only
arduino-cli lib install --git-url https://github.com/mu-opensource/MuVisionSensor3.git
# then append the CONFIG_DISABLE_HAL_LOCKS line to sdkconfig.h as above
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs \
  --build-path build/fly_escape OpenCatEsp32
```

On an Apple Silicon Mac **without Rosetta**, two x86-only tools in that toolchain fail ("Bad CPU
type"): `esptool` 4.5.1 and Arduino's `ctags` 5.8-arduino11. The verified build replaced them
without installing Rosetta: `esptool==4.5.1` from PyPI in a virtualenv, and Homebrew
`universal-ctags` behind a 3-line shim that renames its `typeref:typename:` field to the
`returntype:` field Arduino's prototype generator expects:

```bash
--build-property "tools.esptool_py.path=<venv>/bin" --build-property "tools.esptool_py.cmd=esptool.py" \
--build-property "tools.ctags.path=<dir containing the shim named ctags>"
```

(With Rosetta installed, or in the Arduino IDE on Intel/Rosetta, neither workaround is needed.)

## Flash (for the owner; not done here)

Everything below writes to the robot. Nothing in it has been run.

1. Power the robot off with the battery button (hold 3 s or more) so the board runs on USB only,
   connect USB, and find the port (`ls /dev/cu.*`; the BiBoard V1 needs the CH343 driver).
2. **Back up the whole flash first.** This is the exact-restore path; it includes the saved joint
   and IMU calibration (NVS) of this particular robot:

   ```bash
   esptool.py --chip esp32 --port /dev/cu.usbmodemXXXX --baud 921600 \
     read_flash 0 0x400000 bittle_B10_250330_full_backup.bin
   ```

3. Build from the revision that matches the robot's firmware date to avoid the firmware's
   "newer sketch date -> initialisation mode" path: `git checkout c481e47` (`DATE "250330"`).
   Building from `main` (`DATE "261024"`) also compiles, but on first boot the board would enter
   initialisation mode (joint and IMU calibration prompts).
4. Flash the four images the build produced (same arguments arduino-cli's upload uses for this
   board; `boot_app0.bin` is in `<core>/tools/partitions/`):

   ```bash
   esptool.py --chip esp32 --port /dev/cu.usbmodemXXXX --baud 921600 \
     --before default_reset --after hard_reset write_flash -z \
     --flash_mode dio --flash_freq 80m --flash_size 4MB \
     0x1000  build/fly_escape/OpenCatEsp32.ino.bootloader.bin \
     0x8000  build/fly_escape/OpenCatEsp32.ino.partitions.bin \
     0xe000  <core>/tools/partitions/boot_app0.bin \
     0x10000 build/fly_escape/OpenCatEsp32.ino.bin
   ```

   (On an Intel Mac or with Rosetta, `arduino-cli upload --fqbn
   esp32:esp32:esp32:PartitionScheme=min_spiffs -p /dev/cu.usbmodemXXXX --input-dir
   build/fly_escape OpenCatEsp32` does the same.)
5. Open a serial monitor at 115200 baud, send `XE` to enable the module (hands away from the
   sensor for a second while it calibrates), `X?` to see the module list, `Xe` to disable it.

## Restoring the stock firmware

- **Exact restore from the backup** made in step 2:

  ```bash
  esptool.py --chip esp32 --port /dev/cu.usbmodemXXXX --baud 921600 \
    write_flash 0 bittle_B10_250330_full_backup.bin
  ```

- **Without a backup:** Petoi Desktop App, *Firmware Uploader*, choose Bittle X+Arm and BiBoard
  V1 (docs.petoi.com/desktop-app/firmware-uploader), or build `c481e47` unmodified with the
  settings above and flash it with the command in step 4. Petoi's uploader flow may ask to
  recalibrate. The fly_escape module state lives in the board's saved config; after going back
  to stock, send `X` to close all modules if it is still listed.

## Cost on the BiBoard

Measured by compiling (see the status table): **+20.2 KB of program flash and +4.3 KB of static
RAM** (model state 1.4 KB, scratch 2.8 KB). Run time per 35 ms step is still an **estimate, not
measured**: about 8,800 multiply-adds, 1 `expf` and on average about 130 `tanhf` calls (up to
about 1,400 in the busiest steps), estimated at 0.5 ms on average and about 2 ms worst case at
240 MHz, i.e. roughly 1.5-6% of one core at 28 Hz. See
`packages/flybrain-edge/src/flybrain_edge/export/` for the derivation.
