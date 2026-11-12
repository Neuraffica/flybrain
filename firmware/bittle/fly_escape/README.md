# fly_escape: the looming-escape circuit on the Bittle's own board

> **Status (2026-11-12): a slim build for the robot's real partition layout compiles, with
> 34,432 bytes free in `app0`. Compile-only: not flashed, not run on a robot.**
> OpenCatEsp32 `c481e47` (the robot's firmware B10_250330), Bittle X+Arm on BiBoard V1, esp32
> core 2.0.12, **default 4 MB partition scheme** (the robot's), with this module and three stock
> features left out: the camera/vision module, the quick demo and the micro:bit BLE client
> (section "Slim build"). The build's partition table and bootloader are byte-identical to the
> ones in the robot's flash backup, so flashing only the app image at 0x10000 is sound
> (section "Flash"). Not yet solved: the USB link on the owner's Mac (see "Checked against the
> real robot").

What it does: an OpenCatEsp32 extension module (letter **E**). When enabled it reads the Petoi
double IR distance sensor on GPIO 36 (`ANALOG3`, treated as left) and GPIO 39 (`ANALOG4`, right)
every time the firmware polls modules, averages the readings, and every 35 ms runs one step of
the FlyWire looming-escape circuit (LC4, LPLC2 -> giant fibers; 348 neurons, 2,093 synapse
entries). On a trigger it queues `bk` (walk backward, 1 s) and `up` (stand) through the
firmware's task queue, at most once every 3 s. It calibrates the empty-scene level at
activation, so keep hands away from the sensor for the first ~0.15 s after enabling.

Like every OpenCat sensor module it only runs while the robot is idle (the firmware polls
modules when no command is executing), and it does not go through the serial "read analog
pin" path that stops a running gait when the laptop demo polls the sensor.

## Files

| File | What |
|---|---|
| `src/fly_escape_all.h` | Generated model (single header, functions `static inline`, data `static const` in flash). Regenerate: `uv run flybrain-export escape --single-header --out firmware/bittle/fly_escape/src` |
| `src/flyEscape.h` | The OpenCat module: setup/calibration, polling, stepping, queueing the escape |
| `apply_to_opencat.py` | Applies the module (and, by default, the slim switches) to an OpenCatEsp32 folder exactly as below |

To use the looming-rate baseline instead of the connectome model, add
`#define FLY_ESCAPE_MODEL 1` before `#include "flyEscape.h"` (this also compiles the baseline
in; the default build leaves it out).

## The on-board model (compact layout, lossless)

The generated C stores the same model more compactly than the first export (2026-11-11), with
no change to its decisions:

| model layout | data in flash | module code + data (ESP32 `-Os`) | static RAM | trigger-tick / side mismatches vs Python |
|---|---|---|---|---|
| float32 CSR, looming baseline always in (first export) | 16,008 B | 17,684 B | 2,808 B scratch + 1,448 B state | 0 / 0 |
| compact: uint8 synapse counts x per-class scale, 6 neuron classes, index ranges, no `drive_in` scratch | 7,766 B | 9,452 B (9,285 without the baseline) | 1,404 B + 1,448 B | 0 / 0 |
| **compact, 3 neurons with no path to the readout dropped (default)** | **7,384 B** | **8,903 B** without the baseline | **1,392 B + 1,436 B** | **0 / 0** |

Equivalence was checked with the host harness against the Python model on both real recordings
and all 384 simulated benchmark episodes (42,796 steps, 331 triggers for the connectome model
and 258 for the baseline): no trigger-tick or side mismatches for any layout, giant-fiber
activity within 1.9e-6. The dropped neurons (DNp02, DNp11, one AMMC-A1) have no directed path to
the giant fibers or to the LC4/LPLC2 populations, the only things the readout uses, so they
cannot change a trigger or a side. Counts above 255 (3 pairs) are split into two entries. Not
pursued: int8/int16 weights and pruning small weights; the lossless layout was enough, so
nothing lossy ships.

On the firmware, the module's cost against the same stock build went from **+20,244 B** of
program flash (first export) to **+11,468 B** (+2,856 B static RAM).

## Slim build (fits the robot's default partition table)

The robot uses the default 4 MB layout: `app0` is 1,310,720 bytes. Stock `c481e47` alone is a
1,305,680-byte app image (5,040 bytes free), so even the compact module does not fit without
leaving something out. Measured cost of each stock compile switch (program flash bytes from
`arduino-cli compile`, same revision and settings, each line removing one more feature):

| build (c481e47, BiBoard V1, Bittle X+Arm, default partitions) | program flash | static RAM | saves |
|---|---|---|---|
| stock | 1,299,929 | 50,484 | |
| - quick demo (`QUICK_DEMO`, the `XQ` touch-pad demo on ANALOG1) | 1,299,681 | 50,476 | 248 |
| - camera/vision (`CAMERA`: Mu Vision, Sentry, Grove Vision AI / Seeed SSCMA) | 1,273,301 | 49,916 | 26,380 |
| - voice (`VOICE`, the Bittle X's voice-command module) | 1,270,653 | 49,700 | 2,648 |
| - ultrasonic (`ULTRASONIC`, Petoi RGB ultrasonic sensor) | 1,258,817 | 49,508 | 11,836 |
| - gesture (`GESTURE`, APDS9960 module) | 1,256,933 | 49,460 | 1,884 |
| - PIR (`PIR`) | 1,256,501 | 49,452 | 432 |
| - micro:bit BLE client (`BT_CLIENT` in `src/OpenCat.h`; the robot's own BLE server stays) | 1,241,913 | 49,356 | 14,588 |
| (option) - classic Bluetooth SPP (`BT_SSP` in `src/OpenCat.h`) | 1,227,605 | 48,508 | 14,308 |

The small sensor modules the owner has are cheap and are kept: double touch 900 B, double light
5,120 B, double IR distance 1,328 B (each measured by removing it from the bottom row).
Compiler/link options were not needed.

**Shipped slim build: leave out the camera, the quick demo and the micro:bit BLE client; keep
everything else** (voice, ultrasonic, gesture, PIR, double touch/light/IR, BLE server, classic
Bluetooth, IMU, every skill, `R`, `X`, `P`, calibration storage):

| build | program flash | static RAM | app image (.bin) | free in `app0` |
|---|---|---|---|---|
| stock | 1,299,929 | 50,484 | 1,305,680 | 5,040 |
| stock + fly_escape (compact) | 1,311,397 | 53,340 | does not fit | |
| slim stock (same three features out) | 1,258,993 | 49,820 | 1,264,736 | 45,984 |
| **slim + fly_escape** | **1,270,541** | **52,676** | **1,276,288** | **34,432** |
| (option) slim + also ultrasonic, gesture, PIR out + fly_escape | 1,256,233 | 52,428 | 1,261,984 | 48,736 |
| (option) the above + voice out + fly_escape | 1,253,485 | 52,220 | 1,259,232 | 51,488 |

**What the shipped slim build no longer has:**

- the camera/vision module (`XC`): Mu Vision, Sentry and Grove Vision AI cameras do nothing;
- the quick demo (`XQ`): the touch-pad example on ANALOG1;
- the micro:bit BLE client: the robot no longer connects out to a micro:bit controller. The
  robot's own BLE serial service, which the laptop demos and Flydeck use, is unchanged.

The module letters stay in the `X` list; selecting a removed one does nothing.

## Adding it to OpenCatEsp32

`apply_to_opencat.py` does all of this (checked against OpenCatEsp32 `c481e47` and `f5aba6b`):

```bash
git clone https://github.com/PetoiCamp/OpenCatEsp32 && cd OpenCatEsp32 && git checkout c481e47
python3 <flybrain>/firmware/bittle/fly_escape/apply_to_opencat.py .            # slim (default)
python3 <flybrain>/firmware/bittle/fly_escape/apply_to_opencat.py . --keep-all # module only
```

By hand:

1. Copy `src/flyEscape.h` and `src/fly_escape_all.h` into `OpenCatEsp32/src/`.
2. `src/OpenCat.h`, next to the other `EXTENSION_*` defines (`E` is not used by any of
   `S A T L D I B U G C Q`):

   ```c
   #define EXTENSION_FLY_ESCAPE 'E'  // fly looming escape, IR sensor on ANALOG3, ANALOG4
   ```

   and append one entry to each of the parallel module arrays (they must stay the same length):

   ```c
   int8_t moduleList[] = { ..., EXTENSION_QUICK_DEMO, EXTENSION_FLY_ESCAPE, };
   String moduleNames[] = { ..., "Quick_Demo", "Fly_Escape" };
   bool moduleActivatedQ[] = { ..., 0 };   // every board variant's list: add one trailing 0
   ```

   Slim build: comment out both `#define BT_CLIENT` lines in the same file.
3. `OpenCatEsp32.ino`: `#define FLY_ESCAPE` before `#include "src/OpenCat.h"`; set `#define
   BITTLE`, `#define BiBoard_V1_0` (other board lines commented), `#define ROBOT_ARM`. Slim build:
   comment out `#define CAMERA` and `#define QUICK_DEMO`. Do not enable the stock double-IR
   module (`XD`) at the same time as this one: both use pins 36/39.
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

5. Enable on the robot with the serial command `XE`; `Xe` disables it; `X?` shows module status.
   The module state is saved, so it will be active again after a reboot until disabled.

## Build (as verified, compile only)

Petoi's guide for BiBoard (docs.petoi.com, "Upload sketch for BiBoard") asks for the esp32 core
**2.0.12** ("2.0.13 and above may cause the mainboard to fail to start up"), the extra line
`#define CONFIG_DISABLE_HAL_LOCKS 1` at the end of
`<core>/tools/sdk/esp32/qio_qspi/include/sdkconfig.h`, board **ESP32 Dev Module**; libraries
ArduinoJson, WebSockets, WiFiManager (library manager) and MuVisionSensor3
(https://github.com/mu-opensource/MuVisionSensor3). Verified with ArduinoJson 7.4.3, WebSockets
2.7.2, WiFiManager 2.0.17, MU Vision Sensor 3 1.2.2.

```bash
arduino-cli core install esp32:esp32@2.0.12 \
  --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli lib install ArduinoJson WebSockets WiFiManager
arduino-cli config set library.enable_unsafe_install true   # needed for the next line only
arduino-cli lib install --git-url https://github.com/mu-opensource/MuVisionSensor3.git
# then append the CONFIG_DISABLE_HAL_LOCKS line to sdkconfig.h as above

# main route: the robot's default 4 MB partition table (app0 = 1,310,720 bytes)
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=default \
  --build-path build/fly_escape OpenCatEsp32
```

The tool rejects an image larger than `app0` ("Sketch too big"). Alternative, only if the
partition table is also rewritten on the robot (moves `app1` and `spiffs`; not recommended):
`PartitionScheme=min_spiffs` (1,966,080-byte app partitions), where even the unmodified module
fits without the slim switches (first build, 2026-11-11: 1,320,173 bytes).

On an Apple Silicon Mac **without Rosetta**, two x86-only tools in that toolchain fail ("Bad CPU
type"): `esptool` 4.5.1 and Arduino's `ctags` 5.8-arduino11. The verified builds replaced them
without installing Rosetta: `esptool==4.5.1` from PyPI in a virtualenv, and Homebrew
`universal-ctags` behind a 3-line shim that renames its `typeref:typename:` field to the
`returntype:` field Arduino's prototype generator expects:

```bash
--build-property "tools.esptool_py.path=<venv>/bin" --build-property "tools.esptool_py.cmd=esptool.py" \
--build-property "tools.ctags.path=<dir containing the shim named ctags>"
```

With that shim, builds that keep `QUICK_DEMO` but drop `CAMERA` misplace the generated
prototypes and fail (`'quickDemo' was not declared`); this is a shim artefact, not a firmware
problem, and does not affect the shipped slim build (which drops both). With Rosetta, or in the
Arduino IDE, neither workaround is needed.

## Partition table and boot chain (compared with the robot's backup)

Read from the 4 MB backup (`bittle-d0ef76cdefb0-B10_250330-20261112-1244.bin`):

| | robot (backup) | this build (`PartitionScheme=default`) |
|---|---|---|
| partition table at 0x8000 | nvs 0x9000/20 KB, otadata 0xe000/8 KB, app0 0x10000/0x140000, app1 0x150000/0x140000, spiffs 0x290000/0x160000, coredump 0x3f0000/64 KB | byte-identical, including its MD5 entry |
| bootloader at 0x1000 | ESP-IDF v4.4 bootloader | byte-identical to the 18,992-byte bootloader core 2.0.12 builds |
| app0 at 0x10000 | app built with ESP-IDF v4.4.5 (ac5d805d0e) by arduino-lib-builder, dated Jun 12 2023: the core 2.0.12 library build | same IDF v4.4.5 build, same image header (DIO, 80 MHz, 4 MB) |
| otadata at 0xe000 | entry 0: sequence 1, CRC valid; entry 1: blank | not written by an app-only flash |
| app1 at 0x150000 | not a valid app image | not written |

How to read `otadata`: two 32-byte entries at 0xe000 and 0xf000, each `uint32 ota_seq`, 20 bytes
label, `uint32 ota_state`, `uint32 crc` (CRC-32 of the 4 sequence bytes, as
`esp_rom_crc32_le(0xFFFFFFFF, seq, 4)`, i.e. Python `zlib.crc32(seq, 0xFFFFFFFF)`). The boot
loader takes the valid entry with the highest sequence and boots `app[(seq - 1) % 2]`. Here
sequence 1 with a valid CRC (0x4743989a) selects **app0**; `ota_state` 0xFFFFFFFF means rollback
is not in use. An app-only write to app0 is therefore what boots.

## Flash (for the owner; not done here)

Everything below writes to the robot. Nothing in it has been run. Fix the USB link first (next
section).

1. Power the robot off with the battery button (hold 3 s or more) so the board runs on USB only,
   connect USB, and find the port (`ls /dev/cu.*`).
2. Keep the verified full backup you already have (`write_flash 0 <backup>` restores everything,
   including this robot's calibration in NVS).
3. Build from `c481e47` (same `DATE "250330"` as the robot, so the firmware does not enter its
   "newer sketch date" initialisation mode), slim switches applied, as above.
4. **Write only the app image to app0.** The bootloader, partition table, otadata, NVS
   (calibration) and SPIFFS stay as they are:

   ```bash
   esptool.py --chip esp32 --port /dev/cu.wchusbserialXXXX --baud 921600 \
     --before default_reset --after hard_reset write_flash -z \
     --flash_mode dio --flash_freq 80m --flash_size 4MB \
     0x10000 build/fly_escape/OpenCatEsp32.ino.bin
   ```

   (`arduino-cli upload` would also rewrite the bootloader, the partition table and otadata
   with `boot_app0.bin`; the bootloader and table are identical, but there is no reason to touch
   them.)
5. Open a serial monitor at 115200 baud, check the boot banner says `B10_250330`, send `XE` to
   enable the module (hands away from the sensor for a second while it calibrates), `X?` to see
   the module list, `Xe` to disable it.

## Restoring the stock firmware

- **App only** (the app partition is all that step 4 changed): write the stock app back from the
  backup. Extract it first (app0 is 0x140000 bytes at 0x10000):

  ```bash
  python3 -c "d=open('backup.bin','rb').read(); open('stock_app0.bin','wb').write(d[0x10000:0x150000])"
  esptool.py --chip esp32 --port /dev/cu.wchusbserialXXXX --baud 921600 write_flash 0x10000 stock_app0.bin
  ```

- **Everything:** `esptool.py --chip esp32 --port ... write_flash 0 backup.bin`.
- **Without a backup:** Petoi Desktop App, *Firmware Uploader*, Bittle X+Arm, BiBoard V1
  (docs.petoi.com/desktop-app/firmware-uploader); it may ask to recalibrate.

The fly_escape module state lives in the board's saved config; after going back to stock, send
`X` to close all modules if it is still listed.

## Cost on the BiBoard

Measured by compiling: the compact module adds **11,468 bytes of program flash and 2,856 bytes
of static RAM** to a stock build (first export: 20,244 / 4,288). Run time per 35 ms step is still
an **estimate, not measured**: about 8,400 multiply-adds, 1 `expf` and on average about 130
`tanhf` calls (up to about 1,400 in the busiest steps), estimated at 0.5 ms on average and about
2 ms worst case at 240 MHz, roughly 1.5-6% of one core at 28 Hz. The compact layout adds one
class lookup per synapse, which should not change that estimate materially.

## Checked against the real robot (2026-11-12)

A full backup of the robot's flash was taken and verified; nothing was written.

- **The chip is an ESP32-U4WDH (revision v3.1) with 4 MB of embedded flash.**
- **The robot's partition table is the default 4 MB layout, not Minimal SPIFFS** (table above).
  The first module build (1,320,173 bytes) was 9,453 bytes too big for `app0`; the slim build
  (1,276,288-byte image) fits with 34,432 bytes free. Its partition table and bootloader match
  the robot's byte for byte, so only the app needs writing.
- **Still blocking: the USB link on this Mac corrupts large writes to the board.** With macOS's
  built-in driver for the board's CH343 USB chip (port name `cu.usbmodem...`), esptool fails to
  upload its helper ("Failed to write to target RAM", checksum error or timeout) unless the RAM
  block size is cut to 256 bytes. Reading works. Writing a firmware image over a link in that
  state is not safe, because recovery would depend on the same link. Install WCH's CH343 driver
  (the port then appears as `cu.wchusbserial...`) or flash from another computer.

Backup command that worked here (esptool 5.6.0, RAM block patched to 0x100, 115200 baud, 381 s):
`read-flash 0 0x400000 backup.bin`, then `verify-flash 0 backup.bin` reported a matching
digest. Restore is `write-flash 0 backup.bin`, which needs a healthy link.
