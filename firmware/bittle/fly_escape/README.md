# fly_escape: the looming-escape circuit on the Bittle's own board

> **Status (2026-11-12): a slim build for the robot's real partition layout compiles, with
> 34,432 bytes free in `app0`. That slim build is compile-only. On a second robot (B10_251121,
> larger partitions) the full firmware plus this module was flashed and runs: see "Second
> robot".**
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

## Second robot: Bittle X on B10_251121 (full build, nothing left out)

A second robot (Bittle X with the head, no arm; ESP32-U4WDH v3.1, 4 MB; firmware B10_251121)
was backed up and checked on 2026-11-12. It ships with the **Minimal SPIFFS** layout, so the
slim switches are not needed there. This build was flashed and run on that robot (next
subsection).

| | first robot (B10_250330) | second robot (B10_251121) |
|---|---|---|
| partition table | default: app0/app1 1,310,720 B | min_spiffs: app0 0x10000, app1 0x1f0000, 1,966,080 B each; spiffs 0x3d0000/128 KB |
| stock app image in the backup | 1,305,680 B | 1,764,992 B |
| otadata | sequence 1 -> app0 | sequence 1 -> app0 |
| app1 | not a valid image | blank (all 0xFF) |

Sources: OpenCatEsp32 `32a1fcb` (2025-11-21, the commit that sets `DATE "251121"`; the only
later commit with that date, `b2e5818`, changes web-block files only). Sketch switches as
committed: `BITTLE`, `BiBoard_V1_0`, `ROBOT_ARM` off. Same esp32 core 2.0.12.

```bash
git checkout 32a1fcb
python3 <flybrain>/firmware/bittle/fly_escape/apply_to_opencat.py . --keep-all
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --build-path build/fly_escape .
```

| build (`32a1fcb`, min_spiffs) | program flash | static RAM | app image (.bin) | free in `app0` |
|---|---|---|---|---|
| stock | 1,760,089 | 70,620 | 1,765,840 | 200,240 |
| **stock + fly_escape (all features kept)** | **1,771,477** | **73,476** | **1,777,232** | **188,848** |

Checked against the backup: the build's partition table and bootloader are byte-identical to
the robot's. The stock rebuild is 848 bytes larger than the app in the robot and not
byte-identical to it, so Petoi's build differs slightly from this one (library versions or
build settings; not investigated). The app offset is the same as on the first robot, so the
flash and restore commands below apply with `0x1e0000` as the app0 length when extracting the
stock app, and the banner to expect is `B10_251121`.

### Flashed and run on the second robot (2026-11-12)

The build above was written to the robot's unused `app1` slot and booted from there; the stock
app in `app0` was not touched. From the owner's Mac, with macOS's built-in CH343 driver:

1. Full 4 MB backup, verified (`read-flash`, `verify-flash`).
2. `write-flash 0x1f0000 <fly_escape app .bin>` with esptool 5.6.0 patched to small blocks
   (`ESP_RAM_BLOCK` 0x100 and the stub's `FLASH_WRITE_SIZE` 0x400, at 115200 baud): 1,777,232
   bytes in 109 s, hash verified, then `verify-flash` matched. With the default block sizes the
   same link fails.
3. Boot selector: one 4 KB sector at 0xf000 holding an otadata entry with sequence 2 (bytes
   `02 00 00 00`, 24 x `ff`, CRC `74 37 f6 55`, rest `ff`), so the boot loader picks `app1`.
   Entry 0 at 0xe000 (sequence 1) stays.
4. Result: boots to `Ready!` as `B10_251121` with `Module count: 12` and `E` in the module
   list; the saved servo zero positions and IMU offsets are still used.
5. `XE` over USB serial: `flyEscape baseline L 3595.30 R 3579.58`, then triggers followed by
   `bkF` and `up`. The owner confirmed the robot backed away when a hand approached and only
   then.

**Back to stock:** write the original sector back (`write-flash 0xf000` with 4096 bytes of the
backup from offset 0xf000); the stock app is still in `app0`.

Not measured: run time per step on the board, behaviour on battery without the cable, false
alarm rate over a longer session. This robot's sensor needs no side swap (pin 36 is its left);
the first robot's does, and the module has no swap switch yet.

## Messages, telemetry and Flydeck (2026-11-12)

The module prints through the firmware's `printToAllPorts()` (`src/io.h`), so its messages reach
USB, Bluetooth LE, classic Bluetooth and the Grove serial port alike, one line each, always
`@FE <what>;`. The `;` ends a message even where Bluetooth drops or moves the newline, and `@FE`
appears in no other firmware output, so a reader can pick messages out of a fragmented or glued
stream (`flybrain_edge.bittle.protocol.OnboardEscapeReader`).

| Message | When |
|---|---|
| `@FE on tel=0;` / `@FE off;` | after the module list of every `X?`, and after every module change (`XE`, `Xe`, `X`, `XC`...); `tel=1`: telemetry on |
| `@FE base L=3595 R=3580;` | resting levels (ADC counts, rounded): when measured at activation, and again after every `@FE on` status, so a listener that connects later learns them |
| `@FE trig side=-1 drive=1.05;` | trigger; the escape is queued now (side +1 left, -1 right, 0 ahead; drive >= 1) |
| `@FE done;` | the queued walk and stand are over and the module reads its sensor again |
| `@FE t 3590 3580 0.06 0.01;` | telemetry: raw left, raw right (averages over the 35 ms step), giant-fibre activity left, right as a fraction of that side's threshold (1 = trigger level) |

**Telemetry is off by default** and switches itself off whenever the module is enabled or
disabled, so other Bluetooth clients (Petoi's app) see nothing new unless someone asks.
`XEP` turns it on, `XEp` off, the same pattern as the camera's `XCP` / `XCp`: the firmware
first handles `XE...` as "enable E" (a no-op when it is on; like any `X<letter>` it also
switches other sensor modules off), then passes the command to the module. Do not send `XEp`
to a module that is off: it would switch it on. `Xe` (off) is enough, telemetry stops with it.

Rate: one line every 200 ms (5/s), plus one at each trigger so the trace shows the approach,
which lasts only 70-250 ms. A line is about 28 bytes (3 Bluetooth notifications of 10 bytes),
about 150 B/s. Over Bluetooth the laptop measured about 10 short request/reply lines a second
for the analog polling demo (`packages/flybrain-edge/NOTES.md`); 5 unsolicited lines a second
stay well inside that, leave room for the module's own messages and command echoes, and match
the firmware's own IMU print limit (200 ms).

Why the 35 ms step is not stalled by printing (read from the sources at `32a1fcb` and the esp32
core 2.0.12): `printToAllPorts()` notifies Bluetooth LE only while a client is connected
(`deviceConnected`), in 10-byte pieces; `BLECharacteristic::notify()` does not wait for a
confirmation (only indications do) and returns at once, without even queueing when the client
has notifications off. USB is `Serial.print()` into the UART: a 30-byte line fits its 128-byte
hardware FIFO, about 2.6 ms of wire time at 115200 baud, written while the next step is 35 ms
away. Classic Bluetooth (`SerialBT`) is written only while an SPP client is connected. With
telemetry off and no trigger the module prints nothing at all.

Two hooks in stock files make this work (both added by `apply_to_opencat.py`): one line at the
end of the module list in `showModuleStatus()` (`src/moduleManager.h`), and a `case
EXTENSION_FLY_ESCAPE` next to the camera's in the `X` command (`src/reaction.h`). Cost against
the previous module build (`32a1fcb`, min_spiffs): +580 bytes of program flash, +8 bytes of
static RAM.

| build (`32a1fcb`, min_spiffs) | program flash | static RAM | app image (.bin) | free in `app0` |
|---|---|---|---|---|
| stock | 1,760,089 | 70,620 | 1,765,840 | 200,240 |
| stock + fly_escape with messages and telemetry | 1,772,057 | 73,484 | 1,777,808 | 188,272 |

Laptop side: `flybrain_edge.bittle.onboard.OnboardEscape` (query, switch, telemetry), the Flydeck
demo [`demos/fly-escape-onboard`](../../../demos/fly-escape-onboard/README.md), and the laptop
demo `fly-escape`, which switches an active on-board module off while it runs and back on at
exit.

**Checked on the robot (Bittle X, B10_251121, this build in `app1`, 2026-11-12):**

- USB: `X?` answers with the list, `@FE on tel=0;` and the resting levels; `XEP` gives 5
  telemetry lines a second, `XEp` stops them.
- Bluetooth: telemetry arrives in clean 10-byte pieces about every 210 ms. The Flydeck demo
  showed the live trace and three escapes decided on the robot (left, right, left; drive 1.42,
  1.10, 2.97), each followed by `@FE done;` about 1.2 s later; the owner saw the robot back
  away each time.
- The laptop `fly-escape` demo switched the running module off at start and back on at exit.

Seen on the robot and not fixed:

- A Bluetooth client connecting makes the firmware rest the robot. The module measured its
  resting levels standing, so lying down the sensor sees the floor and the trace sits at about
  0.1 instead of 0. It did not cause triggers.
- Opening the USB serial port resets the board (and drops a Bluetooth client).

Still not measured: run time per step, a long session, classic Bluetooth clients.

## Adding it to OpenCatEsp32

`apply_to_opencat.py` does all of this (checked against OpenCatEsp32 `c481e47`, `32a1fcb` and
`f5aba6b`):

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

   at the end of the module list in `showModuleStatus()`, after
   `printListWithoutString(moduleActivatedQ, moduleCount);`:

   ```c
   #ifdef FLY_ESCAPE
     flyEscapeStatus(moduleActivatedQ[indexOfModule(EXTENSION_FLY_ESCAPE)]);
   #endif
   ```

   in `readSignal()` next to the other module polls (`read_doubleTouch()` etc.):

   ```c
   #ifdef FLY_ESCAPE
       if (moduleActivatedQ[indexOfModule(EXTENSION_FLY_ESCAPE)])
         read_flyEscape();
   #endif
   ```

5. `src/reaction.h`, in the `X` command, first case after `switch (newCmd[0]) {` ("deal with
   the following command"):

   ```c
   #ifdef FLY_ESCAPE
               case EXTENSION_FLY_ESCAPE:
                 {
                   flyEscapeOptions(newCmd, moduleActivatedQ[indexOfModule(EXTENSION_FLY_ESCAPE)]);
                   break;
                 }
   #endif
   ```

6. Enable on the robot with the serial command `XE`; `Xe` disables it; `X?` shows module status;
   `XEP` / `XEp` switch the telemetry line on / off.
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
