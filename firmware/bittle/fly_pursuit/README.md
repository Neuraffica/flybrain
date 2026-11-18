# fly_pursuit: the AI Vision camera read in its own task, and the pursuit circuit on the board

> **Status (2026-11-18): built from `32a1fcb`, flashed and run on a Bittle X (firmware
> B10_251121) over USB and over Bluetooth.** See "Run on the robot". No session longer than a
> minute yet. The first build in this folder's history came from `f5aba6b` and must not be
> flashed (its `DATE` would make the firmware re-initialise the board); the numbers under
> "Build" marked `f5aba6b` are from that build.

## Run on the robot (2026-11-18, USB and Bluetooth)

Image: OpenCatEsp32 `32a1fcb` with fly_escape and fly_pursuit applied by
`apply_to_opencat.py` (no anchor needed fixing), all stock features kept. Program 1,789,073
bytes, static RAM 83,652, app image 1,794,816 bytes (171,264 free in the slot), SHA-256
`8b36356b8e671b121cc6e06c737bc5e8cd14abc3e3c7e107a9b0451e0df760dc`. The build's bootloader and
partition table are byte-identical to the robot's. Written to `app1` (0x1f0000) after a full,
verified 4 MB backup; the stock app in `app0` was not touched.

| Check | Result |
|---|---|
| Boot | `B10_251121`, `Module count: 13`, `E` and `P` in the list, saved servo and IMU calibration used, `@FP off;` |
| `XP~` with no camera plugged in | `@FP on l=1 b=0;` at once, then `@FP err nocam <n>;` every 2 s. Other commands answered at once, `gp` kept printing. |
| `XP~` / `Xp~` twice in one boot, no camera | both worked |
| `XP~` with the camera | `@FP cam run 1;` 0.15 s after the command. 196 results in 12.7 s (15.4 a second), none skipped, median gap 54 ms, longest 112 ms, no error line. |
| `Xp~` then `XP~` again, same boot | `@FP cam stop;`, then `@FP cam run 2;` and 15.5 results a second. This is what the stock module cannot do. |
| IMU while the camera runs | `gp` printed a reading both times it was asked. No `lock` error. |
| Passive state | the robot did not move |
| `XPB~` with a person's face about 50 cm away, moving side to side | The robot sat, then the head followed. 481 results in 31.2 s (15.4 a second), none skipped, a face in 385 of them (80 %). Head pan between -36 and +41 degrees for a face between -42 and +43 degrees from straight ahead. Face offset from the image centre while seen: median 3.0, mean 4.0, 90th percentile 8.0, largest 19.8 degrees. No head reversal that was not the person's own movement. No error line. |

Over Bluetooth (no command sent over USB), same day, second camera start of that boot:

| Check | Result |
|---|---|
| Echoes | `XP~` 0.08 s, `XPB~` 0.85 s (it sits first), `XPb~` 0.04 s, `Xp~` 0.06 s. None missing. |
| Passive, 6 s | 89 results (15.5 a second), none missing over Bluetooth |
| `XPB~`, 30 s, a face moving side to side | 463 results (15.4 a second), none missing over Bluetooth, a face in 378 (82 %); offset from the image centre median 1.2, 90th percentile 9.8 degrees; head pan between -25 and +50 degrees. No error line. |

So 15 result lines a second pass over Bluetooth LE on this link without loss, and with the
camera read by this module no reply went missing (with the stock camera module they did).

So on this robot the pan sign (positive = left) and the sign of x (positive = the dog's right)
are as assumed. Not measured: run time of one control step, the IMU's read rate while the
camera runs, a session longer than a minute, and what happens when the camera's
own side stalls (it did not).

Seen on this robot and unrelated to the module: lying still, the IMU's yaw runs away at about
34 degrees a second with the module off and the camera unplugged; pitch and roll are steady.

What it is: an OpenCatEsp32 extension module, letter **P** (free: `S A T L D I B U G C Q E` are
taken), that talks to the Petoi AI Vision camera (Seeed Grove Vision AI V2, SSCMA firmware) on
the Grove I2C socket with a small client of its own, in its own FreeRTOS task, and reports
every camera result to the host. Optionally (`XPB~`) it runs a circuit of 265 neurons from the
fly connectome (264 of them; see "The circuit") on the board and turns the head toward the
face, the way the laptop demo `demos/fly-pursuit` does over the link.

It is built to avoid the six failures measured with the stock camera module on this robot
(`packages/flybrain-edge/NOTES.md`, "Petoi AI Vision camera"); see "The six failures".

## Files

| File | What |
|---|---|
| `src/flyPursuit.h` | The module: camera task, SSCMA I2C client, bus locking, bus recovery, main-loop hooks, head moves |
| `src/flyPursuitCore.h` | The hardware-free parts: SSCMA reply parser, head control step, line format. Compiled and tested on the laptop |
| `src/fly_pursuit_all.h` | The circuit, generated: `uv run flybrain-export pursuit --single-header --out firmware/bittle/fly_pursuit/src` |
| `apply_to_opencat.py` | Patches an OpenCatEsp32 folder with BOTH modules (fly_escape E and fly_pursuit P), idempotent |

Tests (laptop): `packages/flybrain-edge/tests/test_onboard_pursuit.py` (export, C-vs-Python
equivalence, the core header with `export_stubs/pursuit_core_main.cpp`, the patch script on a
minimal fake sketch).

## Commands

The firmware's `X` command. Capital letters, so end each with `~` (otherwise the firmware waits
for its serial timeout).

| Command | Effect |
|---|---|
| `XP~` | Module on, **passive**: the camera starts continuous detection, a line per result. The robot does not move. |
| `Xp~` | Module off: `AT+BREAK` to the camera, task idles. Also `X~` (all modules off), or enabling another module. |
| `XPP~` / `XPp~` | Result lines on (default) / off. Status and error lines are always printed. |
| `XPB~` | On-board control on: the robot does `sit` (as the stock camera module does), then every result turns the head. |
| `XPb~` | On-board control off; the head stays where it is. |
| `X?` | Module list, then `@FP on ...;` or `@FP off;` |

As with fly_escape: `XP...` is first handled by the firmware as "enable P" (a no-op when it is on;
like any `X<letter>` it switches off the other sensor modules except voice), then the options are
read. So `XPB~` or `XPp~` sent while the module is off switches it on. The module state is saved
to flash by the firmware; on-board control is NOT saved and is off after every start, including
a start at boot (requirement 6).

`E` and `P` cannot be active at the same time (enabling one switches the other off). Do not use
the stock camera module (`XC`) in the same power-on as this one: both talk to 0x62.

## Host lines

Printed with `printToAllPorts()` (USB, Bluetooth LE, classic Bluetooth, Grove serial), one per
line. Every message starts with `@FP ` and ends with `;` (the `;` ends it even when Bluetooth
drops or moves the newline). Fields are separated by one space. Integers are decimal; decimals
have one digit after the point.

```
result   = "@FP " seq " " ( box | "-" ) [ control ] ";"
box      = x " " y " " w " " h " " score
control  = [ " b=" bearing ] " t=" cmd " p=" pan [ " hold" ]
status   = "@FP on l=" ("0"|"1") " b=" ("0"|"1") ";"  |  "@FP off;"
camera   = "@FP cam run " starts ";"  |  "@FP cam stop;"
error    = "@FP err " what " " count ";"
boot     = "@FP boot bus sda=" d " scl=" d " now sda=" d " scl=" d ";"     (USB only)
```

| Field | Meaning |
|---|---|
| `seq` | Camera results received since power-on, 0..65535, wraps. It counts every result, also those not printed (the main loop prints only the newest one it sees), so a jump of more than 1 means results were skipped. |
| `x y` | Centre of the box with the highest score, pixels from the image centre (240 x 240 frame, so -120..120). `x` positive = the dog's RIGHT (measured through the firmware with the stock camera setting, which this module sends too). `y` positive = down (not checked). |
| `w h` | Box width and height, pixels. |
| `score` | Box score, 0-100 (the camera's model reports only boxes above its own threshold, 50 on this camera). |
| `-` | The camera returned a result with no box: nothing seen in this frame. |
| `b=` | (on-board control, only with a box) bearing given to the circuit, degrees, positive = LEFT, after the correction for the head's own turn. |
| `t=` | (on-board control) head turn command after the 60 deg/s limit, deg/s, positive = left; `0.0` while holding. |
| `p=` | (on-board control) pan target, degrees, positive = left, within +-50. |
| `hold` | (on-board control) no face for 3 s: the head holds still. |
| `l= b=` | Result lines on/off, on-board control on/off. |
| `cam run <n>` | The camera confirmed `AT+INVOKE`; `n` = starts since power-on (more than 1 after a recovery or a stall). |
| `cam stop` | The module was switched off and `AT+BREAK` was sent. |
| `count` | How many errors of that kind since power-on (a line is printed whenever it grows). |

Error kinds (`what`):

| `what` | Meaning | What to do |
|---|---|---|
| `i2c` | An I2C transaction with 0x62 failed (no acknowledge, Wire's 50 ms timeout, short read, or a nonsense length) | Nothing; 5 in a row cause a recovery |
| `lock` | The IMU (or gesture/EEPROM code) did not free the bus within 20 ms | Nothing; the transaction is tried again |
| `stall` | Detection was running but no result came for 1.5 s; `AT+INVOKE` is sent again | If it repeats: check the camera |
| `overflow` | A reply did not fit in the 1,024-byte buffer and was dropped | Should not happen with the face model |
| `cmd` | `AT+SENSOR` or `AT+INVOKE` got no reply within 1.5 s or an error code | Retried every 2 s |
| `nocam` | Nothing answers at 0x62 nor at 0x28 | Camera not plugged into the I2C socket (`SCL SDA +5V GND`) |
| `silent` | 0x28 answers but 0x62 does not: the camera is on the bus but its AI (SSCMA) side is not answering. Seen on 2026-11-17; it lasted across ESP32 restarts | **Power-cycle the camera** (switch the robot off, or unplug the camera). Retried every 2 s |
| `recover` | A bus recovery was made (count = recoveries since power-on) | If it repeats: look at the camera and cable |
| `busheld` | SDA or SCL was still low after a recovery | Power-cycle the camera |

Rate: up to one result line per camera frame, about 15 per second (measured 15.4/s on the
camera's own USB port with the same command), about 25-60 bytes each, so up to about 900 B/s.
Over Bluetooth LE that is more than the laptop has been seen to receive cleanly (about 10 short
lines a second, NOTES.md); turn the result lines off (`XPp~`) when only on-board control is
wanted over Bluetooth, or measure first.

Example (USB, passive, then control on):

```
@FP on l=1 b=0;
@FP cam run 1;
@FP 1 -;
@FP 2 31 -12 83 90 76;
@FP on l=1 b=1;
@FP 40 -64 -10 80 88 79 b=16.0 t=24.3 p=3.2;
@FP 233 - t=0.0 p=21.5 hold;
```

## How it works

### Camera client: our own, not Seeed_Arduino_SSCMA

The library (the copy vendored in OpenCatEsp32, `src/Seeed_Arduino_SSCMA`) was read before
deciding. It cannot be bounded enough for this:

- `begin()` mallocs a 64 KB receive buffer and a 2 KB transmit buffer; every reply is copied
  into a fresh `malloc` and parsed into an ArduinoJson 7 `JsonDocument`, which allocates on the
  heap; boxes go into `std::vector`s. There is no `end()`; the buffers are never freed.
- `wait()` spins on `available()` (2 ms delays only) for up to 1-3 s per command, ignoring
  transfer errors; `invoke()` waits for both the response and the result.
- `i2c_read()` asks for 250 bytes per transaction, but the ESP32 core 2.0.12's `Wire` has a
  128-byte receive buffer and `requestFrom()` does not check the size: **the IDF driver writes
  up to 250 bytes into Wire's 128-byte heap buffer**. This is read from the sources, not
  observed; it would corrupt the heap whenever more than 128 bytes are waiting, and is a
  plausible part of the stock module's crashes and stalls.

So the module has a minimal client of its own for the same transport (byte layout copied from
the library): `0x10 0x03 0 0 0 0` asks how many bytes are ready (2-byte answer), `0x10 0x01 len
0 0` then a read fetches them, `0x10 0x02 len <text>` sends an AT command. Reads are made in
64-byte pieces (below Wire's 128), into one static 1,024-byte buffer; a parser of about 150
lines takes complete `\r{...}\n` frames out of it and pulls out only `type`, `name`, `code`,
`perf` and the box with the highest score. Nothing is allocated.

Mode: one `AT+INVOKE=-1,0,1` (run forever, no change filter, results without images), then only
reads; `AT+BREAK` on disable. Each result, with or without a box, becomes one line. Before
starting, the module sends `AT+BREAK` (a detection may be left running from before an ESP32
restart, since the camera keeps its power) and `AT+SENSOR=1,1,4096` (240 x 240, the 90-degree
rotation the stock firmware uses on a Bittle X; 0 on a Bittle X+Arm), exactly as the stock
module does, so the measured sign of x still holds.

### Task

Created once, at the first enable, with `xTaskCreateStaticPinnedToCore` (stack and control
block are static arrays, so creating it cannot fail for lack of memory and nothing is ever
allocated or freed), core 0, priority 1 (the IMU task's core and priority). It is never deleted.
When the module is off it sleeps in `ulTaskNotifyTake` (woken by enable/disable, otherwise every
200 ms). When on, it polls the camera every 10 ms.

### Main loop

All camera I2C traffic is in the task. The main loop calls `read_flyPursuit()` on every pass of
`readSignal()` (next to the voice module's poll, so lines also come while the robot is busy;
not inside the idle-only block where the stock modules are polled). It copies one small struct
(the newest result, a sequence number, error counters; about 90 bytes) inside a
`portENTER_CRITICAL` section on a spinlock, a few microseconds; it never waits for the camera.
There is no `while (flag);` anywhere in the module.

### Sharing the I2C bus with the IMU (how `imu.h` expects it)

The firmware's I2C users share `Wire` with HAL locks disabled (`CONFIG_DISABLE_HAL_LOCKS 1`,
required by Petoi's build guide), and coordinate with four flags in `OpenCat.h`:
`imuLockI2c`, `cameraLockI2c`, `gestureLockI2c`, `eepromLockI2c`. The IMU task (`readIMU()`)
waits while `cameraLockI2c` is up (at most 500 ms, then skips that reading), then raises
`imuLockI2c` for its own transaction.

This module, for each transaction (one command + one read of at most 64 bytes, about 8 ms at
100 kHz):

1. raises `cameraLockI2c` first,
2. waits one tick (1 ms), so that an IMU read that had already passed its own check of
   `cameraLockI2c` has raised `imuLockI2c` by then,
3. waits (`vTaskDelay(1)` steps) until `imuLockI2c`, `gestureLockI2c` and `eepromLockI2c` are
   down, at most 20 ms (then it lowers its flag, counts `lock`, and tries later),
4. does the transaction, lowers `cameraLockI2c`.

The IMU task is never deleted and `updateGyroQ` is never touched. The flags are plain `bool`s,
not atomics; the one-tick wait closes the check-then-set window of `readIMU()` in practice but
is not a proof. Bus load: a result is about 150-250 bytes, so at 15 results a second the camera
holds the bus for very roughly 30-40% of the time at 100 kHz; IMU readings are delayed by up to
one 8 ms transaction, not lost (not measured).

### Timeouts, failures and bus recovery

Every I2C transaction is bounded by `Wire`'s timeout (50 ms, ESP-IDF driver); every wait for the
bus by 20 ms; every wait for an AT reply by 1.5 s (polling, in the task). Failures are counted
(`@FP err ...`). After 5 failed transactions in a row the task:

1. raises `cameraLockI2c` (waiting up to 200 ms for the IMU),
2. removes the I2C driver (`i2cDeinit(0)`, the core's HAL),
3. releases SDA, clocks SCL 9 times as an open-drain GPIO, sends a STOP (SDA low to high while
   SCL is high), checks that both lines are high (else `err busheld`),
4. re-installs the driver on the firmware's pins (SDA 21, SCL 22, the defaults its
   `Wire.begin()` uses) at the clock `Wire` had (`i2cInit(0, SDA, SCL, Wire.getClock())`). This
   is what `Wire.begin()` does, without re-allocating Wire's buffers; the ESP-IDF driver itself
   allocates its small control block again when installed, so a recovery (only a recovery)
   allocates,
5. lowers the flag, counts `recover`, and starts the camera again (`AT+BREAK`, `AT+SENSOR`,
   `AT+INVOKE`; `@FP cam run <n>` when it answers).

### The circuit and on-board control

`fly_pursuit_all.h` is `flybrain_models.pursuit.PursuitCircuit` with mirror averaging and the
shipped `k_out` (4850.86 rad/s per unit of DNa02 difference), float32: rectified-linear units,
tau 20 ms, exponential-Euler sub-steps of at most 10 ms, Gaussian LC10a input (sigma 10 deg),
DNa02 bias 1.0, two copies of the state (real and mirrored bearing), `turn = k_out * 0.5 *
(dA - dB)`. Stored as synapse counts (`uint8`), one sign bit per presynaptic neuron, one
`1 / input_total` per neuron, CSR indices: 1,840 entries, 8,525 bytes of data in flash (the
same as float32 CSR would be 11,570), 4,092 bytes of state in RAM. One LC10a has no path to
DNa02 and is left out (it cannot change the output), so 264 neurons are computed.

On every new result, with on-board control on (all constants are at the top of
`flyPursuitCore.h`, each marked measured / assumed / chosen):

- `dt` = time since the previous result (0.001-0.5 s; after a gap over 1 s the circuit is reset);
- with a box: bearing = `-x * 60 / 240` degrees (positive = left; the 60-degree field of view is
  ASSUMED, as in the laptop demo), minus how far the pan target has moved since the picture was
  taken. "When the picture was taken" is estimated as the time the task asked for the bytes
  minus the camera's own `perf` (preprocess + inference + postprocess), from a 32-entry history
  of pan targets;
- one circuit step: `fp_step(has_face, bearing, dt, arousal 1)` -> turn, rad/s;
- no face for 3 s: hold (command 0). Before that, a frame without a face is given to the circuit
  as "no target", exactly as the laptop demo does;
- command clipped to +-60 deg/s, integrated into the pan target over `min(dt, 0.15 s)` (so one
  late result moves the head by at most 9 degrees), pan target clamped to +-50 degrees;
- the head pan joint only: `targetHead[0]` and `manualHeadQ` are set exactly as the firmware's
  `i 0 <angle>` command sets them (after a posture, that command ends in the posture's
  `perform()` calling `calibratedPWM(0, angle)`), and `calibratedPWM(0, angle)` is called at
  once. No text command is queued. Positive pan = look left (measured, NOTES.md).

Enabling on-board control does `loadBySkillName("sit")` first, as the stock camera module does
when it starts; the passive state never changes posture.

## The six failures, and what this module does about each

| # | Measured with the stock module | Here |
|---|---|---|
| a | Starts once per boot: stop is a stub, the task cannot be created again (`xTaskCreate` error -1) | The task is static, created once, never deleted; off = it idles. `XP~` / `Xp~` any number of times. No allocation on enable or disable. |
| b | Kills the IMU task (`updateGyroQ = false`) | Never touches `updateGyroQ` or `TASK_imu`; shares the bus with the flags as above. |
| c | Prints a position only when it changes, never says "no face" | One line per camera result, `@FP <seq> -;` when there is no box, with a sequence number. |
| d | Replies go missing and the main loop stalls when the camera's I2C side stalls | The main loop never touches the camera or waits for it; every I2C operation has a timeout; failures are counted and reported. |
| e | I2C bus left stuck; the firmware stopped in its boot scan until power was cycled | Bounded transactions; bus recovery after 5 failures; `AT+BREAK` before every start; and `flyPursuitBootBusCheck()` just before the firmware's `Wire.begin()`: if SDA or SCL is low at boot it clocks SCL 9 times and sends a STOP, and prints `@FP boot bus ...;` on USB. This helps only if the camera holds SDA; if it holds SCL low, only a power cycle helps. The cause of the 2026-11-18 hang is not known. |
| f | Saves "camera on" and restarts at boot with its own body-leaning reaction | The module state is saved by the firmware as for every module, but a start at boot is the passive state: lines only, on-board control off, no posture change. This module has no reaction of its own. |

Plus the case reported on 2026-11-18: 0x28 present and 0x62 missing (the camera's AI side not
answering, until the camera loses power) is recognised at every start attempt and reported as
`@FP err silent <n>;`.

## Build

Toolchain as for fly_escape (esp32 core 2.0.12 with `#define CONFIG_DISABLE_HAL_LOCKS 1`
appended to `tools/sdk/esp32/qio_qspi/include/sdkconfig.h`; ArduinoJson 7.4.3, WebSockets 2.7.2,
WiFiManager 2.0.17, MU Vision Sensor 3 1.2.2). On this Mac the core's `ctags` 5.8-arduino12 is
native arm64 now, so no ctags shim was needed; the core's `esptool` is x86-only, so a venv with
`esptool==4.5.1` is passed in (only `elf2image` runs, nothing touches a port).

For the robot (B10_251121):

```bash
git clone https://github.com/PetoiCamp/OpenCatEsp32 && cd OpenCatEsp32 && git checkout 32a1fcb
uv run flybrain-export pursuit --single-header --out <flybrain>/firmware/bittle/fly_pursuit/src
python3 <flybrain>/firmware/bittle/fly_pursuit/apply_to_opencat.py .        # E and P, all stock features
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --build-path build/fly .
```

`apply_to_opencat.py` reuses `fly_escape/apply_to_opencat.py` unchanged (only if E is not there
yet) and then adds P. It relies on two anchors fly_escape's script does not use: the voice
module's `read_voice()` block in `readSignal()` and the `  Wire.begin();` line in `initRobot()`.
Both are in `f5aba6b`; they have not been checked on `32a1fcb`. If the script stops with "did not
match the expected layout", that is where.

### Numbers from this session's build (OpenCatEsp32 `f5aba6b`, NOT the robot's commit)

Same settings for all three (`min_spiffs`, app slot 1,966,080 bytes):

| build (`f5aba6b`) | program flash | static RAM | app image (.bin) | free in the slot |
|---|---|---|---|---|
| stock | 1,767,253 | 70,940 | 1,773,008 | 193,072 |
| stock + fly_escape | 1,779,173 | 73,812 | 1,784,928 | 181,152 |
| **stock + fly_escape + fly_pursuit (all stock features kept)** | **1,796,261** | **83,980** | **1,802,016** | **164,064** |

fly_pursuit itself: +17,088 bytes of program flash and +10,168 bytes of static RAM (4,096 task
stack, 4,092 circuit state, 1,025 reply buffer, the control state with its pan history, the
shared struct). It fits with everything kept; nothing was dropped. On `32a1fcb` expect similar
numbers (the fly_escape build there was 1,777,808 bytes before this module).

Image of this session (compile only; **do not flash**, see the status at the top):
`data/tools/fw/bittlex-flyescape-flypursuit-f5aba6b-COMPILE-ONLY-9c9274b3.bin`, SHA-256
`9c9274b3e27253c992b019f201552b90bb6a5ecc8c02246536d4b9f91564c090`.

## Equivalence of the C circuit with the Python model

`uv run python -m flybrain_edge.export.pursuit_equivalence` (laptop C compiler, the header as a
shared library through `ctypes`). On 29 of the 114 episodes of
`pursuit_tasks.benchmark_set()` (every 4th; 3,150 frames):

| | result |
|---|---|
| Largest difference in the turn command, same inputs frame by frame | 8.7e-4 rad/s (0.05 deg/s); the largest turn in those episodes is 7.9 rad/s |
| Closed loop with the C model driving the simulated robot: largest bearing difference | 0.007 deg |
| Steering curve (-90..90 deg, 1 s held, open loop): largest difference | 1.0e-3 rad/s (peak 7.9 rad/s) |

The test asserts 5e-3 rad/s (0.3 deg/s; the head is limited to 60 deg/s) and 0.1 deg.

Run time per step on the board is NOT measured: per result, two copies x about 7 sub-steps x
1,840 multiply-adds (about 26,000) and 2 x 231 `expf`; estimated around 1 ms at 240 MHz.

## What to check on the robot (in this order; robot on the floor, USB first)

1. Rebuild from `32a1fcb` (above) and check the banner says `B10_251121` and `Module count: 13`
   with `E` and `P` in the `X?` list. Flash to `app1` as for fly_escape, keep the backup.
2. Camera unplugged: `XP~` -> `@FP on l=1 b=0;`, then `@FP err nocam 1;`, `2;` ... every 2 s.
   `gp` must still print IMU values (requirement b). `Xp~` stops the errors.
3. Camera plugged in: `XP~` -> `@FP cam run 1;` and about 15 `@FP` lines a second; `-` with no
   face, boxes with a face. Check the rate (count lines and `seq` gaps over 20 s), and that a
   face on the dog's LEFT gives negative x.
4. `Xp~` -> `@FP cam stop;` and no more lines; `XP~` again -> `@FP cam run 2;` and lines again.
   Repeat 5 times in one power-on (requirement a).
5. While lines run: `gp` repeatedly, and look for `Camera I2C lock timeout` on USB (the IMU
   waited more than 500 ms). Watch for `@FP err lock`.
6. Over Bluetooth with lines on: are command echoes still answered, and how many `@FP` lines
   arrive per second? If too many, `XPp~`.
7. Save and restart: with `P` on, restart the board (EN). It must come up passive: lines only,
   no movement, `l=1 b=0`.
8. `XPB~` with a face about 50 cm away: the robot sits, the head turns toward the face, no
   oscillation; the `t=` and `p=` fields look sane; with the face gone, `hold` after 3 s.
   Positive `p=` must be the dog's left.
9. Pull the camera's plug while lines run: expect `err i2c`, then `err recover 1`, then
   `nocam` every 2 s; plug it back in: `cam run` again, without restarting the ESP32.
10. If 0x62 ever disappears while 0x28 and 0x79 stay (the 2026-11-17 state): expect
    `@FP err silent <n>;` and power-cycle the camera.
11. Measure how long one control step takes (not measured) if the head looks late.
