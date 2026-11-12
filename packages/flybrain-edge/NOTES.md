# Bittle X + Arm serial protocol notes

Read from the firmware source, not from memory. All paths are in
[PetoiCamp/OpenCatEsp32](https://github.com/PetoiCamp/OpenCatEsp32) at commit `f5aba6b`
(2026-10-24, `DATE "261024"`), unless they name
[PetoiCamp/OpenCat](https://github.com/PetoiCamp/OpenCat) (`24563ca`, 2026-09-07). Line numbers
drift between versions, so search for the symbol if a line does not match.

Status: **nothing here has been checked on a real robot.** Items marked UNVERIFIED are
inferences; `uv run flybrain-bittle probe` prints the raw bytes so you can check them.

## Framing and acknowledgement

| Fact | Source |
|---|---|
| 115200 baud on USB (`Serial.begin(115200)`) | `OpenCatEsp32.ino:38` |
| The first byte is the token. Lower-case tokens are ASCII ending in `\n`; upper-case tokens are binary (int8) ending in `~`. Without a terminator the firmware stops reading after 10 ms of silence (`SERIAL_TIMEOUT`). | `src/moduleManager.h:398-445`, `src/OpenCat.h:68` |
| `\r`/`\n` are stripped from ASCII commands, so `\r\n` endings are fine | `src/moduleManager.h:430-439` |
| Every command is acknowledged by printing its token on its own line (`k`, `i`, `g`, `d`, `?`, ...). ardSerial waits for that line, comparing case-insensitively. | `src/reaction.h:1579-1592`, `serialMaster/ardSerial.py:183-221` (OpenCat) |
| Postures (`kup`, `ksit`, `d`) echo after the blocking transition; gaits (`kvtL`) echo at once; a gait with an angle (`kvtL 90`) echoes when the turn completes | `src/reaction.h:1419-1545, 1579-1592` |
| **Two commands that arrive in one burst are read as one buffer and the second is lost** (the read loop swallows everything available, then the `\n` stripping truncates). The host must wait for the echo (or about 20 ms) between commands. | `src/moduleManager.h:405-439` |
| Opening the USB port normally resets the ESP32 (ardSerial waits 3 s); boot ends with `Ready!`. A new or reset board asks `(Y/n)` questions first. | `src/OpenCat.h:657-777` (`Ready!` at :771), `src/imu.h:1013`, `ardSerial.py:686-705` |
| `?` prints the model (`Bittle X+Arm` when built with `ROBOT_ARM`) and the version (`B10_261024`), then `?` | `src/reaction.h:402-415`, `src/OpenCat.h:70-80, 110-112, 666` |
| Unknown tokens print `Undefined token!`, the argument, then the token | `src/reaction.h:1557-1562` |

## IMU

| Fact | Source |
|---|---|
| `v` / `V` exist only in the AVR OpenCat firmware (and old ESP32 builds). On current OpenCatEsp32 they print `Undefined token!` | `src/OpenCat.h:236-237` (OpenCat), `ChangeLog.md:143, 151` (OpenCatEsp32) |
| Current tokens: `gp` prints one IMU line (and turns continuous printing off), `gP` prints continuously. `g?` prints `Gyro state: Update-1 Balance-1 Print-0 Frequency-1`. All echo `g`. | `src/OpenCat.h:315-328`, `src/reaction.h:567-618` |
| Line format (ICM42670): `"ICM:%6.2f%6.2f%6.2f%7.1f%7.1f%7.1f\t"` = accel x, y, z (in units of about 10 per g), yaw, pitch, roll in degrees. MPU6050 boards print `"MCU:..."` with the same fields. With both chips present both appear on one line. Fields have no separator: `-0.34-10.05` is two numbers. | `src/imu.h:596-618` |
| The line ends with `\r\n` on USB only. Over BLE (and SSP) the IMU text is sent without a newline and runs into the next print, e.g. `ICM: ... \tg`. | `src/imu.h:604, 615, 618`, `src/io.h:11-48` |
| **Rate: at most one IMU line per 200 ms (5 Hz)**, for `gp` and `gP` alike (`PRINT6AXIS_MIN_INTERVAL = 200`). A second `gp` within 200 ms prints only the echo. | `src/imu.h:579-592` |
| The IMU itself is read every ~5 ms in a FreeRTOS task | `src/imu.h:946-973` |
| **Yaw sign: positive = counter-clockwise (left turn) seen from above.** The printed value is `-icm.ypr[0]` / `-mpu.ypr[0]`, the same negation `readIMU()` applies "to match polar coordinate convention (positive = counterclockwise)", and turning gaits use it that way (`R` subtracts from the yaw target). UNVERIFIED on hardware; the firmware's own debug string calls `R` "RIGHT (CCW)", which contradicts the code. Check with `probe --rotate-test`; override with `--yaw-sign -1`. | `src/imu.h:600, 611, 705-706, 723-724`, `src/reaction.h:1506-1519` |
| Range: DMP yaw in (-180, 180], degrees. Zero is wherever the IMU booted. | `src/imu.h:250-262` |
| **No raw gyro rate is printed.** Angular velocity has to be derived from the fused yaw. | `src/imu.h:582-639` |
| Quiet output: `gp` stops continuous IMU printing. Other unsolicited prints (exceptions, battery warnings, `Run delay`, cycle counts, BLE connect/disconnect) cannot be turned off; parse around them. | `src/reaction.h:11-226`, `src/skill.h:440-450` |

### Gyro balancing while the dog is turned by hand

`gb` turns the balance reflex off and keeps the IMU running; `gB` turns it back on. Do **not**
use bare `g` (a toggle, state unknown; over BLE it is even rewritten to `gf`/`gF`,
`src/bleUart.h:84-88`) or `gu` (stops IMU updates, so no yaw at all). Recommendation: **off**
while standing and being rotated, so the legs do not twitch while people handle the robot.
With `ROBOT_ARM` the lifted/pushed/knocked reactions are compiled out anyway; fall-over recovery
remains (`src/imu.h:882-897`, `src/reaction.h:26-192`). Send `gb` *after* the stand skill: the
first token after a rest switches balancing back on (`src/reaction.h:370-376`).

## Joints (Bittle X + Arm, firmware built with `ROBOT_ARM`)

| Index | Joint | Firmware limit (deg) | Source |
|---|---|---|---|
| 0 | head pan; the arm's base turns with it | -120..120 | `src/OpenCat.h:583-593` |
| 1 | arm lift (the "head tilt" slot) | -10..180 (-85..85 without `ROBOT_ARM`) | `src/OpenCat.h:585-589` |
| 2 | claw / pincer (the "tail" slot) | -120..120 | `src/OpenCat.h:128` comment, `src/reaction.h:912-931` (`c-2` pincer calibration), docs robot-arm page |
| 8-15 | legs | see source | `src/OpenCat.h:591-592` |

* `calibratedPWM()` clamps every joint to `angleLimit` (`src/motion.h:9`). This kit clamps the
  head pan tighter on the host (default ±80°, `--pan-limit`). UNVERIFIED that ±80° clears the
  front legs and cables.
* Head pan sign: UNVERIFIED. `skill.h` points the head +30° on skills ending in `L`
  (`offsetLR`, `src/skill.h:492`), which suggests positive = look left (CCW). Override with
  `--pan-sign -1`.
* **Moving one joint smoothly at 10-20 Hz: `i 0 <deg>`** (ASCII, simultaneous). While a posture
  is active the firmware only stores the head target and lets the skill loop slew towards it
  by at most 20° per frame, without blocking, keeping balance running; the token is restored
  to `k` afterwards (`src/reaction.h:823-1061, 1582-1586`, `src/skill.h:405-418`).
  Downside: every `i` command beeps briefly (`beep(15 + newCmdIdx, 5)`, `src/reaction.h:358-361`;
  `b0` mutes the buzzer but is stored in the board's config, so the kit does not send it).
* `I` (binary) does not beep, but it leaves the token at `I`, which stops the skill loop, and
  each command runs a blocking `transform()`; the first `I` right after a skill only sets the
  target (`src/reaction.h:1064-1131`, `src/tools.h:197-209`). Selectable with `--joint-token I`.
* `m` moves joints one after another with a 10 ms delay each (`src/reaction.h:946-949`); not
  suitable for streaming.
* Angles must stay <= 125 in binary commands: 126 is `~`, the terminator.

## Skills

From `src/InstinctBittleESP_arm.h:2401`. `R` variants are mirrored from the stored `L` table
(`src/skill.h:33-63, 440-500`); with `ROBOT_ARM` the loader first tries `<name>Arm<L|R|F>`.

| Use | Command |
|---|---|
| stand | `kup` (same angles as `kbalance`; the firmware itself returns to `up` after timed gaits) |
| rest + servos off | `d` (prints `g`, then `d`) |
| step in place turning left / right | `kvtL` / `kvtR`; stop with `kup` |
| closed-loop turn by an angle | `kvtL 90` (firmware stops at +90° yaw and loads `up`, `src/reaction.h:1503-1537`, `src/imu.h:901-934`) |
| other turning gaits | `kwkL`, `kcrL`, `ktrL`, `kbkL` (+ `R`) |

## BLE

| Fact | Source |
|---|---|
| Service `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`; host writes to RX `6E400002-...` (write and write-without-response); robot notifies on TX `6E400003-...` | `src/bleCommon.h:6-8`, `src/bleUart.h:96-130` |
| Advertised name: `uniqueName + "_BLE"`, where `uniqueName` is `Bittle` + 2 random hex digits unless renamed with `n<name>`. A real B10_250330 board advertised `BittleC2_SSP` over LE instead (see the BLE section at the end) | `src/bleUart.h:99`, `src/configConstants.h:495-549`, `src/reaction.h:417-434` |
| Host write chunk: 20 bytes (ardSerial slices at 20; fits the default MTU). A write that arrives while idle starts a new command; later writes are appended until the terminator or 10 ms. | `ardSerial.py:142-150` (OpenCat), `src/bleUart.h:46-90` |
| Robot notifies in 10-byte pieces (this source); firmware B10_250330 sends each print as one notification, without newlines (see the end) | `src/io.h:23-28` |
| **On connect the firmware sends `Petoi <MODEL>`, then forces token `d` (rest) and sends `p`.** | `src/bleUart.h:160-178` |

## Petoi AI Vision camera (Grove Vision AI V2)

| Fact | Source |
|---|---|
| Enable with `XC~` (`X` is upper case, so end with `~`, or the firmware waits 10 ms). `Xc~` disables. | `src/OpenCat.h:375-393`, `src/moduleManager.h:280-330`, docs petoi-ai-vision page |
| `XCp~` prints one result: an `=` line, then `<x-120>\t<y-120>\tsize = <w>\t<h>\t`. `XCP~` prints whenever x or y changes. Frame is 240x240, so x is -120..120 from the image centre. | `src/reaction.h:1166-1190, 1692-1704`, `src/camera.h:182-188, 248-261` |
| With printing on, Grove Vision also prints `Box[i] target=.., score=.., x=.., y=.., w=.., h=..` lines, on USB only | `src/camera.h:768-806` |
| `XCR~` / `XCr~` turn the built-in reaction (head tracking the target) on/off. It moves the head and would fight the compass needle. | `src/reaction.h:1175-1178`, `src/camera.h:267-350` |
| The camera task only runs when no command arrived for a while, and it shares the I2C bus with the IMU (`cameraLockI2c`), so enabling it can delay IMU reads. | `src/moduleManager.h:480-495`, `src/imu.h:654-661` |
| No raw images reach the host | docs petoi-ai-vision page |

So the host **can** read the horizontal position of the first detected box (pixels from the
centre, 240-wide frame) and its size. It **cannot** get images, choose the model over serial,
or know which way +x points relative to the robot without testing. `protocol.parse_camera()`
is an EXPERIMENTAL parser for that line; nothing in the demo uses it.

## Checked on a real Bittle X+Arm (firmware B10_250330, USB, 2026-11-11)

Read-only probe only; the robot has not been moved by this code yet.

- Identifies as `Bittle X+Arm`, version `B10_250330` (older than the source read above, DATE 261024).
- `gp` / `gP` work and echo `g`; `v` answers `Undefined token!`, as expected.
- IMU line is `ICM:  -1.3  -0.8   9.9  130.8   -7.6   -4.5\t`: acceleration has ONE decimal and
  fields are space separated. The parser now accepts 1 or 2 decimals.
- `gP` is NOT capped at 5 Hz on this firmware: about 230 to 360 lines per second arrive, in USB
  bursts, with 0.1 degree resolution. The demo loop now estimates turn rate over a window of at
  least 40 ms instead of between neighbouring samples.
- Yaw is not wrapped to +-180 (values above 400 were seen); the loop unwraps differences, so
  this is harmless.
- With `gP` the lines reach the host in clumps about every 1.4 s (the link is saturated), so
  the demo polls `gp` instead: about 18 readings/s, a few gaps over 0.2 s (longest seen 1.3 s).
- Yaw drifts while the robot stands untouched: about -2.5 deg/s, steady over 30 s. The demo
  measures the drift for 5 s at start (`--drift-cal`) and subtracts it; after that the heading
  stayed within about 3 degrees over 45 s. The firmware's own IMU calibration has not been run.
- Observe mode works end to end: `kup`, `gb`, polling, and `d` on exit.
- Still unverified: yaw sign, pan sign, head/arm motion (needle), hold mode, Bluetooth.
- All of the runs above were made with the battery OFF (USB power only): the firmware printed
  `Low power: 2.18V. The robot won't move.` every 10 s. So no servo has moved yet, `kup` did
  nothing, and the 1.3 s silence every 10 s is the low-power warning. Drift figures were taken
  lying down, not standing. The link now logs this message as an error.
- With the battery on (2026-11-11): `kup` stands the robot, `i 0 <deg>` pans the head (seen on
  video), `kvtL` / `kvtR` step in place, `d` rests. No echo timeouts and no 1.3 s gaps.
- Yaw DOES follow a real turn, with the opposite sign to the one inferred from source: stepping
  left (`kvtL`, 4 s) changed the yaw by -16 deg, stepping right (`kvtR`, 4 s) by +8 deg, against
  a standing drift of about -0.6 deg/s. The demo therefore defaults to `--yaw-sign -1` on a real
  robot. This rests on `kvtL` turning the robot to its own left; not yet confirmed by eye.
- Turning in place on carpet is slow: roughly 2 to 3.5 deg/s after removing drift.
- Still unverified: pan sign (which way the head turns for a positive command), hold mode,
  Bluetooth.
- Sweep mode ran on the real robot (2 cycles, +-25 deg, carpet, 29 s): every leg reached its
  target with no timeout and no stall re-issue. Legs took 1.4 to 7.3 s (about 8 to 20 deg/s,
  faster than the earlier bare `kvtL` test), overshoot after `kup` was 3 to 9 deg, and head
  commands kept flowing during the gait. Whether the arm visibly held its direction (pan sign)
  has not been confirmed by eye.
- Pan sign CONFIRMED with a dedicated test: standing, `i 0 40` held for 10 s turned the head and
  arm to the robot's own LEFT (owner's observation and video). Positive pan = left, so the
  default `--pan-sign 1` is right on this firmware, together with `--yaw-sign -1`. The earlier
  report that "the arm did not move" during sweep was about the arm lift and claw, which the
  demo never moves; the pan servo works.
- Second sweep run (2 cycles, +-35 deg, wooden floor, 32 s): no timeouts or re-issues; the owner
  watched from above and confirmed the arm held its direction while the body turned (video).
  By the IMU, left legs took about 2 s and right legs 7 to 8 s for the same 70 deg. Unexplained:
  either the robot really turns left several times faster or the yaw reading is asymmetric.
  Overshoot after `kup` was up to 7 deg.
- Blindfold experiment on the robot (sweep, 1 cycle, +-35 deg, 25 s pauses, `--blindfold-after 6`):
  - Standing still at about +49 deg, cue removed for 20 s: connectome bump confidence 0.93,
    heading error about -32 deg (the bump slid to roughly +17 deg and stayed); rewired control
    confidence 0.24, error about 134 deg; complementary filter 0 deg.
  - The blindfolded right turn stalled: three re-issues, then the 40 s timeout at +20 deg (only
    29 deg turned). Cause unknown (the right turn is the slow direction in every run). The
    connectome's small error at the end of that leg is a coincidence: its bump did not move.
  - Standing still again for 25 s: connectome confidence 0.93, error about -12 deg; rewired
    0.24, about 115 deg.
- Hold mode on the robot (70 s, driver connectome): turned by hand to about +55 deg, the robot
  stepped right in three bursts (one hit the 6 s per-turn limit) and settled at +12 deg; turned
  by hand to -36 deg, it stepped left and settled at +13 deg. It stops about 12 deg from the
  true target because it steers on the connectome heading, which carries an offset of about
  10 deg, and the exit threshold is 6 deg. Left turns overshoot; right turns are slow.

## Bluetooth LE on a real Bittle X+Arm (firmware B10_250330, 2026-11-11)

Capture: `tests/fixtures/ble_probe_b10_250330.txt` (trimmed `flybrain-bittle probe --ble
BittleC2` output, kept as a test fixture).

- Found by an LE scan under the name **`BittleC2_SSP`**, i.e. the name of its classic-Bluetooth
  SSP interface, not `..._BLE` as the source suggests. The scanner now prefers devices that
  advertise the Nordic UART service and otherwise accepts `Bittle*_BLE` and `Bittle*_SSP`.
- Connecting, writing and notifications work; streaming (`gP`) gives about 200 IMU lines/s.
- **No newlines at all over BLE.** Each print arrives as its own notification: an echo `g`
  alone; `Undefined token!` then `v`; the joint table as `=`, the index row, the angle row
  (`0,\t60,\t...` with commas), `j`. The link therefore treats a notification boundary as a
  line boundary (`protocol.NotificationFramer`); if a newline ever appears (newer source sends
  `\r\n` and 10-byte pieces, `src/io.h:11-48`) it switches to plain stream parsing.
- Some prints are glued inside one notification: an IMU line plus the following echo
  (`ICM: ... 1.8\tg`), two IMU lines, and the identify reply `Bittle X+ArmB10_250330?`
  (model, version and the `?` echo with no separators; it is preceded by a lone `G`). The
  reader splits these (`protocol.split_glued`, `parse_imu`).
- The greeting is `Petoi Bittle` then `p`.
- Before the fix every echo was missed, so every command timed out; with it, the replayed
  capture identifies `Bittle X+Arm` / `B10_250330`, recognises all echoes, parses the joint
  table and counts every IMU reading (tests/test_bittle_ble_capture.py).
- Not yet run live after the fix: needle/sweep over BLE, IMU polling (`gp`) rate over BLE.

### macOS permission

On this Mac a CLI process that touches CoreBluetooth is killed (exit 134, TCC: missing
`NSBluetoothAlwaysUsageDescription`) when started from Terminal.app or the Claude app, because
macOS attributes it to that app and neither declares Bluetooth use. A small app bundle that
declares it, started with `open -W`, gets the normal consent prompt and then works:
`flybrain-bittle macos-ble-app` creates it (`com.neuraffica.flybrain.blerunner`), and
`flybrain-bittle ble-run -- <command>` runs a command through it with the output in the calling
terminal.
- BLE live on the robot through `flybrain-bittle ble-run` (macOS helper bundle; the system
  Bluetooth prompt appeared for it and CLI BLE then worked):
  - Probe: verdict OK after one more fix. Live, the identify reply arrives as `Bittle X+Arm`,
    then `B10_250330` (exactly 10 bytes, so the framer joins it to the next print) + `?`; the
    version/echo pair is now split.
  - Observe mode, 30 s: polling gives about 16 readings/s, no gap over 0.13 s, no echo timeouts.
  - Sweep, 1 cycle at +-25 deg: both legs reached their targets (1.2 s left, 9.9 s right, one
    stall re-issue on the right leg), head commands flowed, the robot rested at the end.
  - Blindfold experiment over BLE (USB cable still plugged in): still and blindfolded for 24 s,
    connectome confidence 0.93 with 23 deg mean error, rewired 0.25 with 128 deg, filter 3 deg.
    After a blindfolded right turn to -38 deg the connectome bump had not followed (55 deg
    error, confidence 0.93). The right leg took 3.5 s this time, so the slow right turn is not
    consistent.
  - Needle over BLE (USB unplugged, 45 s, about 12 readings/s, no gaps or echo timeouts): the
    heading rose steadily to +76 deg and the pan command followed to -68 deg (mean mismatch
    about 8 deg). Whether that rise was the owner turning the robot or drift is not known; in
    the hold run that followed, the heading stayed within 1 deg for 33 s.
  - Hold over BLE (USB unplugged, 60 s): the robot stepped back toward the target twice; one
    turn hit the 6 s limit.
  - Not tried over BLE: Ctrl-C through `ble-run`.
