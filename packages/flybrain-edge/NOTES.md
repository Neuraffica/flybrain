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

### Tried on a real Bittle X (firmware B10_251121, Bluetooth, 2026-11-16)

The camera sat in the dog's mouth, plugged into the I2C Grove socket (marked SCL, SDA, +5V, GND).
Two runs, one worked and one hung, so treat the stock camera path as unreliable until it has
been repeated over USB, where the firmware's own diagnostics are visible.

| Observation | Detail |
|---|---|
| `XCr~` started the camera once | Replies `@FE off;`, `sit`, `X`; the robot sits. A second `XC~` gave only `X`, which per `moduleManager.h` means the module was already active, i.e. a device answered at 0x62 and accepted its settings. |
| No detection in that run | `XCp~` returned `-121.00\t-121.00\tsize = 0\t0` (x = y = -1: nothing seen) and 25 s of `XCP~` printed no line. Nobody was asked to stand in front of the lens, so this says nothing about the model on the camera. Rate and the sign of x are still unmeasured. |
| The same command hung the second time | `@FE off;` and `sit` arrived, the `X` echo never did, the Bluetooth link dropped within seconds. Afterwards the robot answered late and out of order, then could not be connected to. The user saw violent movements, the LED on pin 27 and repeated loud beeps. Cause not established: either the built-in camera reaction (on by default, `cameraReactionQ = true`, it leans the whole body toward the target and the `r` option had not been taken yet) or a restart loop. |
| The module state is saved | `reconfigureTheActiveModule` writes the whole module list to flash on every status change, and the firmware starts saved modules at boot. After a failed start the camera stays "on" in flash and is started again at every boot. |
| A failed start does not clear the saved flag | With the camera unplugged the start fails cleanly and the flag is cleared in RAM only. `Xc~` then changes nothing and saves nothing. Toggling another module forces a save: `Xa~` then `XA~` (voice off and on) wrote the list with the camera off. |
| Starting the camera switches the IMU updates off | `initModule` sets `updateGyroQ = false` even when the start fails, so `gp` prints nothing for the rest of that boot. `gU` switches the updates back on. |

Rules that follow for anything that enables the camera:

- Put the robot on the floor, away from edges. Never on a table.
- Prefer USB for the first tests: `Fail to start`, the I2C scan and the `AI.begin()` result are
  printed to USB only.
- To recover a robot that misbehaves after a camera test: power it off, unplug the camera, power
  it on, then `Xc~`, `Xa~`, `XA~`, `gU`.

### Same robot over USB, and the camera on its own USB-C port (2026-11-16)

| Observation | Detail |
|---|---|
| The robot did not see the camera on I2C | Three boot scans with the camera plugged into the I2C Grove socket, and two more after re-seating the plug: `0x28 Misc.`, `0x69 ICM42670`, `0x79 Misc.`, `0x7E Misc.`, `GroveVisionQ 0`. Nothing at 0x62, the camera's address. No scan without the camera was taken, so whether 0x28/0x79/0x7E belong to the board or to the camera is open (a forum scan of the same module shows 0x28 next to 0x62). The earlier Bluetooth-only conclusion that the camera had been found was an inference and may be wrong. |
| The boot log and module list are only on USB | An EN pulse through RTS (DTR low) restarts the board without download mode; the scan is printed about 0.6 s later. `X?~` prints the module letters and flags. |
| The camera itself is healthy | Plugged into the Mac by its own USB-C port it enumerates as a CH343 serial port (VID:PID 1A86:55D3) and answers the SSCMA AT protocol at 921600 baud: `Grove Vision AI V2`, software `2025.01.02`, model `Face Detection` (YOLO, class `Face`, score threshold 50), sensor `240x240 Auto`. Opening the port restarts the module. |
| Rate | `AT+INVOKE=-1,0,1` gives 15.4 results per second (median gap 68 ms, max 90 ms; 20 ms of that is inference). `AT+BREAK` stops it. |
| A face at about 50 cm | Found in 458 of 462 frames, score 72-83, box about 83 x 90 px of 240. Box fields are `[x, y, w, h, score, target]` with x, y the box centre. Positions move in steps of 7 px. |
| Sign of x | With the camera held in the dog's mouth, a face on the DOG'S LEFT gave x = 41 and a face roughly ahead gave x = 150: x grows toward the dog's right, so a bearing that is positive to the left is `(120 - x) * fov / 240`. This is at the module's default rotation (opt 0); the robot firmware asks for a 90 degree rotation on a "Bittle X", which was not tested. |
| Field of view | Not measured. From the face box at about 50 cm it is roughly 50-65 degrees across, i.e. the narrow lens rather than a 120 degree one. |

### Resolved the same evening: the plug had been in the wrong socket

On the BiBoard V1.0 the four Grove sockets are labelled, in order along the front edge:
`GND +5V 34 35`, `GND +5V 36 39`, `SCL SDA +5V GND` (I2C) and `GND +5V TX2(10) RX2(9)` (UART2).
A video of the incident shows the camera's plug in the end socket by the UART2 label, so the two
Bluetooth runs above were most likely made with the camera off the I2C bus. That explains "no
detection"; it does not explain the hang.

With the plug in the I2C socket, six boot scans in a row showed `0x62 AI Vision` and
`GroveVisionQ 1`. `0x28`, `0x79` and `0x7E` were there with and without the camera on the bus,
so they belong to the board. Then, over USB:

| Observation | Detail |
|---|---|
| `XCr~` works | `+ enable Camera`, the module list with `C` = 1, `Setting updateGyroQ to false...`, `IMU task exiting`, the I2C scan, `sit`, `Setup Grove Vision AI Module`, `Standard AI.begin() succeeded!`, `Set sensor angle and resolution...`, then the `X` echo 1.5 s after the command, then `Camera task activated.` |
| A face is reported | `XCp~` returns `=` and e.g. `53.00\t-44.00\tsize = 83\t90`. Box sizes match the direct USB measurement (83-90 px wide). |
| `XCP~` is not a rate | With a face held still it printed 3 lines in 15 s: it prints only when x or y changes, and positions are quantised in 7 px steps. Measuring the rate needs a moving target or polling `XCp~`. |
| `Xc~` after a successful start | `- disable Camera`, module list back to `C` = 0 (saved). `gU` then restores IMU prints. |
| Opening the USB port can restart the board | It did on most opens here, even with DTR and RTS held low. A command sent in the first ~11 s is lost: wait for `Ready!`. |
| The voice module is on by default | During one session it printed `XA+kdg` and the robot ran `up` by itself. Expect the robot to react to sounds unless voice is switched off (`Xa~`). |

Measured through the firmware with a face swaying in front of the camera (plug in the I2C
socket, voice switched off first with `Xa~`):

| | Over USB | Over Bluetooth |
|---|---|---|
| `XCr~` until the `X` echo | 1.5 s | 2.5 s |
| Firmware init path | `Standard AI.begin() succeeded!` | `Standard AI.begin() failed, using manual method...` then `Manual initialization succeeded!` |
| `XCP~` lines with a moving face | 77 in 20 s (3.9/s) | 78 in 20 s (3.9/s) |
| Shortest steady gap between lines | 63-70 ms, i.e. the camera's own 15 Hz | 60 ms at the 10th percentile, median 151 ms |
| Longest gap | 1.3 s | 2.5 s |

- A new line can arrive every camera frame (about 68 ms), but only when x or y changed, so a
  still target produces nothing: treat "no line" as "same place", and use a timeout to decide
  that the target is gone (there is no "lost" message).
- Sign of x through the firmware: a face on the DOG'S LEFT gave `x - 120` of about -72. The same
  as on the camera's own port: negative is left. Bearing positive-left = `-(x - 120) * fov / 240`.
- Polling `XCp~` every 100 ms is worse than `XCP~`: it answered 8.8 times a second but the value
  changed only 2.2 times a second, because the camera task yields to incoming commands.
- Under Bluetooth the standard start of the camera fails and the firmware's fallback takes over.
  It worked here; it is the part to suspect if a start hangs.

### The camera starts properly only once per boot

Four start/stop cycles in one Bluetooth session, with the firmware's prints logged over USB:

| Cycle | USB print when the camera task is made | Result |
|---|---|---|
| 1 (first start after a restart) | `Camera task created successfully` | positions arrive |
| 2, 3, 4 (after `Xc~`, same boot) | `Failed to create camera task, error code: -1` | `XCr~` still echoes `X` and `XCp~` still answers, but with the LAST position from before: no new reading ever arrives |

- Error -1 from `xTaskCreatePinnedToCore` is "could not allocate memory". `stopModule` for the
  camera is a stub (`// cameraStop();   // Todo`), so a second start in the same boot finds no
  room for the task. Over Bluetooth nothing reports this: the failure message is USB-only.
- So: one camera start per power-on. To use the camera again, restart the board (power button, or
  an EN pulse over USB). The firmware has no restart command that is safe to send (its
  `ESP.restart()` calls sit behind the Wi-Fi setup paths).
- A stale answer looks exactly like a still target. After a start, a program can only tell the
  two apart by moving the head and seeing whether the position follows.
- Replies can also go missing over Bluetooth while the camera runs: in two of the four cycles the
  echo of the first `XCp~` never reached the host, although USB shows the firmware answered at
  once. Do not treat a missing echo as a hung robot; the command has usually been carried out.
  This is what the "hang" in the second Bluetooth run most likely was.
- Head pan with the camera on: `i 0 -25` brought a target that was on the dog's right closer to
  the image centre, so negative pan looks right and positive looks left (one reading, with an
  uncontrolled target; repeat with a fixed face before relying on it).

- With a small face photo in view (box only 34 px wide, i.e. a weak target) and the head stepped
  through known angles, the camera rarely reported a new position after a head move. The two
  steps that did give one: `i 0 -10` moved a centred target to `x - 120 = -44` (about 4.2 px per
  degree, i.e. a field of view near 57 degrees, and again negative pan = look right). Sweeping
  the head +-10 degrees at 4 to 20 joint commands a second gave under 1 line a second. Whether
  that is the weak target or the commands starving the camera task is not settled.
- Aiming aid that worked: beep (`b 16 16`) on every new position for 30 s while the person moves
  the target; 57 positions and 24 beeps in one run.

### The fly-pursuit head loop on the robot (2026-11-18)

`fly-pursuit` with the camera read through the firmware, connectome controller, gain 1, head up
to 60 deg/s, 8 head commands a second, a person's face about 50 cm away:

| Run | Result |
|---|---|
| USB, 45 s, face held still | Clean start (the program restarts the board first) and shut-down. Head settled about 3 degrees off centre and stayed; turn commands near zero. 72 positions, 50 head commands, no missing echo. |
| Bluetooth, 60 s, face moved left and right by up to about 55 degrees | Head followed between -27 and +44 degrees, lagging by roughly 0.5 to 1 s, with no oscillation (3 reversals, each a movement of the person). 229 positions (3.8 a second), 199 head commands, no missing echo. Measured bearing of the face in the camera: median 0.5, mean 2.5, 90th percentile 7.5, largest 16.8 degrees. |

- Sending 8 head commands a second does not starve the camera task: positions kept arriving at
  about 4 a second while the head and the face moved.
- The correction for the head's own motion (subtract how far the head has turned since the
  position was measured) removed the +-19 degree swings of the first, uncorrected run.
- When the face moves faster than the head can follow, the camera stops reporting for a moment
  (the box leaves the frame or blurs) and the head stops where the corrected bearing reaches
  zero, then catches up on the next position.

Still open: how fast joint commands can be sent before the camera task falls behind (needs a
large, fixed face), and a proper field-of-view measurement.

What this suggests: read the camera on the laptop over its own USB cable and steer the robot over
Bluetooth. That path gives 15 Hz and the score, leaves the robot's IMU and other modules alone,
and does not use the firmware's camera module at all. The cost is a cable from the dog's mouth to
the laptop.

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

### Lost stand at the end of a sweep leg (BLE)

Seen on the robot over BLE (sweep, 2 cycles, ±25°): after the first two legs the loop sent
`kup` and entered its pause, but the heading kept rising at about 25°/s for the whole 3 s pause
(+29° to +105°) and only reversed when the next leg's `kvtR` arrived. The third and fourth stops
worked; over USB it never happened in four runs (overshoot 3-9°).

Cause, as far as the code shows (not confirmed with a byte capture of the failing moment):
the demo sent the stand without waiting for its echo, so the next write (an IMU poll `gp` every
50 ms, or a head `i` command) could follow 20 ms later. While the firmware's main loop is busy
(a gait frame), a BLE write that arrives before `readBle()` has consumed the previous one is
appended to it (`src/bleUart.h:46-90`: `bleMessageShift` is still 0). The buffer `up\ngp` is
then parsed as one command with an embedded newline, so the skill name `up\ngp` is not found
and nothing is loaded: the gait keeps running and the firmware still echoes `k`. Over USB
`read_serial()` truncates at the first newline instead, which keeps the FIRST command and loses
the second (`src/moduleManager.h:430-445`), which fits USB never losing a stand. Starting a
turn from standing survives because the loop is fast then.

Fixes (flybrain-edge `robot.py`, fly-compass `loop.py`):

- Skills sent with `wait=False` go through a command thread that holds the write lock until
  the `k` echo, so nothing can be written in between; the control loop is not blocked.
- Every command waits `min_gap` after the previous one (USB 20 ms, BLE 50 ms), and after a
  command whose echo was not seen (timeout, or a write without echo) the next write waits a
  guard interval (USB 0.1 s, BLE 0.5 s).
- A stop watchdog in hold and sweep: 1.5 s after a stand, if the IMU heading still changes
  faster than 3°/s (over 1 s of samples), the stand is re-sent once a second, at most 4 times,
  then `d` and observe mode. A new turn never starts before the stop is confirmed.
- The simulator can reproduce the merge (`MockBittle(merge_window_s=...)`), with the BLE and USB
  behaviours above; tests show the old way of sending loses the stand there and the new one
  does not.

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
    about 8 deg). The owner confirmed he was turning the robot slowly one way, so this is
    the needle tracking a real turn, not drift.
  - Hold over BLE (USB unplugged, 60 s): the robot stepped back toward the target twice; one
    turn hit the 6 s limit.
  - Ctrl-C through `ble-run`, sent mid-turn during a sweep ("turning left" at +37 deg): the app
    printed "stopping (Ctrl-C)", exited with code 0 after 1.5 s and left no process behind.
    The owner saw the robot stop stepping and lie down. (The joint read-back afterwards cannot
    show this: the firmware forces rest on every BLE connection.)
- The firmware's yaw is an unbounded running total (it drifts up by roughly 0.6 deg/s even at
  rest). After about 4 h powered it passed 10000 and overflowed its print field, gluing onto the
  acceleration value before it: `ICM:  -0.3   0.6   9.910565.4   -1.4    3.5`. Every IMU line
  then failed to parse and the demo ran with no heading at all (no error, no motion). The
  parser now handles the glued form. Power-cycling the board resets the total.

Checked on the robot after the fix (sweep over BLE, 3 cycles, +-25 deg): no runaway in any of
the six pauses. Each leg now ends about 20 deg past its target (for example target +25, robot
settles at +44): roughly 10 deg while the link waits for the `kup` echo and another 10 to 15
deg during the pause. Over USB the overshoot was 3 to 9 deg. The stop watchdog fired once
(still moving at 3.3 deg/s 1.5 s after the stand) and one re-send was enough. Not yet re-run
over USB with the fix.

## Analog reads: the infrared proximity sensor (firmware B10_250330, USB, 2026-11-11)

A Petoi double infrared proximity sensor on the Grove socket labelled 39/36 can be read from the
host without enabling the firmware's sensor module (which would start its own reactions; enabling
the camera with `XC~` made the robot do `sit` by itself, so the kit sends no `X` command).

| Fact | Source |
|---|---|
| Token `R` (`T_READ`), binary: pairs of type and pin byte, terminated by `~`. Type `a` (`TYPE_ANALOG`) = analogRead, `d` = digitalRead. Pins must stay below 126 (`~`). | `src/OpenCat.h:364-367` |
| For each pair the firmware prints `=` and the reading, then echoes `R`. | `src/reaction.h:1101-1112, 1579-1581` |
| Captured: TX `Ra$~` (pin 36) -> RX `=\r\n3381\r\n`, then `R\r\n`. TX `Ra$a'~` (pins 36, 39) -> `=\r\n3376\r\n=\r\n3453\r\n`, `R\r\n`. Four pins in one command also work (`Ra"a#a$a'~`); unconnected pins read about 0. | robot, USB |
| About 28 reads/s of two pins over USB with nothing else polled; the robot did not move. | robot, USB |
| No beep for `R` (excluded with the binary joint tokens). | `src/reaction.h:358-361` |
| **Side effect: the read path is shared with the binary joint tokens.** After it, the token is not restored to `k` (`R` is not in the list at `src/reaction.h:1582-1586`) and `resetCmd()` clears it, so a running gait stops (`skill->perform()` is no longer called). When the previous token was not a skill (or a stale `nonHeadJointQ` is set) it also re-asserts the current joint angles as a posture (`transform` + `convertTargetToPosture`). So: never read analog pins while a gait runs; the demo pauses reads during an escape. | `src/reaction.h:1064-1131`, `src/tools.h:197-209` |
| BLE: not yet captured. By analogy with the other prints on B10_250330, `=`, the reading and the echo should arrive as separate notifications, possibly glued (`=3381`, `3453R`); the parser accepts all of these. | inferred |

Sensor behaviour: about 3400 (+-50) with nothing in front, about 0 with a hand close; graded only
over a narrow band, so a hand approach gives 1 to 6 intermediate samples (70 to 250 ms). The two
channels fall at slightly different times. Which pin is the robot's left is not established
(`flybrain-bittle probe --analog 36,39 --side-test` checks it).

### Fly escape on the robot (2026-11-11, USB, firmware B10_250330)

- `probe --analog 36,39`: 83.7 reads/s, all complete; resting about 3250-3470.
- Observe mode: calibrated (left 3373 +- 18, right 3382 +- 13), about 27 samples/s in the app.
- Escape mode with the first fitted model: 6 escapes in 45 s (`bkF` for 1 s, then `up`), every
  stand confirmed by its echo, one trigger correctly ignored inside the 3 s minimum interval.
  The connectome and rewired models only fired when the pin-39 channel dropped; two approaches
  seen only by pin 36 were missed while both baselines caught them. The owner saw the same: the
  dog backed away from a hand on its left and not on its right. Cause: the right giant fiber
  gets 967 LC4+LPLC2 synapses against 634 on the left and one shared threshold was fitted on
  data with no one-sided approaches. Fixed in the model (a threshold per giant fiber).
- On this robot the pin-39 half of the sensor was on the robot's left: use `--swap-sides`.
- A first escape attempt was refused by calibration because a hand was in front of the sensor
  during the 2 s window ("right channel rests at 116"). Working as designed.
- Not yet run on the robot: the both-sides fix, escape over BLE, `back-turn`.

### Fly escape on the robot, day 2 (2026-11-12)

- **Daylight blinds the sensor.** Next to a window at midday both channels read about 50-170
  instead of about 3400, as if something were permanently in front of it; shading it with a hand
  only raised the reading by about 50. Moved away from the window, the same sensor read
  3150-3300 at once. Calibration refused to start in the blinded state, as designed. Run this
  demo away from windows or shield the sensor.
- **Both-sides fix confirmed** (USB, `--swap-sides`, 90 s): 6 escapes, every stand confirmed,
  4 for a hand on the robot's left (pin 39 dropping) and 2 on its right (pin 36 dropping), with
  the reported side matching the channel each time. Two more triggers were refused by the
  6-per-minute cap. Of 10 trigger events in the log: connectome 8, looming-rate 9, proximity 6,
  rewired 4, random 0. No ground truth for how many approaches there really were.
- **Analog reads over BLE work**: about 9.6 samples/s in the app (USB: about 26/s).
- **Escape mode over BLE works** (`ble-run`, `--swap-sides`, 90 s, away from the window): 3
  escapes, every stand confirmed, one trigger refused inside the 3 s minimum interval. Of 4
  trigger events: connectome 4, rewired 3, proximity 3, looming-rate 2, random 0. No approach
  seen only by the right-hand channel occurred in this run, so right-only over BLE is untested.
- The robot advertised as `BittleC2_BLE` today and `BittleC2_SSP` yesterday.

### Flydeck on the real robot (2026-11-12)

(This was the AppleScript applet made by `install-app`, since replaced by the self-contained
Flydeck.app from `uv run flydeck build-app`; the hub and demos below are the same code.)

- `uv run flydeck install-app` created `~/Applications/Flydeck.app`; opening it started the hub
  and a browser tab, with three demo cards.
- The first two Bluetooth scans hung for 45 s ("no answer within 45 s") while macOS was showing
  its Bluetooth consent prompt for Flydeck. After the owner allowed it, Scan found the robot
  (`BittleC2_SSP`, -68 dBm) in about 5 s. So the hub, started in the background by the applet,
  IS attributed to Flydeck for Bluetooth; the scan just has no way to say "waiting for
  permission".
- Check over BLE: Bittle X+Arm, B10_250330, battery 7.66 V. The firmware does answer the
  voltage query.
- Hello robot started from the app over BLE: state RUNNING, embedded dashboard live, six head
  wags counted, battery 7.58 V. Stop returned the app to STOPPED.
- Fly escape started from the app over BLE (mode escape, swap sides ticked, space confirmed):
  calibrated (left 3161 +- 13, right 3093 +- 12), state RUNNING with the dashboard embedded, 6
  escapes in about a minute (3 reported right, 2 left, 1 ahead), every stand confirmed; further
  triggers were held back by the 6-per-minute cap and the 3 s minimum interval.
- Quit while that demo was running (the page's quit request, sent directly because the browser
  automation cannot accept its confirmation dialog): the hub and the demo were gone 2 s later,
  no process left behind, log ends "flydeck: bye".
- Not yet tried through the app: fly-compass on the robot, the Dock's Cmd-Q, and Force Quit
  while a demo runs. Whether the robot lay down on quit was not measured by the host.
