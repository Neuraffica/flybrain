# Hardware assessment

What the provided devices can do, how to talk to them, and where the model should run.
Researched 2026-11-11 from vendor docs. Items marked **(verify)** were not confirmed on a
primary page or the vendor pages disagree; check them on a real unit before the event.

## Seeed XIAO ESP32-S3 (Sense)

| | |
|---|---|
| MCU | ESP32-S3R8, dual-core Xtensa LX7, 240 MHz, with vector instructions used by ESP-NN |
| Memory | 8 MB PSRAM, 8 MB flash |
| Radio | 2.4 GHz WiFi, BLE 5.0 |
| I/O | 11 GPIO, 1 UART, 1 I2C, 1 SPI, USB-C, LiPo charger |
| Sense board adds | Camera (OV2640 / OV3660 / OV5640 depending on batch), PDM microphone, SD slot |
| Missing | No IMU on either variant. Add one over I2C or read the robot's |

Inference speed: Espressif's own benchmark for the int8 person-detection model (96x96
grayscale, about 250 KB) is 54 ms per invoke on ESP32-S3 with ESP-NN, against 2300 ms
without it and 380 ms on a classic ESP32. So about 18 fps for model invoke alone, less once
camera capture and preprocessing are added.

Runtimes: TFLite Micro with ESP-NN (confirmed), Edge Impulse (Seeed ships Arduino libraries;
needs PSRAM enabled). ESP-DL, MicroPython + ulab and emlearn are likely to work but were not
confirmed **(verify)**.

Our own budget estimate for a sparse recurrent net written in plain C (not measured): one
step costs one multiply-add per synapse. A reservoir of 2,000 neurons and 100,000 synapses
at 50 Hz is 5 million multiply-adds per second, well inside what the chip can do, and takes
about 300 KB as int8 weights with int16 column indices. Measuring this is the first job of
`flybrain-edge`.

## Petoi Bittle X (with arm, camera and sensor pack)

| | |
|---|---|
| Main board | BiBoard V1, ESP32 (classic, dual-core LX6). Petoi pages disagree on the module: WROOM-32D with 16 MB flash or U4WDH with 4 MB **(verify)** |
| Joints | 8 leg servos + 1 neck. The arm variant adds 2 servos (arm, claw) on the head |
| IMU | MPU6050 or ICM42670, on board |
| Battery | 7.4 V 1000 mAh, about 1 hour |
| Links | USB-C serial at 115200, Bluetooth classic SPP and BLE (Nordic-UART style), WiFi, second UART for a companion computer, 4 Grove ports |
| Camera | Petoi AI Vision module (Cortex-M55 + Ethos-U55). Runs SenseCraft models and reports detections. It does not stream raw images |
| Sensor pack | Light, touch, gesture, PIR, IR distance, ultrasonic, voice (Grove) |

### How you control it

The OpenCat firmware takes short text commands over any serial link:

| Token | Effect |
|---|---|
| `kwkF`, `kwkL`, `kbk`, `kbalance` | Run a built-in skill (walk forward, walk left, back, stand) |
| `i 8 30 9 -20 ...` | Move listed joints to angles at the same time (`I` is the binary form) |
| `m 0 45` | Move joints one after another |
| `j` | Print all joint angles |
| `v` / `V` | Print IMU data (output format **(verify)**) |
| `f` | Servo position feedback (feedback servos only) |
| `K` | Upload and run a temporary skill (a new gait) without reflashing |
| `G` | Toggle gyro balance |
| `XS` | Switch command input to the second UART |
| `d` | Rest |

Python: `ardSerial.py` and the `PetoiRobot` library (`autoConnect()`, `sendSkillStr()`,
`rotateJoints()`, `getAngleList()`). Documented for USB serial only.

Gaits are tables of int8 joint angles: 8 values per frame for joints 8 to 15. A custom
gait is either streamed frame by frame with `i`/`I`, or uploaded once with `K`.

Streaming rate is undocumented. A full ASCII `i` command is about 100 bytes, roughly 9 ms
on the wire at 115200 baud, so 20 to 50 Hz looks feasible. **Measure it** (verify).

## Where should the model run?

Short answer: you do not need Bluetooth or WiFi. A small model can run fully on board.
Three tiers, all supported by the starter kit:

| Tier | Brain runs on | Link to the dog | Good for | Risk |
|---|---|---|---|---|
| A. On-device | XIAO ESP32-S3 mounted on the dog | Wired UART to BiBoard (3.3 V both sides) | Reflexes, looming, heading, small reservoirs, tiny CNNs. Best fit for the "run it on a device" rule | Wiring and mounting; UART2 pins on BiBoard V1 **(verify)** |
| B. Tethered laptop | Laptop | USB serial cable | Fastest to start, any model size, easy debugging | Cable limits movement |
| C. Wireless laptop or server | Laptop / GPU box | BLE for commands, WiFi for camera frames | Whole-brain simulation, flyvis, anything too big for the MCU | 2.4 GHz congestion with 50+ boards in one room; latency jitter |

Recommendations:

- **Do not run large models on the BiBoard.** It is a classic ESP32 without vector
  instructions and it is busy driving servos. Small sensor reflexes are the exception: the
  exported fly-escape circuit (351 neurons, 2,212 synapses) needs about 16 KB of flash, 4 KB
  of RAM and an estimated 0.5 to 2 ms per 35 ms step, and can run as an OpenCat firmware
  module (see `firmware/bittle/fly_escape/`). It compiles into the stock firmware for
  BiBoard V1, adding 20.2 KB of flash and 4.3 KB of RAM; it has not been flashed or run on
  a robot, and its run time is an estimate. Anything bigger, or anything that needs images, goes on the XIAO or a laptop.
- **Default to tier B in the first hour, then move to tier A.** The same Python model that
  drives the dog over USB gets exported to C and flashed on the XIAO.
- **The XIAO Sense is the eye.** The Petoi camera module cannot give raw frames, so any
  fly-vision work (motion detection, looming) needs the XIAO camera. Mount it on the head.
- **For tier C, do not depend on venue WiFi.** Put the XIAO in access-point mode or bring a
  dedicated router per table. BLE is fine for skill commands (a few bytes at 10 to 20 Hz)
  and too slow for images.
- **Decide before the event whether tier C satisfies rule 3.** A laptop driving a robot is
  arguably not "running on a device". Suggested ruling: the control loop that touches the
  motors must run on the robot or MCU; a server may add slower, higher-level signals.

### Things to test on a real unit before Dec 5

1. Max `i`/`I` command rate over USB, BLE and UART2 before servos stutter.
2. UART2 pins and `XS` behaviour on BiBoard V1 with a XIAO attached.
3. IMU output format and rate from `v`.
4. XIAO camera frame rate at 96x96 grayscale with a model running.
5. Battery life with the XIAO powered from the Grove 5 V rail.
6. Arm joint indices and whether the arm blocks the head-mounted camera.

## Sources

- https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/
- https://github.com/espressif/esp-nn
- https://www.petoi.com/pages/bittle-x-robot-dog-with-arm-specifications
- https://docs.petoi.com/biboard/biboard-v1-guide
- https://docs.petoi.com/apis/serial-protocol
- https://docs.petoi.com/apis/python-api
- https://docs.petoi.com/applications/skill-creation
- https://docs.petoi.com/extensible-modules/petoi-ai-vision
- https://docs.petoi.com/extensible-modules/robot-arm
- https://docs.petoi.com/apis/raspberry-pi-serial-port-as-an-interfac/for-biboard-v1
- https://github.com/PetoiCamp/OpenCatEsp32
