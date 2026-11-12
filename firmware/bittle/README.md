# Bittle firmware notes

Planned: the OpenCat firmware version to flash, any patches (for example a faster IMU readout), and wiring for the XIAO on the BiBoard's second UART. Upstream: https://github.com/PetoiCamp/OpenCatEsp32

## Modules

- [`fly_escape/`](fly_escape/README.md): draft OpenCatEsp32 extension (letter `E`) that runs the
  exported FlyWire looming-escape circuit on the BiBoard itself. A slim build (camera, quick demo
  and micro:bit BLE client left out) compiles for BiBoard V1 with the robot's default partition
  table, 34 KB free in `app0` (esp32 core 2.0.12; module +11.5 KB flash, +2.9 KB RAM); not yet
  flashed or run on a robot.
