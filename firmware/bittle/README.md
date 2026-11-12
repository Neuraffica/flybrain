# Bittle firmware notes

Planned: the OpenCat firmware version to flash, any patches (for example a faster IMU readout), and wiring for the XIAO on the BiBoard's second UART. Upstream: https://github.com/PetoiCamp/OpenCatEsp32

## Modules

- [`fly_escape/`](fly_escape/README.md): draft OpenCatEsp32 extension (letter `E`) that runs the
  exported FlyWire looming-escape circuit on the BiBoard itself. Compiles for BiBoard V1
  (esp32 core 2.0.12; +20.2 KB flash, +4.3 KB RAM); not yet flashed or run on a robot.
