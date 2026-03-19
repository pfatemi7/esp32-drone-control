# ESP32 Drone Control

Arduino firmware for an ESP32-based companion computer that talks to a Pixhawk flight controller over **MAVLink** (ArduCopter). It runs a small mission state machine (GUIDED mode, arm, takeoff, hold, altitude monitoring) and reads range from a **LIDAR-Lite v3** via PWM.

The sketch targets a **Heltec** ESP32 board (with OLED) using the Heltec unofficial library.

## Features

- MAVLink companion heartbeat and command/ack handling toward the flight controller
- Configurable UART pins and baud for the Pixhawk telemetry port
- LIDAR-Lite v3 distance via trigger/monitor pins and pulse timing
- On-screen status via the built-in OLED

## Hardware notes

- **Pixhawk ↔ ESP32 UART:** defaults in the sketch use `Serial1` with RX/TX and baud defined at the top of `esp32_drone_control.ino`—match these to your wiring.
- **LIDAR-Lite v3 (PWM):** trigger and monitor GPIOs are set in the sketch; adjust if your wiring differs.

## Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) or compatible toolchain for ESP32
- Board support and libraries as referenced in the sketch (`heltec_unofficial`, ESP32 `WiFi`, MAVLink C headers included as `MAVLink.h`—ensure your build includes the correct MAVLink headers for your setup)

## License

Add a license if you plan to distribute the project.
