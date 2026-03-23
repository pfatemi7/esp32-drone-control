# ESP32 Drone Control

Arduino firmware for an **ESP32** companion computer (Heltec board with OLED) that talks to a **Pixhawk** flight controller over **MAVLink** (ArduCopter). It runs a mission state machine: GUIDED mode, arm, takeoff, **forward flight** with velocity setpoints, **LIDAR obstacle** detection, and landing.

## Project layout

| Path | Purpose |
|------|---------|
| **`esp32_drone_control/esp32_drone_control.ino`** | **Main sketch** — full mission: takeoff, forward motion, obstacle sensing, landing |
| `README.md` | This file |

Open the folder **`esp32_drone_control`** as the Arduino sketch folder (the `.ino` filename must match the folder name).

## Mission overview

1. **BOOT / link** — Wait for FC heartbeat, set **GUIDED** mode.
2. **Arm** — Send arm command.
3. **Takeoff** — Command takeoff to a target relative altitude (`TARGET_TAKEOFF_ALT_M`).
4. **Altitude check** — After a settle delay, verify altitude is within tolerance.
5. **Forward flight** — Periodically send **forward velocity** (`FORWARD_SPEED_MPS`) so the vehicle moves forward while holding altitude.
6. **Obstacle** — LIDAR range is monitored; if distance stays below `OBSTACLE_TRIGGER_M` for enough consecutive hits (`OBSTACLE_CONSECUTIVE_HITS_REQUIRED`), the companion commands **LAND**.
7. **Landing / disarm** — Landing command and optional near-ground disarm logic.

Tunable constants (UART pins, baud, speeds, thresholds) are at the top of `esp32_drone_control.ino`.

## LIDAR (LIDAR-Lite v3 PWM)

- Distance is read via **trigger + monitor** GPIOs and pulse width (see defines in the sketch).
- **Invalid / zero / near-zero** readings are **not** shown as `0.0 m` or “no signal”: they are reported and displayed as **`40.0 m`** (`LIDAR_NO_SIGNAL_REPORT_DISTANCE_M`) so they do not look like an immediate obstacle.
- Very small readings below `LIDAR_MIN_VALID_DISTANCE_M` should not count as obstacles (see obstacle logic in the sketch).

## Hardware notes

- **Pixhawk ↔ ESP32 UART:** `Serial1` — set `PIXHAWK_RX_PIN`, `PIXHAWK_TX_PIN`, and `PIXHAWK_BAUD` to match your wiring (defaults are in the sketch).
- **LIDAR-Lite v3 (PWM):** `TRIGGER_PIN` / `MONITOR_PIN` — adjust if your wiring differs.
- LIDAR PWM may be **5 V**; level-shift or divide for **3.3 V** ESP32 inputs if required.

## Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) or compatible toolchain for ESP32
- Libraries: **`heltec_unofficial`**, ESP32 core, and **MAVLink** headers available as `MAVLink.h` (match your build to the dialect your project uses)

## License

Add a license if you plan to distribute the project.
