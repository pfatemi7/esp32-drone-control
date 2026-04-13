# ESP32 Kamikaze / Drone Control

Arduino firmware for an **ESP32** companion (Heltec V4 with OLED and SX1262 LoRa) that talks to a **Pixhawk** flight controller over **MAVLink** (ArduCopter). A handheld **commander** board sends commands over **LoRa**; the companion runs guided missions, reads **LIDAR-Lite v3** over PWM for obstacle handling, and periodically sends **GPS telemetry** back to the commander.

## Project layout

| Path | Purpose |
|------|---------|
| **`esp32_drone_control/esp32_drone_control.ino`** | **Drone companion** — MAVLink to FC, LoRa command RX + ACK, LoRa telemetry TX (~5 s), LIDAR, onboard mission (square + obstacle) |
| **`esp32_drone_control/heltec_v4_pa.h`** | Heltec V4 external PA / RF path init for SX1262 |
| **`kamikaze_commander_nogps/kamikaze_commander_nogps.ino`** | **Handheld commander** — sends `CMD_START_AUTO_MISSION` (PRG), receives telemetry, shows drone lat/lon on OLED |
| **`kamikaze_commander_nogps/heltec_v4_pa.h`** | Same PA helper as drone (match hardware) |
| `README.md` | This file |

Open each **folder** as its own Arduino sketch (folder name must match the `.ino` name).

## LoRa protocol (summary)

- **Commands → drone:** `CommandPacket` (magic `KC`, CRC16-CCITT-false), same binary layout on drone and commander.
- **ACKs ← drone:** `CommandAckPacket` (magic `KA`).
- **Telemetry ← drone (every ~5 s):** `TelemetryPacket` (magic `KT`) — lat/lon (deg × 1e7), relative altitude (cm), FC mode/flags — so the commander can display position without its own GPS.

Radio parameters (915 MHz, SF12, etc.) must match between sketches.

## Drone mission (`CMD_START_AUTO_MISSION` from commander PRG)

After GUIDED, arm, and takeoff to `TARGET_TAKEOFF_ALT_M`, the companion runs a **2 m square** in **NED** sense relative to the captured origin: **north → east → south → west**, using **WGS84-style offsets** (Earth radius `6378137` m) to compute target lat/lon, then `SET_POSITION_TARGET_GLOBAL_INT` for each corner.

**Horizontal speed** to those targets is **not** set in this firmware; ArduPilot chooses speed from parameters such as **`WPNAV_SPEED`** (and acceleration limits).

During **`SQUARE_NAV`**, **LIDAR** is still evaluated each control tick: if range is valid and below **`OBSTACLE_TRIGGER_M`** for **`OBSTACLE_CONSECUTIVE_HITS_REQUIRED`** consecutive hits, the companion commands **LAND** (same idea as the older forward-flight mission).

## LIDAR (LIDAR-Lite v3 PWM)

- Distance via **trigger + monitor** GPIOs (see defines in `esp32_drone_control.ino`).
- Invalid / near-zero readings are mapped to a **max-range display** (`LIDAR_NO_SIGNAL_REPORT_DISTANCE_M`, default 40 m) so they do not look like an immediate obstacle.
- Readings below `LIDAR_MIN_VALID_DISTANCE_M` reset / skip obstacle counting as in the sketch.

## Hardware notes

- **Pixhawk ↔ ESP32 UART:** `Serial1` — set `PIXHAWK_RX_PIN`, `PIXHAWK_TX_PIN`, and `PIXHAWK_BAUD` in the drone sketch.
- **LIDAR PWM** may be **5 V**; use a divider or level shifter for **3.3 V** ESP32 inputs if needed.
- **Heltec LoRa:** ensure `heltec_v4_pa_init()` matches your board revision.

## Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) or compatible ESP32 toolchain
- Libraries: **`heltec_unofficial`**, ESP32 core, **MAVLink** as `MAVLink.h` (dialect must match your FC / generated headers)

## License

Add a license if you plan to distribute the project.
