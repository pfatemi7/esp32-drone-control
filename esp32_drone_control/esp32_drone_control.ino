#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include <WiFi.h>
#include <math.h>
extern "C" {
#include <MAVLink.h>
}

static constexpr uint8_t OLED_CONTRAST = 255;
static constexpr int VEXT_CTRL_PIN = 36;

// ========================= NEW: Companion UART configuration =========================
// Change only these three constants to match your Heltec<->Pixhawk wiring.
static constexpr int PIXHAWK_RX_PIN = 44;      // ESP32 RX  <- Pixhawk TX
static constexpr int PIXHAWK_TX_PIN = 43;      // ESP32 TX  -> Pixhawk RX
static constexpr uint32_t PIXHAWK_BAUD = 921600;
// =====================================================================================

// LIDAR-Lite v3 PWM wiring.
static constexpr uint8_t TRIGGER_PIN = 47;
static constexpr uint8_t MONITOR_PIN = 48;

// One LIDAR reading approximately every 100 ms.
static constexpr uint32_t READ_PERIOD_MS = 100;
// Keep timeout short so distance checks do not starve velocity/altitude control updates.
static constexpr uint32_t PULSE_TIMEOUT_US = 12000;

static uint32_t last_read_ms = 0;
static uint32_t last_pulse_us = 0;
static float last_distance_cm = -1.0f;
static bool last_read_ok = false;
static const char *last_pulse_polarity = "NONE";
static const char *last_read_source = "NONE";
static const char *last_trigger_mode = "NONE";
static uint32_t timeout_count = 0;

// ========================= NEW: Flight-control mission constants =========================
static constexpr uint8_t COMPANION_SYS_ID = 200;
static constexpr uint8_t COMPANION_COMP_ID = MAV_COMP_ID_ONBOARD_COMPUTER;

static constexpr uint32_t COMPANION_HEARTBEAT_PERIOD_MS = 1000;
static constexpr uint32_t FC_HEARTBEAT_TIMEOUT_MS = 4000;

static constexpr uint32_t FC_COMMAND_RETRY_MS = 1200;
static constexpr float TARGET_TAKEOFF_ALT_M = 1.5f;
static constexpr float ALTITUDE_TOLERANCE_M = 0.1f;
static constexpr float LAND_NEAR_GROUND_M = 0.15f;
static constexpr float OBSTACLE_TRIGGER_M = 1.0f;
static constexpr float LIDAR_MIN_VALID_DISTANCE_M = 0.01f;
static constexpr float LIDAR_NO_SIGNAL_REPORT_DISTANCE_M = 40.0f;
static constexpr uint8_t OBSTACLE_CONSECUTIVE_HITS_REQUIRED = 1;
static constexpr float FORWARD_SPEED_MPS = 0.5f;
static constexpr uint32_t FORWARD_COMMAND_PERIOD_MS = 100;
static constexpr uint32_t BOOT_WAIT_MS = 5000;
static constexpr uint32_t ALTITUDE_SETTLE_WAIT_MS = 5000;
static constexpr uint32_t STREAM_REQUEST_PERIOD_MS = 5000;
static constexpr uint16_t STREAM_RATE_HZ = 4;

// ArduCopter custom modes requested.
static constexpr uint32_t ARDUCOPTER_MODE_GUIDED = 4;
static constexpr uint32_t ARDUCOPTER_MODE_LAND = 9;
// ==========================================================================================

// ========================= NEW: Flight-control mission state =========================
enum MissionState : uint8_t {
  BOOT_WAIT = 0,
  WAIT_HEARTBEAT,
  SET_GUIDED,
  ARMING,
  TAKEOFF,
  WAIT_ALTITUDE_CHECK,
  FORWARD_FLIGHT,
  LANDING,
  DISARMING,
  FAILSAFE_LAND,
  DONE
};

static HardwareSerial &pixhawkSerial = Serial1;
static MissionState mission_state = BOOT_WAIT;
static uint32_t mission_state_enter_ms = 0;
static bool mission_state_action_sent = false;
static uint32_t last_command_sent_ms = 0;
static uint32_t boot_start_ms = 0;
static uint8_t obstacle_hit_count = 0;

static uint8_t fc_target_sysid = 0;
static uint8_t fc_target_compid = 0;
static bool fc_seen_heartbeat = false;
static uint32_t last_fc_heartbeat_ms = 0;
static uint32_t last_companion_heartbeat_ms = 0;
static bool fc_is_armed = false;
static float fc_relative_alt_m = NAN;
static uint32_t fc_custom_mode = 0;
static char fc_mode_text[16] = "UNKNOWN";
static uint16_t last_ack_command = 0;
static uint8_t last_ack_result = 255;
static uint32_t last_stream_request_ms = 0;
static uint32_t last_forward_command_ms = 0;
static bool fc_gpi_ever_received = false;
// =====================================================================================

// ========================= NEW: Flight-control helpers (prototypes) =========================
static const char *missionStateName(MissionState state);
static const char *arducopterModeName(uint32_t customMode);
static const char *mavResultName(uint8_t result);
static bool isAltValid();
static bool isAltitudeWithinTargetBand();
static bool shouldRetryStateAction(uint32_t now);
static void changeMissionState(MissionState next, const char *reason, uint32_t now);
static bool shouldRunHeartbeatFailsafe(MissionState state);
static bool sendMavlinkMessage(const mavlink_message_t &msg);
static void sendCompanionHeartbeat(uint32_t now);
static void processIncomingMavlink(uint32_t now);
static void sendSetMode(uint32_t customMode, const char *label);
static void sendArmDisarm(bool arm, const char *label);
static void sendTakeoff(float altMeters);
static void sendForwardVelocity(float forwardSpeedMps);
static void sendLand(const char *label);
static void runMissionStateMachine(uint32_t now);
static bool isFcLinkConnected(uint32_t now);
static void requestDataStreams(uint32_t now);
// =============================================================================================

static void drawScreen() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "LIDAR-Lite v3 PWM");
  display.drawString(0, 12, "TRIG:47 MON:48");

  char line3[32];
  if (last_read_ok) {
    snprintf(line3, sizeof(line3), "Pulse: %lu us", (unsigned long)last_pulse_us);
  } else {
    snprintf(line3, sizeof(line3), "Pulse: timeout");
  }
  display.drawString(0, 24, line3);

  display.setFont(ArialMT_Plain_16);
  char line4[32];
  float reportedDistanceM = LIDAR_NO_SIGNAL_REPORT_DISTANCE_M;
  if (last_read_ok) {
    const float measuredM = last_distance_cm / 100.0f;
    if (measuredM >= LIDAR_MIN_VALID_DISTANCE_M) {
      reportedDistanceM = measuredM;
    }
  }
  snprintf(line4, sizeof(line4), "%.2f m", reportedDistanceM);
  display.drawString(0, 38, line4);

  // ========================= NEW: Flight-control OLED extension =========================
  display.setFont(ArialMT_Plain_10);
  char line5[96];
  const bool linkConnected = isFcLinkConnected(millis());
  if (isnan(fc_relative_alt_m)) {
    snprintf(line5, sizeof(line5), "Link:%s %s Alt:-- %s %s",
             linkConnected ? "OK" : "WAIT",
             missionStateName(mission_state),
             fc_is_armed ? "ARMED" : "DISARMED",
             fc_mode_text);
  } else {
    snprintf(line5, sizeof(line5), "Link:%s %s Alt:%.2f %s %s",
             linkConnected ? "OK" : "WAIT",
             missionStateName(mission_state),
             fc_relative_alt_m,
             fc_is_armed ? "ARMED" : "DISARMED",
             fc_mode_text);
  }
  display.drawString(0, 54, line5);
  // ======================================================================================

  display.display();
}

// Trigger a new PWM distance measurement and read the pulse width.
static bool readPulseEitherPolarity(uint8_t pin, uint32_t &pulseUs, const char *source) {
  pulseUs = pulseIn(pin, HIGH, PULSE_TIMEOUT_US);
  if (pulseUs > 0) {
    last_pulse_polarity = "HIGH";
    last_read_source = source;
    return true;
  }
  pulseUs = pulseIn(pin, LOW, PULSE_TIMEOUT_US);
  if (pulseUs > 0) {
    last_pulse_polarity = "LOW";
    last_read_source = source;
    return true;
  }
  return false;
}

static bool tryReadOnce(uint32_t &pulseUs, float &distanceCm) {
  if (readPulseEitherPolarity(MONITOR_PIN, pulseUs, "MON") ||
      readPulseEitherPolarity(TRIGGER_PIN, pulseUs, "TRIG")) {
    distanceCm = pulseUs / 10.0f;  // cm
    return true;
  }
  return false;
}

static bool readLidarPwm(uint32_t &pulseUs, float &distanceCm) {
  // Trigger mode A: idle HIGH, short LOW pulse (common mode).
  last_trigger_mode = "LOW_PULSE";
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(20);
  digitalWrite(TRIGGER_PIN, HIGH);
  if (tryReadOnce(pulseUs, distanceCm)) {
    return true;
  }

  // Trigger mode B: idle LOW, short HIGH pulse (for inverted trigger wiring).
  last_trigger_mode = "HIGH_PULSE";
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(20);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIGGER_PIN, LOW);
  if (tryReadOnce(pulseUs, distanceCm)) {
    return true;
  }

  last_pulse_polarity = "NONE";
  last_read_source = "NONE";
  last_trigger_mode = "NONE";
  return false;
}

// ========================= NEW: Flight-control helper implementations =========================
static const char *missionStateName(MissionState state) {
  switch (state) {
    case BOOT_WAIT:
      return "BOOT_WAIT";
    case WAIT_HEARTBEAT:
      return "WAIT_HB";
    case SET_GUIDED:
      return "SET_GUIDED";
    case ARMING:
      return "ARMING";
    case TAKEOFF:
      return "TAKEOFF";
    case WAIT_ALTITUDE_CHECK:
      return "ALT_CHECK";
    case FORWARD_FLIGHT:
      return "FORWARD";
    case LANDING:
      return "LANDING";
    case DISARMING:
      return "DISARMING";
    case FAILSAFE_LAND:
      return "FAILSAFE";
    case DONE:
      return "DONE";
    default:
      return "UNKNOWN";
  }
}

static const char *arducopterModeName(uint32_t customMode) {
  switch (customMode) {
    case 0:
      return "STABILIZE";
    case 3:
      return "AUTO";
    case 4:
      return "GUIDED";
    case 5:
      return "LOITER";
    case 9:
      return "LAND";
    default:
      return "UNKNOWN";
  }
}

static const char *mavResultName(uint8_t result) {
  switch (result) {
    case MAV_RESULT_ACCEPTED:
      return "ACCEPTED";
    case MAV_RESULT_TEMPORARILY_REJECTED:
      return "TEMP_REJECTED";
    case MAV_RESULT_DENIED:
      return "DENIED";
    case MAV_RESULT_UNSUPPORTED:
      return "UNSUPPORTED";
    case MAV_RESULT_FAILED:
      return "FAILED";
    case MAV_RESULT_IN_PROGRESS:
      return "IN_PROGRESS";
    case MAV_RESULT_CANCELLED:
      return "CANCELLED";
    default:
      return "UNKNOWN";
  }
}

static bool isAltValid() {
  return !isnan(fc_relative_alt_m);
}

static bool isAltitudeWithinTargetBand() {
  if (!isAltValid()) {
    return false;
  }
  return fabsf(fc_relative_alt_m - TARGET_TAKEOFF_ALT_M) <= ALTITUDE_TOLERANCE_M;
}

static bool isFcLinkConnected(uint32_t now) {
  if (!fc_seen_heartbeat) {
    return false;
  }
  return (now - last_fc_heartbeat_ms) <= FC_HEARTBEAT_TIMEOUT_MS;
}

static void requestDataStreams(uint32_t now) {
  if (!fc_seen_heartbeat) {
    return;
  }
  if (now - last_stream_request_ms < STREAM_REQUEST_PERIOD_MS) {
    return;
  }
  last_stream_request_ms = now;

  mavlink_message_t msg;

  mavlink_msg_request_data_stream_pack(
    COMPANION_SYS_ID, COMPANION_COMP_ID, &msg,
    fc_target_sysid, fc_target_compid,
    MAV_DATA_STREAM_POSITION,
    STREAM_RATE_HZ,
    1);
  sendMavlinkMessage(msg);

  mavlink_msg_request_data_stream_pack(
    COMPANION_SYS_ID, COMPANION_COMP_ID, &msg,
    fc_target_sysid, fc_target_compid,
    MAV_DATA_STREAM_EXTRA2,
    STREAM_RATE_HZ,
    1);
  sendMavlinkMessage(msg);

  Serial.print("[MAVLINK] Requested data streams (POSITION+EXTRA2) at ");
  Serial.print(STREAM_RATE_HZ);
  Serial.println(" Hz");
}

static bool shouldRetryStateAction(uint32_t now) {
  return (!mission_state_action_sent) || (now - last_command_sent_ms >= FC_COMMAND_RETRY_MS);
}

static void changeMissionState(MissionState next, const char *reason, uint32_t now) {
  if (mission_state == next) {
    return;
  }
  Serial.print("[MISSION] ");
  Serial.print(missionStateName(mission_state));
  Serial.print(" -> ");
  Serial.print(missionStateName(next));
  Serial.print(" | reason: ");
  Serial.println(reason);
  mission_state = next;
  mission_state_enter_ms = now;
  mission_state_action_sent = false;
  if (next != FORWARD_FLIGHT) {
    obstacle_hit_count = 0;
  }
  if (next != FORWARD_FLIGHT) {
    last_forward_command_ms = 0;
  }
}

static bool shouldRunHeartbeatFailsafe(MissionState state) {
  switch (state) {
    case SET_GUIDED:
    case ARMING:
    case TAKEOFF:
    case WAIT_ALTITUDE_CHECK:
    case FORWARD_FLIGHT:
    case LANDING:
    case DISARMING:
      return true;
    default:
      return false;
  }
}

static bool sendMavlinkMessage(const mavlink_message_t &msg) {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const uint16_t msgLen = mavlink_msg_to_send_buffer(buffer, &msg);
  const size_t written = pixhawkSerial.write(buffer, msgLen);
  return written == msgLen;
}

static void sendCompanionHeartbeat(uint32_t now) {
  if (now - last_companion_heartbeat_ms < COMPANION_HEARTBEAT_PERIOD_MS) {
    return;
  }

  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
    COMPANION_SYS_ID,
    COMPANION_COMP_ID,
    &msg,
    MAV_TYPE_ONBOARD_CONTROLLER,
    MAV_AUTOPILOT_INVALID,
    0,
    0,
    MAV_STATE_ACTIVE);

  if (sendMavlinkMessage(msg)) {
    last_companion_heartbeat_ms = now;
  }
}

static void processIncomingMavlink(uint32_t now) {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (pixhawkSerial.available() > 0) {
    const uint8_t c = static_cast<uint8_t>(pixhawkSerial.read());
    if (!mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      continue;
    }

    switch (msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);

        if (hb.autopilot != MAV_AUTOPILOT_INVALID) {
          const bool wasConnected = fc_seen_heartbeat;
          const bool wasArmed = fc_is_armed;
          const uint32_t oldMode = fc_custom_mode;

          fc_seen_heartbeat = true;
          last_fc_heartbeat_ms = now;
          fc_target_sysid = msg.sysid;
          fc_target_compid = msg.compid;
          fc_is_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
          fc_custom_mode = hb.custom_mode;
          snprintf(fc_mode_text, sizeof(fc_mode_text), "%s", arducopterModeName(fc_custom_mode));

          if (!wasConnected) {
            Serial.print("[MAVLINK] FC heartbeat detected sys=");
            Serial.print(fc_target_sysid);
            Serial.print(" comp=");
            Serial.println(fc_target_compid);
          }
          if (wasArmed != fc_is_armed) {
            Serial.print("[MAVLINK] Armed state changed: ");
            Serial.println(fc_is_armed ? "ARMED" : "DISARMED");
          }
          if (oldMode != fc_custom_mode) {
            Serial.print("[MAVLINK] Mode changed custom=");
            Serial.print(fc_custom_mode);
            Serial.print(" (");
            Serial.print(fc_mode_text);
            Serial.println(")");
          }
        }
      } break;

      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t gpi;
        mavlink_msg_global_position_int_decode(&msg, &gpi);
        fc_relative_alt_m = gpi.relative_alt / 1000.0f;  // mm -> m
        if (!fc_gpi_ever_received) {
          fc_gpi_ever_received = true;
          Serial.print("[MAVLINK] First GLOBAL_POSITION_INT received, relative_alt=");
          Serial.print(fc_relative_alt_m, 3);
          Serial.println(" m");
        }
      } break;

      case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);
        last_ack_command = ack.command;
        last_ack_result = ack.result;
        Serial.print("[MAVLINK] COMMAND_ACK cmd=");
        Serial.print(last_ack_command);
        Serial.print(" result=");
        Serial.print(last_ack_result);
        Serial.print(" (");
        Serial.print(mavResultName(last_ack_result));
        Serial.println(")");
      } break;

      default:
        break;
    }
  }
}

static void sendSetMode(uint32_t customMode, const char *label) {
  if (!fc_seen_heartbeat) {
    Serial.println("[MAVLINK] Cannot set mode: waiting for FC heartbeat");
    return;
  }

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    COMPANION_SYS_ID, COMPANION_COMP_ID, &msg,
    fc_target_sysid, fc_target_compid,
    MAV_CMD_DO_SET_MODE, 0,
    static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),  // param1: use custom mode
    static_cast<float>(customMode),                          // param2: ArduCopter custom mode
    0, 0, 0, 0, 0);

  const bool ok = sendMavlinkMessage(msg);
  Serial.print("[MAVLINK] ");
  Serial.print(label);
  Serial.print(" custom_mode=");
  Serial.print(customMode);
  Serial.print(" -> ");
  Serial.println(ok ? "sent" : "send failed");
}

static void sendArmDisarm(bool arm, const char *label) {
  if (!fc_seen_heartbeat) {
    Serial.println("[MAVLINK] Cannot arm/disarm: waiting for FC heartbeat");
    return;
  }

  mavlink_message_t msg;
  const float forceArmMagic = 2989.0f;      // ArduPilot force-arm code
  const float forceDisarmMagic = 21196.0f;  // ArduPilot force-disarm code
  mavlink_msg_command_long_pack(
    COMPANION_SYS_ID, COMPANION_COMP_ID, &msg,
    fc_target_sysid, fc_target_compid,
    MAV_CMD_COMPONENT_ARM_DISARM, 0,
    arm ? 1.0f : 0.0f,  // param1
    arm ? forceArmMagic : forceDisarmMagic,  // param2: force arm/disarm
    0, 0, 0, 0, 0);

  const bool ok = sendMavlinkMessage(msg);
  Serial.print("[MAVLINK] ");
  Serial.print(label);
  Serial.print(" (force)");
  Serial.print(" -> ");
  Serial.println(ok ? "sent" : "send failed");
}

static void sendTakeoff(float altMeters) {
  if (!fc_seen_heartbeat) {
    Serial.println("[MAVLINK] Cannot takeoff: waiting for FC heartbeat");
    return;
  }

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    COMPANION_SYS_ID, COMPANION_COMP_ID, &msg,
    fc_target_sysid, fc_target_compid,
    MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, altMeters);

  const bool ok = sendMavlinkMessage(msg);
  Serial.print("[MAVLINK] TAKEOFF alt=");
  Serial.print(altMeters, 2);
  Serial.print("m -> ");
  Serial.println(ok ? "sent" : "send failed");
}

static void sendForwardVelocity(float forwardSpeedMps) {
  if (!fc_seen_heartbeat) {
    Serial.println("[MAVLINK] Cannot send forward velocity: waiting for FC heartbeat");
    return;
  }

  mavlink_message_t msg;
  // Ignore position/acceleration/yaw fields, use only velocity (vx, vy, vz).
  const uint16_t typeMask = 0x0DC7;

  mavlink_msg_set_position_target_local_ned_pack(
    COMPANION_SYS_ID, COMPANION_COMP_ID, &msg,
    millis(),
    fc_target_sysid, fc_target_compid,
    MAV_FRAME_BODY_NED,
    typeMask,
    0.0f, 0.0f, 0.0f,          // position ignored
    forwardSpeedMps, 0.0f, 0.0f,  // vx, vy, vz (hold altitude with vz=0)
    0.0f, 0.0f, 0.0f,          // acceleration ignored
    0.0f, 0.0f);               // yaw and yaw_rate ignored

  const bool ok = sendMavlinkMessage(msg);
  if (!ok) {
    Serial.println("[MAVLINK] FORWARD velocity command send failed");
  }
}

static void sendLand(const char *label) {
  if (!fc_seen_heartbeat) {
    Serial.println("[MAVLINK] Cannot land: waiting for FC heartbeat");
    return;
  }

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    COMPANION_SYS_ID, COMPANION_COMP_ID, &msg,
    fc_target_sysid, fc_target_compid,
    MAV_CMD_NAV_LAND, 0,
    0, 0, 0, 0, 0, 0, 0);

  const bool ok = sendMavlinkMessage(msg);
  Serial.print("[MAVLINK] ");
  Serial.print(label);
  Serial.print(" (NAV_LAND) -> ");
  Serial.println(ok ? "sent" : "send failed");
}

static void runMissionStateMachine(uint32_t now) {
  if (fc_seen_heartbeat &&
      shouldRunHeartbeatFailsafe(mission_state) &&
      (now - last_fc_heartbeat_ms > FC_HEARTBEAT_TIMEOUT_MS) &&
      mission_state != FAILSAFE_LAND &&
      mission_state != DONE) {
    Serial.println("[FAILSAFE] FC heartbeat lost. Commanding LAND.");
    sendLand("FAILSAFE LAND");
    mission_state_action_sent = true;
    last_command_sent_ms = now;
    changeMissionState(FAILSAFE_LAND, "heartbeat timeout", now);
    return;
  }

  switch (mission_state) {
    case BOOT_WAIT: {
      if (now - boot_start_ms >= BOOT_WAIT_MS) {
        changeMissionState(WAIT_HEARTBEAT, "boot wait complete (5s)", now);
      }
    } break;

    case WAIT_HEARTBEAT: {
      if (fc_seen_heartbeat) {
        changeMissionState(SET_GUIDED, "FC heartbeat received", now);
      }
    } break;

    case SET_GUIDED: {
      if (shouldRetryStateAction(now)) {
        sendSetMode(ARDUCOPTER_MODE_GUIDED, "SET_GUIDED");
        mission_state_action_sent = true;
        last_command_sent_ms = now;
      }
      if (fc_custom_mode == ARDUCOPTER_MODE_GUIDED) {
        changeMissionState(ARMING, "GUIDED mode confirmed", now);
      }
    } break;

    case ARMING: {
      if (shouldRetryStateAction(now)) {
        sendArmDisarm(true, "ARM command");
        mission_state_action_sent = true;
        last_command_sent_ms = now;
      }
      if (fc_is_armed) {
        changeMissionState(TAKEOFF, "vehicle armed", now);
      }
    } break;

    case TAKEOFF: {
      if (!mission_state_action_sent) {
        sendTakeoff(TARGET_TAKEOFF_ALT_M);
        mission_state_action_sent = true;
        last_command_sent_ms = now;
        changeMissionState(WAIT_ALTITUDE_CHECK, "takeoff command sent, waiting 5s", now);
      }
    } break;

    case WAIT_ALTITUDE_CHECK: {
      if (now - mission_state_enter_ms < ALTITUDE_SETTLE_WAIT_MS) {
        break;
      }

      if (isAltitudeWithinTargetBand()) {
        Serial.print("[MISSION] Altitude check passed after 5s, current=");
        Serial.print(fc_relative_alt_m, 3);
        Serial.println(" m");
        changeMissionState(FORWARD_FLIGHT, "altitude stable, start forward flight + obstacle monitor", now);
      } else {
        Serial.print("[MISSION] Altitude check failed after 5s, current=");
        if (isAltValid()) {
          Serial.print(fc_relative_alt_m, 3);
          Serial.println(" m. Re-sending takeoff command.");
        } else {
          Serial.println("no altitude data. Re-sending takeoff command.");
        }
        changeMissionState(TAKEOFF, "altitude not close to 1.0m", now);
      }
    } break;

    case FORWARD_FLIGHT: {
      // Resend GUIDED velocity setpoint periodically so FC keeps speed/altitude stable.
      if ((now - last_forward_command_ms) >= FORWARD_COMMAND_PERIOD_MS) {
        sendForwardVelocity(FORWARD_SPEED_MPS);
        last_forward_command_ms = now;
      }

      if (last_read_ok) {
        const float obstacle_m = last_distance_cm / 100.0f;
        if (obstacle_m < LIDAR_MIN_VALID_DISTANCE_M) {
          // Ignore invalid zero/near-zero PWM readings (remapped to 40 m for display).
          if (obstacle_hit_count > 0) {
            Serial.println("[MISSION] Ignoring invalid LIDAR distance, counter reset");
            obstacle_hit_count = 0;
          }
        } else if (obstacle_m <= OBSTACLE_TRIGGER_M) {
          if (obstacle_hit_count < 255) {
            obstacle_hit_count++;
          }
          Serial.print("[MISSION] Obstacle hit ");
          Serial.print(obstacle_hit_count);
          Serial.print("/");
          Serial.print(OBSTACLE_CONSECUTIVE_HITS_REQUIRED);
          Serial.print(" at ");
          Serial.print(obstacle_m, 3);
          Serial.println(" m");
        } else if (obstacle_hit_count > 0) {
          Serial.print("[MISSION] Obstacle counter reset, distance=");
          Serial.print(obstacle_m, 3);
          Serial.println(" m");
          obstacle_hit_count = 0;
        }
      }

      if (obstacle_hit_count >= OBSTACLE_CONSECUTIVE_HITS_REQUIRED) {
        sendLand("LIDAR obstacle LAND");
        mission_state_action_sent = true;
        last_command_sent_ms = now;
        changeMissionState(LANDING, "obstacle threshold reached", now);
      }
    } break;

    case LANDING: {
      if (shouldRetryStateAction(now)) {
        sendLand("LANDING keepalive LAND");
        mission_state_action_sent = true;
        last_command_sent_ms = now;
      }

      if (!fc_is_armed) {
        changeMissionState(DONE, "vehicle auto-disarmed after landing", now);
      } else if (isAltValid() && fc_relative_alt_m <= LAND_NEAR_GROUND_M) {
        Serial.print("[MISSION] Near ground at ");
        Serial.print(fc_relative_alt_m, 3);
        Serial.println(" m, proceeding to disarm");
        changeMissionState(DISARMING, "near-ground altitude reached", now);
      }
    } break;

    case DISARMING: {
      if (!fc_is_armed) {
        changeMissionState(DONE, "disarm confirmed", now);
      } else if (shouldRetryStateAction(now)) {
        sendArmDisarm(false, "DISARM command");
        mission_state_action_sent = true;
        last_command_sent_ms = now;
      }
    } break;

    case FAILSAFE_LAND: {
      if (fc_seen_heartbeat && shouldRetryStateAction(now)) {
        sendLand("FAILSAFE repeat LAND");
        mission_state_action_sent = true;
        last_command_sent_ms = now;
      }

      if (!fc_is_armed) {
        changeMissionState(DONE, "failsafe landing auto-disarm complete", now);
      } else if (isAltValid() && fc_relative_alt_m <= LAND_NEAR_GROUND_M) {
        changeMissionState(DISARMING, "failsafe near-ground, disarm", now);
      }
    } break;

    case DONE: {
      if (!mission_state_action_sent) {
        Serial.println("[MISSION] Cycle complete. Waiting 5 seconds before restart.");
        mission_state_action_sent = true;
      }
      if (now - mission_state_enter_ms >= BOOT_WAIT_MS) {
        boot_start_ms = now;
        changeMissionState(BOOT_WAIT, "restart mission cycle", now);
      }
    } break;

    default:
      break;
  }
}
// =============================================================================================

void setup() {
  Serial.begin(115200);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(MONITOR_PIN, INPUT_PULLUP);
  digitalWrite(TRIGGER_PIN, HIGH);

  // OLED init (Heltec)
  pinMode(VEXT_CTRL_PIN, OUTPUT);
  digitalWrite(VEXT_CTRL_PIN, LOW);  // VEXT is active-low on Heltec V4
  delay(50);
  WiFi.mode(WIFI_OFF);
  btStop();
  heltec_setup();
  delay(100);
  display.normalDisplay();
  display.setContrast(OLED_CONTRAST);
  display.displayOn();
  drawScreen();

  // ========================= NEW: Pixhawk MAVLink UART init =========================
  pixhawkSerial.begin(PIXHAWK_BAUD, SERIAL_8N1, PIXHAWK_RX_PIN, PIXHAWK_TX_PIN);
  boot_start_ms = millis();
  mission_state_enter_ms = boot_start_ms;
  Serial.println("Companion MAVLink mission initialized");
  Serial.print("Pixhawk UART RX=");
  Serial.print(PIXHAWK_RX_PIN);
  Serial.print(" TX=");
  Serial.print(PIXHAWK_TX_PIN);
  Serial.print(" BAUD=");
  Serial.println(PIXHAWK_BAUD);
  // ================================================================================

  Serial.println("LIDAR-Lite v3 PWM reader ready");
  Serial.println("Trigger pin: 47, Monitor pin: 48");
}

void loop() {
  unsigned long now = millis();

  // ========================= NEW: MAVLink I/O + mission state machine =========================
  processIncomingMavlink(now);
  sendCompanionHeartbeat(now);
  requestDataStreams(now);
  runMissionStateMachine(now);
  // ============================================================================================

  // Keep distance checks on fixed timing without skipping control loop iterations.
  if (now - last_read_ms >= READ_PERIOD_MS) {
    last_read_ms = now;

    uint32_t pulseUs = 0;
    float distanceCm = 0.0f;

    last_read_ok = readLidarPwm(pulseUs, distanceCm);
    if (last_read_ok) {
      if (distanceCm < (LIDAR_MIN_VALID_DISTANCE_M * 100.0f)) {
        // Treat zero/near-zero invalid reads as max-range report distance.
        distanceCm = LIDAR_NO_SIGNAL_REPORT_DISTANCE_M * 100.0f;
      }
      last_pulse_us = pulseUs;
      last_distance_cm = distanceCm;
      Serial.print("Distance: ");
      Serial.print(distanceCm / 100.0f, 3);
      Serial.print(" m (");
      Serial.print(distanceCm, 1);
      Serial.print(" cm), pulse ");
      Serial.print(last_pulse_polarity);
      Serial.print(" from ");
      Serial.print(last_read_source);
      Serial.print(", trig ");
      Serial.println(last_trigger_mode);
      timeout_count = 0;
    } else {
      timeout_count++;
      Serial.print("LIDAR read timeout, monitor level=");
      Serial.print(digitalRead(MONITOR_PIN));
      Serial.print(", trig level=");
      Serial.println(digitalRead(TRIGGER_PIN));
      if (timeout_count % 20 == 0) {
        Serial.println("Hint: LIDAR-Lite v3 typically needs a clean 5V supply.");
        Serial.println("Hint: PWM output can be 5V; use a divider/level-shifter into ESP32.");
      }
    }
  }

  drawScreen();
}
