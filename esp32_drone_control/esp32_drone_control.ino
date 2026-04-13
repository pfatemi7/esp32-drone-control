#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include "heltec_v4_pa.h"
#include <WiFi.h>
#include <math.h>
#include <string.h>
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

// ========================= LoRa command RX (from commander) =========================
static constexpr float LORA_FREQ_MHZ = 915.0f;
static constexpr float LORA_BW_KHZ = 125.0f;
static constexpr uint8_t LORA_SF = 12;
static constexpr uint8_t LORA_CR = 5;
static constexpr uint8_t LORA_SYNC_WORD = 0x12;
static constexpr int8_t LORA_TX_POWER_DBM = 22;
static constexpr uint16_t LORA_PREAMBLE = 8;
static constexpr uint8_t COMMAND_MAGIC_0 = 'K';
static constexpr uint8_t COMMAND_MAGIC_1 = 'C';
static constexpr uint8_t COMMAND_ACK_MAGIC_0 = 'K';
static constexpr uint8_t COMMAND_ACK_MAGIC_1 = 'A';
static constexpr uint8_t COMMAND_VERSION = 1;
static constexpr uint8_t COMMAND_ALT_REF_RELATIVE_HOME = 1;
static constexpr uint32_t COMMAND_LINK_TIMEOUT_MS = 4500;
static constexpr uint32_t COMMAND_ARM_TIMEOUT_MS = 12000;
static constexpr uint32_t COMMAND_GUIDED_TIMEOUT_MS = 8000;
static constexpr uint32_t COMMAND_TAKEOFF_TIMEOUT_MS = 12000;
static constexpr uint32_t COMMAND_LAND_TIMEOUT_MS = 120000;

static constexpr uint8_t TELEM_MAGIC_0 = 'K';
static constexpr uint8_t TELEM_MAGIC_1 = 'T';
static constexpr uint32_t TELEM_SEND_INTERVAL_MS = 5000;

#pragma pack(push, 1)
struct CommandPacket {
  uint8_t magic[2];
  uint8_t version;
  uint8_t command_id;
  uint16_t seq;
  uint32_t t_ms;
  uint8_t alt_ref;
  uint8_t flags;
  uint16_t reserved;
  int32_t target_lat_e7;
  int32_t target_lon_e7;
  int32_t target_alt_cm;
  uint16_t crc16;
};

struct CommandAckPacket {
  uint8_t magic[2];
  uint8_t version;
  uint8_t command_id;
  uint16_t seq;
  uint8_t status;
  uint8_t fc_mode;
  uint8_t fc_flags;
  int32_t rel_alt_cm;
  uint16_t crc16;
};

struct TelemetryPacket {
  uint8_t magic[2];
  uint8_t version;
  int32_t lat_e7;
  int32_t lon_e7;
  int32_t rel_alt_cm;
  uint8_t fc_mode;
  uint8_t fc_flags;
  uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(CommandPacket) == 28, "CommandPacket size mismatch");
static_assert(sizeof(CommandAckPacket) == 15, "CommandAckPacket size mismatch");
static_assert(sizeof(TelemetryPacket) == 19, "TelemetryPacket size mismatch");

enum CommandId : uint8_t {
  CMD_SET_GUIDED = 1,
  CMD_ARM = 2,
  CMD_TAKEOFF = 3,
  CMD_SET_TARGET = 4,
  CMD_LAND = 5,
  // One-shot from kamikaze_commander_nogps PRG: run onboard LIDAR + forward mission.
  CMD_START_AUTO_MISSION = 6
};

enum CommandAckStatus : uint8_t {
  ACK_ACCEPTED = 1,
  ACK_COMPLETE = 2,
  ACK_REJECTED = 3,
  ACK_FAILED = 4
};

enum CommandExecState : uint8_t {
  COMMAND_IDLE = 0,
  COMMAND_ACTIVE,
  COMMAND_BLOCKED_LINK_LOSS
};

static bool lora_ok = false;
static CommandExecState command_exec_state = COMMAND_IDLE;
static CommandPacket active_command = {};
static bool active_command_present = false;
static bool active_command_sent = false;
static uint32_t active_command_last_send_ms = 0;
static uint32_t active_command_start_ms = 0;
static uint32_t last_commander_packet_ms = 0;
static uint16_t last_completed_seq = 0xFFFF;
static uint8_t last_completed_cmd = 0xFF;
static uint8_t last_completed_status = 0;
static uint32_t last_completed_ack_sent_ms = 0;
static uint32_t last_completed_time_ms = 0;
static constexpr uint32_t COMPLETED_ACK_RESEND_MS = 500;
static constexpr uint32_t COMPLETED_ACK_RESEND_DURATION_MS = 15000;
static int32_t target_lat_e7 = 0;
static int32_t target_lon_e7 = 0;
static float target_alt_m = 0.0f;
static bool target_set = false;
// ============================================================================================

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
static constexpr uint8_t OBSTACLE_CONSECUTIVE_HITS_REQUIRED = 1;
static constexpr float LIDAR_MIN_VALID_DISTANCE_M = 0.01f;
static constexpr float LIDAR_NO_SIGNAL_REPORT_DISTANCE_M = 40.0f;
static constexpr uint32_t BOOT_WAIT_MS = 5000;
static constexpr uint32_t ALTITUDE_SETTLE_WAIT_MS = 5000;
static constexpr uint32_t STREAM_REQUEST_PERIOD_MS = 5000;
static constexpr uint16_t STREAM_RATE_HZ = 4;
static constexpr float SQUARE_LEG_DISTANCE_M = 2.0f;
static constexpr float SQUARE_WAYPOINT_REACHED_THRESHOLD_M = 0.8f;

// ArduCopter custom modes requested.
static constexpr uint32_t ARDUCOPTER_MODE_GUIDED = 4;
static constexpr uint32_t ARDUCOPTER_MODE_RTL    = 6;
static constexpr uint32_t ARDUCOPTER_MODE_LAND   = 9;

static constexpr float    WAYPOINT_REACHED_THRESHOLD_M = 1.5f;
static constexpr uint32_t COMMAND_SET_TARGET_TIMEOUT_MS = 120000;
static constexpr double   EARTH_RADIUS_M = 6378137.0;
// ==========================================================================================

// ========================= NEW: Flight-control mission state =========================
enum MissionState : uint8_t {
  BOOT_WAIT = 0,
  WAIT_HEARTBEAT,
  SET_GUIDED,
  ARMING,
  TAKEOFF,
  WAIT_ALTITUDE_CHECK,
  SQUARE_NAV,
  LANDING,
  DISARMING,
  FAILSAFE_LAND,
  DONE
};

static HardwareSerial &pixhawkSerial = Serial1;
// Local MAVLink mission runs only after LoRa CMD_START_AUTO_MISSION (commander PRG).
static bool local_mission_armed = false;
static MissionState mission_state = DONE;
static uint32_t mission_state_enter_ms = 0;
static bool mission_state_action_sent = false;
static uint32_t last_command_sent_ms = 0;
static uint32_t boot_start_ms = 0;
static uint8_t square_leg_index = 0;
static int32_t square_origin_lat_e7 = 0;
static int32_t square_origin_lon_e7 = 0;
static int32_t square_target_lat_e7 = 0;
static int32_t square_target_lon_e7 = 0;
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
static uint32_t last_ack_ms = 0;
static uint32_t last_stream_request_ms = 0;
static bool fc_gpi_ever_received = false;
static int32_t fc_lat_e7 = 0;
static int32_t fc_lon_e7 = 0;
static uint32_t last_telem_send_ms = 0;
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
static bool sendTargetPositionGlobal(int32_t latE7, int32_t lonE7, float altRelM);
static void sendLand(const char *label);
static void runMissionStateMachine(uint32_t now);
static void runCommanderCommandMachine(uint32_t now);
static bool isFcLinkConnected(uint32_t now);
static void requestDataStreams(uint32_t now);
static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);
static bool initLoraReceiver();
static bool receiveCommandPacket(uint32_t now);
static bool sendCommandAckPacket(uint16_t seq, uint8_t commandId, CommandAckStatus status);
static bool commandAckShowsFailureSince(uint16_t command, uint32_t sinceMs);
static float horizontalDistanceE7(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
static void offsetLatLonByMeters(int32_t baseLatE7, int32_t baseLonE7,
                                 float northMeters, float eastMeters,
                                 int32_t &outLatE7, int32_t &outLonE7);
// =============================================================================================

static void drawScreen() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "LIDAR-Lite v3 PWM");
  display.drawString(0, 12, "TRIG:47 MON:48");

  char line3[40];
  if (fc_seen_heartbeat) {
    snprintf(line3, sizeof(line3), "Mode: %s", fc_mode_text);
  } else {
    snprintf(line3, sizeof(line3), "Mode: (no FC)");
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
  const char *cmdState = "IDLE";
  if (active_command_present) {
    cmdState = "CMD";
  } else if (command_exec_state == COMMAND_BLOCKED_LINK_LOSS) {
    cmdState = "LINK_BLK";
  } else if (local_mission_armed) {
    cmdState = "AUTO";
  }
  if (isnan(fc_relative_alt_m)) {
    snprintf(line5, sizeof(line5), "Link:%s %s %s Alt:-- %s",
             linkConnected ? "OK" : "WAIT",
             cmdState,
             local_mission_armed ? missionStateName(mission_state) : "---",
             fc_is_armed ? "ARMED" : "DISARMED");
  } else {
    snprintf(line5, sizeof(line5), "Link:%s %s %s Alt:%.2f %s",
             linkConnected ? "OK" : "WAIT",
             cmdState,
             local_mission_armed ? missionStateName(mission_state) : "---",
             fc_relative_alt_m,
             fc_is_armed ? "ARMED" : "DISARMED");
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

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static bool initLoraReceiver() {
  // Heltec V4 external PA should be initialized before SX1262 begin().
  heltec_v4_pa_init();

  const int st = radio.begin(
    LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC_WORD, LORA_TX_POWER_DBM, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("[LORA] init failed, code=");
    Serial.println(st);
    return false;
  }

  radio.setCRC(true);
  radio.setDio2AsRfSwitch(true);
  radio.setRxBoostedGainMode(true);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);

  Serial.printf("[LORA] RX ready %.1f MHz BW%.0f SF%u CR%u SW0x%02X\n",
                LORA_FREQ_MHZ, LORA_BW_KHZ, static_cast<unsigned>(LORA_SF),
                static_cast<unsigned>(LORA_CR), static_cast<unsigned>(LORA_SYNC_WORD));
  return true;
}

static bool sendCommandAckPacket(uint16_t seq, uint8_t commandId, CommandAckStatus status) {
  if (!lora_ok) {
    return false;
  }
  CommandAckPacket ack = {};
  ack.magic[0] = COMMAND_ACK_MAGIC_0;
  ack.magic[1] = COMMAND_ACK_MAGIC_1;
  ack.version = COMMAND_VERSION;
  ack.command_id = commandId;
  ack.seq = seq;
  ack.status = static_cast<uint8_t>(status);
  ack.fc_mode = static_cast<uint8_t>(fc_custom_mode & 0xFF);
  uint8_t flags = 0;
  if (fc_is_armed) flags |= 0x01;
  if (isFcLinkConnected(millis())) flags |= 0x02;
  if (isAltValid()) flags |= 0x04;
  ack.fc_flags = flags;
  ack.rel_alt_cm = isAltValid() ? static_cast<int32_t>(lroundf(fc_relative_alt_m * 100.0f)) : INT32_MIN;
  ack.crc16 = crc16_ccitt_false(
    reinterpret_cast<const uint8_t *>(&ack),
    sizeof(CommandAckPacket) - sizeof(ack.crc16));

  radio.standby();
  delay(4);
  radio.clearIrqFlags(0xFFFF);
  const int st = radio.transmit(reinterpret_cast<uint8_t *>(&ack), sizeof(ack));
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  return st == RADIOLIB_ERR_NONE;
}

static bool receiveCommandPacket(uint32_t now) {
  if (!lora_ok) {
    return false;
  }

  const uint16_t irqFlags = radio.getIrqFlags();
  if (irqFlags & (RADIOLIB_SX126X_IRQ_HEADER_ERR | RADIOLIB_SX126X_IRQ_CRC_ERR)) {
    radio.clearIrqFlags(0xFFFF);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    return false;
  }
  if (!(irqFlags & RADIOLIB_SX126X_IRQ_RX_DONE)) {
    return false;
  }

  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_RX_DONE);
  const int plen = radio.getPacketLength();
  if (plen != static_cast<int>(sizeof(CommandPacket))) {
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    return false;
  }

  CommandPacket pkt = {};
  const int st = radio.readData(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  if (st != RADIOLIB_ERR_NONE) {
    return false;
  }

  if (pkt.magic[0] != COMMAND_MAGIC_0 || pkt.magic[1] != COMMAND_MAGIC_1) {
    return false;
  }
  if (pkt.version != COMMAND_VERSION) {
    return false;
  }
  const uint16_t calc = crc16_ccitt_false(
    reinterpret_cast<const uint8_t *>(&pkt),
    sizeof(CommandPacket) - sizeof(pkt.crc16));
  if (calc != pkt.crc16) {
    return false;
  }

  last_commander_packet_ms = now;

  if (pkt.seq == last_completed_seq && pkt.command_id == last_completed_cmd) {
    sendCommandAckPacket(pkt.seq, pkt.command_id, static_cast<CommandAckStatus>(last_completed_status));
    return true;
  }
  if (active_command_present &&
      pkt.seq == active_command.seq &&
      pkt.command_id == active_command.command_id) {
    sendCommandAckPacket(pkt.seq, pkt.command_id, ACK_ACCEPTED);
    return true;
  }
  if (active_command_present &&
      (pkt.seq != active_command.seq || pkt.command_id != active_command.command_id)) {
    sendCommandAckPacket(pkt.seq, pkt.command_id, ACK_REJECTED);
    return true;
  }

  if (pkt.alt_ref != COMMAND_ALT_REF_RELATIVE_HOME) {
    sendCommandAckPacket(pkt.seq, pkt.command_id, ACK_REJECTED);
    return true;
  }

  active_command = pkt;
  active_command_present = true;
  active_command_sent = false;
  active_command_last_send_ms = 0;
  active_command_start_ms = now;
  command_exec_state = COMMAND_ACTIVE;
  sendCommandAckPacket(pkt.seq, pkt.command_id, ACK_ACCEPTED);
  Serial.printf("[COMMAND] RX cmd=%u seq=%u altRef=REL_HOME alt=%.2f\n",
                static_cast<unsigned>(pkt.command_id),
                static_cast<unsigned>(pkt.seq),
                pkt.target_alt_cm / 100.0f);
  return true;
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
    case SQUARE_NAV:
      return "SQUARE";
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
    case 6:
      return "RTL";
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
  if (next == SQUARE_NAV) {
    obstacle_hit_count = 0;
  }
}

static bool shouldRunHeartbeatFailsafe(MissionState state) {
  switch (state) {
    case SET_GUIDED:
    case ARMING:
    case TAKEOFF:
    case WAIT_ALTITUDE_CHECK:
    case SQUARE_NAV:
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
        fc_lat_e7 = gpi.lat;
        fc_lon_e7 = gpi.lon;
        if (!fc_gpi_ever_received) {
          fc_gpi_ever_received = true;
          Serial.printf("[MAVLINK] First GPI: lat=%.7f lon=%.7f alt=%.3f m\n",
                        fc_lat_e7 * 1e-7, fc_lon_e7 * 1e-7, fc_relative_alt_m);
        }
      } break;

      case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);
        last_ack_command = ack.command;
        last_ack_result = ack.result;
        last_ack_ms = now;
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

  // ArduPilot libraries/GCS_MAVLink/GCS.h:
  //   magic_force_arm_value = 2989       -> skip arming checks when arming
  //   magic_force_arm_disarm_value = 21196 -> force disarm (in flight, etc.)
  // MAVLink doc also lists 21196 as generic "force"; ArduPilot distinguishes the two.
  const float param2 = arm ? 2989.0f : 21196.0f;

  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
    COMPANION_SYS_ID, COMPANION_COMP_ID, &msg,
    fc_target_sysid, fc_target_compid,
    MAV_CMD_COMPONENT_ARM_DISARM, 0,
    arm ? 1.0f : 0.0f,
    param2,
    0, 0, 0, 0, 0);

  const bool ok = sendMavlinkMessage(msg);
  Serial.print("[MAVLINK] ");
  Serial.print(label);
  Serial.print(arm ? " (force arm p2=2989) -> " : " (force disarm p2=21196) -> ");
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

static bool sendTargetPositionGlobal(int32_t latE7, int32_t lonE7, float altRelM) {
  if (!fc_seen_heartbeat) {
    Serial.println("[MAVLINK] Cannot send target: waiting for FC heartbeat");
    return false;
  }

  mavlink_message_t msg;
  const uint16_t typeMask =
    (1 << 3) | (1 << 4) | (1 << 5) |
    (1 << 6) | (1 << 7) | (1 << 8) |
    (1 << 10) | (1 << 11);

  mavlink_msg_set_position_target_global_int_pack(
    COMPANION_SYS_ID, COMPANION_COMP_ID, &msg,
    millis(),
    fc_target_sysid, fc_target_compid,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    typeMask,
    latE7,
    lonE7,
    altRelM,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f);

  const bool ok = sendMavlinkMessage(msg);
  Serial.print("[MAVLINK] SET_TARGET_GLOBAL_INT -> ");
  Serial.println(ok ? "sent" : "send failed");
  return ok;
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

static void offsetLatLonByMeters(int32_t baseLatE7, int32_t baseLonE7,
                                 float northMeters, float eastMeters,
                                 int32_t &outLatE7, int32_t &outLonE7) {
  const double baseLatDeg = static_cast<double>(baseLatE7) * 1e-7;
  const double baseLonDeg = static_cast<double>(baseLonE7) * 1e-7;
  const double baseLatRad = baseLatDeg * DEG_TO_RAD;

  const double latDeg = baseLatDeg + (static_cast<double>(northMeters) / EARTH_RADIUS_M) * RAD_TO_DEG;
  double lonDeg = baseLonDeg;

  const double cosLat = cos(baseLatRad);
  if (fabs(cosLat) > 1e-6) {
    lonDeg = baseLonDeg + (static_cast<double>(eastMeters) / (EARTH_RADIUS_M * cosLat)) * RAD_TO_DEG;
  }

  outLatE7 = static_cast<int32_t>(lround(latDeg * 1e7));
  outLonE7 = static_cast<int32_t>(lround(lonDeg * 1e7));
}

static void runMissionStateMachine(uint32_t now) {
  if (!local_mission_armed) {
    return;
  }

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
        if (!fc_gpi_ever_received) {
          Serial.println("[MISSION] Waiting for GPS before square mission starts");
          break;
        }
        Serial.print("[MISSION] Altitude check passed after 5s, current=");
        Serial.print(fc_relative_alt_m, 3);
        Serial.println(" m");
        square_origin_lat_e7 = fc_lat_e7;
        square_origin_lon_e7 = fc_lon_e7;
        square_leg_index = 0;
        changeMissionState(SQUARE_NAV, "altitude stable, start 2m square mission", now);
      } else {
        Serial.print("[MISSION] Altitude check failed after 5s, current=");
        if (isAltValid()) {
          Serial.print(fc_relative_alt_m, 3);
          Serial.println(" m. Re-sending takeoff command.");
        } else {
          Serial.println("no altitude data. Re-sending takeoff command.");
        }
        changeMissionState(TAKEOFF, "altitude not close to target", now);
      }
    } break;

    case SQUARE_NAV: {
      if (!fc_gpi_ever_received) {
        break;
      }

      if (last_read_ok) {
        const float obstacle_m = last_distance_cm / 100.0f;
        if (obstacle_m < LIDAR_MIN_VALID_DISTANCE_M) {
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
        break;
      }

      if (square_leg_index >= 4) {
        sendLand("SQUARE complete LAND");
        mission_state_action_sent = true;
        last_command_sent_ms = now;
        changeMissionState(LANDING, "square mission complete", now);
        break;
      }

      if (!mission_state_action_sent) {
        float northOffsetM = 0.0f;
        float eastOffsetM = 0.0f;
        if (square_leg_index == 0) {
          northOffsetM = SQUARE_LEG_DISTANCE_M;                    // 2m north
        } else if (square_leg_index == 1) {
          northOffsetM = SQUARE_LEG_DISTANCE_M; eastOffsetM = SQUARE_LEG_DISTANCE_M;   // then 2m east
        } else if (square_leg_index == 2) {
          eastOffsetM = SQUARE_LEG_DISTANCE_M;                     // then 2m south
        } else {
          northOffsetM = 0.0f; eastOffsetM = 0.0f;                // then 2m west (back to origin)
        }

        offsetLatLonByMeters(square_origin_lat_e7, square_origin_lon_e7,
                             northOffsetM, eastOffsetM,
                             square_target_lat_e7, square_target_lon_e7);
        sendTargetPositionGlobal(square_target_lat_e7, square_target_lon_e7, TARGET_TAKEOFF_ALT_M);
        mission_state_action_sent = true;
        last_command_sent_ms = now;
        Serial.printf("[MISSION] Square leg %u/4 target lat=%.7f lon=%.7f\n",
                      static_cast<unsigned>(square_leg_index + 1),
                      square_target_lat_e7 * 1e-7, square_target_lon_e7 * 1e-7);
      } else if (shouldRetryStateAction(now)) {
        sendTargetPositionGlobal(square_target_lat_e7, square_target_lon_e7, TARGET_TAKEOFF_ALT_M);
        last_command_sent_ms = now;
      }

      const float distToLegTargetM = horizontalDistanceE7(fc_lat_e7, fc_lon_e7,
                                                          square_target_lat_e7, square_target_lon_e7);
      if (distToLegTargetM <= SQUARE_WAYPOINT_REACHED_THRESHOLD_M) {
        Serial.printf("[MISSION] Reached leg %u target (dist=%.2fm)\n",
                      static_cast<unsigned>(square_leg_index + 1), distToLegTargetM);
        square_leg_index++;
        mission_state_action_sent = false;
        if (square_leg_index >= 4) {
          sendLand("SQUARE complete LAND");
          mission_state_action_sent = true;
          last_command_sent_ms = now;
          changeMissionState(LANDING, "all square legs reached", now);
        }
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
        Serial.println("[MISSION] Square mission complete. Waiting for next command.");
        local_mission_armed = false;
        mission_state_action_sent = true;
      }
    } break;

    default:
      break;
  }
}

static bool commandAckShowsFailureSince(uint16_t command, uint32_t sinceMs) {
  if (last_ack_command != command) {
    return false;
  }
  if (last_ack_ms < sinceMs) {
    return false;
  }
  return last_ack_result == MAV_RESULT_DENIED ||
         last_ack_result == MAV_RESULT_TEMPORARILY_REJECTED ||
         last_ack_result == MAV_RESULT_FAILED ||
         last_ack_result == MAV_RESULT_UNSUPPORTED ||
         last_ack_result == MAV_RESULT_CANCELLED;
}

static float horizontalDistanceE7(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  const double lat1Rad = (static_cast<double>(lat1) * 1e-7) * DEG_TO_RAD;
  const double lon1Rad = (static_cast<double>(lon1) * 1e-7) * DEG_TO_RAD;
  const double lat2Rad = (static_cast<double>(lat2) * 1e-7) * DEG_TO_RAD;
  const double lon2Rad = (static_cast<double>(lon2) * 1e-7) * DEG_TO_RAD;
  const double dLat = lat2Rad - lat1Rad;
  const double dLon = lon2Rad - lon1Rad;
  const double sinHalfLat = sin(dLat * 0.5);
  const double sinHalfLon = sin(dLon * 0.5);
  double a = sinHalfLat * sinHalfLat +
             cos(lat1Rad) * cos(lat2Rad) * sinHalfLon * sinHalfLon;
  if (a < 0.0) a = 0.0;
  if (a > 1.0) a = 1.0;
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return static_cast<float>(EARTH_RADIUS_M * c);
}

static void runCommanderCommandMachine(uint32_t now) {
  if (active_command_present &&
      (now - last_commander_packet_ms) > COMMAND_LINK_TIMEOUT_MS &&
      !fc_is_armed) {
    command_exec_state = COMMAND_BLOCKED_LINK_LOSS;
    active_command_present = false;
    active_command_sent = false;
    last_completed_time_ms = now;
    Serial.println("[COMMAND] Link timeout before airborne phase: command aborted");
  }

  if (!active_command_present) {
    if (command_exec_state == COMMAND_ACTIVE) {
      command_exec_state = COMMAND_IDLE;
      last_completed_ack_sent_ms = 0;
      last_completed_time_ms = now;
    }
    if (last_completed_cmd != 0xFF &&
        (now - last_completed_time_ms) < COMPLETED_ACK_RESEND_DURATION_MS &&
        (now - last_completed_ack_sent_ms) >= COMPLETED_ACK_RESEND_MS) {
      sendCommandAckPacket(last_completed_seq, last_completed_cmd,
                           static_cast<CommandAckStatus>(last_completed_status));
      last_completed_ack_sent_ms = now;
    }
    return;
  }

  const uint8_t cmdId = active_command.command_id;
  const bool shouldSend = (!active_command_sent) || (now - active_command_last_send_ms >= FC_COMMAND_RETRY_MS);

  if (cmdId == CMD_START_AUTO_MISSION) {
    if (local_mission_armed && mission_state != DONE) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_REJECTED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_REJECTED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
      Serial.println("[COMMAND] START_AUTO_MISSION rejected (mission already running)");
      return;
    }
    local_mission_armed = true;
    boot_start_ms = now;
    changeMissionState(BOOT_WAIT, "LoRa CMD_START_AUTO_MISSION (commander PRG)", now);
    sendCommandAckPacket(active_command.seq, cmdId, ACK_COMPLETE);
    last_completed_seq = active_command.seq;
    last_completed_cmd = cmdId;
    last_completed_status = ACK_COMPLETE;
    active_command_present = false;
    command_exec_state = COMMAND_IDLE;
    Serial.println("[COMMAND] START_AUTO_MISSION accepted; onboard MAVLink mission armed");
    return;
  }

  if (cmdId == CMD_SET_GUIDED) {
    if (shouldSend) {
      sendSetMode(ARDUCOPTER_MODE_GUIDED, "CMD_SET_GUIDED");
      active_command_sent = true;
      active_command_last_send_ms = now;
    }
    if (commandAckShowsFailureSince(MAV_CMD_DO_SET_MODE, active_command_start_ms)) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_FAILED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_FAILED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
      return;
    }
    if (fc_custom_mode == ARDUCOPTER_MODE_GUIDED) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_COMPLETE);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_COMPLETE;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
    }
    if (fc_custom_mode != ARDUCOPTER_MODE_GUIDED &&
        (now - active_command_start_ms) > COMMAND_GUIDED_TIMEOUT_MS) {
      Serial.println("[COMMAND] GUIDED timeout");
      sendCommandAckPacket(active_command.seq, cmdId, ACK_FAILED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_FAILED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
    }
    return;
  }

  if (cmdId == CMD_ARM) {
    if (shouldSend) {
      sendArmDisarm(true, "CMD_ARM");
      active_command_sent = true;
      active_command_last_send_ms = now;
    }
    if (commandAckShowsFailureSince(MAV_CMD_COMPONENT_ARM_DISARM, active_command_start_ms)) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_FAILED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_FAILED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
      return;
    }
    if (fc_is_armed) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_COMPLETE);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_COMPLETE;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
    }
    if (!fc_is_armed && (now - active_command_start_ms) > COMMAND_ARM_TIMEOUT_MS) {
      Serial.print("[COMMAND] ARM timeout; last ACK cmd=");
      Serial.print(last_ack_command);
      Serial.print(" result=");
      Serial.println(last_ack_result);
      sendCommandAckPacket(active_command.seq, cmdId, ACK_FAILED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_FAILED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
    }
    return;
  }

  if (cmdId == CMD_TAKEOFF) {
    if (!fc_is_armed || fc_custom_mode != ARDUCOPTER_MODE_GUIDED || !fc_gpi_ever_received) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_REJECTED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_REJECTED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
      return;
    }
    if (shouldSend) {
      const float cmdAlt = active_command.target_alt_cm / 100.0f;
      sendTakeoff(cmdAlt);
      active_command_sent = true;
      active_command_last_send_ms = now;
    }
    if (commandAckShowsFailureSince(MAV_CMD_NAV_TAKEOFF, active_command_start_ms)) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_FAILED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_FAILED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
      return;
    }
    const float cmdAlt = active_command.target_alt_cm / 100.0f;
    if (isAltValid() && fabsf(fc_relative_alt_m - cmdAlt) <= ALTITUDE_TOLERANCE_M) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_COMPLETE);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_COMPLETE;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
    }
    if ((now - active_command_start_ms) > COMMAND_TAKEOFF_TIMEOUT_MS) {
      Serial.println("[COMMAND] TAKEOFF timeout");
      sendCommandAckPacket(active_command.seq, cmdId, ACK_FAILED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_FAILED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
    }
    return;
  }

  if (cmdId == CMD_SET_TARGET) {
    if (!active_command_sent) {
      if (!fc_is_armed || fc_custom_mode != ARDUCOPTER_MODE_GUIDED || !fc_gpi_ever_received) {
        sendCommandAckPacket(active_command.seq, cmdId, ACK_REJECTED);
        last_completed_seq = active_command.seq;
        last_completed_cmd = cmdId;
        last_completed_status = ACK_REJECTED;
        active_command_present = false;
        command_exec_state = COMMAND_IDLE;
        return;
      }
    }

    if (shouldSend) {
      const float cmdAlt = active_command.target_alt_cm / 100.0f;
      if (sendTargetPositionGlobal(active_command.target_lat_e7, active_command.target_lon_e7, cmdAlt)) {
        target_lat_e7 = active_command.target_lat_e7;
        target_lon_e7 = active_command.target_lon_e7;
        target_alt_m = cmdAlt;
        target_set = true;
        active_command_sent = true;
        active_command_last_send_ms = now;
      }
    }

    if (active_command_sent && fc_gpi_ever_received) {
      const float dist = horizontalDistanceE7(fc_lat_e7, fc_lon_e7, target_lat_e7, target_lon_e7);
      if (dist <= WAYPOINT_REACHED_THRESHOLD_M) {
        Serial.printf("[COMMAND] Waypoint reached (dist=%.2fm), switching to RTL\n", dist);
        sendSetMode(ARDUCOPTER_MODE_RTL, "AUTO_RTL_AFTER_WAYPOINT");
        sendCommandAckPacket(active_command.seq, cmdId, ACK_COMPLETE);
        last_completed_seq = active_command.seq;
        last_completed_cmd = cmdId;
        last_completed_status = ACK_COMPLETE;
        active_command_present = false;
        command_exec_state = COMMAND_IDLE;
        return;
      }
    }

    if ((now - active_command_start_ms) > COMMAND_SET_TARGET_TIMEOUT_MS) {
      Serial.println("[COMMAND] SET_TARGET timeout — waypoint not reached");
      sendCommandAckPacket(active_command.seq, cmdId, ACK_FAILED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_FAILED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
    }
    return;
  }

  if (cmdId == CMD_LAND) {
    if (!fc_is_armed) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_REJECTED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_REJECTED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
      return;
    }
    if (shouldSend) {
      sendLand("CMD_LAND");
      active_command_sent = true;
      active_command_last_send_ms = now;
    }
    if (commandAckShowsFailureSince(MAV_CMD_NAV_LAND, active_command_start_ms)) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_FAILED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_FAILED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
      return;
    }
    const bool nearGround = isAltValid() && fc_relative_alt_m <= LAND_NEAR_GROUND_M;
    const bool landMode = (fc_custom_mode == ARDUCOPTER_MODE_LAND);
    if (!fc_is_armed || (landMode && nearGround)) {
      sendCommandAckPacket(active_command.seq, cmdId, ACK_COMPLETE);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_COMPLETE;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
      return;
    }
    if ((now - active_command_start_ms) > COMMAND_LAND_TIMEOUT_MS) {
      Serial.println("[COMMAND] LAND timeout");
      sendCommandAckPacket(active_command.seq, cmdId, ACK_FAILED);
      last_completed_seq = active_command.seq;
      last_completed_cmd = cmdId;
      last_completed_status = ACK_FAILED;
      active_command_present = false;
      command_exec_state = COMMAND_IDLE;
    }
    return;
  }

  sendCommandAckPacket(active_command.seq, cmdId, ACK_REJECTED);
  last_completed_seq = active_command.seq;
  last_completed_cmd = cmdId;
  last_completed_status = ACK_REJECTED;
  active_command_present = false;
  command_exec_state = COMMAND_IDLE;
}
// =============================================================================================

static bool sendTelemetryPacket(uint32_t now) {
  if (!lora_ok) return false;

  TelemetryPacket pkt = {};
  pkt.magic[0] = TELEM_MAGIC_0;
  pkt.magic[1] = TELEM_MAGIC_1;
  pkt.version = COMMAND_VERSION;
  pkt.lat_e7 = fc_lat_e7;
  pkt.lon_e7 = fc_lon_e7;
  pkt.rel_alt_cm = isAltValid()
    ? static_cast<int32_t>(lroundf(fc_relative_alt_m * 100.0f))
    : INT32_MIN;
  pkt.fc_mode = static_cast<uint8_t>(fc_custom_mode & 0xFF);
  uint8_t flags = 0;
  if (fc_is_armed)          flags |= 0x01;
  if (isFcLinkConnected(now)) flags |= 0x02;
  if (isAltValid())         flags |= 0x04;
  if (fc_gpi_ever_received) flags |= 0x08;
  pkt.fc_flags = flags;
  pkt.crc16 = crc16_ccitt_false(
    reinterpret_cast<const uint8_t *>(&pkt),
    sizeof(TelemetryPacket) - sizeof(pkt.crc16));

  radio.standby();
  delay(4);
  radio.clearIrqFlags(0xFFFF);
  const int st = radio.transmit(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);

  if (st == RADIOLIB_ERR_NONE) {
    Serial.printf("[TELEM] Sent GPS lat=%.7f lon=%.7f alt=%dcm\n",
                  pkt.lat_e7 * 1e-7, pkt.lon_e7 * 1e-7, pkt.rel_alt_cm);
    return true;
  }
  Serial.printf("[TELEM] TX_ERR %d\n", st);
  return false;
}

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
  mission_state = DONE;
  mission_state_enter_ms = boot_start_ms;
  mission_state_action_sent = true;
  local_mission_armed = false;
  Serial.println("Companion MAVLink mission initialized");
  Serial.print("Pixhawk UART RX=");
  Serial.print(PIXHAWK_RX_PIN);
  Serial.print(" TX=");
  Serial.print(PIXHAWK_TX_PIN);
  Serial.print(" BAUD=");
  Serial.println(PIXHAWK_BAUD);
  // ================================================================================

  lora_ok = initLoraReceiver();
  Serial.print("LoRa command RX: ");
  Serial.println(lora_ok ? "ready" : "init failed");

  Serial.println("LIDAR-Lite v3 PWM reader ready");
  Serial.println("Trigger pin: 47, Monitor pin: 48");
}

void loop() {
  unsigned long now = millis();

  // Commander-driven control: parse LoRa commands, monitor FC state continuously.
  receiveCommandPacket(now);
  processIncomingMavlink(now);
  sendCompanionHeartbeat(now);
  requestDataStreams(now);
  runCommanderCommandMachine(now);
  runMissionStateMachine(now);

  if (now - last_telem_send_ms >= TELEM_SEND_INTERVAL_MS) {
    sendTelemetryPacket(now);
    last_telem_send_ms = now;
  }

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
