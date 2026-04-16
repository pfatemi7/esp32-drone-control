// Kamikaze Commander (no GPS) — LoRa command transmitter
//
// Same binary protocol as kamikaze_commander.ino plus CMD_LAND (id 5).
// PRG button: sends CMD_START_AUTO_MISSION (6); drone runs onboard LIDAR + MAVLink mission.
//
// Open this folder as the Arduino sketch (name must match .ino).
// Drone side: esp32_drone_control.ino handles CMD_START_AUTO_MISSION.

#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include "heltec_v4_pa.h"
#include <WiFi.h>
#include <math.h>
#include <string.h>

static constexpr int VEXT_CTRL_PIN = 36;
static constexpr uint8_t OLED_CONTRAST = 255;

static constexpr float    LORA_FREQ_MHZ   = 915.0f;
static constexpr float    LORA_BW_KHZ     = 125.0f;
static constexpr uint8_t  LORA_SF         = 12;
static constexpr uint8_t  LORA_CR         = 5;
static constexpr uint8_t  LORA_SYNC_WORD  = 0x12;
static constexpr int8_t   LORA_TX_POWER   = 22;
static constexpr uint16_t LORA_PREAMBLE   = 8;

static constexpr float  TAKEOFF_ALT_M  = 3.0f;

static constexpr uint8_t CMD_MAGIC_0 = 'K';
static constexpr uint8_t CMD_MAGIC_1 = 'C';
static constexpr uint8_t CMD_ACK_MAGIC_0 = 'K';
static constexpr uint8_t CMD_ACK_MAGIC_1 = 'A';
static constexpr uint8_t TELEM_MAGIC_0 = 'K';
static constexpr uint8_t TELEM_MAGIC_1 = 'T';
static constexpr uint8_t CMD_VERSION = 1;
static constexpr uint8_t ALT_REF_RELATIVE_HOME = 1;
static constexpr uint8_t MAX_COMMAND_RETRIES = 15;
static constexpr uint32_t COMMAND_ACK_TIMEOUT_MS = 2500;

enum CommandId : uint8_t {
  CMD_SET_GUIDED = 1,
  CMD_ARM = 2,
  CMD_TAKEOFF = 3,
  CMD_SET_TARGET = 4,
  CMD_LAND = 5,
  CMD_START_AUTO_MISSION = 6
};

enum CommandAckStatus : uint8_t {
  ACK_ACCEPTED = 1,
  ACK_COMPLETE = 2,
  ACK_REJECTED = 3,
  ACK_FAILED = 4
};

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

static_assert(sizeof(CommandPacket) == 28, "CommandPacket packing unexpected");
static_assert(sizeof(CommandAckPacket) == 15, "CommandAckPacket packing unexpected");
static_assert(sizeof(TelemetryPacket) == 19, "TelemetryPacket packing unexpected");

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

enum SequenceStage : uint8_t {
  SEQ_IDLE = 0,
  SEQ_WAIT_MISSION,
  SEQ_DONE,
  SEQ_ERROR
};

static uint16_t tx_seq = 0;
static char     last_status[64] = "IDLE";
static bool     radio_ok = false;
static uint32_t last_draw_ms = 0;
static SequenceStage sequence_stage = SEQ_IDLE;
static uint32_t last_send_ms = 0;
static uint8_t retry_count = 0;
static uint16_t inflight_seq = 0;
static CommandId inflight_cmd = CMD_SET_GUIDED;
static int32_t target_alt_cm = 0;

static bool     telem_received   = false;
static int32_t  drone_lat_e7     = 0;
static int32_t  drone_lon_e7     = 0;
static int32_t  drone_alt_cm     = 0;
static uint8_t  drone_fc_mode    = 0;
static uint8_t  drone_fc_flags   = 0;
static uint32_t last_telem_rx_ms = 0;

static void drawScreen() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  display.drawString(0, 0, "Cmdr  PRG=AUTO msn");

  char latLine[32], lonLine[32];
  if (telem_received) {
    snprintf(latLine, sizeof(latLine), "Lat: %.7f", drone_lat_e7 * 1e-7);
    snprintf(lonLine, sizeof(lonLine), "Lon: %.7f", drone_lon_e7 * 1e-7);
  } else {
    snprintf(latLine, sizeof(latLine), "Lat: --");
    snprintf(lonLine, sizeof(lonLine), "Lon: --");
  }
  display.drawString(0, 12, latLine);
  display.drawString(0, 24, lonLine);

  char altLine[48];
  if (telem_received && drone_alt_cm != INT32_MIN) {
    const bool armed = (drone_fc_flags & 0x01) != 0;
    snprintf(altLine, sizeof(altLine), "Alt:%.2fm %s",
             drone_alt_cm / 100.0f, armed ? "ARM" : "DISARM");
  } else {
    snprintf(altLine, sizeof(altLine), "Alt:-- DISARM");
  }
  display.drawString(0, 36, altLine);

  char stLine[64];
  snprintf(stLine, sizeof(stLine), "Seq:%u %s St:%s",
           (unsigned)tx_seq, radio_ok ? "OK" : "ERR", last_status);
  display.drawString(0, 48, stLine);
  display.display();
}

static const char *commandName(CommandId id) {
  switch (id) {
    case CMD_SET_GUIDED: return "SET_GUIDED";
    case CMD_ARM: return "ARM";
    case CMD_TAKEOFF: return "TAKEOFF";
    case CMD_SET_TARGET: return "SET_TARGET";
    case CMD_LAND: return "LAND";
    case CMD_START_AUTO_MISSION: return "START_AUTO";
    default: return "UNKNOWN";
  }
}

static void handleAck(const CommandAckPacket &ack);

static void pollRadio() {
  if (!radio_ok) return;

  const uint16_t irqFlags = radio.getIrqFlags();
  if (irqFlags & (RADIOLIB_SX126X_IRQ_HEADER_ERR | RADIOLIB_SX126X_IRQ_CRC_ERR)) {
    radio.clearIrqFlags(0xFFFF);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    return;
  }
  if (!(irqFlags & RADIOLIB_SX126X_IRQ_RX_DONE)) {
    return;
  }

  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_RX_DONE);
  const int plen = radio.getPacketLength();

  uint8_t buf[32];
  if (plen < 3 || plen > static_cast<int>(sizeof(buf))) {
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    return;
  }

  const int st = radio.readData(buf, plen);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  if (st != RADIOLIB_ERR_NONE) return;

  if (plen == static_cast<int>(sizeof(CommandAckPacket)) &&
      buf[0] == CMD_ACK_MAGIC_0 && buf[1] == CMD_ACK_MAGIC_1) {
    CommandAckPacket ack;
    memcpy(&ack, buf, sizeof(ack));
    if (ack.version != CMD_VERSION) return;
    const uint16_t calc = crc16_ccitt_false(buf, sizeof(CommandAckPacket) - sizeof(ack.crc16));
    if (calc != ack.crc16) return;
    handleAck(ack);
    return;
  }

  if (plen == static_cast<int>(sizeof(TelemetryPacket)) &&
      buf[0] == TELEM_MAGIC_0 && buf[1] == TELEM_MAGIC_1) {
    TelemetryPacket telem;
    memcpy(&telem, buf, sizeof(telem));
    if (telem.version != CMD_VERSION) return;
    const uint16_t calc = crc16_ccitt_false(buf, sizeof(TelemetryPacket) - sizeof(telem.crc16));
    if (calc != telem.crc16) return;
    drone_lat_e7   = telem.lat_e7;
    drone_lon_e7   = telem.lon_e7;
    drone_alt_cm   = telem.rel_alt_cm;
    drone_fc_mode  = telem.fc_mode;
    drone_fc_flags = telem.fc_flags;
    telem_received = true;
    last_telem_rx_ms = millis();
    Serial.printf("[TELEM] RX lat=%.7f lon=%.7f alt=%.2fm\n",
                  telem.lat_e7 * 1e-7, telem.lon_e7 * 1e-7, telem.rel_alt_cm / 100.0f);
    return;
  }
}

static bool sendCommandPacket(CommandId cmd, uint32_t now) {
  CommandPacket pkt = {};
  pkt.magic[0] = CMD_MAGIC_0;
  pkt.magic[1] = CMD_MAGIC_1;
  pkt.version = CMD_VERSION;
  pkt.command_id = static_cast<uint8_t>(cmd);
  pkt.seq = tx_seq;
  pkt.t_ms = now;
  pkt.alt_ref = ALT_REF_RELATIVE_HOME;
  pkt.flags = 0;
  pkt.reserved = 0;
  pkt.target_lat_e7 = 0;
  pkt.target_lon_e7 = 0;
  pkt.target_alt_cm = (cmd == CMD_TAKEOFF) ? target_alt_cm : 0;
  pkt.crc16 = crc16_ccitt_false(
    reinterpret_cast<const uint8_t *>(&pkt),
    sizeof(CommandPacket) - sizeof(pkt.crc16));

  radio.standby();
  delay(4);
  radio.clearIrqFlags(0xFFFF);
  const int st = radio.transmit(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  if (st != RADIOLIB_ERR_NONE) {
    snprintf(last_status, sizeof(last_status), "TX_ERR %d", st);
    return false;
  }

  inflight_seq = tx_seq;
  inflight_cmd = cmd;
  last_send_ms = now;
  snprintf(last_status, sizeof(last_status), "TX %s #%u", commandName(cmd), (unsigned)tx_seq);
  Serial.printf("[CMD] Sent %s seq=%u\n", commandName(cmd), (unsigned)tx_seq);
  return true;
}

static void startSequence() {
  if (!radio_ok) {
    snprintf(last_status, sizeof(last_status), "NO RADIO");
    return;
  }
  target_alt_cm = (int32_t)lroundf(TAKEOFF_ALT_M * 100.0f);
  sequence_stage = SEQ_WAIT_MISSION;
  retry_count = 0;
  last_send_ms = 0;
  inflight_seq = 0;
  snprintf(last_status, sizeof(last_status), "SEQ START");
  Serial.println("[CMD] TX START_AUTO_MISSION (drone runs local mission)");
}

static void handleAck(const CommandAckPacket &ack) {
  if (sequence_stage == SEQ_IDLE || sequence_stage == SEQ_DONE || sequence_stage == SEQ_ERROR) {
    return;
  }
  if (ack.seq != inflight_seq || ack.command_id != static_cast<uint8_t>(inflight_cmd)) {
    return;
  }

  if (ack.status == ACK_ACCEPTED) {
    snprintf(last_status, sizeof(last_status), "ACK ACC %s", commandName(inflight_cmd));
    return;
  }

  if (ack.status == ACK_COMPLETE) {
    Serial.printf("[CMD] COMPLETE %s seq=%u\n", commandName(inflight_cmd), (unsigned)ack.seq);
    tx_seq++;
    retry_count = 0;
    last_send_ms = 0;
    if (sequence_stage == SEQ_WAIT_MISSION) {
      sequence_stage = SEQ_DONE;
      snprintf(last_status, sizeof(last_status), "SEQ DONE");
    }
    return;
  }

  snprintf(last_status, sizeof(last_status), "ACK FAIL %s", commandName(inflight_cmd));
  sequence_stage = SEQ_ERROR;
}


static void runSequence(uint32_t now) {
  if (sequence_stage == SEQ_IDLE || sequence_stage == SEQ_DONE || sequence_stage == SEQ_ERROR) {
    return;
  }

  const CommandId cmd = CMD_START_AUTO_MISSION;

  const bool shouldSend = (last_send_ms == 0) || ((now - last_send_ms) >= COMMAND_ACK_TIMEOUT_MS);
  if (!shouldSend) {
    return;
  }
  if (retry_count >= MAX_COMMAND_RETRIES) {
    snprintf(last_status, sizeof(last_status), "SEQ TIMEOUT");
    sequence_stage = SEQ_ERROR;
    return;
  }
  if (sendCommandPacket(cmd, now)) {
    retry_count++;
  }
}

void setup() {
  pinMode(VEXT_CTRL_PIN, OUTPUT);
  digitalWrite(VEXT_CTRL_PIN, LOW);
  delay(50);

  WiFi.mode(WIFI_OFF);
  btStop();

  heltec_setup();
  delay(100);
  display.normalDisplay();
  display.setContrast(OLED_CONTRAST);
  display.displayOn();

  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Kamikaze Commander (no GPS) booting...");

  heltec_v4_pa_init();

  const int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                             LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setCRC(true);
    radio.setDio2AsRfSwitch(true);
    radio.setRxBoostedGainMode(true);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
    radio_ok = true;
    Serial.printf("LoRa ready: %.1f MHz SF%u\n", LORA_FREQ_MHZ, (unsigned)LORA_SF);
  } else {
    radio_ok = false;
    snprintf(last_status, sizeof(last_status), "RADIO ERR %d", st);
  }

  snprintf(last_status, sizeof(last_status), radio_ok ? "PRG=START" : "RADIO ERR");
  drawScreen();
}

void loop() {
  heltec_loop();

  const uint32_t now = millis();

  if (button.isSingleClick()) {
    if (sequence_stage == SEQ_IDLE || sequence_stage == SEQ_DONE || sequence_stage == SEQ_ERROR) {
      startSequence();
    } else {
      snprintf(last_status, sizeof(last_status), "SEQ BUSY");
    }
    drawScreen();
    last_draw_ms = now;
  }

  pollRadio();
  runSequence(now);

  if (now - last_draw_ms >= 300) {
    drawScreen();
    last_draw_ms = now;
  }
}
