// =======================
// HELTEC ESP32 LORA V4 - POWER AMPLIFIER CONFIGURATION
// =======================
// The V4 board has an external PA that MUST be enabled for proper TX range.
// Without this, TX power is severely degraded (-127 dBm at 1 meter!)
//
// Reference: http://community.heltec.cn/t/problem-with-esp32-lora-range/22436
// Reference: http://community.heltec.cn/t/how-to-get-28dbm-on-the-module-heltec-esp32-v4/22101
//
// Include this file and call heltec_v4_pa_init() in setup() BEFORE initializing the radio.

#ifndef HELTEC_V4_PA_H
#define HELTEC_V4_PA_H

#include <Arduino.h>

#define LORA_PA_POWER   7
#define LORA_PA_EN      2
#define LORA_PA_TX_EN   46

static inline void heltec_v4_pa_init() {
  pinMode(LORA_PA_POWER, OUTPUT);
  digitalWrite(LORA_PA_POWER, HIGH);
  pinMode(LORA_PA_EN, OUTPUT);
  digitalWrite(LORA_PA_EN, HIGH);
  pinMode(LORA_PA_TX_EN, OUTPUT);
  digitalWrite(LORA_PA_TX_EN, HIGH);
  delay(10);
  Serial.println("[V4 PA] Power Amplifier initialized");
}

static inline void heltec_v4_pa_tx_mode() {
  digitalWrite(LORA_PA_TX_EN, HIGH);
}

static inline void heltec_v4_pa_rx_mode() {
  digitalWrite(LORA_PA_TX_EN, HIGH);
}

#endif
