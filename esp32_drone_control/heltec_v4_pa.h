// =======================
// HELTEC ESP32 LORA V4 - POWER AMPLIFIER CONFIGURATION
// =======================
// The V4 board has an external PA that MUST be enabled for proper TX range.
// Include this file and call heltec_v4_pa_init() in setup() BEFORE radio.begin().

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

#endif
