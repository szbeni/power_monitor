#pragma once

#include "secrets.h"

// MQTT base topic (same style as Sofar2mqtt)
#ifndef DEVICE_NAME
#define DEVICE_NAME "sofaress"
#endif

// ---------------------------------------------------------------------------
// Wiring — ESP32-C3 SuperMini
//
// SoftSerial is gone: C3 has two HW UARTs (UART0 + UART1). USB CDC is Serial.
//
// JSY-MK-194G TTL — HardwareSerial Serial1 (MycilaJSY):
//   GPIO20  ESP RX <- JSY TX
//   GPIO21  ESP TX -> JSY RX
//   3V3     JSY VCC (3.3 V is fine)
//   GND     shared
//
// Sofar RS485 (MAX485 / MAX3485) — HardwareSerial Serial0:
//   GPIO4   DE/RE (HIGH=TX, LOW=RX; leave unused if module has no DE/RE)
//   GPIO5   RS485 RX  (ESP RX <- module RO)
//   GPIO6   RS485 TX  (ESP TX -> module DI)
//
// Debug — USB CDC Serial @ 115200
//
// Put the grid / ESS CT through JSY channel 2 (load2 / CT2).
// ---------------------------------------------------------------------------

#ifndef SOFAR_DE_PIN
#define SOFAR_DE_PIN 4
#endif
#ifndef SOFAR_RX_PIN
#define SOFAR_RX_PIN 5
#endif
#ifndef SOFAR_TX_PIN
#define SOFAR_TX_PIN 6
#endif

#ifndef JSY_RX_PIN
#define JSY_RX_PIN 20 // ESP RX <- JSY TX
#endif
#ifndef JSY_TX_PIN
#define JSY_TX_PIN 21 // ESP TX -> JSY RX
#endif

// After connect, bump JSY to 38400 for best ESS reactivity (~330 ms detect)
#ifndef JSY_TARGET_BAUD
#define JSY_TARGET_BAUD 38400
#endif

#ifndef LOG_BAUD
#define LOG_BAUD 115200
#endif
#define Log Serial

#ifndef SOFAR_SLAVE_ID
#define SOFAR_SLAVE_ID 0x01
#endif

#ifdef INVERTER_ME3000
#ifndef MAX_POWER_W
#define MAX_POWER_W 3000
#endif
#else
#ifndef MAX_POWER_W
#define MAX_POWER_W 3600
#endif
#endif

// ESS closed-loop control (JSY load2 → Sofar passive charge/discharge)
#ifndef ESS_ENABLE
#define ESS_ENABLE 1
#endif

// 1 = charge from excess export only (command ≤ 0); never discharge
#ifndef ESS_CHARGE_ONLY
#define ESS_CHARGE_ONLY 0
#endif

// SOC protect: below low threshold ESS becomes charge-only; resumes bidirectional at low+hyst.
#ifndef ESS_SOC_PROTECT_ENABLE
#define ESS_SOC_PROTECT_ENABLE 1
#endif
#ifndef ESS_SOC_PROTECT_LOW
#define ESS_SOC_PROTECT_LOW 15
#endif
#ifndef ESS_SOC_PROTECT_HYST
#define ESS_SOC_PROTECT_HYST 5
#endif

// How often to sample JSY snapshot and update Sofar (ms).
// ~500 ms tracks MycilaJSY @ 38400 (~330 ms change detect) without flooding RS485.
#ifndef ESS_LOOP_INTERVAL_MS
#define ESS_LOOP_INTERVAL_MS 500
#endif

// Deadband around 0 W — freeze integrator / avoid chatter
#ifndef ESS_DEADBAND_W
#define ESS_DEADBAND_W 10
#endif

// Minimum command change before re-sending to inverter (W)
#ifndef ESS_MIN_DELTA_W
#define ESS_MIN_DELTA_W 5
#endif

// Re-send same setpoint at least this often (ms) so inverter stays in mode
#ifndef ESS_REFRESH_MS
#define ESS_REFRESH_MS 60000
#endif

// ESS near-zero hold (W): stay in charge/discharge instead of standby to avoid
// Sofar internal relay chatter when PI output is inside the deadband.
#ifndef ESS_HOLD_MIN_W
#define ESS_HOLD_MIN_W 30
#endif

// Dual battery banks via battery_selector MQTT (A = prefer, B = secondary).
#ifndef DUAL_BATT_ENABLE
#define DUAL_BATT_ENABLE 1
#endif
#ifndef DUAL_BATT_EMPTY_SOC
#define DUAL_BATT_EMPTY_SOC 10
#endif
#ifndef DUAL_BATT_FULL_SOC
#define DUAL_BATT_FULL_SOC 98
#endif
// Contactor + BMS handover time before the inverter reports the new pack.
#ifndef DUAL_BATT_SETTLE_MS
#define DUAL_BATT_SETTLE_MS 15000
#endif
#ifndef DUAL_BATT_COOLDOWN_MS
#define DUAL_BATT_COOLDOWN_MS 60000
#endif
#ifndef DUAL_BATT_CONFIRM_TIMEOUT_MS
#define DUAL_BATT_CONFIRM_TIMEOUT_MS 10000
#endif
// Must allow several confirmation samples (one per ESS cycle) after settling.
#ifndef DUAL_BATT_SOC_SYNC_TIMEOUT_MS
#define DUAL_BATT_SOC_SYNC_TIMEOUT_MS 30000
#endif

// SOC validation — a mis-paired Modbus reply looks like a valid register value,
// so suspicious readings must repeat before they are believed.
// Samples that must agree (within SOC_CONFIRM_TOLERANCE) for a suspicious value.
#ifndef SOC_CONFIRM_SAMPLES
#define SOC_CONFIRM_SAMPLES 2
#endif
// Samples required when the value contradicts what we last knew for this bank.
#ifndef SOC_CONFIRM_SAMPLES_STRICT
#define SOC_CONFIRM_SAMPLES_STRICT 4
#endif
// Two samples count as agreeing within this many % (BMS ticks while settling).
#ifndef SOC_CONFIRM_TOLERANCE
#define SOC_CONFIRM_TOLERANCE 2
#endif
// Step (%) vs the live SOC that is treated as a jump needing confirmation.
#ifndef SOC_JUMP_PCT
#define SOC_JUMP_PCT 20
#endif
// Deviation (%) from the bank's last-known SOC that triggers strict confirmation.
#ifndef SOC_EXPECT_DEVIATION_PCT
#define SOC_EXPECT_DEVIATION_PCT 15
#endif
#ifndef DUAL_BATT_SELECTOR_TOPIC
#define DUAL_BATT_SELECTOR_TOPIC "battery_selector"
#endif
#ifndef DUAL_BATT_A_KWH
#define DUAL_BATT_A_KWH 10.0f
#endif
#ifndef DUAL_BATT_B_KWH
#define DUAL_BATT_B_KWH 5.0f
#endif
#ifndef DUAL_BATT_DEFAULT_BANK
#define DUAL_BATT_DEFAULT_BANK 1 // 1=A, 2=B
#endif

// PI gains for zero-export (Ts = ESS_LOOP_INTERVAL_MS). Tunable via MQTT set/kp|ki.
// u = Kp*e + Ki*∫e  with e = grid_power_w (+import → +discharge).
#ifndef ESS_KP
#define ESS_KP 0.2f
#endif
#ifndef ESS_KI
#define ESS_KI 0.1f
#endif

#ifndef HEARTBEAT_INTERVAL_MS
#define HEARTBEAT_INTERVAL_MS 9000
#endif

#ifndef MQTT_STATE_INTERVAL_MS
#define MQTT_STATE_INTERVAL_MS 5000
#endif

// Home Assistant MQTT discovery (entities under homeassistant/*/sofaress/*/config)
#ifndef HA_MQTT_DISCOVERY
#define HA_MQTT_DISCOVERY 1
#endif

#ifndef HA_DISCOVERY_PREFIX
#define HA_DISCOVERY_PREFIX "homeassistant"
#endif

#ifndef HA_DEVICE_NAME
#define HA_DEVICE_NAME "Sofar ESS"
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 8 // onboard blue LED on most C3 SuperMini boards
#endif

// WiFi: SuperMini onboard LDO often droops during TX peaks → AUTH_EXPIRE.
#ifndef WIFI_TX_POWER
#define WIFI_TX_POWER WIFI_POWER_5dBm
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 10000
#endif

#ifndef WIFI_RETRY_INTERVAL_MS
#define WIFI_RETRY_INTERVAL_MS 15000
#endif

// 1 = enable ArduinoOTA (LAN upload via espota)
#ifndef OTA_ENABLE
#define OTA_ENABLE 1
#endif
