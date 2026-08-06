#pragma once

#include "secrets.h"

// MQTT base topic
#ifndef DEVICE_NAME
#define DEVICE_NAME "battery_selector"
#endif

// ---------------------------------------------------------------------------
// Wiring — ESP32 DevKit (pins match battery_selector.yaml)
//
//   Two batteries (A / B), four relay channels switched together:
//
//   GPIO32  Relay 1 — power
//   GPIO33  Relay 2 — CAN bus A
//   GPIO25  Relay 3 — CAN bus B
//   GPIO26  Relay 4 — unused (left de-energized)
//   GPIO23  Status LED
//   GND     shared
//
//   Relays de-energized (NC) → Battery A
//   Relays energized   (NO) → Battery B
//
//   Battery A/B selection drives Relay 1–3 together; Relay 4 stays off.
// ---------------------------------------------------------------------------

#ifndef RELAY1_PIN
#define RELAY1_PIN 32  // power
#endif
#ifndef RELAY2_PIN
#define RELAY2_PIN 33  // CAN bus A
#endif
#ifndef RELAY3_PIN
#define RELAY3_PIN 25  // CAN bus B
#endif
#ifndef RELAY4_PIN
#define RELAY4_PIN 26  // unused
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 23
#endif

// 1 = HIGH turns relay ON (ESPHome gpio switch default).
// Set to 0 for active-low relay modules.
#ifndef RELAY_ACTIVE_HIGH
#define RELAY_ACTIVE_HIGH 1
#endif

// Physical relay count on the board
#ifndef NUM_RELAYS
#define NUM_RELAYS 4
#endif

// Relays that switch with battery select (power + CAN A + CAN B)
#ifndef NUM_ACTIVE_RELAYS
#define NUM_ACTIVE_RELAYS 3
#endif

// Selectable banks: 1 = Battery A, 2 = Battery B
#ifndef NUM_BATTERIES
#define NUM_BATTERIES 2
#endif

// Boot selection: 1 = Battery A (all relays de-energized / NC)
#ifndef DEFAULT_BATTERY
#define DEFAULT_BATTERY 1
#endif

// How often to re-publish selected battery (ms)
#ifndef REPORT_INTERVAL_MS
#define REPORT_INTERVAL_MS 5000
#endif

#ifndef LOG_BAUD
#define LOG_BAUD 115200
#endif
#define Log Serial

#ifndef WIFI_TX_POWER
#define WIFI_TX_POWER WIFI_POWER_11dBm
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 15000
#endif

#ifndef WIFI_RETRY_INTERVAL_MS
#define WIFI_RETRY_INTERVAL_MS 15000
#endif

// 1 = enable ArduinoOTA (LAN upload via espota)
#ifndef OTA_ENABLE
#define OTA_ENABLE 1
#endif

// Home Assistant MQTT discovery (select entity under homeassistant/select/…)
#ifndef HA_MQTT_DISCOVERY
#define HA_MQTT_DISCOVERY 1
#endif

#ifndef HA_DISCOVERY_PREFIX
#define HA_DISCOVERY_PREFIX "homeassistant"
#endif

#ifndef HA_DEVICE_NAME
#define HA_DEVICE_NAME "Battery Selector"
#endif

// Labels published on …/selected and used as MQTT select options in HA
#ifndef HA_OPTION_A
#define HA_OPTION_A "Battery A"
#endif
#ifndef HA_OPTION_B
#define HA_OPTION_B "Battery B"
#endif
