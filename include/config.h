#pragma once

#include "secrets.h"

// ---------------------------------------------------------------------------
// Wiring (ESP32-C3 SuperMini <-> JSY-MK-194G TTL)
//
//   ESP32-C3          JSY-MK-194G
//   --------          -----------
//   GPIO20 (RX)  <--  TX
//   GPIO21 (TX)  -->  RX
//   GND          ---  GND
//   5V           ---  VCC   (JSY usually wants 5V supply)
//
// Cross TX/RX. Share GND. If the JSY UART TX is 5V logic, use a level
// shifter (or divider) into GPIO20 — the C3 is NOT 5V tolerant.
// ---------------------------------------------------------------------------

#ifndef JSY_RX_PIN
#define JSY_RX_PIN 20 // ESP RX <- JSY TX
#endif

#ifndef JSY_TX_PIN
#define JSY_TX_PIN 21 // ESP TX -> JSY RX
#endif

// Channel used for the ESS loop (JSY-MK-194G CT2 / load2)
#ifndef ESS_CHANNEL
#define ESS_CHANNEL 2
#endif

// ESS tick interval (ms): serial / UDP / optional MQTT
#ifndef ESS_PUBLISH_INTERVAL_MS
#define ESS_PUBLISH_INTERVAL_MS 250
#endif

// 1 = send load2 power over UDP each ESS tick.
#ifndef ESS_UDP_ENABLE
#define ESS_UDP_ENABLE 0
#endif

// 1 = publish load2/power over MQTT each ESS tick.
#ifndef ESS_MQTT_ENABLE
#define ESS_MQTT_ENABLE 1
#endif

// HTTP server port for GET /api/power and GET /
#ifndef HTTP_PORT
#define HTTP_PORT 80
#endif

// MQTT:
//   <root>/load2/power     float W @ ESS_PUBLISH_INTERVAL_MS (ESS, non-retained)
//   <root>/tele/SENSOR     JSON snapshot @ HA_PUBLISH_INTERVAL_MS (HA)
//   <root>/status          online/offline (availability + LWT)
#ifndef MQTT_TOPIC_ROOT
#define MQTT_TOPIC_ROOT "power_monitor/jsy"
#endif

// Home Assistant MQTT discovery for tele/SENSOR JSON. Set to 0 to disable.
#ifndef HA_MQTT_DISCOVERY
#define HA_MQTT_DISCOVERY 1
#endif

#ifndef HA_DISCOVERY_PREFIX
#define HA_DISCOVERY_PREFIX "homeassistant"
#endif

#ifndef HA_DEVICE_NAME
#define HA_DEVICE_NAME "JSY Load2"
#endif

#ifndef HA_PUBLISH_INTERVAL_MS
#define HA_PUBLISH_INTERVAL_MS 10000
#endif

// After connect, bump JSY to 38400 for best ESS reactivity (~330 ms detect)
#ifndef JSY_TARGET_BAUD
#define JSY_TARGET_BAUD 38400
#endif

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 8 // onboard blue LED on most C3 SuperMini boards
#endif

// WiFi: SuperMini onboard LDO often droops during TX peaks → AUTH_EXPIRE.
// Lower TX power cuts peak current. wifi_power_t enum value (see WiFiGeneric.h).
// SuperMini LDO: keep TX low so MQTT at 250 ms does not brown out the radio.
#ifndef WIFI_TX_POWER
#define WIFI_TX_POWER WIFI_POWER_5dBm
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 10000
#endif

#ifndef WIFI_BOOT_ATTEMPTS
#define WIFI_BOOT_ATTEMPTS 3
#endif

#ifndef WIFI_RETRY_INTERVAL_MS
#define WIFI_RETRY_INTERVAL_MS 15000
#endif
