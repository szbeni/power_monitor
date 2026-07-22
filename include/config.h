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

// Publish / print interval for the ESS consumer (ms)
#ifndef ESS_PUBLISH_INTERVAL_MS
#define ESS_PUBLISH_INTERVAL_MS 250
#endif

// Serial status print interval (ms); 0 disables periodic serial dumps
#ifndef SERIAL_STATUS_INTERVAL_MS
#define SERIAL_STATUS_INTERVAL_MS 2000
#endif

// HTTP server port for GET /api/power and GET /
#ifndef HTTP_PORT
#define HTTP_PORT 80
#endif

// MQTT topic root. Messages:
//   <root>/load2/power          float W  (signed active power)
//   <root>/load2/voltage        float V
//   <root>/load2/current        float A
//   <root>/load2/power_factor   float
//   <root>/load2/json           full JSON snapshot
#ifndef MQTT_TOPIC_ROOT
#define MQTT_TOPIC_ROOT "power_monitor/jsy"
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
#ifndef WIFI_TX_POWER
#define WIFI_TX_POWER WIFI_POWER_8_5dBm
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
