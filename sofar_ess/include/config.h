#pragma once

#include "secrets.h"

// MQTT base topic (same style as Sofar2mqtt)
#ifndef DEVICE_NAME
#define DEVICE_NAME "SofarEss"
#endif

// ---------------------------------------------------------------------------
// Wiring — ESP8266 NodeMCU
//
// Sofar RS485 (MAX485 / MAX3485) — SoftwareSerial:
//   D5  DE/RE (HIGH=TX, LOW=RX; leave unused if module has no DE/RE)
//   D6  RS485 RX  (ESP RX <- module RO)
//   D7  RS485 TX  (ESP TX -> module DI)
//
// JSY-MK-194G TTL — SoftwareSerial @ 9600:
//   D1  JSY RX  (ESP RX <- JSY TX)
//   D2  JSY TX  (ESP TX -> JSY RX)
//   5V  JSY VCC
//   GND shared
//
// Debug — hardware UART0 (USB):
//   RX/TX  Serial @ 115200
//
// Put the grid / ESS CT through JSY channel 2 (load2 / CT2).
// Only one SoftSerial listens at a time; firmware alternates.
// ---------------------------------------------------------------------------

#ifndef SOFAR_DE_PIN
#define SOFAR_DE_PIN D5
#endif
#ifndef SOFAR_RX_PIN
#define SOFAR_RX_PIN D6
#endif
#ifndef SOFAR_TX_PIN
#define SOFAR_TX_PIN D7
#endif

#ifndef JSY_RX_PIN
#define JSY_RX_PIN D1
#endif
#ifndef JSY_TX_PIN
#define JSY_TX_PIN D2
#endif

#ifndef LOG_BAUD
#define LOG_BAUD 115200
#endif
#define Log Serial

#ifndef SOFAR_SLAVE_ID
#define SOFAR_SLAVE_ID 0x01
#endif

#ifndef JSY_SLAVE_ID
#define JSY_SLAVE_ID 0x01
#endif

#ifndef JSY_BAUD
#define JSY_BAUD 9600
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

// How often to read JSY and update Sofar (ms)
#ifndef ESS_LOOP_INTERVAL_MS
#define ESS_LOOP_INTERVAL_MS 2000
#endif

// Deadband around 0 W — avoid chatter
#ifndef ESS_DEADBAND_W
#define ESS_DEADBAND_W 80
#endif

// Minimum command change before re-sending to inverter (W)
#ifndef ESS_MIN_DELTA_W
#define ESS_MIN_DELTA_W 50
#endif

// Re-send same setpoint at least this often (ms) so inverter stays in mode
#ifndef ESS_REFRESH_MS
#define ESS_REFRESH_MS 15000
#endif

#ifndef HEARTBEAT_INTERVAL_MS
#define HEARTBEAT_INTERVAL_MS 9000
#endif

#ifndef MQTT_STATE_INTERVAL_MS
#define MQTT_STATE_INTERVAL_MS 10000
#endif
