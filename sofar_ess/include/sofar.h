#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

#include "config.h"

enum class SofarMode : uint8_t {
  Unknown = 0,
  Standby,
  Auto,
  Charge,
  Discharge,
};

struct SofarStatus {
  uint16_t runState = 0;
  uint16_t gridPowerRaw = 0;

  // 0x020D Charge/Discharge power (Sofar map name). ESS sign: +discharge, −charge.
  uint16_t chargeDischargePowerRaw = 0;
  int16_t chargeDischargePowerW = 0;

  // 0x020E / 0x020F — DC terminal V×I (preferred measured battery power).
  float batteryVoltageV = 0.0f;
  float batteryCurrentA = 0.0f;
  int16_t batteryDcPowerW = 0; // V×I, ESS sign +discharge / −charge
  bool batteryDcValid = false;
  uint32_t batteryDcMs = 0;
  uint32_t chargeDischargeMs = 0;

  uint16_t batterySoc = 0;
  bool socValid = false;
  bool ok = false;

  // Inverter fault message list (0x0201–0x0205): word N = byte(2N+1)<<8 | byte(2N).
  uint16_t fault[5] = {};
  bool faultValid = false;
  // Inverter alert (0x022B), low byte = alert byte0.
  uint16_t alert = 0;
  bool alertValid = false;
  // Battery fault message list (0x023D–0x0241).
  uint16_t battFault[5] = {};
  bool battFaultValid = false;

  // PV strings (0x0250–0x0255) + today generation (0x0218). Low-priority poll.
  float pv1VoltageV = 0.0f;
  float pv1CurrentA = 0.0f;
  float pv1PowerW = 0.0f;
  float pv2VoltageV = 0.0f;
  float pv2CurrentA = 0.0f;
  float pv2PowerW = 0.0f;
  float pvTotalW = 0.0f; // PV1 + PV2
  bool pvValid = false;
  float pvTodayKwh = 0.0f;
  bool pvTodayValid = false;

  // Last Modbus failure reason (timeout, crc, exception, …).
  char lastError[24] = "";
  // SOC reads not accepted (out of range / awaiting confirmation). Diagnostic.
  uint16_t socRejects = 0;
};

bool sofarBegin(HardwareSerial& bus);
void sofarHeartbeat();
bool sofarStandby();
bool sofarAuto();
bool sofarCharge(uint16_t watts);
bool sofarDischarge(uint16_t watts);
// Read one status register into cache (round-robin). Non-blocking aside from one Modbus txn.
bool sofarPollStatusField(SofarStatus& cache);
// Clear live SOC validity and jump-confirm pending (call after battery bank switch).
void sofarInvalidateSoc(SofarStatus& cache);
// Cross-check hint: SOC we expect from the bank being connected (its last known
// value). A reading far from it must confirm SOC_CONFIRM_SAMPLES_STRICT times.
// Cleared once a SOC is accepted.
void sofarSetSocExpectation(bool have, uint8_t expected);
// When true, sofarPollStatusField prefers SOC reads until a SOC is accepted.
void sofarPreferSocPoll(bool enable);
// One dedicated SOC Modbus read + acceptSoc (for post-switch sync).
bool sofarPollSocNow(SofarStatus& cache);
SofarMode sofarLastMode();
uint16_t sofarLastSetpointW();

// Last passive (0x42) reply: result = low byte, status = high byte.
// Status bits: 0 charge enabled, 1 discharge enabled, 2 battery full / charge
// prohibited, 3 battery flat / discharge prohibited.
bool sofarPassiveStatusValid();
uint8_t sofarLastPassiveStatus();
uint8_t sofarLastPassiveResult();
bool sofarChargeProhibited();
bool sofarDischargeProhibited();

const char* sofarRunStateName(uint16_t runState);
// Append active fault/alert bit names into out (comma-separated). Returns out.
size_t sofarFormatFaults(const SofarStatus& s, char* out, size_t outLen);
size_t sofarFormatAlerts(const SofarStatus& s, char* out, size_t outLen);
size_t sofarFormatBattFaults(const SofarStatus& s, char* out, size_t outLen);
