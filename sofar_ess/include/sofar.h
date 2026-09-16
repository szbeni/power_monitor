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

struct SofarBatteryParams {
  uint16_t raw[12] = {}; // 0x10B1 .. 0x10BC
  bool regOk[12] = {};
  char regWhy[12][16] = {}; // per-register: "ok", "exception", "no-rx", …
  bool valid = false;       // true when all 12 registers read OK
  uint32_t updatedMs = 0;
  uint32_t lastPollMs = 0;
  uint16_t pollAttempts = 0;
  char pollDiag[64] = "";  // summary e.g. "7/12 ok"
  char regDiag[256] = {};  // per-register detail for MQTT debug
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

  // Battery profile settings 0x10B1–0x10BC (FC 0x04 read / FC 0x13 write).
  SofarBatteryParams battParams;

  // Last Modbus failure reason (timeout, crc, exception, …).
  char lastError[24] = "";
  // SOC reads not accepted (out of range / awaiting confirmation). Diagnostic.
  uint16_t socRejects = 0;
};

constexpr uint8_t SOFAR_BATT_PARAM_COUNT = 12;
constexpr uint16_t SOFAR_BATT_PARAM_REG_BASE = 0x10B1;

struct SofarBattParamMeta {
  uint16_t reg;
  const char* key;       // MQTT suffix e.g. "capacity_ah"
  const char* haName;    // HA display name
  float scale;
  uint16_t rawMin;
  uint16_t rawMax;
  float step;
  const char* unit;
  uint8_t decimals;
  bool writable;
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

// Battery profile 0x10B1–0x10BC (hybrid map, FC 0x04/0x13).
void sofarBattParamProbeBegin(SofarBatteryParams& p);
bool sofarPollBatteryParamReg(SofarStatus& cache, uint8_t index);
void sofarBattParamBuildDiag(SofarBatteryParams& p);
uint8_t sofarBattParamOkCount(const SofarBatteryParams& p);
bool sofarPollBatteryParams(SofarStatus& cache); // legacy: all regs in one call
bool sofarWriteBatteryParam(SofarStatus& cache, uint8_t index, uint16_t raw);
uint8_t sofarBattParamCount();
const SofarBattParamMeta* sofarBattParamMeta(uint8_t index);
const SofarBattParamMeta* sofarBattParamMetaByKey(const char* key);
int8_t sofarBattParamIndexByKey(const char* key);
float sofarBattParamScaled(const SofarBatteryParams& p, uint8_t index);
bool sofarBattParamScaledToRaw(uint8_t index, float scaled, uint16_t& raw);
bool sofarBattParamValidateWrite(const SofarBatteryParams& p, uint8_t index, uint16_t raw);
const char* sofarLastModbusError();
