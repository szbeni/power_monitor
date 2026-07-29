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
  uint16_t batteryPowerRaw = 0;
  // Signed battery power (W): +discharge, −charge. 0 if idle / unknown.
  int16_t batteryPowerW = 0;
  uint16_t batterySoc = 0;
  bool socValid = false;
  bool ok = false;
};

bool sofarBegin(HardwareSerial& bus);
void sofarHeartbeat();
bool sofarStandby();
bool sofarAuto();
bool sofarCharge(uint16_t watts);
bool sofarDischarge(uint16_t watts);
// Read one status register into cache (round-robin). Non-blocking aside from one Modbus txn.
bool sofarPollStatusField(SofarStatus& cache);
SofarMode sofarLastMode();
uint16_t sofarLastSetpointW();
