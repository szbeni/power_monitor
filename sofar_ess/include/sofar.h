#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

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
  uint16_t batterySoc = 0;
  bool ok = false;
};

bool sofarBegin(SoftwareSerial& bus);
void sofarHeartbeat();
bool sofarStandby();
bool sofarAuto();
bool sofarCharge(uint16_t watts);
bool sofarDischarge(uint16_t watts);
bool sofarReadStatus(SofarStatus& out);
SofarMode sofarLastMode();
uint16_t sofarLastSetpointW();
