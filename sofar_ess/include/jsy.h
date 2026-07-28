#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

struct JsyLoad2 {
  float voltage = NAN;
  float current = NAN;
  float activePower = NAN; // signed W; >0 import, <0 export (CT orientation)
  float powerFactor = NAN;
  float frequency = NAN;
  float energyImportWh = NAN;  // channel2 positive energy (from grid)
  float energyExportWh = NAN;  // channel2 negative energy (to grid)
  bool ok = false;
};

bool jsyBegin(SoftwareSerial& bus);
bool jsyReadLoad2(SoftwareSerial& bus, JsyLoad2& out);
