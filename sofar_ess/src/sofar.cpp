#include "sofar.h"

#include "config.h"
#include "modbus_util.h"

namespace {
SoftwareSerial* bus_ = nullptr;
SofarMode lastMode_ = SofarMode::Unknown;
uint16_t lastSetpointW_ = 0;

constexpr uint8_t FN_READ = 0x03;
constexpr uint8_t FN_PASSIVE = 0x42;
constexpr uint16_t CMD_STANDBY = 0x0100;
constexpr uint16_t CMD_DISCHARGE = 0x0101;
constexpr uint16_t CMD_CHARGE = 0x0102;
constexpr uint16_t CMD_AUTO = 0x0103;
constexpr uint16_t PARAM_STANDBY = 0x5555;
constexpr uint16_t REG_RUNSTATE = 0x0200;
constexpr uint16_t REG_BATTW = 0x020d;
constexpr uint16_t REG_BATTSOC = 0x0210;
constexpr uint16_t REG_GRIDW = 0x0212;

struct Resp {
  uint8_t data[32];
  uint8_t dataSize = 0;
  bool ok = false;
};

void flushBus() {
  bus_->flush();
  delay(50);
  while (bus_->available()) {
    bus_->read();
  }
}

bool listen(Resp& resp, uint8_t expectId) {
  uint8_t frame[64];
  uint8_t n = 0;
  uint8_t fn = 0;
  uint8_t dataBytes = 0;
  bool done = false;
  resp = {};

  while (!done && n < sizeof(frame)) {
    int tries = 0;
    while (!bus_->available() && tries++ < 8) {
      delay(40);
    }
    if (tries >= 8) {
      break;
    }

    frame[n] = bus_->read();
    switch (n) {
      case 0:
        if (frame[0] != expectId) {
          n = 0;
          continue;
        }
        break;
      case 1:
        fn = frame[1];
        (void)fn;
        break;
      case 2:
        dataBytes = frame[2];
        if (dataBytes + 5 > sizeof(frame)) {
          return false;
        }
        break;
      default:
        if (n >= dataBytes + 3) {
          done = true;
        }
        break;
    }
    n++;
  }

  if (n < 5 || !modbusCheckCrc(frame, n)) {
    return false;
  }

  resp.dataSize = dataBytes;
  memcpy(resp.data, frame + 3, dataBytes);
  resp.ok = true;
  return true;
}

bool sendRaw(uint8_t* frame, size_t size, Resp* resp) {
  modbusCalcCrc(frame, size);
  flushBus();
  digitalWrite(SOFAR_DE_PIN, HIGH);
  delay(1);
  bus_->write(frame, size);
  bus_->flush();
  digitalWrite(SOFAR_DE_PIN, LOW);

  if (!resp) {
    // Heartbeat: still drain briefly
    Resp dummy;
    listen(dummy, SOFAR_SLAVE_ID);
    return true;
  }
  return listen(*resp, SOFAR_SLAVE_ID);
}

bool sendPassive(uint16_t cmd, uint16_t param) {
  uint8_t frame[] = {
      SOFAR_SLAVE_ID,
      FN_PASSIVE,
      uint8_t(cmd >> 8),
      uint8_t(cmd & 0xff),
      uint8_t(param >> 8),
      uint8_t(param & 0xff),
      0,
      0};
  Resp rs;
  if (!sendRaw(frame, sizeof(frame), &rs) || rs.dataSize < 2) {
    Log.println("[sofar] passive cmd failed");
    return false;
  }
  const uint16_t code = (uint16_t(rs.data[0]) << 8) | rs.data[1];
  Log.printf("[sofar] passive 0x%04X param=%u -> 0x%04X\n", cmd, param, code);
  return (code & 0xff) == 0;
}

bool readReg(uint16_t reg, uint16_t& value) {
  uint8_t frame[] = {
      SOFAR_SLAVE_ID, FN_READ, uint8_t(reg >> 8), uint8_t(reg & 0xff), 0x00, 0x01, 0, 0};
  Resp rs;
  if (!sendRaw(frame, sizeof(frame), &rs) || rs.dataSize < 2) {
    return false;
  }
  value = (uint16_t(rs.data[0]) << 8) | rs.data[1];
  return true;
}
} // namespace

bool sofarBegin(SoftwareSerial& bus) {
  bus_ = &bus;
  pinMode(SOFAR_DE_PIN, OUTPUT);
  digitalWrite(SOFAR_DE_PIN, LOW);
  bus_->begin(9600);
  bus_->listen();
  delay(50);
  return true;
}

void sofarHeartbeat() {
  if (!bus_) {
    return;
  }
  bus_->listen();
  uint8_t frame[] = {SOFAR_SLAVE_ID, 0x49, 0x22, 0x01, 0x22, 0x02, 0x00, 0x00};
  sendRaw(frame, sizeof(frame), nullptr);
}

bool sofarStandby() {
  bus_->listen();
  const bool ok = sendPassive(CMD_STANDBY, PARAM_STANDBY);
  if (ok) {
    lastMode_ = SofarMode::Standby;
    lastSetpointW_ = 0;
  }
  return ok;
}

bool sofarAuto() {
  bus_->listen();
  const bool ok = sendPassive(CMD_AUTO, 0);
  if (ok) {
    lastMode_ = SofarMode::Auto;
    lastSetpointW_ = 0;
  }
  return ok;
}

bool sofarCharge(uint16_t watts) {
  if (watts == 0 || watts > MAX_POWER_W) {
    return false;
  }
  bus_->listen();
  const bool ok = sendPassive(CMD_CHARGE, watts);
  if (ok) {
    lastMode_ = SofarMode::Charge;
    lastSetpointW_ = watts;
  }
  return ok;
}

bool sofarDischarge(uint16_t watts) {
  if (watts == 0 || watts > MAX_POWER_W) {
    return false;
  }
  bus_->listen();
  const bool ok = sendPassive(CMD_DISCHARGE, watts);
  if (ok) {
    lastMode_ = SofarMode::Discharge;
    lastSetpointW_ = watts;
  }
  return ok;
}

bool sofarReadStatus(SofarStatus& out) {
  bus_->listen();
  out = {};
  uint16_t v = 0;
  if (!readReg(REG_RUNSTATE, v)) {
    return false;
  }
  out.runState = v;
  if (!readReg(REG_GRIDW, v)) {
    return false;
  }
  out.gridPowerRaw = v;
  if (!readReg(REG_BATTW, v)) {
    return false;
  }
  out.batteryPowerRaw = v;
  if (!readReg(REG_BATTSOC, v)) {
    return false;
  }
  out.batterySoc = v;
  out.ok = true;
  return true;
}

SofarMode sofarLastMode() {
  return lastMode_;
}

uint16_t sofarLastSetpointW() {
  return lastSetpointW_;
}
