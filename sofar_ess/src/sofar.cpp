#include "sofar.h"

#include "config.h"
#include "modbus_util.h"

namespace {
HardwareSerial* bus_ = nullptr;
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

constexpr uint32_t kListenTimeoutMs = 400;

// 0x020d as signed int16 ×10 → W.
// Sofar: charge +, discharge −. Our ESS convention: +discharge / −charge.
int16_t decodeBatteryPowerW(uint16_t /*runState*/, uint16_t raw) {
  const int32_t w = -int32_t(int16_t(raw)) * 10;
  if (w > 32767) {
    return 32767;
  }
  if (w < -32768) {
    return -32768;
  }
  return int16_t(w);
}

struct Resp {
  uint8_t data[32];
  uint8_t dataSize = 0;
  bool ok = false;
};

void flushBus() {
  bus_->flush();
  delay(5);
  while (bus_->available()) {
    bus_->read();
  }
}

// Returns false and sets why on failure (static buffer, overwritten each call).
const char* listenFailWhy_ = "";

bool listen(Resp& resp, uint8_t expectId) {
  uint8_t frame[64];
  uint8_t n = 0;
  uint8_t fn = 0;
  uint8_t dataBytes = 0;
  bool done = false;
  uint16_t junk = 0;
  resp = {};
  listenFailWhy_ = "timeout";

  const uint32_t deadline = millis() + kListenTimeoutMs;

  while (!done && n < sizeof(frame) && (int32_t)(millis() - deadline) < 0) {
    if (!bus_->available()) {
      delay(2);
      continue;
    }

    frame[n] = bus_->read();
    switch (n) {
      case 0:
        if (frame[0] != expectId) {
          junk++;
          n = 0;
          continue;
        }
        break;
      case 1:
        fn = frame[1];
        (void)fn;
        break;
      case 2:
        // Byte 2 is data-byte count for normal replies (same as Sofar2mqtt).
        dataBytes = frame[2];
        if (dataBytes + 5 > sizeof(frame)) {
          listenFailWhy_ = "len";
          return false;
        }
        break;
      default:
        // Need indices [0 .. dataBytes+4] inclusive = dataBytes+5 bytes total
        // (3 header + data + 2 CRC). Sofar2mqtt sets done when inByteNum > dataBytes+3.
        if (n > dataBytes + 3) {
          done = true;
        }
        break;
    }
    n++;
  }

  if (!done || n < 5) {
    if (junk) {
      listenFailWhy_ = "timeout+junk";
    } else if (n == 0) {
      listenFailWhy_ = "no-rx";
    } else {
      listenFailWhy_ = "short";
    }
    return false;
  }
  if (!modbusCheckCrc(frame, n)) {
    listenFailWhy_ = "crc";
    Log.printf("[sofar] CRC fail n=%u:", n);
    for (uint8_t i = 0; i < n && i < 16; i++) {
      Log.printf(" %02X", frame[i]);
    }
    Log.println();
    return false;
  }

  resp.dataSize = dataBytes;
  memcpy(resp.data, frame + 3, dataBytes);
  resp.ok = true;
  listenFailWhy_ = "";
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
    Log.printf("[sofar] passive 0x%04X FAIL (%s)\n", cmd, listenFailWhy_);
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
    Log.printf("[sofar] read 0x%04X FAIL (%s)\n", reg, listenFailWhy_);
    return false;
  }
  value = (uint16_t(rs.data[0]) << 8) | rs.data[1];
  return true;
}
} // namespace

bool sofarBegin(HardwareSerial& bus) {
  bus_ = &bus;
  pinMode(SOFAR_DE_PIN, OUTPUT);
  digitalWrite(SOFAR_DE_PIN, LOW);
  bus_->begin(9600, SERIAL_8N1, SOFAR_RX_PIN, SOFAR_TX_PIN);
  delay(50);
  Log.printf("[sofar] UART0 RX=%d TX=%d DE=%d slave=0x%02X\n",
             SOFAR_RX_PIN,
             SOFAR_TX_PIN,
             SOFAR_DE_PIN,
             SOFAR_SLAVE_ID);
  return true;
}

void sofarHeartbeat() {
  if (!bus_) {
    return;
  }
  uint8_t frame[] = {SOFAR_SLAVE_ID, 0x49, 0x22, 0x01, 0x22, 0x02, 0x00, 0x00};
  sendRaw(frame, sizeof(frame), nullptr);
}

bool sofarStandby() {
  const bool ok = sendPassive(CMD_STANDBY, PARAM_STANDBY);
  if (ok) {
    lastMode_ = SofarMode::Standby;
    lastSetpointW_ = 0;
  }
  return ok;
}

bool sofarAuto() {
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
  const bool ok = sendPassive(CMD_DISCHARGE, watts);
  if (ok) {
    lastMode_ = SofarMode::Discharge;
    lastSetpointW_ = watts;
  }
  return ok;
}

bool sofarPollStatusField(SofarStatus& cache) {
  // One register per call so ESS timing stays predictable (~one Modbus txn).
  static uint8_t phase = 0;
  uint16_t v = 0;
  bool ok = false;

  switch (phase % 4) {
    case 0:
      ok = readReg(REG_RUNSTATE, v);
      if (ok) {
        cache.runState = v;
        cache.batteryPowerW = decodeBatteryPowerW(cache.runState, cache.batteryPowerRaw);
      }
      break;
    case 1:
      ok = readReg(REG_GRIDW, v);
      if (ok) {
        cache.gridPowerRaw = v;
      }
      break;
    case 2:
      ok = readReg(REG_BATTW, v);
      if (ok) {
        cache.batteryPowerRaw = v;
        cache.batteryPowerW = decodeBatteryPowerW(cache.runState, cache.batteryPowerRaw);
      }
      break;
    default:
      ok = readReg(REG_BATTSOC, v);
      if (ok) {
        cache.batterySoc = v;
      }
      break;
  }
  phase++;

  if (ok) {
    cache.ok = true;
  }
  return ok;
}

SofarMode sofarLastMode() {
  return lastMode_;
}

uint16_t sofarLastSetpointW() {
  return lastSetpointW_;
}
