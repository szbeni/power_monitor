#include "jsy.h"

#include "config.h"
#include "modbus_util.h"

// JSY-MK-194 on SoftwareSerial @ 9600.
// Logical register addresses are 4-byte units.
// Read 14 regs from 0x0048 → 56 data bytes.
namespace {
constexpr uint16_t REG_START = 0x0048;
constexpr uint8_t REG_COUNT = 0x0E;
constexpr size_t DATA_BYTES = 56;

constexpr size_t off(uint16_t reg) {
  return size_t(reg - REG_START) * 4;
}

bool readFrame(SoftwareSerial& bus, uint8_t* dataOut) {
  uint8_t req[] = {
      JSY_SLAVE_ID,
      0x03,
      uint8_t(REG_START >> 8),
      uint8_t(REG_START & 0xff),
      0x00,
      REG_COUNT,
      0,
      0};
  modbusCalcCrc(req, sizeof(req));

  while (bus.available()) {
    bus.read();
  }

  bus.write(req, sizeof(req));
  bus.flush();

  uint8_t frame[3 + DATA_BYTES + 2];
  size_t n = 0;
  const uint32_t start = millis();
  while (n < sizeof(frame) && millis() - start < 400) {
    if (!bus.available()) {
      delay(2);
      continue;
    }
    frame[n++] = bus.read();
    if (n == 1 && frame[0] != JSY_SLAVE_ID) {
      n = 0;
    } else if (n == 2 && frame[1] != 0x03) {
      n = 0;
    } else if (n == 3 && frame[2] != DATA_BYTES) {
      if (frame[2] < 4 || frame[2] > 80) {
        n = 0;
      }
    } else if (n >= 3 && n >= size_t(5 + frame[2])) {
      break;
    }
  }

  if (n < 5 || frame[2] != DATA_BYTES) {
    return false;
  }
  if (!modbusCheckCrc(frame, 3 + DATA_BYTES + 2)) {
    return false;
  }
  memcpy(dataOut, frame + 3, DATA_BYTES);
  return true;
}
} // namespace

bool jsyBegin(SoftwareSerial& bus) {
  bus.begin(JSY_BAUD);
  bus.listen();
  delay(20);
  return true;
}

bool jsyReadLoad2(SoftwareSerial& bus, JsyLoad2& out) {
  out = {};
  bus.listen();

  uint8_t data[DATA_BYTES];
  if (!readFrame(bus, data)) {
    return false;
  }

  const uint8_t sign2 = data[off(0x004E) + 1];
  const float sign = sign2 ? -1.0f : 1.0f;

  out.frequency = beU32(data + off(0x004F)) * 0.01f;
  out.voltage = beU32(data + off(0x0050)) * 0.0001f;
  out.current = beU32(data + off(0x0051)) * 0.0001f;
  out.activePower = beU32(data + off(0x0052)) * 0.0001f * sign;
  // 0x0053 / 0x0055: Wh with scale 0.1 (same as MycilaJSY)
  out.energyImportWh = beU32(data + off(0x0053)) * 0.1f;
  out.powerFactor = beU32(data + off(0x0054)) * 0.001f;
  out.energyExportWh = beU32(data + off(0x0055)) * 0.1f;
  out.ok = out.frequency > 0 && !isnan(out.activePower);
  return out.ok;
}
