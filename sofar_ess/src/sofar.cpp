#include "sofar.h"

#include "config.h"
#include "modbus_util.h"

namespace {
HardwareSerial* bus_ = nullptr;
SofarMode lastMode_ = SofarMode::Unknown;
uint16_t lastSetpointW_ = 0;

constexpr uint8_t FN_READ = 0x03;
constexpr uint8_t FN_PASSIVE = 0x42;
constexpr uint8_t FN_HEARTBEAT = 0x49;
constexpr uint16_t CMD_STANDBY = 0x0100;
constexpr uint16_t CMD_DISCHARGE = 0x0101;
constexpr uint16_t CMD_CHARGE = 0x0102;
constexpr uint16_t CMD_AUTO = 0x0103;
constexpr uint16_t PARAM_STANDBY = 0x5555;
constexpr uint16_t REG_RUNSTATE = 0x0200;
constexpr uint16_t REG_FAULT1 = 0x0201;   // fault bytes 0–1 … through REG_FAULT1+4
constexpr uint16_t REG_BATTW = 0x020d;
constexpr uint16_t REG_BATTSOC = 0x0210;
constexpr uint16_t REG_GRIDW = 0x0212;
constexpr uint16_t REG_PVDAY = 0x0218;    // today generation ×0.01 kWh
constexpr uint16_t REG_ALERT = 0x022b;
constexpr uint16_t REG_BATTFAULT1 = 0x023d; // batt fault bytes 0–1 … +4
constexpr uint16_t REG_PV1_V = 0x0250;    // PV1/2 block: V×0.1, I×0.01 A, P×0.01 kW

constexpr uint32_t kListenTimeoutMs = 400;
// A reply that is still in flight when we transmit would be read as the answer
// to the *next* request (FN 0x03 replies carry no register address, so nothing
// else catches it). Drain until the bus has been idle for one Modbus t3.5 gap
// (~4 ms @ 9600 8N1) instead of a fixed delay.
constexpr uint32_t kBusQuietMs = 6;
constexpr uint32_t kBusDrainMaxMs = 250;

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
  const uint32_t start = millis();
  uint32_t lastByte = start;
  uint16_t dropped = 0;
  while (millis() - lastByte < kBusQuietMs) {
    if (bus_->available()) {
      bus_->read();
      dropped++;
      lastByte = millis();
    } else {
      delay(1);
    }
    if (millis() - start > kBusDrainMaxMs) {
      break;
    }
  }
  if (dropped) {
    Log.printf("[sofar] drained %u stale byte(s)\n", dropped);
  }
}

// Returns false and sets why on failure (static buffer, overwritten each call).
const char* listenFailWhy_ = "";

bool listen(Resp& resp, uint8_t expectId, uint8_t expectFn, bool verbose = true) {
  uint8_t frame[64];
  uint8_t n = 0;
  uint8_t fn = 0;
  uint8_t dataBytes = 0;
  uint8_t expectLen = 0; // total frame length once known, 0 = still unknown
  bool done = false;
  bool exception = false;
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

    if (n == 0) {
      if (frame[0] != expectId) {
        junk++;
        continue;
      }
    } else if (n == 1) {
      fn = frame[1];
      if (fn == uint8_t(expectFn | 0x80)) {
        // Modbus exception: [id][fn|0x80][code][crc][crc] — byte 2 is the
        // exception code, not a byte count.
        exception = true;
        expectLen = 5;
      } else if (fn != expectFn) {
        listenFailWhy_ = "fn";
        if (verbose) {
          Log.printf("[sofar] unexpected fn 0x%02X (want 0x%02X)\n", fn, expectFn);
        }
        return false;
      }
    } else if (n == 2 && !exception) {
      // Byte 2 is data-byte count for normal replies (same as Sofar2mqtt).
      dataBytes = frame[2];
      if (dataBytes < 2 || dataBytes > sizeof(resp.data)) {
        listenFailWhy_ = "len";
        if (verbose) {
          Log.printf("[sofar] bad byte count %u\n", dataBytes);
        }
        return false;
      }
      expectLen = dataBytes + 5; // 3 header + data + 2 CRC
    }

    n++;
    done = (expectLen != 0) && (n >= expectLen);
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
    if (verbose) {
      Log.printf("[sofar] CRC fail n=%u:", n);
      for (uint8_t i = 0; i < n && i < 16; i++) {
        Log.printf(" %02X", frame[i]);
      }
      Log.println();
    }
    return false;
  }
  if (exception) {
    listenFailWhy_ = "exception";
    if (verbose) {
      Log.printf("[sofar] exception fn=0x%02X code=0x%02X\n", fn, frame[2]);
    }
    return false;
  }

  resp.dataSize = dataBytes;
  memcpy(resp.data, frame + 3, dataBytes);
  resp.ok = true;
  listenFailWhy_ = "";
  return true;
}

bool sendRaw(uint8_t* frame, size_t size, Resp* resp, uint8_t expectFn) {
  modbusCalcCrc(frame, size);
  flushBus();
  digitalWrite(SOFAR_DE_PIN, HIGH);
  delay(1);
  bus_->write(frame, size);
  bus_->flush();
  digitalWrite(SOFAR_DE_PIN, LOW);

  if (!resp) {
    // Fire-and-forget (heartbeat): consume the reply so it can't be mistaken
    // for the next read's answer, but its framing is undocumented — don't log.
    Resp dummy;
    listen(dummy, SOFAR_SLAVE_ID, expectFn, false);
    return true;
  }
  return listen(*resp, SOFAR_SLAVE_ID, expectFn);
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
  if (!sendRaw(frame, sizeof(frame), &rs, FN_PASSIVE) || rs.dataSize != 2) {
    Log.printf("[sofar] passive 0x%04X FAIL (%s)\n", cmd, listenFailWhy_);
    return false;
  }
  const uint16_t code = (uint16_t(rs.data[0]) << 8) | rs.data[1];
  Log.printf("[sofar] passive 0x%04X param=%u -> 0x%04X\n", cmd, param, code);
  return (code & 0xff) == 0;
}

bool readRegs(uint16_t reg, uint16_t count, uint16_t* values) {
  if (count == 0 || count > 16 || !values) {
    listenFailWhy_ = "bad-count";
    return false;
  }
  uint8_t frame[] = {
      SOFAR_SLAVE_ID,
      FN_READ,
      uint8_t(reg >> 8),
      uint8_t(reg & 0xff),
      uint8_t(count >> 8),
      uint8_t(count & 0xff),
      0,
      0};
  Resp rs;
  const uint8_t expectBytes = uint8_t(count * 2);
  if (!sendRaw(frame, sizeof(frame), &rs, FN_READ) || rs.dataSize != expectBytes) {
    Log.printf("[sofar] read 0x%04X x%u FAIL (%s)\n", reg, count, listenFailWhy_);
    return false;
  }
  for (uint16_t i = 0; i < count; i++) {
    values[i] = (uint16_t(rs.data[i * 2]) << 8) | rs.data[i * 2 + 1];
  }
  return true;
}

bool readReg(uint16_t reg, uint16_t& value) {
  return readRegs(reg, 1, &value);
}

void setLastError(SofarStatus& cache, const char* why) {
  if (!why) {
    why = "";
  }
  strncpy(cache.lastError, why, sizeof(cache.lastError) - 1);
  cache.lastError[sizeof(cache.lastError) - 1] = '\0';
}
} // namespace

// Append ",name" (or just "name" if empty) for each set bit in byte using names[8].
static size_t appendBits(char* out, size_t outLen, size_t used, uint8_t bits, const char* const names[8]) {
  for (uint8_t b = 0; b < 8; b++) {
    if (!(bits & (1u << b)) || !names[b] || !names[b][0]) {
      continue;
    }
    const size_t nlen = strlen(names[b]);
    const size_t need = (used ? 1 : 0) + nlen;
    if (used + need >= outLen) {
      break;
    }
    if (used) {
      out[used++] = ',';
    }
    memcpy(out + used, names[b], nlen);
    used += nlen;
  }
  if (outLen) {
    out[used < outLen ? used : outLen - 1] = '\0';
  }
  return used;
}

const char* sofarRunStateName(uint16_t runState) {
  switch (runState) {
    case 0:
      return "wait";
    case 1:
      return "check";
    case 2:
      return "normal";
    case 3:
      return "check_discharge";
    case 4:
      return "discharge";
    case 5:
      return "eps";
    case 6:
      return "fault";
    case 7:
      return "permanent_fault";
    default:
      return "unknown";
  }
}

size_t sofarFormatFaults(const SofarStatus& s, char* out, size_t outLen) {
  if (!out || outLen == 0) {
    return 0;
  }
  out[0] = '\0';
  if (!s.faultValid) {
    return 0;
  }

  // PDF: Fault Message bytes 0–9. Only named (non-reserved) bits listed.
  static const char* const byte0[8] = {
      "GridOVP", "GridUVP", "GridOFP", "GridUFP", "BatOVP", nullptr, nullptr, nullptr};
  static const char* const byte1[8] = {
      "HW_LLCBus_OVP", "HW_Boost_OVP", "HwBuckBoostOCP", "HwBatOCP", nullptr, nullptr, "HwAcOCP", nullptr};
  static const char* const byte2[8] = {
      "HwADFaultIGrid", "HwADFaultDCI", "HwADFaultVGrid", nullptr, "MChip_Fault", "HwAuxPowerFault", nullptr, nullptr};
  static const char* const byte3[8] = {
      "LLCBusOVP", "SwBusOVP", "BatOCP", "DciOCP", "SwOCPInstant", "BuckOCP", "AcRmsOCP", nullptr};
  static const char* const byte4[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
  static const char* const byte5[8] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
  static const char* const byte6[8] = {
      "ConsistentFault_VGrid",
      "ConsistentFault_FGrid",
      "ConsistentFault_DCI",
      "BatCommunication",
      "SpiCommLose",
      "SciCommLose",
      "RecoverRelayFail",
      nullptr};
  static const char* const byte7[8] = {
      "OverTempFault_BAT", "OverTempFault_HeatSink", "OverTempFault_Env", nullptr, nullptr, nullptr, nullptr, nullptr};
  static const char* const byte8[8] = {
      "unrecoverHwAcOCP",
      "unrecoverBusOVP",
      "unrecoverBatOCP_EPS",
      nullptr,
      nullptr,
      "unrecoverOCPInstant",
      nullptr,
      nullptr};
  static const char* const byte9[8] = {
      nullptr, nullptr, "unrecoverEEPROM_W", "unrecoverEEPROM_R", "unrecoverRelayFail", nullptr, nullptr, nullptr};

  static const char* const* const bytes[10] = {
      byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9};

  size_t used = 0;
  for (uint8_t wi = 0; wi < 5; wi++) {
    const uint8_t lo = uint8_t(s.fault[wi] & 0xff);         // byte 2*wi
    const uint8_t hi = uint8_t((s.fault[wi] >> 8) & 0xff);  // byte 2*wi+1
    used = appendBits(out, outLen, used, lo, bytes[wi * 2]);
    used = appendBits(out, outLen, used, hi, bytes[wi * 2 + 1]);
  }
  if (used == 0 && (s.runState == 6 || s.runState == 7)) {
    // Fault state but no named bits — still surface raw words.
    snprintf(out, outLen, "raw:%04X:%04X:%04X:%04X:%04X",
             s.fault[0], s.fault[1], s.fault[2], s.fault[3], s.fault[4]);
    return strlen(out);
  }
  return used;
}

size_t sofarFormatAlerts(const SofarStatus& s, char* out, size_t outLen) {
  if (!out || outLen == 0) {
    return 0;
  }
  out[0] = '\0';
  if (!s.alertValid) {
    return 0;
  }
  static const char* const alert0[8] = {
      "OverTempAlarm",
      "OverFreqAlarm",
      "RemoteDerate",
      "RemoteOff",
      nullptr,
      nullptr,
      nullptr,
      "BatLowVoltageAlarm"};
  // 0x022B: low byte = alert byte0 (PDF).
  return appendBits(out, outLen, 0, uint8_t(s.alert & 0xff), alert0);
}

size_t sofarFormatBattFaults(const SofarStatus& s, char* out, size_t outLen) {
  if (!out || outLen == 0) {
    return 0;
  }
  out[0] = '\0';
  if (!s.battFaultValid) {
    return 0;
  }
  static const char* const byte0[8] = {
      "BatOCD", "BatSCD", "BatOV", "BatUV", "BatOTD", "BatOTC", "BatUTD", "BatUTC"};
  size_t used = appendBits(out, outLen, 0, uint8_t(s.battFault[0] & 0xff), byte0);
  // Higher batt-fault bytes are reserved in the PDF; still show raw if any set.
  bool anyHigher = false;
  for (uint8_t i = 0; i < 5; i++) {
    const uint16_t mask = (i == 0) ? 0xff00u : 0xffffu;
    if (s.battFault[i] & mask) {
      anyHigher = true;
      break;
    }
  }
  if (used == 0 && anyHigher) {
    snprintf(out,
             outLen,
             "raw:%04X:%04X:%04X:%04X:%04X",
             s.battFault[0],
             s.battFault[1],
             s.battFault[2],
             s.battFault[3],
             s.battFault[4]);
    return strlen(out);
  }
  return used;
}

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
  uint8_t frame[] = {SOFAR_SLAVE_ID, FN_HEARTBEAT, 0x22, 0x01, 0x22, 0x02, 0x00, 0x00};
  sendRaw(frame, sizeof(frame), nullptr, FN_HEARTBEAT);
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

// SoC must be 0..100. A large step is more likely a mis-paired reply than a
// real jump, so make it prove itself on the next poll before we believe it.
static uint16_t socPending_ = 0;
static bool socHavePending_ = false;
static bool preferSocPoll_ = false;
static bool requireSocConfirm_ = false;

static bool acceptSoc(SofarStatus& cache, uint16_t v) {
  if (v > 100) {
    Log.printf("[sofar] SoC %u out of range — ignored\n", v);
    socHavePending_ = false;
    return false;
  }

  // After a bank switch, always require two matching reads before trusting SOC.
  if (requireSocConfirm_) {
    if (!(socHavePending_ && socPending_ == v)) {
      Log.printf("[sofar] SoC post-switch %u — waiting for confirmation\n", v);
      socPending_ = v;
      socHavePending_ = true;
      return false;
    }
    requireSocConfirm_ = false;
  }

  const int delta = int(v) - int(cache.batterySoc);
  const bool jump = cache.socValid && (delta > 20 || delta < -20);
  if (jump && !(socHavePending_ && socPending_ == v)) {
    Log.printf("[sofar] SoC jump %u -> %u — waiting for confirmation\n", cache.batterySoc, v);
    socPending_ = v;
    socHavePending_ = true;
    return false;
  }

  socHavePending_ = false;
  cache.batterySoc = v;
  cache.socValid = true;
  return true;
}

void sofarInvalidateSoc(SofarStatus& cache) {
  cache.socValid = false;
  cache.batterySoc = 0;
  socHavePending_ = false;
  socPending_ = 0;
  requireSocConfirm_ = true;
  Log.println("[sofar] SoC invalidated (awaiting new bank)");
}

void sofarPreferSocPoll(bool enable) {
  preferSocPoll_ = enable;
}

bool sofarPollSocNow(SofarStatus& cache) {
  if (!bus_) {
    return false;
  }
  uint16_t v = 0;
  if (!readReg(REG_BATTSOC, v)) {
    setLastError(cache, listenFailWhy_);
    return false;
  }
  const bool ok = acceptSoc(cache, v);
  if (!ok) {
    setLastError(cache, "soc-reject");
  } else {
    cache.ok = true;
    setLastError(cache, "");
  }
  return ok;
}

bool sofarPollStatusField(SofarStatus& cache) {
  // One Modbus txn per call so ESS timing stays predictable.
  // Phases: run → grid → batt → soc → fault[5] → alert → battFault[5]
  //         → pv strings[6] → pv today. PV is low-priority telemetry.
  static uint8_t phase = 0;
  static uint8_t failStreak = 0;
  uint16_t v = 0;
  bool ok = false;

  // After a bank switch, bias every poll toward SOC until acceptSoc succeeds.
  if (preferSocPoll_) {
    phase = 3;
  }

  switch (phase % 9) {
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
    case 3:
      ok = readReg(REG_BATTSOC, v);
      if (ok) {
        ok = acceptSoc(cache, v);
        if (!ok) {
          setLastError(cache, "soc-reject");
        } else if (preferSocPoll_) {
          preferSocPoll_ = false;
        }
      }
      break;
    case 4:
      ok = readRegs(REG_FAULT1, 5, cache.fault);
      if (ok) {
        cache.faultValid = true;
        if (cache.runState == 6 || cache.runState == 7 ||
            cache.fault[0] || cache.fault[1] || cache.fault[2] || cache.fault[3] || cache.fault[4]) {
          char buf[160];
          sofarFormatFaults(cache, buf, sizeof(buf));
          Log.printf("[sofar] run=%u(%s) faults=%s raw=%04X %04X %04X %04X %04X\n",
                     cache.runState,
                     sofarRunStateName(cache.runState),
                     buf[0] ? buf : "(none)",
                     cache.fault[0],
                     cache.fault[1],
                     cache.fault[2],
                     cache.fault[3],
                     cache.fault[4]);
        }
      }
      break;
    case 5:
      ok = readReg(REG_ALERT, v);
      if (ok) {
        cache.alert = v;
        cache.alertValid = true;
        if (v) {
          char buf[96];
          sofarFormatAlerts(cache, buf, sizeof(buf));
          Log.printf("[sofar] alert=0x%04X %s\n", v, buf[0] ? buf : "(unnamed)");
        }
      }
      break;
    case 6:
      ok = readRegs(REG_BATTFAULT1, 5, cache.battFault);
      if (ok) {
        cache.battFaultValid = true;
        if (cache.battFault[0] || cache.battFault[1] || cache.battFault[2] || cache.battFault[3] ||
            cache.battFault[4]) {
          char buf[96];
          sofarFormatBattFaults(cache, buf, sizeof(buf));
          Log.printf("[sofar] batt_fault=%s raw=%04X %04X %04X %04X %04X\n",
                     buf[0] ? buf : "(none)",
                     cache.battFault[0],
                     cache.battFault[1],
                     cache.battFault[2],
                     cache.battFault[3],
                     cache.battFault[4]);
        }
      }
      break;
    case 7: {
      // PV1/PV2 V/I/P as one 6-register block (hybrid ME3000SP map).
      uint16_t pv[6] = {};
      ok = readRegs(REG_PV1_V, 6, pv);
      if (ok) {
        cache.pv1VoltageV = float(int16_t(pv[0])) * 0.1f;
        cache.pv1CurrentA = float(int16_t(pv[1])) * 0.01f;
        cache.pv1PowerW = float(int16_t(pv[2])) * 0.01f * 1000.0f;
        cache.pv2VoltageV = float(int16_t(pv[3])) * 0.1f;
        cache.pv2CurrentA = float(int16_t(pv[4])) * 0.01f;
        cache.pv2PowerW = float(int16_t(pv[5])) * 0.01f * 1000.0f;
        cache.pvTotalW = cache.pv1PowerW + cache.pv2PowerW;
        cache.pvValid = true;
      }
      break;
    }
    default:
      ok = readReg(REG_PVDAY, v);
      if (ok) {
        cache.pvTodayKwh = float(v) * 0.01f;
        cache.pvTodayValid = true;
      }
      break;
  }

  // Advancing unconditionally means one dropped reply offsets the round robin
  // forever, and FN 0x03 replies carry no register address to catch it with.
  // Retry the same register, but give up after a few tries so an unsupported
  // one can't stall the rotation.
  if (ok) {
    failStreak = 0;
    phase++;
    cache.ok = true;
    setLastError(cache, "");
  } else {
    if (listenFailWhy_[0] && strcmp(cache.lastError, "soc-reject") != 0) {
      setLastError(cache, listenFailWhy_);
    } else if (!cache.lastError[0]) {
      setLastError(cache, listenFailWhy_);
    }
    if (++failStreak >= 3) {
      failStreak = 0;
      phase++;
    }
  }
  return ok;
}

SofarMode sofarLastMode() {
  return lastMode_;
}

uint16_t sofarLastSetpointW() {
  return lastSetpointW_;
}
