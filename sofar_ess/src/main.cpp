#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <MycilaJSY.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <cmath>
#include <cinttypes>
#include <cstring>

#include "config.h"
#include "sofar.h"

// ESP32-C3 has only 2 UARTs; map Serial2 helpers like Mycila examples.
#ifndef SOC_UART_HP_NUM
#define SOC_UART_HP_NUM SOC_UART_NUM
#endif
#if SOC_UART_HP_NUM < 3
#define Serial2 Serial1
#endif

// UART0 (Serial0) = Sofar RS485, UART1 (Serial2 alias) = JSY. USB CDC is Serial.
static Mycila::JSY jsy;
static Mycila::JSY::Data jsyData;
static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t lastJsyReadMs = 0;
static bool jsyReady = false;

static WiFiClient wifi;
static PubSubClient mqtt(wifi);

static bool essEnabled = ESS_ENABLE;
// EPS switch (HA / set/eps). When on and grid is lost, send 0x41 enable.
static bool epsEnabled = EPS_ENABLE_DEFAULT;
enum class EpsApplied : uint8_t { Unknown, Enable, Disable };
static EpsApplied epsApplied = EpsApplied::Unknown;
static uint32_t epsLastSendMs = 0;
static bool gridDown = false;      // from inverter grid voltage (epsUpdateGridState)
static float gridVoltageV = NAN;   // inverter grid A voltage, NAN if stale
static uint32_t lastGridSampleMs = 0;
static uint8_t gridLowCount = 0;
static uint8_t gridHighCount = 0;
static bool essChargeOnlyUser = ESS_CHARGE_ONLY;
static bool essSocProtectEnabled = ESS_SOC_PROTECT_ENABLE;
static uint8_t essSocProtectLow = ESS_SOC_PROTECT_LOW;
static uint8_t essSocProtectHyst = ESS_SOC_PROTECT_HYST;
static bool essSocProtectActive = false; // low SOC → charge-only
static uint8_t essSocProtectHigh = ESS_SOC_PROTECT_HIGH;
static uint8_t essSocProtectHighHyst = ESS_SOC_PROTECT_HIGH_HYST;
static bool essSocFullActive = false; // high SOC → no charge
static float essKp = ESS_KP;
static float essKi = ESS_KI;
static float essDeadbandW = float(ESS_DEADBAND_W);
static float essMinDeltaW = float(ESS_MIN_DELTA_W);
static uint16_t essHoldMinW = ESS_HOLD_MIN_W;
static float essIntegral = 0.0f;
static float lastGridPowerW = NAN;
static float lastEnergyImportWh = NAN;
static float lastEnergyExportWh = NAN;
static SofarStatus lastSofar{};
static bool lastSofarOk = false;
static uint32_t lastEssMs = 0;
static uint32_t lastHeartbeatMs = 0;
static uint32_t lastMqttStateMs = 0;
static uint32_t lastBattParamPollMs = 0;
static uint32_t lastBattParamRegMs = 0;
static uint32_t lastEnergyPublishMs = 0;
static bool battParamPollNow = true;
static bool battParamProbing = false;
static uint8_t battParamProbeIdx = 0;
static uint32_t lastCmdMs = 0;
static int16_t lastCmdW = 0; // +discharge, -charge, 0=standby
static uint32_t essChargeBlockedFullMs = 0; // last charge suppressed by full protect
static uint32_t essDischargeBlockedLowMs = 0; // last discharge suppressed by low protect
static uint8_t chargeTargetSoc = 0; // 0 = no limit; manual charge stops at target → standby
// Manual charge session. The command is a max charge power: actual power below it
// (BMS limit, taper) is normal and never a fault. 0 = no session.
static uint16_t maxChargeW = MANUAL_CHARGE_DEFAULT_W; // stored max, used by "charge true"
static uint16_t manualChargeW = 0;
static uint32_t manualChargeSentMs = 0;
static uint32_t manualChargeTryMs = 0;
static uint32_t followProhibitSinceMs = 0; // BMS charge-prohibit debounce (manual session)

enum class FollowRecoverState : uint8_t { Idle = 0, Standby, Hold };
enum class FollowRecovery : uint8_t { None = 0, SwapEmpty, SwapFull, StandbyReset };

static FollowRecoverState followRecoverState = FollowRecoverState::Idle;
static uint32_t followRecoverStateMs = 0;
static uint32_t followGraceUntilMs = 0;
static uint32_t followFailSinceMs = 0;
static int16_t followActualW = 0;
static bool followHaveActual = false;
static bool followOk = true;
static int16_t followErrorW = 0;
static bool followGraceActive = false;
static FollowRecovery followLastRecovery = FollowRecovery::None;
static int16_t followPrevActualW = 0;
static uint32_t followRecoverLastMs = 0;
static uint32_t followRecoverWindowStartMs = 0;
static uint8_t followRecoverWindowCount = 0;

#if DUAL_BATT_ENABLE
enum class DualBattState : uint8_t {
  Idle = 0,
  Standby,
  CommandRelay,
  WaitConfirm,
  Settling,
  SyncSoc,
  Error,
};

static bool dualBattEnabled = true;
static uint8_t dualBattActive = DUAL_BATT_DEFAULT_BANK; // 1=A, 2=B
static uint8_t dualBattForce = 0;                       // 0=auto, 1=A, 2=B
static uint8_t dualBattTarget = 0;                      // bank being switched to (0=none)
static DualBattState dualBattState = DualBattState::Idle;
static uint8_t dualBattEmptySoc = DUAL_BATT_EMPTY_SOC;
static uint8_t dualBattFullSoc = DUAL_BATT_FULL_SOC;
static uint8_t dualBattSocA = 0;
static uint8_t dualBattSocB = 0;
static bool dualBattSocAValid = false;
static bool dualBattSocBValid = false;
static uint32_t dualBattStateMs = 0;
static uint32_t dualBattLastSwitchMs = 0;
static uint8_t dualBattAutoDwellMin = DUAL_BATT_AUTO_DWELL_MIN; // Auto-only switch dwell (min)
static bool dualBattSelectedMatch = false;
static uint8_t dualBattReportedSelected = 0; // from selector MQTT, 0=unknown
static int16_t dualBattResumeCmdW = 0;
static bool dualBattStickyEmptyA = false;
static bool dualBattStickyEmptyB = false;
static bool dualBattStickyFullA = false;
static bool dualBattStickyFullB = false;
// SOC of each bank when its sticky full flag was set. The flag clears once the
// bank's SOC falls DUAL_BATT_FULL_CLEAR_DROP below this (see dualBattExpireStickyFull).
static uint8_t dualBattFullMarkSocA = 0;
static uint8_t dualBattFullMarkSocB = 0;
#endif

#if HA_MQTT_DISCOVERY
static bool haDiscoverySent = false;
#endif

static void resetEssIntegral() {
  essIntegral = 0.0f;
}

// Lowest SOC for discharge / dual-bank empty — follows ESS protect low when enabled.
static uint8_t essDischargeFloorSoc() {
  if (essSocProtectEnabled) {
    return essSocProtectLow;
  }
#if DUAL_BATT_ENABLE
  return dualBattEmptySoc;
#else
  return uint8_t(FOLLOW_EMPTY_SOC);
#endif
}

static const char* followRecoveryName(FollowRecovery r) {
  switch (r) {
    case FollowRecovery::SwapEmpty:
      return "swap_empty";
    case FollowRecovery::SwapFull:
      return "swap_full";
    case FollowRecovery::StandbyReset:
      return "standby_reset";
    default:
      return "none";
  }
}

static void beginFollowGrace() {
  followGraceUntilMs = millis() + uint32_t(FOLLOW_GRACE_MS);
  followFailSinceMs = 0;
  followProhibitSinceMs = 0;
  followGraceActive = true;
  followOk = true;
  followErrorW = 0;
}

static Preferences nvsPrefs;
static constexpr const char* kNvsNs = "sofar_ess";

static void nvsPutBool(const char* key, bool v) {
  if (nvsPrefs.begin(kNvsNs, false)) {
    nvsPrefs.putBool(key, v);
    nvsPrefs.end();
  }
}

static void nvsPutU8(const char* key, uint8_t v) {
  if (nvsPrefs.begin(kNvsNs, false)) {
    nvsPrefs.putUChar(key, v);
    nvsPrefs.end();
  }
}

static void nvsPutU16(const char* key, uint16_t v) {
  if (nvsPrefs.begin(kNvsNs, false)) {
    nvsPrefs.putUInt(key, v);
    nvsPrefs.end();
  }
}

static void nvsPutFloat(const char* key, float v) {
  if (nvsPrefs.begin(kNvsNs, false)) {
    nvsPrefs.putFloat(key, v);
    nvsPrefs.end();
  }
}

static void loadSettingsFromNvs() {
  if (!nvsPrefs.begin(kNvsNs, true)) {
    return;
  }
  essEnabled = nvsPrefs.getBool("ess_en", essEnabled);
  essChargeOnlyUser = nvsPrefs.getBool("chg_only", essChargeOnlyUser);
  epsEnabled = nvsPrefs.getBool("eps_en", epsEnabled);
  essSocProtectEnabled = nvsPrefs.getBool("soc_prot_en", essSocProtectEnabled);
  essSocProtectLow = nvsPrefs.getUChar("soc_lo", essSocProtectLow);
  essSocProtectHyst = nvsPrefs.getUChar("soc_hyst", essSocProtectHyst);
  essSocProtectHigh = nvsPrefs.getUChar("soc_hi", essSocProtectHigh);
  essSocProtectHighHyst = nvsPrefs.getUChar("soc_hi_hyst", essSocProtectHighHyst);
  essKp = nvsPrefs.getFloat("kp", essKp);
  essKi = nvsPrefs.getFloat("ki", essKi);
  essDeadbandW = nvsPrefs.getFloat("deadband", essDeadbandW);
  essMinDeltaW = nvsPrefs.getFloat("min_delta", essMinDeltaW);
  essHoldMinW = uint16_t(nvsPrefs.getUInt("hold_min", essHoldMinW));
  chargeTargetSoc = nvsPrefs.getUChar("chg_tgt", chargeTargetSoc);
  maxChargeW = uint16_t(nvsPrefs.getUInt("chg_max_w", maxChargeW));
  if (maxChargeW > MAX_POWER_W) {
    maxChargeW = MAX_POWER_W;
  } else if (maxChargeW < 200) {
    maxChargeW = 200;
  }
#if DUAL_BATT_ENABLE
  dualBattEmptySoc = nvsPrefs.getUChar("db_empty", dualBattEmptySoc);
  dualBattFullSoc = nvsPrefs.getUChar("db_full", dualBattFullSoc);
  dualBattAutoDwellMin = nvsPrefs.getUChar("db_dwell", dualBattAutoDwellMin);
  dualBattForce = nvsPrefs.getUChar("db_force", dualBattForce);
  dualBattEnabled = nvsPrefs.getBool("db_en", dualBattEnabled);
  if (essSocProtectEnabled && dualBattEmptySoc > essSocProtectLow) {
    dualBattEmptySoc = essSocProtectLow;
    nvsPutU8("db_empty", dualBattEmptySoc);
  }
#endif
  nvsPrefs.end();
  Log.println("[nvs] settings loaded");
}

static bool mqttPublish(const char* suffix, const String& payload, bool retain = false) {
  if (!mqtt.connected()) {
    return false;
  }
  String topic = String(DEVICE_NAME) + "/" + suffix;
  if (!mqtt.publish(topic.c_str(), payload.c_str(), retain)) {
    Log.printf("[mqtt] publish fail %s (len=%u)\n",
               topic.c_str(),
               static_cast<unsigned>(payload.length()));
    return false;
  }
  return true;
}

static bool essEffectiveChargeOnly() {
  return essChargeOnlyUser || essSocProtectActive;
}

static bool essNoCharge() {
  return essSocFullActive;
}

static void publishSocProtectParams() {
  mqttPublish("ess/soc_protect/enabled", essSocProtectEnabled ? "true" : "false", true);
  mqttPublish("ess/soc_protect/low", String(essSocProtectLow), true);
  mqttPublish("ess/soc_protect/hyst", String(essSocProtectHyst), true);
  mqttPublish("ess/soc_protect/active", essSocProtectActive ? "true" : "false", true);
  mqttPublish("ess/soc_protect/high", String(essSocProtectHigh), true);
  mqttPublish("ess/soc_protect/high_hyst", String(essSocProtectHighHyst), true);
  mqttPublish("ess/soc_protect/full_active", essSocFullActive ? "true" : "false", true);
}

static void publishEssParams() {
  mqttPublish("ess/kp", String(essKp, 3), true);
  mqttPublish("ess/ki", String(essKi, 3), true);
  mqttPublish("ess/deadband", String(essDeadbandW, 1), true);
  mqttPublish("ess/min_delta", String(essMinDeltaW, 1), true);
  mqttPublish("ess/hold_min", String(essHoldMinW), true);
  mqttPublish("ess/charge_only", essChargeOnlyUser ? "true" : "false", true);
  mqttPublish("ess/effective_charge_only", essEffectiveChargeOnly() ? "true" : "false", true);
  publishSocProtectParams();
}

#if DUAL_BATT_ENABLE
static bool dualBattBusy();
#endif

static void publishBattParamDiag(const char* skippedReason = nullptr) {
  if (skippedReason && skippedReason[0]) {
    mqttPublish("batt_param/poll_diag", skippedReason);
    mqttPublish("batt_param/last_error", skippedReason);
    return;
  }
  mqttPublish("batt_param/poll_diag", lastSofar.battParams.pollDiag[0] ? lastSofar.battParams.pollDiag : "unknown");
  if (lastSofar.battParams.regDiag[0]) {
    mqttPublish("batt_param/reg_diag", lastSofar.battParams.regDiag);
  }
  mqttPublish("batt_param/last_error", lastSofar.lastError[0] ? lastSofar.lastError : sofarLastModbusError());
  mqttPublish("batt_param/poll_attempts", String(lastSofar.battParams.pollAttempts));
  mqttPublish("batt_param/last_poll_ms", String(lastSofar.battParams.lastPollMs));
  const uint8_t okCount = sofarBattParamOkCount(lastSofar.battParams);
  mqttPublish("batt_param/probe_progress",
              String(okCount) + "/" + String(sofarBattParamCount()));
}

static void publishBattParamReg(uint8_t index) {
  const SofarBattParamMeta* m = sofarBattParamMeta(index);
  if (!m) {
    return;
  }
  char regHex[8];
  snprintf(regHex, sizeof(regHex), "%04x", m->reg);
  const char* why = lastSofar.battParams.regWhy[index][0] ? lastSofar.battParams.regWhy[index] : "?";
  mqttPublish((String("batt_param/reg/") + regHex).c_str(), why);

  if (!lastSofar.battParams.regOk[index]) {
    return;
  }
  const float scaled = sofarBattParamScaled(lastSofar.battParams, index);
  String topic = String("batt_param/") + m->key;
  if (m->decimals == 0) {
    mqttPublish(topic.c_str(), String(lroundf(scaled)));
  } else {
    mqttPublish(topic.c_str(), String(scaled, unsigned(m->decimals)));
  }
  char hex[8];
  snprintf(hex, sizeof(hex), "%04X", lastSofar.battParams.raw[index]);
  mqttPublish((String("batt_param/raw/") + regHex).c_str(), hex);
}

static void publishBattParams() {
  const uint8_t okCount = sofarBattParamOkCount(lastSofar.battParams);
  mqttPublish("batt_param/valid", lastSofar.battParams.valid ? "true" : "false");
  publishBattParamDiag();
  for (uint8_t i = 0; i < sofarBattParamCount(); i++) {
    publishBattParamReg(i);
  }
  if (okCount == 0) {
    return;
  }
  if (lastSofar.battParams.valid) {
    lastSofar.battParams.updatedMs = millis();
  }
}

static bool battParamPollAllowed() {
#if DUAL_BATT_ENABLE
  if (dualBattBusy()) {
    return false;
  }
#endif
  return true;
}

static void energyMeterPublishTick() {
  const uint32_t now = millis();
  if ((now - lastEnergyPublishMs) < uint32_t(ENERGY_METER_PUBLISH_MS)) {
    return;
  }
  if (isnan(lastEnergyImportWh) && isnan(lastEnergyExportWh)) {
    return;
  }
  lastEnergyPublishMs = now;
  if (!isnan(lastEnergyImportWh)) {
    const String v = String(lastEnergyImportWh, 1);
    mqttPublish("ess/energy_import_wh", v);
    mqttPublish("grid_import_wh", v);
  }
  if (!isnan(lastEnergyExportWh)) {
    const String v = String(lastEnergyExportWh, 1);
    mqttPublish("ess/energy_export_wh", v);
    mqttPublish("grid_export_wh", v);
  }
}

static void battParamProbeFinish(uint32_t now) {
  sofarBattParamBuildDiag(lastSofar.battParams);
  lastBattParamPollMs = now;
  battParamProbing = false;
  battParamProbeIdx = 0;
  publishBattParams();
  Log.printf("[batt_param] probe done — %s\n%s\n",
             lastSofar.battParams.pollDiag,
             lastSofar.battParams.regDiag);
}

static void battParamPollTick(bool force = false) {
#if !BATT_PARAM_ENABLE
  static bool publishedDisabled = false;
  if (!publishedDisabled) {
    mqttPublish("batt_param/valid", "false", true);
    mqttPublish("batt_param/poll_diag", "disabled", true);
    publishedDisabled = true;
  }
  return;
#else
  const uint32_t now = millis();

  if (!battParamProbing) {
    const uint32_t interval =
        lastSofar.battParams.valid ? uint32_t(BATT_PARAM_POLL_MS) : uint32_t(BATT_PARAM_RETRY_MS);
    if (!force && !battParamPollNow && (now - lastBattParamPollMs) < interval) {
      return;
    }
    if (!battParamPollAllowed()) {
      if (battParamPollNow) {
        publishBattParamDiag("skipped: dual_batt_busy");
      }
      return;
    }
    battParamPollNow = false;
    lastSofar.battParams.pollAttempts++;
    lastSofar.battParams.lastPollMs = now;
    sofarBattParamProbeBegin(lastSofar.battParams);
    battParamProbing = true;
    battParamProbeIdx = 0;
    lastBattParamRegMs = 0; // first register immediately
    mqttPublish("batt_param/probe_progress", "0/" + String(sofarBattParamCount()));
    Log.println("[batt_param] probe start");
  }

  if (!battParamPollAllowed()) {
    return; // pause mid-cycle — resume when dual-batt idle
  }
  if (lastBattParamRegMs != 0 && (now - lastBattParamRegMs) < uint32_t(BATT_PARAM_REG_GAP_MS)) {
    return;
  }

  const bool ok = sofarPollBatteryParamReg(lastSofar, battParamProbeIdx);
  if (ok) {
    lastSofarOk = true;
  }
  publishBattParamReg(battParamProbeIdx);
  const SofarBattParamMeta* probed = sofarBattParamMeta(battParamProbeIdx);
  Log.printf("[batt_param] 0x%04X %s\n",
             probed ? unsigned(probed->reg) : unsigned(0x10B1 + battParamProbeIdx),
             lastSofar.battParams.regWhy[battParamProbeIdx]);

  battParamProbeIdx++;
  lastBattParamRegMs = now;

  if (battParamProbeIdx >= sofarBattParamCount()) {
    battParamProbeFinish(now);
  } else {
    const uint8_t done = battParamProbeIdx;
    mqttPublish("batt_param/probe_progress", String(done) + "/" + String(sofarBattParamCount()));
  }
#endif
}

static bool handleBattParamSet(const String& key, const String& msg) {
#if !BATT_PARAM_ENABLE
  Log.println("[batt_param] disabled at compile time");
  return false;
#else
  const int8_t idx = sofarBattParamIndexByKey(key.c_str());
  if (idx < 0) {
    Log.printf("[batt_param] unknown key %s\n", key.c_str());
    return false;
  }
  const SofarBattParamMeta* m = sofarBattParamMeta(uint8_t(idx));
  if (!m || !m->writable) {
    Log.printf("[batt_param] %s not writable\n", key.c_str());
    return false;
  }
  uint16_t raw = 0;
  if (!sofarBattParamScaledToRaw(uint8_t(idx), msg.toFloat(), raw)) {
    Log.printf("[batt_param] %s scale failed\n", key.c_str());
    return false;
  }
  if (!sofarBattParamValidateWrite(lastSofar.battParams, uint8_t(idx), raw)) {
    Log.printf("[batt_param] %s raw=%u out of range\n", key.c_str(), raw);
    return false;
  }

  const bool wasEss = essEnabled;
  const int16_t resumeCmd = lastCmdW;
  const SofarMode resumeMode = sofarLastMode();
  sofarStandby();

  const bool ok = sofarWriteBatteryParam(lastSofar, uint8_t(idx), raw);
  if (!ok) {
    publishBattParamDiag();
    Log.printf("[batt_param] write %s failed (%s)\n", key.c_str(), lastSofar.lastError);
    if (wasEss) {
      if (resumeCmd > 0) {
        sofarDischarge(uint16_t(resumeCmd));
      } else if (resumeCmd < 0) {
        sofarCharge(uint16_t(-resumeCmd));
      }
    } else if (resumeMode == SofarMode::Auto) {
      sofarAuto();
    }
    return false;
  }

  publishBattParams();
  lastBattParamPollMs = millis();

  if (wasEss) {
    if (resumeCmd > 0) {
      sofarDischarge(uint16_t(resumeCmd));
    } else if (resumeCmd < 0) {
      sofarCharge(uint16_t(-resumeCmd));
    }
  } else if (resumeMode == SofarMode::Auto) {
    sofarAuto();
  }
  Log.printf("[batt_param] %s set to %s (raw=%u)\n", key.c_str(), msg.c_str(), raw);
  return true;
#endif
}

#if DUAL_BATT_ENABLE
static void dualBattMarkActiveFull();
#endif

// Bidirectional ESS → charge-only when SOC drops; no-charge when SOC is full.
static void updateEssSocProtect() {
  if (!essSocProtectEnabled || !essEnabled) {
    bool changed = false;
    if (essSocProtectActive) {
      essSocProtectActive = false;
      changed = true;
    }
    if (essSocFullActive) {
      essSocFullActive = false;
      changed = true;
    }
    if (changed) {
      publishSocProtectParams();
      mqttPublish("ess/effective_charge_only", essEffectiveChargeOnly() ? "true" : "false", true);
    }
    return;
  }
  if (!lastSofar.socValid) {
    return;
  }

  const uint8_t soc = uint8_t(lastSofar.batterySoc);
  const uint8_t resumeLowAt =
      uint8_t(min(100u, unsigned(essSocProtectLow) + unsigned(essSocProtectHyst)));
  const int resumeHighAt = int(essSocProtectHigh) - int(essSocProtectHighHyst);

  // Low SOC → charge-only
  if (!essSocProtectActive && soc < essSocProtectLow) {
    essSocProtectActive = true;
    if (essIntegral > 0.0f) {
      essIntegral = 0.0f;
    }
    Log.printf("[ess] SOC %u < %u — charge-only protect\n", soc, essSocProtectLow);
    publishSocProtectParams();
    mqttPublish("ess/effective_charge_only", "true", true);
  } else if (essSocProtectActive && soc >= resumeLowAt) {
    essSocProtectActive = false;
    essDischargeBlockedLowMs = 0;
    Log.printf("[ess] SOC %u >= %u — bidirectional (low) resumed\n", soc, resumeLowAt);
    publishSocProtectParams();
    mqttPublish("ess/effective_charge_only", essEffectiveChargeOnly() ? "true" : "false", true);
  }

  // High SOC → no charge
  if (!essSocFullActive && soc >= essSocProtectHigh) {
    essSocFullActive = true;
    if (essIntegral < 0.0f) {
      essIntegral = 0.0f;
    }
    Log.printf("[ess] SOC %u >= %u — full protect (no charge)\n", soc, essSocProtectHigh);
#if DUAL_BATT_ENABLE
    // The latch holds while SOC sags back below the threshold, so remember
    // which pack is full — otherwise the handover policy stops seeing it.
    dualBattMarkActiveFull();
#endif
    publishSocProtectParams();
  } else if (essSocFullActive && int(soc) <= resumeHighAt) {
    essSocFullActive = false;
    essChargeBlockedFullMs = 0;
    Log.printf("[ess] SOC %u <= %d — charge allowed again\n", soc, resumeHighAt);
    publishSocProtectParams();
  }
}

struct Load2Snapshot {
  float voltage = NAN;
  float current = NAN;
  float activePower = NAN;
  float energyImportWh = NAN;
  float energyExportWh = NAN;
  float frequency = NAN;
  bool connected = false;
};

static const char* modeName(SofarMode m) {
  switch (m) {
    case SofarMode::Standby:
      return "standby";
    case SofarMode::Auto:
      return "auto";
    case SofarMode::Charge:
      return "charge";
    case SofarMode::Discharge:
      return "discharge";
    default:
      return "unknown";
  }
}

static Load2Snapshot captureLoad2() {
  Load2Snapshot s;
  Mycila::JSY::Data local;

  portENTER_CRITICAL(&dataMux);
  local = jsyData;
  portEXIT_CRITICAL(&dataMux);

  const auto& ch = local.channel2();
  s.voltage = ch.voltage;
  s.current = ch.current;
  s.activePower = ch.activePower;
  s.energyImportWh = ch.activeEnergyImported;
  s.energyExportWh = ch.activeEnergyReturned;
  s.frequency = local.aggregate.frequency;
  s.connected = s.frequency > 0 && !std::isnan(s.activePower);
  return s;
}



static void publishChargeTargetSoc() {
  mqttPublish("charge/target_soc", String(chargeTargetSoc), true);
}

static void publishManualCharge() {
  mqttPublish("charge/max_w", String(maxChargeW), true);
  mqttPublish("charge/manual_w", String(manualChargeW), true);
}

// Ends the manual charge session so manualChargeTick() stops re-asserting it.
static void endManualCharge() {
  if (manualChargeW != 0) {
    manualChargeW = 0;
    followProhibitSinceMs = 0;
    publishManualCharge();
  }
}

#if DUAL_BATT_ENABLE
static const char* dualBattStateName(DualBattState s) {
  switch (s) {
    case DualBattState::Idle:
      return "idle";
    case DualBattState::Standby:
      return "standby";
    case DualBattState::CommandRelay:
      return "switching";
    case DualBattState::WaitConfirm:
      return "switching";
    case DualBattState::Settling:
      return "settling";
    case DualBattState::SyncSoc:
      return "syncing";
    case DualBattState::Error:
      return "error";
    default:
      return "unknown";
  }
}

static const char* dualBattLabel(uint8_t bank) {
  return (bank == 2) ? "B" : "A";
}

static const char* dualBattHaLabel(uint8_t bank) {
  return (bank == 2) ? "Battery B" : "Battery A";
}

static const char* dualBattForceLabel() {
  if (dualBattForce == 1) {
    return "A";
  }
  if (dualBattForce == 2) {
    return "B";
  }
  return "off";
}

static bool dualBattBusy() {
  return dualBattState != DualBattState::Idle && dualBattState != DualBattState::Error;
}

static float dualBattCombinedSoc() {
  if (!dualBattSocAValid || !dualBattSocBValid) {
    return NAN;
  }
  const float totalKwh = DUAL_BATT_A_KWH + DUAL_BATT_B_KWH;
  if (totalKwh <= 0.0f) {
    return NAN;
  }
  return (float(dualBattSocA) * DUAL_BATT_A_KWH +
          float(dualBattSocB) * DUAL_BATT_B_KWH) /
         totalKwh;
}

static void publishDualBatt() {
  mqttPublish("battery/active", dualBattLabel(dualBattActive), true);
  mqttPublish("battery/dual_state", dualBattStateName(dualBattState), true);
  mqttPublish("battery/force", dualBattForceLabel(), true);
  mqttPublish("battery/empty", String(dualBattEmptySoc), true);
  mqttPublish("battery/full", String(dualBattFullSoc), true);
  mqttPublish("battery/auto_dwell_min", String(dualBattAutoDwellMin), true);
  mqttPublish("battery/dual_enabled", dualBattEnabled ? "true" : "false", true);
  if (dualBattSocAValid) {
    mqttPublish("battery/a/soc", String(dualBattSocA), true);
  } else {
    mqttPublish("battery/a/soc", "null", true);
  }
  if (dualBattSocBValid) {
    mqttPublish("battery/b/soc", String(dualBattSocB), true);
  } else {
    mqttPublish("battery/b/soc", "null", true);
  }
  const float combinedSoc = dualBattCombinedSoc();
  mqttPublish("battery/combined_soc",
              isnan(combinedSoc) ? String("null") : String(combinedSoc, 1),
              true);
  // HA select state: Auto / Battery A / Battery B
  const char* forceHa = "Auto";
  if (dualBattForce == 1) {
    forceHa = "Battery A";
  } else if (dualBattForce == 2) {
    forceHa = "Battery B";
  }
  mqttPublish("battery/force_select", forceHa, true);
}

static void dualBattSetState(DualBattState next) {
  if (dualBattState == next) {
    return;
  }
  dualBattState = next;
  dualBattStateMs = millis();
  Log.printf("[dual] state=%s active=%s target=%s\n",
             dualBattStateName(dualBattState),
             dualBattLabel(dualBattActive),
             dualBattTarget ? dualBattLabel(dualBattTarget) : "-");
  mqttPublish("battery/dual_state", dualBattStateName(dualBattState), true);
}

static void dualBattUpdateCacheFromLive() {
  if (!lastSofar.socValid || dualBattBusy()) {
    return;
  }
  const uint8_t soc = uint8_t(lastSofar.batterySoc);
  bool changed = false;
  if (dualBattActive == 2) {
    changed = !dualBattSocBValid || dualBattSocB != soc;
    dualBattSocB = soc;
    dualBattSocBValid = true;
  } else {
    changed = !dualBattSocAValid || dualBattSocA != soc;
    dualBattSocA = soc;
    dualBattSocAValid = true;
  }
  if (changed) {
    publishDualBatt();
  }
}

static bool dualBattPublishSelect(uint8_t bank) {
  if (!mqtt.connected()) {
    return false;
  }
  const String topic = String(DUAL_BATT_SELECTOR_TOPIC) + "/set/select";
  const char* payload = dualBattHaLabel(bank);
  const bool ok = mqtt.publish(topic.c_str(), payload, false);
  Log.printf("[dual] select -> %s (%s)\n", payload, ok ? "ok" : "fail");
  return ok;
}

static uint8_t dualBattParseBank(const String& msg, bool allowAuto) {
  if (allowAuto &&
      (msg.equalsIgnoreCase("auto") || msg.equalsIgnoreCase("off") || msg == "0" ||
       msg.equalsIgnoreCase("false"))) {
    return 0;
  }
  if (msg == "1" || msg.equalsIgnoreCase("A") || msg.equalsIgnoreCase("Battery A") ||
      msg.equalsIgnoreCase("battery_a") || msg.equalsIgnoreCase("batterya")) {
    return 1;
  }
  if (msg == "2" || msg.equalsIgnoreCase("B") || msg.equalsIgnoreCase("Battery B") ||
      msg.equalsIgnoreCase("battery_b") || msg.equalsIgnoreCase("batteryb")) {
    return 2;
  }
  return 255; // invalid
}

static void dualBattOnSelected(const String& msg) {
  const uint8_t bank = dualBattParseBank(msg, false);
  if (bank == 255) {
    Log.printf("[dual] ignore selected payload '%s'\n", msg.c_str());
    return;
  }
  dualBattReportedSelected = bank;
  if (dualBattState == DualBattState::WaitConfirm && dualBattTarget == bank) {
    dualBattSelectedMatch = true;
    Log.printf("[dual] selector confirmed %s\n", dualBattLabel(bank));
  }
}

static void dualBattBeginSwitch(uint8_t bank) {
  if (bank != 1 && bank != 2) {
    return;
  }
  if (bank == dualBattActive && dualBattState == DualBattState::Idle) {
    Log.printf("[dual] already on %s\n", dualBattLabel(bank));
    return;
  }
  if (dualBattBusy()) {
    Log.println("[dual] switch ignored — already switching");
    return;
  }
  dualBattTarget = bank;
  dualBattSelectedMatch = false;
  dualBattResumeCmdW = lastCmdW;
  dualBattSetState(DualBattState::Standby);
}

// urgent = protection or safety handover (BMS refuse, active bank empty,
// overnight target). Auto-only dwell does not apply to urgent requests.
static bool dualBattRequestBank(uint8_t bank, bool urgent = false) {
  // dualBattEnabled ("Automatic A-first policy") only gates preference switching.
  // Urgent handovers (overnight target, BMS refuse, bank empty/full) still run.
  if (!urgent && !dualBattEnabled && dualBattForce == 0) {
    return false;
  }
  if (bank == dualBattActive || bank == 0) {
    return false;
  }
  const uint32_t now = millis();
  if (dualBattLastSwitchMs != 0 && (now - dualBattLastSwitchMs) < DUAL_BATT_COOLDOWN_MS) {
    Log.printf("[dual] cooldown %lu ms left\n",
               static_cast<unsigned long>(DUAL_BATT_COOLDOWN_MS - (now - dualBattLastSwitchMs)));
    return false;
  }
  const uint32_t dwellMs = uint32_t(dualBattAutoDwellMin) * 60000UL;
  if (!urgent && dualBattForce == 0 && dwellMs != 0 && dualBattLastSwitchMs != 0 &&
      (now - dualBattLastSwitchMs) < dwellMs) {
    Log.printf("[dual] auto dwell %lu s left\n",
               static_cast<unsigned long>((dwellMs - (now - dualBattLastSwitchMs)) / 1000UL));
    return false;
  }
  dualBattBeginSwitch(bank);
  return true;
}

static bool dualBattBankEmpty(uint8_t bank) {
  const uint8_t emptyTh = essDischargeFloorSoc();
  if (bank == 2) {
    return dualBattStickyEmptyB || (dualBattSocBValid && dualBattSocB <= emptyTh);
  }
  return dualBattStickyEmptyA || (dualBattSocAValid && dualBattSocA <= emptyTh);
}

static bool dualBattBankFull(uint8_t bank) {
  if (bank == 2) {
    return dualBattStickyFullB || (dualBattSocBValid && dualBattSocB >= dualBattFullSoc);
  }
  return dualBattStickyFullA || (dualBattSocAValid && dualBattSocA >= dualBattFullSoc);
}

static bool dualBattBankKnown(uint8_t bank) {
  return bank == 2 ? dualBattSocBValid : dualBattSocAValid;
}

// Charge-side "done" for handover: physically full, or at the manual charge
// target. The target check is live and not sticky, so the pack can still be
// charged toward dualBattFullSoc later (daytime ESS).
static bool dualBattBankChargeDone(uint8_t bank) {
  if (dualBattBankFull(bank)) {
    return true;
  }
  if (chargeTargetSoc == 0 || !dualBattBankKnown(bank)) {
    return false;
  }
  const uint8_t soc = (bank == 2) ? dualBattSocB : dualBattSocA;
  return soc >= chargeTargetSoc;
}

static bool dualBattCooldownActive() {
  return dualBattLastSwitchMs != 0 &&
         (millis() - dualBattLastSwitchMs) < uint32_t(DUAL_BATT_COOLDOWN_MS);
}

static void dualBattMarkActiveEmpty() {
  if (dualBattActive == 2) {
    dualBattStickyEmptyB = true;
    dualBattStickyFullB = false;
  } else {
    dualBattStickyEmptyA = true;
    dualBattStickyFullA = false;
  }
  Log.printf("[dual] mark %s empty (sticky)\n", dualBattLabel(dualBattActive));
}

static void dualBattMarkActiveFull() {
  // Record the SOC at the moment of marking. Re-marking an already-full bank
  // keeps the original reference so the drop check stays meaningful.
  uint8_t soc = dualBattFullSoc;
  if (lastSofar.socValid) {
    soc = uint8_t(lastSofar.batterySoc);
  } else if (dualBattActive == 2 ? dualBattSocBValid : dualBattSocAValid) {
    soc = dualBattActive == 2 ? dualBattSocB : dualBattSocA;
  }
  if (dualBattActive == 2) {
    if (!dualBattStickyFullB) {
      dualBattFullMarkSocB = soc;
    }
    dualBattStickyFullB = true;
    dualBattStickyEmptyB = false;
  } else {
    if (!dualBattStickyFullA) {
      dualBattFullMarkSocA = soc;
    }
    dualBattStickyFullA = true;
    dualBattStickyEmptyA = false;
  }
  Log.printf("[dual] mark %s full (sticky) at SOC %u\n", dualBattLabel(dualBattActive), unsigned(soc));
}

// The full flag is only valid while the pack still holds charge. Clear it once a
// bank's known SOC has dropped DUAL_BATT_FULL_CLEAR_DROP below the marked level,
// so a pack that was full and then discharged is not locked out of charging.
static void dualBattExpireStickyFull() {
  if (dualBattStickyFullA && dualBattSocAValid &&
      int(dualBattSocA) <= int(dualBattFullMarkSocA) - int(DUAL_BATT_FULL_CLEAR_DROP)) {
    dualBattStickyFullA = false;
    Log.printf("[dual] A no longer full (SOC %u, marked at %u)\n",
               unsigned(dualBattSocA),
               unsigned(dualBattFullMarkSocA));
  }
  if (dualBattStickyFullB && dualBattSocBValid &&
      int(dualBattSocB) <= int(dualBattFullMarkSocB) - int(DUAL_BATT_FULL_CLEAR_DROP)) {
    dualBattStickyFullB = false;
    Log.printf("[dual] B no longer full (SOC %u, marked at %u)\n",
               unsigned(dualBattSocB),
               unsigned(dualBattFullMarkSocB));
  }
}

static void dualBattClearStickyOnFollow(int16_t cmdW, bool following) {
  if (!following) {
    return;
  }
  if (cmdW > int16_t(essHoldMinW)) {
    if (dualBattActive == 2) {
      dualBattStickyEmptyB = false;
    } else {
      dualBattStickyEmptyA = false;
    }
  } else if (cmdW < -int16_t(essHoldMinW)) {
    if (dualBattActive == 2) {
      dualBattStickyFullB = false;
    } else {
      dualBattStickyFullA = false;
    }
  }
}

static bool dualBattActiveStickyEmpty() {
  return dualBattActive == 2 ? dualBattStickyEmptyB : dualBattStickyEmptyA;
}

static bool dualBattActiveStickyFull() {
  return dualBattActive == 2 ? dualBattStickyFullB : dualBattStickyFullA;
}

static bool dualBattTrySwapOther(bool forEmpty, bool urgent = false) {
  // Forced bank: no handover. Auto with A-first off: urgent handovers still run.
  if (dualBattForce != 0 || dualBattBusy()) {
    return false;
  }
  if (!urgent && !dualBattEnabled) {
    return false;
  }
  const uint8_t other = (dualBattActive == 2) ? 1 : 2;
  if (forEmpty) {
    if (dualBattBankKnown(other) && dualBattBankEmpty(other)) {
      return false;
    }
  } else if (dualBattBankKnown(other) && dualBattBankChargeDone(other)) {
    return false;
  }
  return dualBattRequestBank(other, urgent);
}

static void dualBattEvaluatePolicy() {
  if (!dualBattEnabled || dualBattBusy()) {
    return;
  }

  // Force override: keep requested bank connected.
  if (dualBattForce == 1 || dualBattForce == 2) {
    if (dualBattForce != dualBattActive) {
      dualBattRequestBank(dualBattForce, true); // forced: no Auto dwell
    }
    return;
  }

  // Intent from last ESS/manual command. Near-zero hold does not switch banks.
  // Full protect replaces the charge command with a discharge hold, so a charge
  // it just suppressed still counts as intent — that is when the other bank is
  // needed most.
  const bool chargeBlockedFull =
      essChargeBlockedFullMs != 0 &&
      (millis() - essChargeBlockedFullMs) < uint32_t(DUAL_BATT_FULL_HANDOVER_MS);
  const bool dischargeBlockedLow =
      essDischargeBlockedLowMs != 0 &&
      (millis() - essDischargeBlockedLowMs) < uint32_t(DUAL_BATT_FULL_HANDOVER_MS);
  const bool wantDischargeCmd = lastCmdW > int16_t(essHoldMinW);
  const bool wantCharge = lastCmdW < -int16_t(essHoldMinW) || (chargeBlockedFull && !wantDischargeCmd);
  // Low protect turns the discharge into a -hold charge, so it can look like
  // charge intent. Count the suppressed discharge as discharge intent.
  const bool wantDischarge = wantDischargeCmd || (dischargeBlockedLow && !wantCharge);
  if (!wantDischarge && !wantCharge) {
    return;
  }

  const bool aEmpty = dualBattBankEmpty(1);
  const bool bEmpty = dualBattBankEmpty(2);
  // Charge handover: a bank at the manual charge target counts as done, so
  // charging moves to the other bank instead of stopping at the target.
  const bool aFull = dualBattBankChargeDone(1);
  const bool bFull = dualBattBankChargeDone(2);
  // Leaving an empty bank is urgent (bypasses the Auto dwell). Prefer-A on
  // discharge and charge is unchanged.
  if (wantDischarge) {
    if (dualBattActive == 1) {
      if (aEmpty && !bEmpty) {
        dualBattRequestBank(2, true);
      }
    } else {
      if (!aEmpty) {
        dualBattRequestBank(1, bEmpty);
      }
    }
  } else if (wantCharge) {
    if (dualBattActive == 1) {
      if (aFull && !bFull) {
        dualBattRequestBank(2);
      }
    } else {
      if (!aFull) {
        dualBattRequestBank(1);
      }
    }
  }
}

static void dualBattTick() {
  const uint32_t now = millis();
  dualBattExpireStickyFull();

  switch (dualBattState) {
    case DualBattState::Idle:
      dualBattEvaluatePolicy();
      break;

    case DualBattState::Standby:
      if (sofarStandby()) {
        lastCmdW = 0;
        lastCmdMs = now;
        beginFollowGrace();
        mqttPublish("ess/mode", modeName(SofarMode::Standby));
        dualBattSetState(DualBattState::CommandRelay);
      } else if (now - dualBattStateMs > 5000) {
        Log.println("[dual] standby timeout");
        dualBattSetState(DualBattState::Error);
      }
      break;

    case DualBattState::CommandRelay:
      dualBattSelectedMatch = false;
      if (dualBattPublishSelect(dualBattTarget)) {
        dualBattSetState(DualBattState::WaitConfirm);
      } else if (now - dualBattStateMs > 5000) {
        Log.println("[dual] MQTT select publish failed");
        dualBattSetState(DualBattState::Error);
      }
      break;

    case DualBattState::WaitConfirm:
      if (dualBattSelectedMatch ||
          (dualBattReportedSelected != 0 && dualBattReportedSelected == dualBattTarget)) {
        dualBattSelectedMatch = true;
        dualBattSetState(DualBattState::Settling);
      } else if (now - dualBattStateMs > DUAL_BATT_CONFIRM_TIMEOUT_MS) {
        Log.println("[dual] selector confirm timeout");
        dualBattSetState(DualBattState::Error);
      }
      break;

    case DualBattState::Settling:
      if (now - dualBattStateMs >= DUAL_BATT_SETTLE_MS) {
        sofarInvalidateSoc(lastSofar);
        // Cross-check the first post-switch reading against this bank's last
        // known SOC; a contradicting value needs strict confirmation.
        if (dualBattTarget == 2) {
          sofarSetSocExpectation(dualBattSocBValid, dualBattSocB);
        } else {
          sofarSetSocExpectation(dualBattSocAValid, dualBattSocA);
        }
        sofarPreferSocPoll(true);
        dualBattSetState(DualBattState::SyncSoc);
      }
      break;

    case DualBattState::SyncSoc:
      if (sofarPollSocNow(lastSofar) && lastSofar.socValid) {
        dualBattActive = dualBattTarget;
        dualBattTarget = 0;
        dualBattLastSwitchMs = now;
        sofarPreferSocPoll(false);
        if (dualBattActive == 2) {
          dualBattSocB = uint8_t(lastSofar.batterySoc);
          dualBattSocBValid = true;
        } else {
          dualBattSocA = uint8_t(lastSofar.batterySoc);
          dualBattSocAValid = true;
        }
        lastSofarOk = true;
        dualBattSetState(DualBattState::Idle);
        publishDualBatt();
        bool resumed = true;
        if (dualBattResumeCmdW > 0) {
          resumed = sofarDischarge(uint16_t(dualBattResumeCmdW));
        } else if (dualBattResumeCmdW < 0) {
          resumed = sofarCharge(uint16_t(-dualBattResumeCmdW));
        }
        if (dualBattResumeCmdW != 0 && resumed) {
          lastCmdW = dualBattResumeCmdW;
          lastCmdMs = now;
        }
        dualBattResumeCmdW = 0;
        beginFollowGrace();
        Log.printf("[dual] synced %s SOC=%u\n",
                   dualBattLabel(dualBattActive),
                   unsigned(lastSofar.batterySoc));
      } else if (now - dualBattStateMs > DUAL_BATT_SOC_SYNC_TIMEOUT_MS) {
        Log.println("[dual] SOC sync timeout");
        sofarPreferSocPoll(false);
        // Still adopt the bank — BMS may need longer; live SOC will catch up.
        dualBattActive = dualBattTarget;
        dualBattTarget = 0;
        dualBattLastSwitchMs = now;
        dualBattSetState(DualBattState::Error);
        publishDualBatt();
      }
      break;

    case DualBattState::Error:
      // Auto-recover to idle after cooldown so policy can retry.
      if (now - dualBattStateMs > DUAL_BATT_COOLDOWN_MS) {
        dualBattTarget = 0;
        dualBattSetState(DualBattState::Idle);
        publishDualBatt();
      }
      break;
  }
}
#endif // DUAL_BATT_ENABLE

// Manual charge (ESS off): stop at chargeTargetSoc and go standby. The target is
// a pack-wide (combined) SOC: when the active bank reaches it while the combined
// SOC is still short, hand over to the other bank first.
static void checkChargeTargetSoc() {
  if (essEnabled || chargeTargetSoc == 0) {
    return;
  }
  if (sofarLastMode() != SofarMode::Charge) {
    return;
  }
  if (!lastSofar.socValid) {
    return;
  }
  const float target = float(chargeTargetSoc);
  const float activeSoc = float(lastSofar.batterySoc);
  float effectiveSoc = activeSoc;
  bool stopAtTarget = false;
#if DUAL_BATT_ENABLE
  // A bank switch owns the Sofar state; never standby mid-switch.
  if (dualBattBusy()) {
    return;
  }
  const float combinedSoc = dualBattCombinedSoc();
  if (!isnan(combinedSoc)) {
    effectiveSoc = combinedSoc;
  }
  // Unknown combined (other bank not seen yet) counts as short: the other bank
  // may still take charge.
  const bool combinedShort = isnan(combinedSoc) || combinedSoc < target;
  // Runs regardless of the A-first switch: with it off, the pack would otherwise
  // never reach the target (combined can't get there) and keep charging the
  // active bank into BMS refusal.
  if (combinedShort) {
    // Active bank reached the target (or is physically full) but the pack is
    // still short: hand over to the other bank and keep charging.
    const bool activeAtLimit =
        activeSoc >= target || activeSoc >= float(dualBattFullSoc);
    if (activeAtLimit && dualBattForce == 0) {
      if (dualBattTrySwapOther(false, true)) { // overnight target handover: urgent
        Log.printf("[charge] target SOC %u — bank %s at limit, switching to %s\n",
                   chargeTargetSoc,
                   dualBattLabel(dualBattActive),
                   dualBattLabel(dualBattTarget));
        return;
      }
      if (dualBattCooldownActive()) {
        return; // retry on next poll once the switch cooldown passes
      }
    }
    if (activeSoc < float(dualBattFullSoc)) {
      return; // keep charging the active bank
    }
    // Physically full and no other bank can take charge: stop below.
    stopAtTarget = true;
  }
#endif
  if (!stopAtTarget && effectiveSoc < target) {
    return;
  }

  const uint8_t hit = chargeTargetSoc;
  if (sofarStandby()) {
    lastCmdW = 0;
    lastCmdMs = millis();
    beginFollowGrace();
    chargeTargetSoc = 0;
    publishChargeTargetSoc();
    endManualCharge(); // target reached: stop re-asserting the charge
    Log.printf("[charge] target SOC %u reached (effective %.1f%%) — standby\n",
               hit,
               effectiveSoc);
    mqttPublish("ess/mode", modeName(sofarLastMode()));
  }
}

#if HA_MQTT_DISCOVERY
struct HaSensor {
  const char* objectId;
  const char* name;
  const char* valueTemplate;
  const char* unit;         // nullptr if none
  const char* deviceClass;  // nullptr if none
  const char* stateClass;   // nullptr if none
};

static const HaSensor kHaSensors[] = {
    {"grid_power", "Grid Power", "{{ value_json.grid_power_w }}", "W", "power", "measurement"},
    {"grid_voltage", "Grid Voltage", "{{ value_json.grid_voltage_v }}", "V", "voltage", "measurement"},
    {"grid_frequency", "Grid Frequency", "{{ value_json.grid_frequency_hz }}", "Hz", "frequency", "measurement"},
    {"charge_discharge_power",
     "Charge/Discharge Power",
     "{{ value_json.charge_discharge_power_w }}",
     "W",
     "power",
     "measurement"},
    {"battery_dc_power", "Battery DC Power", "{{ value_json.battery_dc_power_w }}", "W", "power", "measurement"},
    {"battery_voltage", "Battery Voltage", "{{ value_json.battery_voltage_v }}", "V", "voltage", "measurement"},
    {"battery_current", "Battery Current", "{{ value_json.battery_current_a }}", "A", "current", "measurement"},
    {"ess_command", "ESS Command", "{{ value_json.ess_command_w }}", "W", "power", "measurement"},
    {"follow_error", "Follow Error", "{{ value_json.follow_error_w }}", "W", "power", "measurement"},
    {"follow_recovery", "Follow Recovery", "{{ value_json.follow_recovery }}", nullptr, nullptr, nullptr},
    {"battery_soc", "Battery SOC", "{{ value_json.battery_soc }}", "%", "battery", "measurement"},
    {"battery_a_soc", "Battery A SOC", "{{ value_json.battery_a_soc }}", "%", "battery", "measurement"},
    {"battery_b_soc", "Battery B SOC", "{{ value_json.battery_b_soc }}", "%", "battery", "measurement"},
    {"battery_combined_soc", "Battery Combined SOC", "{{ value_json.battery_combined_soc }}", "%", "battery", "measurement"},
    {"battery_active", "Battery Active", "{{ value_json.battery_active }}", nullptr, nullptr, nullptr},
    {"battery_dual_state", "Battery Dual State", "{{ value_json.battery_dual_state }}", nullptr, nullptr, nullptr},
    {"pv1_power", "PV1 Power", "{{ value_json.pv1_power_w }}", "W", "power", "measurement"},
    {"pv1_voltage", "PV1 Voltage", "{{ value_json.pv1_voltage_v }}", "V", "voltage", "measurement"},
    {"pv1_current", "PV1 Current", "{{ value_json.pv1_current_a }}", "A", "current", "measurement"},
    {"pv2_power", "PV2 Power", "{{ value_json.pv2_power_w }}", "W", "power", "measurement"},
    {"pv2_voltage", "PV2 Voltage", "{{ value_json.pv2_voltage_v }}", "V", "voltage", "measurement"},
    {"pv2_current", "PV2 Current", "{{ value_json.pv2_current_a }}", "A", "current", "measurement"},
    {"pv_total", "Sofar PV Total", "{{ value_json.pv_total_w }}", "W", "power", "measurement"},
    {"pv_today", "Sofar PV Today", "{{ value_json.pv_today_kwh }}", "kWh", "energy", "total_increasing"},
    {"energy_import", "Energy Import", "{{ value_json.energy_import_wh }}", "Wh", "energy", "total_increasing"},
    {"energy_export", "Energy Export", "{{ value_json.energy_export_wh }}", "Wh", "energy", "total_increasing"},
    {"sofar_mode", "Sofar Mode", "{{ value_json.sofar_mode }}", nullptr, nullptr, nullptr},
    {"run_state", "Run State", "{{ value_json.run_state }}", nullptr, nullptr, nullptr},
    {"run_state_name", "Run State Name", "{{ value_json.run_state_name }}", nullptr, nullptr, nullptr},
    {"fault_message", "Sofar Fault", "{{ value_json.fault_message }}", nullptr, nullptr, nullptr},
    {"alert_message", "Sofar Alert", "{{ value_json.alert_message }}", nullptr, nullptr, nullptr},
    {"batt_fault_message", "Battery Fault", "{{ value_json.batt_fault_message }}", nullptr, nullptr, nullptr},
    {"sofar_last_error", "Sofar Last Error", "{{ value_json.sofar_last_error }}", nullptr, nullptr, nullptr},
    {"soc_rejects", "SOC Rejects", "{{ value_json.soc_rejects }}", nullptr, nullptr, "total_increasing"},
};

static void haFillDevice(JsonObject device) {
  device["identifiers"][0] = DEVICE_NAME;
  device["name"] = HA_DEVICE_NAME;
  device["manufacturer"] = "Sofar";
  device["model"] = "ESS + JSY-MK-194G";
  device["sw_version"] = "sofar_ess";
}

static bool haPublishConfig(const char* component, const char* objectId, const String& payload) {
  char topic[160];
  snprintf(topic, sizeof(topic), "%s/%s/%s/%s/config", HA_DISCOVERY_PREFIX, component, DEVICE_NAME, objectId);
  if (!mqtt.publish(topic, payload.c_str(), true)) {
    Log.printf("[ha] discovery fail %s/%s (len=%u)\n",
               component,
               objectId,
               static_cast<unsigned>(payload.length()));
    return false;
  }
  mqtt.loop();
  delay(10);
  return true;
}

static void publishHaDiscovery() {
  if (!mqtt.connected()) {
    return;
  }

  const String stateTopic = String(DEVICE_NAME) + "/state";
  const String availTopic = String(DEVICE_NAME) + "/status";
  const String setEssTopic = String(DEVICE_NAME) + "/set/ess";
  const String setChargeOnlyTopic = String(DEVICE_NAME) + "/set/charge_only";
  const String setStandbyTopic = String(DEVICE_NAME) + "/set/standby";
  const String setAutoTopic = String(DEVICE_NAME) + "/set/auto";

  for (const HaSensor& sensor : kHaSensors) {
    JsonDocument doc;
    doc["name"] = sensor.name;
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_%s", DEVICE_NAME, sensor.objectId);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = stateTopic;
    doc["value_template"] = sensor.valueTemplate;
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    if (sensor.unit) {
      doc["unit_of_measurement"] = sensor.unit;
    }
    if (sensor.deviceClass) {
      doc["device_class"] = sensor.deviceClass;
    }
    if (sensor.stateClass) {
      doc["state_class"] = sensor.stateClass;
    }
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("sensor", sensor.objectId, payload);
  }

  // Sofar RS485 health (true = at least one register read succeeded)
  {
    JsonDocument doc;
    doc["name"] = "Sofar OK";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_sofar_ok", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = stateTopic;
    doc["value_template"] = "{{ 'true' if value_json.sofar_ok else 'false' }}";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["device_class"] = "connectivity";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("binary_sensor", "sofar_ok", payload);
  }

  // Inverter in fault / permanent fault run-state
  {
    JsonDocument doc;
    doc["name"] = "Sofar Fault Active";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_sofar_fault_active", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = stateTopic;
    doc["value_template"] = "{{ 'true' if value_json.fault_active else 'false' }}";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["device_class"] = "problem";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("binary_sensor", "sofar_fault_active", payload);
  }

  {
    JsonDocument doc;
    doc["name"] = "ESS Follow OK";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_follow_ok", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = stateTopic;
    doc["value_template"] = "{{ 'true' if value_json.follow_ok else 'false' }}";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("binary_sensor", "follow_ok", payload);
  }

  {
    JsonDocument doc;
    doc["name"] = "ESS Follow Grace";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_follow_grace", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = stateTopic;
    doc["value_template"] = "{{ 'true' if value_json.follow_grace else 'false' }}";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("binary_sensor", "follow_grace", payload);
  }

  // ESS enable switch (state from dedicated topic — string true/false)
  {
    JsonDocument doc;
    doc["name"] = "ESS Enabled";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_ess_enabled", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/ess/enabled";
    doc["command_topic"] = setEssTopic;
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("switch", "ess_enabled", payload);
  }

  // EPS switch: on = enable backup output when grid is lost; off = disable
  {
    JsonDocument doc;
    doc["name"] = "EPS Enabled";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_eps_enabled", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/ess/eps_enabled";
    doc["command_topic"] = String(DEVICE_NAME) + "/set/eps";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["icon"] = "mdi:power-plug-battery";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("switch", "eps_enabled", payload);
  }

  // Grid lost, from inverter grid voltage (see epsUpdateGridState)
  {
    JsonDocument doc;
    doc["name"] = "Grid Down";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_grid_down", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = stateTopic;
    doc["value_template"] = "{{ 'true' if value_json.grid_down else 'false' }}";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["device_class"] = "problem";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("binary_sensor", "grid_down", payload);
  }

  // Charge-only: soak export, never discharge
  {
    JsonDocument doc;
    doc["name"] = "ESS Charge Only";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_ess_charge_only", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/ess/charge_only";
    doc["command_topic"] = setChargeOnlyTopic;
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("switch", "ess_charge_only", payload);
  }

  // SOC protect: auto charge-only below low threshold, resume at low + hyst
  {
    JsonDocument doc;
    doc["name"] = "ESS SOC Protect";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_ess_soc_protect", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/ess/soc_protect/enabled";
    doc["command_topic"] = String(DEVICE_NAME) + "/set/soc_protect";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("switch", "ess_soc_protect", payload);
  }

  {
    JsonDocument doc;
    doc["name"] = "ESS SOC Protect Active";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_ess_soc_protect_active", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/ess/soc_protect/active";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("binary_sensor", "ess_soc_protect_active", payload);
  }

  {
    JsonDocument doc;
    doc["name"] = "ESS SOC Full Active";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_ess_soc_full_active", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/ess/soc_protect/full_active";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("binary_sensor", "ess_soc_full_active", payload);
  }

  // PI gain numbers (MQTT set/kp|ki)
  struct {
    const char* objectId;
    const char* name;
    const char* stateSuffix;
    const char* commandSuffix;
    float minV;
    float maxV;
    float step;
  } numbers[] = {
      {"kp", "ESS Kp", "ess/kp", "set/kp", 0.0f, 5.0f, 0.05f},
      {"ki", "ESS Ki", "ess/ki", "set/ki", 0.0f, 5.0f, 0.05f},
      {"deadband", "ESS Deadband", "ess/deadband", "set/deadband", 0.0f, 500.0f, 5.0f},
      {"min_delta", "ESS Min Delta", "ess/min_delta", "set/min_delta", 0.0f, 500.0f, 5.0f},
      {"hold_min", "ESS Hold Min", "ess/hold_min", "set/hold_min", 1.0f, float(MAX_POWER_W), 5.0f},
      // Max charge power for manual charge (Charge button / set/charge true). The
      // pack may take less (BMS limit, taper); this is a ceiling, not a target.
      {"max_charge_w", "Max Charge Power", "charge/max_w", "set/max_charge_w", 200.0f, float(MAX_POWER_W), 50.0f},
      {"soc_protect_low", "ESS SOC Protect Low", "ess/soc_protect/low", "set/soc_protect_low", 1.0f, 100.0f, 1.0f},
      {"soc_protect_hyst", "ESS SOC Protect Hysteresis", "ess/soc_protect/hyst", "set/soc_protect_hyst", 0.0f, 30.0f, 1.0f},
      {"soc_protect_high", "ESS SOC Protect High", "ess/soc_protect/high", "set/soc_protect_high", 50.0f, 100.0f, 1.0f},
      {"soc_protect_high_hyst",
       "ESS SOC Protect High Hysteresis",
       "ess/soc_protect/high_hyst",
       "set/soc_protect_high_hyst",
       0.0f,
       30.0f,
       1.0f},
#if DUAL_BATT_ENABLE
      {"dual_batt_empty", "Battery Empty SOC", "battery/empty", "set/dual_batt_empty", 0.0f, 100.0f, 1.0f},
      {"dual_batt_full", "Battery Full SOC", "battery/full", "set/dual_batt_full", 0.0f, 100.0f, 1.0f},
      {"dual_batt_dwell", "Battery Auto Dwell", "battery/auto_dwell_min", "set/dual_batt_dwell", 0.0f, 240.0f, 1.0f},
#endif
  };
  for (const auto& n : numbers) {
    JsonDocument doc;
    doc["name"] = n.name;
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_%s", DEVICE_NAME, n.objectId);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/" + n.stateSuffix;
    doc["command_topic"] = String(DEVICE_NAME) + "/" + n.commandSuffix;
    doc["min"] = n.minV;
    doc["max"] = n.maxV;
    doc["step"] = n.step;
    doc["mode"] = "box";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("number", n.objectId, payload);
  }

#if BATT_PARAM_ENABLE
  for (uint8_t i = 0; i < sofarBattParamCount(); i++) {
    const SofarBattParamMeta* m = sofarBattParamMeta(i);
    if (!m || !m->writable) {
      continue;
    }
    JsonDocument doc;
    doc["name"] = m->haName;
    char objectId[64];
    snprintf(objectId, sizeof(objectId), "batt_param_%s", m->key);
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_%s", DEVICE_NAME, objectId);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/batt_param/" + m->key;
    doc["command_topic"] = String(DEVICE_NAME) + "/set/batt_param/" + m->key;
    doc["min"] = float(m->rawMin) * m->scale;
    doc["max"] = float(m->rawMax) * m->scale;
    doc["step"] = m->step;
    if (m->unit[0]) {
      doc["unit_of_measurement"] = m->unit;
    }
    doc["mode"] = "box";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("number", objectId, payload);
  }

  {
    JsonDocument doc;
    doc["name"] = "Battery Params Valid";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_batt_param_valid", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/batt_param/valid";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("binary_sensor", "batt_param_valid", payload);
  }

  {
    JsonDocument doc;
    doc["name"] = "Battery Params Poll Diag";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_batt_param_poll_diag", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/batt_param/poll_diag";
    doc["icon"] = "mdi:bug-outline";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("sensor", "batt_param_poll_diag", payload);
  }
#endif

  // Charge target SOC (manual charge stops at this %)
  {
    JsonDocument doc;
    doc["name"] = "Charge Target SOC";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_charge_target_soc", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/charge/target_soc";
    doc["command_topic"] = String(DEVICE_NAME) + "/set/charge_target_soc";
    doc["min"] = 0;
    doc["max"] = 100;
    doc["step"] = 1;
    doc["unit_of_measurement"] = "%";
    doc["mode"] = "box";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("number", "charge_target_soc", payload);
  }

  // Manual Sofar buttons (disable ESS when pressed — same as MQTT set/)
  {
    const String setResetITopic = String(DEVICE_NAME) + "/set/reset_i";
    const char* standbyCmd = setStandbyTopic.c_str();
    const char* autoCmd = setAutoTopic.c_str();
    const char* resetICmd = setResetITopic.c_str();
    const String setChargeTopic = String(DEVICE_NAME) + "/set/charge";
    const char* chargeCmd = setChargeTopic.c_str();
    struct {
      const char* objectId;
      const char* name;
      const char* commandTopic;
    } buttons[] = {
        {"charge", "Charge", chargeCmd}, // starts a manual charge at the Max Charge Power
        {"standby", "Standby", standbyCmd},
        {"auto", "Auto", autoCmd},
        {"reset_i", "Reset Integral", resetICmd},
    };
    for (const auto& btn : buttons) {
      JsonDocument doc;
      doc["name"] = btn.name;
      char uniqueId[96];
      snprintf(uniqueId, sizeof(uniqueId), "%s_%s", DEVICE_NAME, btn.objectId);
      doc["unique_id"] = uniqueId;
      doc["command_topic"] = btn.commandTopic;
      doc["payload_press"] = "true";
      doc["availability_topic"] = availTopic;
      doc["payload_available"] = "online";
      doc["payload_not_available"] = "offline";
      haFillDevice(doc["device"].to<JsonObject>());

      String payload;
      serializeJson(doc, payload);
      haPublishConfig("button", btn.objectId, payload);
    }
  }

#if DUAL_BATT_ENABLE
  // Dual-battery auto enable
  {
    JsonDocument doc;
    doc["name"] = "Dual Battery Auto";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_dual_batt", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/battery/dual_enabled";
    doc["command_topic"] = String(DEVICE_NAME) + "/set/dual_batt";
    doc["payload_on"] = "true";
    doc["payload_off"] = "false";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("switch", "dual_batt", payload);
  }

  // Force bank select: Auto / Battery A / Battery B
  {
    JsonDocument doc;
    doc["name"] = "Battery Bank";
    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_battery_bank", DEVICE_NAME);
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = String(DEVICE_NAME) + "/battery/force_select";
    doc["command_topic"] = String(DEVICE_NAME) + "/set/battery";
    doc["options"][0] = "Auto";
    doc["options"][1] = "Battery A";
    doc["options"][2] = "Battery B";
    doc["availability_topic"] = availTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    haFillDevice(doc["device"].to<JsonObject>());

    String payload;
    serializeJson(doc, payload);
    haPublishConfig("select", "battery_bank", payload);
  }
#endif

  haDiscoverySent = true;
  Log.println("[ha] MQTT discovery published");
}
#endif // HA_MQTT_DISCOVERY

static bool followReadActual(int16_t& actual) {
  const uint32_t now = millis();
  if (lastSofar.batteryDcValid && lastSofar.batteryDcMs != 0 &&
      (now - lastSofar.batteryDcMs) <= uint32_t(FOLLOW_ACTUAL_STALE_MS)) {
    actual = lastSofar.batteryDcPowerW;
    return true;
  }
  if (lastSofar.chargeDischargeMs != 0 &&
      (now - lastSofar.chargeDischargeMs) <= uint32_t(FOLLOW_ACTUAL_STALE_MS)) {
    actual = lastSofar.chargeDischargePowerW;
    return true;
  }
  return false;
}

static bool followIsRamping(int16_t cmd, int16_t actual) {
  if ((cmd > 0 && actual <= 0) || (cmd < 0 && actual >= 0)) {
    return false;
  }
  return abs(int(actual)) >= abs(int(followPrevActualW)) + FOLLOW_RAMP_STEP_W;
}

static bool followNotFollowing(int16_t cmd, int16_t actual) {
  const int absCmd = abs(int(cmd));
  int thresh = int(FOLLOW_RATIO * float(absCmd));
  if (thresh < FOLLOW_ACTUAL_FLOOR_W) {
    thresh = FOLLOW_ACTUAL_FLOOR_W;
  }
  // A pack close to full takes less than commanded but keeps charging to a
  // higher SOC. Only near-zero acceptance means it can no longer take charge.
  if (cmd < 0 && thresh > FOLLOW_TAPER_MIN_W) {
    thresh = FOLLOW_TAPER_MIN_W;
  }
  const bool sameDir = (cmd > 0 && actual > 0) || (cmd < 0 && actual < 0);
  if (!sameDir) {
    return true;
  }
  if (abs(int(actual)) < thresh) {
    return !followIsRamping(cmd, actual);
  }
  return false;
}

static bool followRunStateRamping() {
  if (!lastSofarOk) {
    return false;
  }
  const uint16_t rs = lastSofar.runState;
  return rs == 0 || rs == 1 || rs == 3;
}

static bool followRecoverAllowed() {
  const uint32_t now = millis();
  if (followRecoverLastMs != 0 &&
      (now - followRecoverLastMs) < uint32_t(FOLLOW_RECOVER_COOLDOWN_MS)) {
    Log.printf("[follow] recover cooldown %lu ms left\n",
               static_cast<unsigned long>(FOLLOW_RECOVER_COOLDOWN_MS -
                                          (now - followRecoverLastMs)));
    return false;
  }
  if (followRecoverWindowStartMs == 0 ||
      (now - followRecoverWindowStartMs) >= uint32_t(FOLLOW_RECOVER_WINDOW_MS)) {
    followRecoverWindowStartMs = now;
    followRecoverWindowCount = 0;
  }
  if (followRecoverWindowCount >= FOLLOW_RECOVER_MAX) {
    Log.println("[follow] recover cap reached");
    return false;
  }
  return true;
}

static void followStartStandbyRecover() {
  if (!essEnabled || followRecoverState != FollowRecoverState::Idle) {
    return;
  }
#if DUAL_BATT_ENABLE
  if (dualBattBusy()) {
    return;
  }
#endif
  if (!followRecoverAllowed()) {
    return;
  }
  followRecoverWindowCount++;
  followRecoverLastMs = millis();
  followLastRecovery = FollowRecovery::StandbyReset;
  resetEssIntegral();
  lastCmdW = 0;
  followFailSinceMs = 0;
  followRecoverState = FollowRecoverState::Standby;
  followRecoverStateMs = millis();
  Log.println("[follow] standby recover");
  mqttPublish("ess/follow_recovery", followRecoveryName(followLastRecovery));
}

static void followHandleLimit(bool empty) {
  followFailSinceMs = 0;
#if DUAL_BATT_ENABLE
  if (empty) {
    dualBattMarkActiveEmpty();
    followLastRecovery = FollowRecovery::SwapEmpty;
  } else {
    dualBattMarkActiveFull();
    followLastRecovery = FollowRecovery::SwapFull;
  }
  mqttPublish("ess/follow_recovery", followRecoveryName(followLastRecovery));
  if (dualBattTrySwapOther(empty, true)) { // BMS refuse: urgent
    Log.printf("[follow] swap for BMS %s\n", empty ? "empty" : "full");
    return;
  }
  Log.printf("[follow] BMS %s — no other bank to swap\n", empty ? "empty" : "full");
#else
  (void)empty;
#endif
}

static void followRecoverTick() {
  const uint32_t now = millis();
  switch (followRecoverState) {
    case FollowRecoverState::Idle:
      break;
    case FollowRecoverState::Standby:
      if (sofarStandby()) {
        lastCmdW = 0;
        lastCmdMs = now;
        beginFollowGrace();
        mqttPublish("ess/mode", modeName(SofarMode::Standby));
        followRecoverState = FollowRecoverState::Hold;
        followRecoverStateMs = now;
      } else if (now - followRecoverStateMs > 5000) {
        Log.println("[follow] recover standby timeout");
        followRecoverState = FollowRecoverState::Idle;
      }
      break;
    case FollowRecoverState::Hold:
      if (now - followRecoverStateMs >= uint32_t(FOLLOW_RECOVER_HOLD_MS)) {
        followRecoverState = FollowRecoverState::Idle;
        beginFollowGrace();
        Log.println("[follow] recover resume ESS");
      }
      break;
  }
}

static void followTick() {
  const uint32_t now = millis();
  followGraceActive = followGraceUntilMs != 0 && int32_t(now - followGraceUntilMs) < 0;
  if (!followGraceActive) {
    followGraceUntilMs = 0;
  }

#if DUAL_BATT_ENABLE
  if (dualBattBusy()) {
    followOk = true;
    followErrorW = 0;
    followFailSinceMs = 0;
    return;
  }
#endif
  if (followRecoverState != FollowRecoverState::Idle) {
    followOk = true;
    followErrorW = 0;
    return;
  }

  int16_t actual = 0;
  followHaveActual = followReadActual(actual);
  if (followHaveActual) {
    followActualW = actual;
  }

  const int16_t cmd = lastCmdW;
  const bool wantDischarge = cmd > int16_t(essHoldMinW);
  const bool wantCharge = cmd < -int16_t(essHoldMinW);
  const bool cmdLarge = abs(int(cmd)) >= FOLLOW_MIN_CMD_W;

  if (followGraceActive) {
    followOk = true;
    followErrorW = 0;
    followFailSinceMs = 0;
    if (followHaveActual) {
      followPrevActualW = actual;
    }
    return;
  }

  // Manual charge session: the command is only a max power. A low actual power
  // (BMS limit, taper) is not a fault, and the prohibit bit is sticky between
  // passive replies, so only act on it after it persisted for FOLLOW_CONFIRM_MS.
  const bool manualSession = manualChargeW != 0 && wantCharge;
  if (manualSession) {
    if (!sofarChargeProhibited()) {
      followProhibitSinceMs = 0;
    } else if (followProhibitSinceMs == 0) {
      followProhibitSinceMs = now;
    }
    if (!sofarChargeProhibited() ||
        (now - followProhibitSinceMs) < uint32_t(FOLLOW_CONFIRM_MS)) {
      followOk = true;
      followFailSinceMs = 0;
      if (followHaveActual) {
        followErrorW = int16_t(int(cmd) - int(actual)); // informational only
#if DUAL_BATT_ENABLE
        dualBattClearStickyOnFollow(cmd, actual < -int16_t(essHoldMinW));
#endif
        followPrevActualW = actual;
      } else {
        followErrorW = 0;
      }
      return;
    }
    // Prohibited for the full confirm window: fall through to the limit handling.
  } else {
    followProhibitSinceMs = 0;
  }

  if (wantCharge && sofarChargeProhibited()) {
#if DUAL_BATT_ENABLE
    if (!dualBattActiveStickyFull()) {
      followHandleLimit(false);
    } else {
      dualBattTrySwapOther(false, true); // already marked full — keep retrying handover (urgent)
    }
#else
    followHandleLimit(false);
#endif
    if (followHaveActual) {
      followPrevActualW = actual;
    }
    return;
  }
  if (wantDischarge && sofarDischargeProhibited()) {
#if DUAL_BATT_ENABLE
    if (!dualBattActiveStickyEmpty()) {
      followHandleLimit(true);
    } else {
      dualBattTrySwapOther(true, true); // already marked empty — keep retrying handover (urgent)
    }
#else
    followHandleLimit(true);
#endif
    if (followHaveActual) {
      followPrevActualW = actual;
    }
    return;
  }

  if (!cmdLarge || followRunStateRamping() || !lastSofarOk || !followHaveActual) {
    followOk = true;
    followErrorW = 0;
    followFailSinceMs = 0;
    if (followHaveActual) {
      followPrevActualW = actual;
    }
    return;
  }

  followErrorW = int16_t(int(cmd) - int(actual));
  const bool notFollow = followNotFollowing(cmd, actual);
  followOk = !notFollow;

  if (!notFollow) {
    followFailSinceMs = 0;
#if DUAL_BATT_ENABLE
    dualBattClearStickyOnFollow(cmd, true);
#endif
    followPrevActualW = actual;
    return;
  }

  const bool socOk = lastSofar.socValid;
  const uint8_t soc = socOk ? uint8_t(lastSofar.batterySoc) : 0;

  if (followFailSinceMs == 0) {
    followFailSinceMs = now;
  }

  followPrevActualW = actual;

  if ((now - followFailSinceMs) < uint32_t(FOLLOW_CONFIRM_MS)) {
    return;
  }

  followFailSinceMs = 0;
  Log.printf("[follow] mismatch cmd=%d actual=%d soc=%u\n",
             int(cmd),
             int(actual),
             unsigned(socOk ? soc : 255));

  if (wantCharge && (sofarChargeProhibited() || (socOk && soc >= FOLLOW_FULL_SOC))) {
    followHandleLimit(false);
    return;
  }
  if (wantDischarge &&
      (sofarDischargeProhibited() || (socOk && soc <= essDischargeFloorSoc()))) {
    followHandleLimit(true);
    return;
  }

  followStartStandbyRecover();
}

// Manual charge session: manualChargeW is a max charge power, so the pack taking
// less is fine. Keep the command alive: re-send periodically (this also refreshes
// the inverter passive status bits) and after any unplanned standby, for example
// a bank-switch resume that failed once. ESS mode does this itself every cycle.
static void manualChargeTick() {
  if (manualChargeW == 0) {
    return;
  }
  if (essEnabled) {
    endManualCharge();
    return;
  }
  if (followRecoverState != FollowRecoverState::Idle) {
    return;
  }
#if DUAL_BATT_ENABLE
  if (dualBattBusy()) {
    return; // bank switch owns Sofar commands and resumes the charge itself
  }
#endif
  const uint32_t now = millis();
  const bool modeOk =
      sofarLastMode() == SofarMode::Charge && sofarLastSetpointW() == manualChargeW;
  const bool due =
      manualChargeSentMs == 0 ||
      (now - manualChargeSentMs) >= uint32_t(MANUAL_CHARGE_REFRESH_MS);
  if (modeOk && !due) {
    return;
  }
  if (manualChargeTryMs != 0 && (now - manualChargeTryMs) < 2000) {
    return; // do not hammer RS485 when sends fail
  }
  manualChargeTryMs = now;
  if (sofarCharge(manualChargeW)) {
    manualChargeSentMs = now;
    lastCmdW = -int16_t(manualChargeW);
    lastCmdMs = now;
    if (!modeOk) {
      beginFollowGrace();
      Log.printf("[charge] manual charge re-asserted at %uW max\n", manualChargeW);
      mqttPublish("ess/mode", modeName(sofarLastMode()));
    }
  } else {
    Log.println("[charge] manual charge re-send failed — will retry");
  }
}

static void applyEss(float gridPowerW) {
  lastGridPowerW = gridPowerW;
  if (followRecoverState != FollowRecoverState::Idle) {
    return;
  }
#if DUAL_BATT_ENABLE
  if (dualBattBusy()) {
    return; // bank switch owns Sofar commands
  }
#endif
  updateEssSocProtect();

  // PI on grid residual: drive P → 0.
  // e = P (+import → more discharge / +u; −export → more charge / −u)
  // Use measured dt so status/heartbeat stalls don't violate fixed-Ts integration.
  static uint32_t lastPiMs = 0;
  const uint32_t nowPi = millis();
  float Ts = float(ESS_LOOP_INTERVAL_MS) / 1000.0f;
  if (lastPiMs != 0) {
    Ts = float(nowPi - lastPiMs) / 1000.0f;
  }
  if (Ts < 0.05f) {
    Ts = 0.05f;
  } else if (Ts > 2.0f) {
    Ts = 2.0f;
  }
  lastPiMs = nowPi;

  const float e = gridPowerW;
  const float maxW = float(MAX_POWER_W);
  const bool chargeOnly = essEffectiveChargeOnly();
  const bool noCharge = essNoCharge();
  // Charge-only: allow charge (−) and hold, never discharge (+).
  // Full / charge-rejected: allow discharge (+) only, never charge (−).
  const float uMax = chargeOnly ? 0.0f : maxW;
  const float uMin = noCharge ? 0.0f : -maxW;

  float uProbe = essKp * e + essKi * essIntegral;
  const bool saturated = (uProbe > uMax) || (uProbe < uMin);

  // Integrate only outside deadband and when not pushing further into saturation.
  if (fabsf(e) >= essDeadbandW && !saturated) {
    essIntegral += e * Ts;
    const float iLim = (essKi > 1e-6f) ? (maxW / essKi) : maxW;
    const float iMax = chargeOnly ? 0.0f : iLim;
    const float iMin = noCharge ? 0.0f : -iLim;
    if (essIntegral > iMax) {
      essIntegral = iMax;
    } else if (essIntegral < iMin) {
      essIntegral = iMin;
    }
  }

  float u = essKp * e + essKi * essIntegral;
  if (noCharge && u < -essDeadbandW) {
    // Export the pack could soak if it were not full — dual-battery handover
    // reads this as charge intent.
    essChargeBlockedFullMs = nowPi;
  }
  // Low-SOC protect cancelled a real discharge: the bank policy reads this as
  // discharge intent so an empty connected bank can hand over. User charge-only
  // is a mode choice, not a bank fault, so it does not count.
  if (essSocProtectActive && !essChargeOnlyUser && u > essDeadbandW && uMax <= 0.0f) {
    essDischargeBlockedLowMs = nowPi;
  }
  if (u > uMax) {
    u = uMax;
  } else if (u < uMin) {
    u = uMin;
  }

  int16_t targetW = 0;
  if (u > essDeadbandW) {
    targetW = int16_t(u);
  } else if (u < -essDeadbandW) {
    targetW = int16_t(u);
  } else if (fabsf(e) < essDeadbandW) {
    // Flat grid and small command → hold; bleed I so we don't stick forever.
    targetW = 0;
    essIntegral *= 0.9f;
    if (fabsf(essIntegral) < 1.0f) {
      essIntegral = 0.0f;
    }
  } else {
    // e still large but u in deadband — hold off commanding tiny watts
    targetW = 0;
  }

  // Hard clamps.
  if (chargeOnly && targetW > 0) {
    targetW = 0;
  }
  if (noCharge && targetW < 0) {
    targetW = 0;
  }

  // Near-zero: hold min charge/discharge instead of standby (Sofar relay chatter).
  if (targetW == 0) {
    const int16_t hold = int16_t(essHoldMinW);
    if (noCharge) {
      targetW = hold; // discharge hold when full / charge rejected
    } else if (chargeOnly || lastCmdW < 0 || sofarLastMode() == SofarMode::Charge) {
      targetW = -hold;
    } else if (lastCmdW > 0 || sofarLastMode() == SofarMode::Discharge) {
      targetW = hold;
    } else {
      targetW = -hold; // default: charge hold
    }
  }

  const uint32_t now = millis();
  const bool changed = fabsf(float(targetW) - float(lastCmdW)) >= essMinDeltaW;
  const bool refresh = (now - lastCmdMs) >= ESS_REFRESH_MS;
  if (!changed && !refresh && lastCmdMs != 0) {
    return;
  }

  bool ok = false;
  if (targetW > 0) {
    ok = sofarDischarge(uint16_t(targetW));
  } else {
    ok = sofarCharge(uint16_t(-targetW));
  }

  if (ok) {
    lastCmdW = targetW;
    lastCmdMs = now;
  }

  Log.printf("[ess] grid=%.1fW u=%.1f I=%.1f dt=%.2f kp=%.2f ki=%.2f cmd=%d (%s%s%s%s) ok=%d\n",
             gridPowerW,
             u,
             essIntegral,
             Ts,
             essKp,
             essKi,
             int(targetW),
             modeName(sofarLastMode()),
             chargeOnly ? " charge_only" : "",
             essSocProtectActive ? " soc_low" : "",
             essSocFullActive ? " soc_full" : "",
             int(ok));
  mqttPublish("ess/grid_power", String(gridPowerW, 1));
  mqttPublish("ess/command_w", String(targetW));
  mqttPublish("ess/mode", modeName(sofarLastMode()));
  mqttPublish("ess/integral", String(essIntegral, 1));
  if (lastSofar.batteryDcValid) {
    mqttPublish("ess/battery_dc_power", String(lastSofar.batteryDcPowerW));
  }
  mqttPublish("ess/charge_discharge_power", String(lastSofar.chargeDischargePowerW));
}

// --- EPS (backup output, Sofar function 0x41) ---------------------------------

static void epsPublishState() {
  mqttPublish("ess/eps_enabled", epsEnabled ? "true" : "false", true);
}

// Send 0x41 and remember what the inverter was last told.
static bool epsSendFrame(bool enable) {
  epsApplied = enable ? EpsApplied::Enable : EpsApplied::Disable;
  epsLastSendMs = millis();
  return sofarSetEps(enable);
}

// Grid state from the inverter's own grid A voltage (0x0206). Lost after
// EPS_GRID_DEBOUNCE consecutive reads below EPS_GRID_LOST_V; back after the same
// count above EPS_GRID_RETURN_V. Stale or missing readings leave the state unchanged.
static void epsUpdateGridState() {
  if (!lastSofar.gridVoltageValid || millis() - lastSofar.gridVoltageMs > EPS_GRID_STALE_MS) {
    gridVoltageV = NAN;
    return;
  }
  const float v = lastSofar.gridVoltageV;
  if (lastSofar.gridVoltageMs == lastGridSampleMs) {
    return; // no new reading since the last ESS cycle
  }
  lastGridSampleMs = lastSofar.gridVoltageMs;
  gridVoltageV = v;

  if (!gridDown) {
    gridHighCount = 0;
    gridLowCount = (v < EPS_GRID_LOST_V) ? uint8_t(gridLowCount + 1) : 0;
    if (gridLowCount >= EPS_GRID_DEBOUNCE) {
      gridDown = true;
      gridLowCount = 0;
      epsApplied = EpsApplied::Unknown; // new outage: enable right away
      Log.printf("[eps] grid lost (inverter %.1f V)\n", v);
    }
  } else {
    gridLowCount = 0;
    gridHighCount = (v > EPS_GRID_RETURN_V) ? uint8_t(gridHighCount + 1) : 0;
    if (gridHighCount >= EPS_GRID_DEBOUNCE) {
      gridDown = false;
      gridHighCount = 0;
      Log.printf("[eps] grid back (inverter %.1f V)\n", v);
    }
  }
}

// Every ESS cycle. Switch off → send disable. Switch on + grid lost → send enable,
// re-sent every EPS_RESEND_INTERVAL_MS while the outage lasts.
static void epsTick() {
  epsUpdateGridState();

  if (!epsEnabled) {
    if (epsApplied != EpsApplied::Disable) {
      epsSendFrame(false);
    }
    return;
  }
  if (!gridDown) {
    return;
  }
  if (epsApplied == EpsApplied::Enable && millis() - epsLastSendMs < EPS_RESEND_INTERVAL_MS) {
    return;
  }
  epsSendFrame(true);
}

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length + 1);
  for (unsigned i = 0; i < length; i++) {
    msg += char(payload[i]);
  }

  const String t(topic);
  Log.printf("[mqtt] %s = %s\n", topic, msg.c_str());

#if DUAL_BATT_ENABLE
  if (t == String(DUAL_BATT_SELECTOR_TOPIC) + "/selected") {
    dualBattOnSelected(msg);
    return;
  }
#endif

  // Only handle sofaress/set/* below
  if (!t.startsWith(String(DEVICE_NAME) + "/set/")) {
    return;
  }

  const String battPrefix = String(DEVICE_NAME) + "/set/batt_param/";
  if (t.startsWith(battPrefix)) {
    const String key = t.substring(battPrefix.length());
    handleBattParamSet(key, msg);
    return;
  }

  const String cmd = t.substring(t.lastIndexOf('/') + 1);

  if (cmd == "ess") {
    // true / on / 1 → enable ESS (leave charge_only as-is)
    // charge_only / excess / battery_save → enable + charge-only
    // full / bidirectional → enable + allow discharge
    // false / 0 / off → disable
    if (msg == "false" || msg == "0" || msg == "off") {
      essEnabled = false;
      resetEssIntegral();
    } else if (msg == "charge_only" || msg == "excess" || msg == "battery_save") {
      essEnabled = true;
      essChargeOnlyUser = true;
      if (essIntegral > 0.0f) {
        essIntegral = 0.0f;
      }
    } else if (msg == "full" || msg == "bidirectional") {
      essEnabled = true;
      essChargeOnlyUser = false;
    } else {
      essEnabled = true;
    }
    if (essEnabled) {
      endManualCharge(); // ESS owns the command again
    }
    mqttPublish("ess/enabled", essEnabled ? "true" : "false", true);
    nvsPutBool("ess_en", essEnabled);
    if (msg == "charge_only" || msg == "excess" || msg == "battery_save") {
      nvsPutBool("chg_only", essChargeOnlyUser);
    } else if (msg == "full" || msg == "bidirectional") {
      nvsPutBool("chg_only", essChargeOnlyUser);
    }
    publishEssParams();
    return;
  }

  if (cmd == "charge_only") {
    essChargeOnlyUser = (msg != "false" && msg != "0" && msg != "off");
    if (essChargeOnlyUser && essIntegral > 0.0f) {
      essIntegral = 0.0f;
    }
    Log.printf("[ess] charge_only=%d\n", int(essChargeOnlyUser));
    nvsPutBool("chg_only", essChargeOnlyUser);
    publishEssParams();
    return;
  }

  if (cmd == "soc_protect" || cmd == "soc_protect_enabled") {
    essSocProtectEnabled = (msg != "false" && msg != "0" && msg != "off");
    if (!essSocProtectEnabled) {
      essSocProtectActive = false;
      essSocFullActive = false;
    }
    Log.printf("[ess] soc_protect=%d\n", int(essSocProtectEnabled));
    nvsPutBool("soc_prot_en", essSocProtectEnabled);
    publishEssParams();
    return;
  }

  if (cmd == "soc_protect_low" || cmd == "soc_low") {
    const int v = msg.toInt();
    if (v >= 1 && v <= 100) {
      essSocProtectLow = uint8_t(v);
#if DUAL_BATT_ENABLE
      if (essSocProtectEnabled && dualBattEmptySoc > essSocProtectLow) {
        dualBattEmptySoc = essSocProtectLow;
        nvsPutU8("db_empty", dualBattEmptySoc);
        mqttPublish("battery/empty", String(dualBattEmptySoc), true);
      }
#endif
      Log.printf("[ess] soc_protect_low=%u\n", essSocProtectLow);
      nvsPutU8("soc_lo", essSocProtectLow);
      publishEssParams();
    } else {
      Log.println("[ess] soc_protect_low out of range (1..100)");
    }
    return;
  }

  if (cmd == "soc_protect_hyst" || cmd == "soc_hyst") {
    const int v = msg.toInt();
    if (v >= 0 && v <= 30) {
      essSocProtectHyst = uint8_t(v);
      Log.printf("[ess] soc_protect_hyst=%u\n", essSocProtectHyst);
      nvsPutU8("soc_hyst", essSocProtectHyst);
      publishEssParams();
    } else {
      Log.println("[ess] soc_protect_hyst out of range (0..30)");
    }
    return;
  }

  if (cmd == "soc_protect_high" || cmd == "soc_high") {
    const int v = msg.toInt();
    if (v >= 50 && v <= 100) {
      essSocProtectHigh = uint8_t(v);
      Log.printf("[ess] soc_protect_high=%u\n", essSocProtectHigh);
      nvsPutU8("soc_hi", essSocProtectHigh);
      publishEssParams();
    } else {
      Log.println("[ess] soc_protect_high out of range (50..100)");
    }
    return;
  }

  if (cmd == "soc_protect_high_hyst" || cmd == "soc_high_hyst") {
    const int v = msg.toInt();
    if (v >= 0 && v <= 30) {
      essSocProtectHighHyst = uint8_t(v);
      Log.printf("[ess] soc_protect_high_hyst=%u\n", essSocProtectHighHyst);
      nvsPutU8("soc_hi_hyst", essSocProtectHighHyst);
      publishEssParams();
    } else {
      Log.println("[ess] soc_protect_high_hyst out of range (0..30)");
    }
    return;
  }

  if (cmd == "kp") {
    const float v = msg.toFloat();
    if (v >= 0.0f && v <= 5.0f) {
      essKp = v;
      Log.printf("[ess] kp=%.3f\n", essKp);
      nvsPutFloat("kp", essKp);
      publishEssParams();
    } else {
      Log.println("[ess] kp out of range (0..5)");
    }
    return;
  }

  if (cmd == "ki") {
    const float v = msg.toFloat();
    if (v >= 0.0f && v <= 5.0f) {
      essKi = v;
      Log.printf("[ess] ki=%.3f\n", essKi);
      nvsPutFloat("ki", essKi);
      publishEssParams();
    } else {
      Log.println("[ess] ki out of range (0..5)");
    }
    return;
  }

  if (cmd == "deadband") {
    const float v = msg.toFloat();
    if (v >= 0.0f && v <= 500.0f) {
      essDeadbandW = v;
      Log.printf("[ess] deadband=%.1fW\n", essDeadbandW);
      nvsPutFloat("deadband", essDeadbandW);
      publishEssParams();
    } else {
      Log.println("[ess] deadband out of range (0..500)");
    }
    return;
  }

  if (cmd == "min_delta") {
    const float v = msg.toFloat();
    if (v >= 0.0f && v <= 500.0f) {
      essMinDeltaW = v;
      Log.printf("[ess] min_delta=%.1fW\n", essMinDeltaW);
      nvsPutFloat("min_delta", essMinDeltaW);
      publishEssParams();
    } else {
      Log.println("[ess] min_delta out of range (0..500)");
    }
    return;
  }

  if (cmd == "hold_min" || cmd == "hold_min_w") {
    const int v = msg.toInt();
    if (v >= 1 && v <= int(MAX_POWER_W)) {
      essHoldMinW = uint16_t(v);
      Log.printf("[ess] hold_min=%uW\n", essHoldMinW);
      nvsPutU16("hold_min", essHoldMinW);
      publishEssParams();
    } else {
      Log.println("[ess] hold_min out of range");
    }
    return;
  }

  if (cmd == "reset_i" || cmd == "reset_integral") {
    resetEssIntegral();
    Log.println("[ess] integral reset");
    mqttPublish("ess/integral", "0.0");
    return;
  }

  if (cmd == "charge_target_soc") {
    const int v = msg.toInt();
    if (v >= 0 && v <= 100) {
      chargeTargetSoc = uint8_t(v);
      Log.printf("[charge] target_soc=%u\n", chargeTargetSoc);
      nvsPutU8("chg_tgt", chargeTargetSoc);
      publishChargeTargetSoc();
    } else {
      Log.println("[charge] target_soc out of range (0..100)");
    }
    return;
  }

  if (cmd == "max_charge_w" || cmd == "max_charge_power") {
    const int v = msg.toInt();
    if (v >= 200 && v <= int(MAX_POWER_W)) {
      maxChargeW = uint16_t(v);
      nvsPutU16("chg_max_w", maxChargeW);
      Log.printf("[charge] max_charge_w=%u\n", maxChargeW);
      if (manualChargeW != 0) {
        manualChargeW = maxChargeW; // running session follows the new maximum
        manualChargeSentMs = 0;     // re-send on the next tick
      }
      publishManualCharge();
    } else {
      Log.println("[charge] max_charge_w out of range (200..MAX)");
    }
    return;
  }

#if DUAL_BATT_ENABLE
  if (cmd == "dual_batt" || cmd == "dual_batt_enabled") {
    dualBattEnabled = (msg != "false" && msg != "0" && msg != "off");
    Log.printf("[dual] enabled=%d\n", int(dualBattEnabled));
    nvsPutBool("db_en", dualBattEnabled);
    publishDualBatt();
    return;
  }

  if (cmd == "dual_batt_empty" || cmd == "battery_empty") {
    const int v = msg.toInt();
    if (v >= 0 && v <= 100) {
      dualBattEmptySoc = uint8_t(v);
      Log.printf("[dual] empty=%u\n", dualBattEmptySoc);
      nvsPutU8("db_empty", dualBattEmptySoc);
      publishDualBatt();
    } else {
      Log.println("[dual] empty out of range (0..100)");
    }
    return;
  }

  if (cmd == "dual_batt_full" || cmd == "battery_full") {
    const int v = msg.toInt();
    if (v >= 0 && v <= 100) {
      dualBattFullSoc = uint8_t(v);
      Log.printf("[dual] full=%u\n", dualBattFullSoc);
      nvsPutU8("db_full", dualBattFullSoc);
      publishDualBatt();
    } else {
      Log.println("[dual] full out of range (0..100)");
    }
    return;
  }

  if (cmd == "dual_batt_dwell" || cmd == "auto_dwell_min") {
    const int v = msg.toInt();
    if (v >= 0 && v <= 240) {
      dualBattAutoDwellMin = uint8_t(v);
      Log.printf("[dual] auto_dwell=%u min\n", dualBattAutoDwellMin);
      nvsPutU8("db_dwell", dualBattAutoDwellMin);
      publishDualBatt();
    } else {
      Log.println("[dual] auto dwell out of range (0..240 min)");
    }
    return;
  }

  if (cmd == "battery" || cmd == "battery_bank" || cmd == "force_battery") {
    const uint8_t bank = dualBattParseBank(msg, true);
    if (bank == 255) {
      Log.println("[dual] battery payload invalid (A/B/auto)");
      return;
    }
    dualBattForce = bank; // 0=auto
    nvsPutU8("db_force", dualBattForce);
    publishDualBatt();
    if (bank == 0) {
      Log.println("[dual] force cleared — auto policy");
    } else {
      Log.printf("[dual] force %s\n", dualBattLabel(bank));
      if (bank != dualBattActive) {
        dualBattBeginSwitch(bank); // force ignores cooldown
      }
    }
    return;
  }
#endif

  // EPS switch: persisted; epsTick() enforces it (on + grid lost → enable, off → disable).
  if (cmd == "eps") {
    epsEnabled = !(msg == "false" || msg == "0" || msg == "off");
    nvsPutBool("eps_en", epsEnabled);
    epsPublishState();
    Log.printf("[eps] switch=%d\n", int(epsEnabled));
    return;
  }

  // EPS messages: do not change ESS state. Reply on response/eps_enable|eps_disable.
  if (cmd == "eps_enable" || cmd == "eps_disable") {
    const bool okEps = epsSendFrame(cmd == "eps_enable");
    mqttPublish((String("response/") + cmd).c_str(), okEps ? "0" : "1");
    return;
  }

  // Manual Sofar2mqtt-compatible overrides disable ESS until re-enabled
  // "charge true|start|on" (HA button) uses the stored max charge power.
  const int value = (cmd == "charge" && (msg == "true" || msg == "start" || msg == "on"))
                        ? int(maxChargeW)
                        : msg.toInt();
  bool handled = false;

  if (cmd == "standby" && msg != "false") {
    essEnabled = false;
    resetEssIntegral();
    chargeTargetSoc = 0;
    publishChargeTargetSoc();
    endManualCharge();
    handled = sofarStandby();
    if (handled) {
      beginFollowGrace();
    }
  } else if (cmd == "auto") {
    essEnabled = false;
    resetEssIntegral();
    endManualCharge();
    if (msg == "true") {
      handled = sofarAuto();
    }
  } else if (cmd == "charge" && value > 0 && value <= MAX_POWER_W) {
    essEnabled = false;
    resetEssIntegral();
    // Start a manual charge session; value is the max charge power. The session
    // re-sends it, so a failed first send is retried by manualChargeTick().
    manualChargeW = uint16_t(value);
    manualChargeSentMs = 0;
    manualChargeTryMs = 0;
    followProhibitSinceMs = 0;
    handled = sofarCharge(uint16_t(value));
    if (handled) {
      manualChargeSentMs = millis();
      beginFollowGrace();
    }
    publishManualCharge();
  } else if (cmd == "discharge" && value > 0 && value <= MAX_POWER_W) {
    essEnabled = false;
    resetEssIntegral();
    endManualCharge();
    handled = sofarDischarge(uint16_t(value));
  }

  if (handled) {
    lastCmdMs = millis();
    lastCmdW = (cmd == "charge") ? -value : (cmd == "discharge") ? value : 0;
    const String respTopic = String("response/") + cmd;
    mqttPublish(respTopic.c_str(), handled ? "0" : "1");
    mqttPublish("ess/enabled", "false", true);
  }
}

static void ensureMqtt() {
  if (MQTT_HOST[0] == '\0' || mqtt.connected()) {
    return;
  }
  Log.printf("[mqtt] connect %s:%d\n", MQTT_HOST, MQTT_PORT);
  String willTopic = String(DEVICE_NAME) + "/status";
  bool ok;
  if (MQTT_USER[0] != '\0') {
    ok = mqtt.connect(DEVICE_NAME, MQTT_USER, MQTT_PASSWORD, willTopic.c_str(), 0, true, "offline");
  } else {
    ok = mqtt.connect(DEVICE_NAME, willTopic.c_str(), 0, true, "offline");
  }
  if (!ok) {
    Log.printf("[mqtt] failed rc=%d\n", mqtt.state());
    return;
  }
  Log.println("[mqtt] connected");
  mqtt.publish(willTopic.c_str(), "online", true);
  mqtt.subscribe((String(DEVICE_NAME) + "/set/#").c_str());
#if DUAL_BATT_ENABLE
  mqtt.subscribe((String(DUAL_BATT_SELECTOR_TOPIC) + "/selected").c_str());
#endif
  mqttPublish("ess/enabled", essEnabled ? "true" : "false", true);
  epsPublishState();
  publishEssParams();
  publishChargeTargetSoc();
  publishManualCharge();
#if DUAL_BATT_ENABLE
  publishDualBatt();
#endif
  battParamPollNow = true;
  lastEnergyPublishMs = 0;
#if HA_MQTT_DISCOVERY
  haDiscoverySent = false;
  publishHaDiscovery();
#endif
}

static void connectWifi() {
  if (strcmp(WIFI_SSID, "your-wifi-ssid") == 0) {
    Log.println("[wifi] edit include/secrets.h");
    return;
  }
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (!WiFi.setTxPower(WIFI_TX_POWER)) {
    Log.println("[wifi] setTxPower failed");
  }
  WiFi.setHostname(DEVICE_NAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Log.printf("[wifi] connecting to %s", WIFI_SSID);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Log.print('.');
  }
  Log.println();
  if (WiFi.status() == WL_CONNECTED) {
    Log.printf("[wifi] OK %s rssi=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Log.println("[wifi] failed");
  }
}

#if OTA_ENABLE
static bool otaReady = false;

static void startOta() {
  if (otaReady || WiFi.status() != WL_CONNECTED) {
    return;
  }

  ArduinoOTA.setHostname(DEVICE_NAME);
  if (OTA_PASSWORD[0] != '\0') {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    Log.println("[ota] updating — pausing ESS traffic");
    // Avoid Sofar/JSY activity during flash write.
  });
  ArduinoOTA.onEnd([]() { Log.println("[ota] done — rebooting"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint8_t lastPct = 255;
    const uint8_t pct = total ? uint8_t((progress * 100u) / total) : 0;
    if (pct != lastPct && (pct % 10u) == 0) {
      lastPct = pct;
      Log.printf("[ota] %u%%\n", pct);
    }
  });
  ArduinoOTA.onError([](ota_error_t err) {
    Log.printf("[ota] error %u\n", static_cast<unsigned>(err));
  });

  ArduinoOTA.begin();
  otaReady = true;
  Log.printf("[ota] ready as '%s' @ %s (espota)\n",
             DEVICE_NAME,
             WiFi.localIP().toString().c_str());
}
#endif

static void publishState() {
  // Cache only — never block ESS with a multi-register Sofar read here.
  String json = "{";
  json += "\"ess_enabled\":";
  json += essEnabled ? "true" : "false";
  json += ",\"ess_charge_only\":";
  json += essChargeOnlyUser ? "true" : "false";
  json += ",\"ess_effective_charge_only\":";
  json += essEffectiveChargeOnly() ? "true" : "false";
  json += ",\"ess_soc_protect_enabled\":";
  json += essSocProtectEnabled ? "true" : "false";
  json += ",\"ess_soc_protect_low\":";
  json += String(essSocProtectLow);
  json += ",\"ess_soc_protect_hyst\":";
  json += String(essSocProtectHyst);
  json += ",\"ess_soc_protect_active\":";
  json += essSocProtectActive ? "true" : "false";
  json += ",\"ess_soc_protect_high\":";
  json += String(essSocProtectHigh);
  json += ",\"ess_soc_protect_high_hyst\":";
  json += String(essSocProtectHighHyst);
  json += ",\"ess_soc_full_active\":";
  json += essSocFullActive ? "true" : "false";
  json += ",\"grid_power_w\":";
  json += isnan(lastGridPowerW) ? "null" : String(lastGridPowerW, 1);
  json += ",\"energy_import_wh\":";
  json += isnan(lastEnergyImportWh) ? "null" : String(lastEnergyImportWh, 1);
  json += ",\"energy_export_wh\":";
  json += isnan(lastEnergyExportWh) ? "null" : String(lastEnergyExportWh, 1);
  json += ",\"ess_command_w\":";
  json += String(lastCmdW);
  json += ",\"ess_kp\":";
  json += String(essKp, 3);
  json += ",\"ess_ki\":";
  json += String(essKi, 3);
  json += ",\"ess_deadband_w\":";
  json += String(essDeadbandW, 1);
  json += ",\"ess_min_delta_w\":";
  json += String(essMinDeltaW, 1);
  json += ",\"ess_hold_min_w\":";
  json += String(essHoldMinW);
  json += ",\"ess_integral\":";
  json += String(essIntegral, 1);
  json += ",\"follow_ok\":";
  json += followOk ? "true" : "false";
  json += ",\"follow_error_w\":";
  json += String(followErrorW);
  json += ",\"follow_grace\":";
  json += followGraceActive ? "true" : "false";
  json += ",\"follow_recovery\":\"";
  json += followRecoveryName(followLastRecovery);
  json += "\",\"passive_status\":";
  json += sofarPassiveStatusValid() ? String(sofarLastPassiveStatus()) : String("null");
  json += ",\"passive_result\":";
  json += sofarPassiveStatusValid() ? String(sofarLastPassiveResult()) : String("null");
  json += ",\"passive_charge_enabled\":";
  json += (sofarPassiveStatusValid() && (sofarLastPassiveStatus() & 0x01)) ? "true" : "false";
  json += ",\"passive_discharge_enabled\":";
  json += (sofarPassiveStatusValid() && (sofarLastPassiveStatus() & 0x02)) ? "true" : "false";
  json += ",\"passive_charge_prohibited\":";
  json += sofarChargeProhibited() ? "true" : "false";
  json += ",\"passive_discharge_prohibited\":";
  json += sofarDischargeProhibited() ? "true" : "false";
  json += ",\"sofar_mode\":\"";
  json += modeName(sofarLastMode());
  json += "\",\"charge_target_soc\":";
  json += String(chargeTargetSoc);
  json += ",\"max_charge_w\":";
  json += String(maxChargeW);
  json += ",\"manual_charge_w\":";
  json += String(manualChargeW);
#if DUAL_BATT_ENABLE
  json += ",\"battery_active\":\"";
  json += dualBattLabel(dualBattActive);
  json += "\",\"battery_force\":\"";
  json += dualBattForceLabel();
  json += "\",\"battery_dual_state\":\"";
  json += dualBattStateName(dualBattState);
  json += "\",\"battery_dual_enabled\":";
  json += dualBattEnabled ? "true" : "false";
  json += ",\"battery_empty_soc\":";
  json += String(dualBattEmptySoc);
  json += ",\"ess_discharge_floor_soc\":";
  json += String(essDischargeFloorSoc());
  json += ",\"battery_full_soc\":";
  json += String(dualBattFullSoc);
  json += ",\"battery_a_soc\":";
  json += dualBattSocAValid ? String(dualBattSocA) : String("null");
  json += ",\"battery_b_soc\":";
  json += dualBattSocBValid ? String(dualBattSocB) : String("null");
  json += ",\"battery_combined_soc\":";
  {
    const float combinedSoc = dualBattCombinedSoc();
    json += isnan(combinedSoc) ? String("null") : String(combinedSoc, 1);
  }
  json += ",\"battery_a_kwh\":";
  json += String(DUAL_BATT_A_KWH, 1);
  json += ",\"battery_b_kwh\":";
  json += String(DUAL_BATT_B_KWH, 1);
#endif
  json += ",\"eps_enabled\":";
  json += epsEnabled ? "true" : "false";
  json += ",\"grid_down\":";
  json += gridDown ? "true" : "false";
  json += ",\"grid_voltage_v\":";
  json += isnan(gridVoltageV) ? String("null") : String(gridVoltageV, 1);
  json += ",\"grid_frequency_hz\":";
  json += lastSofar.gridFreqValid ? String(lastSofar.gridFreqHz, 2) : String("null");
  json += ",\"sofar_ok\":";
  json += lastSofarOk ? "true" : "false";
  json += ",\"sofar_last_error\":\"";
  json += lastSofar.lastError;
  json += "\"";
  json += ",\"soc_rejects\":";
  json += String(lastSofar.socRejects);

  char faultMsg[160] = "";
  char alertMsg[96] = "";
  char battFaultMsg[96] = "";
  sofarFormatFaults(lastSofar, faultMsg, sizeof(faultMsg));
  sofarFormatAlerts(lastSofar, alertMsg, sizeof(alertMsg));
  sofarFormatBattFaults(lastSofar, battFaultMsg, sizeof(battFaultMsg));

  const bool faultActive =
      lastSofarOk && (lastSofar.runState == 6 || lastSofar.runState == 7 || faultMsg[0] != '\0');
  json += ",\"fault_active\":";
  json += faultActive ? "true" : "false";
  json += ",\"run_state_name\":\"";
  json += sofarRunStateName(lastSofar.runState);
  json += "\",\"fault_message\":\"";
  json += faultMsg;
  json += "\",\"alert_message\":\"";
  json += alertMsg;
  json += "\",\"batt_fault_message\":\"";
  json += battFaultMsg;
  json += "\"";

  if (lastSofarOk) {
    json += ",\"run_state\":";
    json += String(lastSofar.runState);
    json += ",\"battery_soc\":";
    json += lastSofar.socValid ? String(lastSofar.batterySoc) : String("null");
    json += ",\"charge_discharge_power_w\":";
    json += String(lastSofar.chargeDischargePowerW);
    json += ",\"battery_dc_power_w\":";
    json += lastSofar.batteryDcValid ? String(lastSofar.batteryDcPowerW) : String("null");
    json += ",\"battery_voltage_v\":";
    json += lastSofar.batteryDcValid ? String(lastSofar.batteryVoltageV, 1) : String("null");
    json += ",\"battery_current_a\":";
    json += lastSofar.batteryDcValid ? String(lastSofar.batteryCurrentA, 2) : String("null");
    json += ",\"sofar_grid_raw\":";
    json += String(lastSofar.gridPowerRaw);
  }
  if (lastSofar.pvValid) {
    json += ",\"pv1_voltage_v\":";
    json += String(lastSofar.pv1VoltageV, 1);
    json += ",\"pv1_current_a\":";
    json += String(lastSofar.pv1CurrentA, 2);
    json += ",\"pv1_power_w\":";
    json += String(lastSofar.pv1PowerW, 0);
    json += ",\"pv2_voltage_v\":";
    json += String(lastSofar.pv2VoltageV, 1);
    json += ",\"pv2_current_a\":";
    json += String(lastSofar.pv2CurrentA, 2);
    json += ",\"pv2_power_w\":";
    json += String(lastSofar.pv2PowerW, 0);
    json += ",\"pv_total_w\":";
    json += String(lastSofar.pvTotalW, 0);
  } else {
    json += ",\"pv1_voltage_v\":null,\"pv1_current_a\":null,\"pv1_power_w\":null";
    json += ",\"pv2_voltage_v\":null,\"pv2_current_a\":null,\"pv2_power_w\":null";
    json += ",\"pv_total_w\":null";
  }
  json += ",\"pv_today_kwh\":";
  json += lastSofar.pvTodayValid ? String(lastSofar.pvTodayKwh, 2) : String("null");
  json += ",\"batt_param_valid\":";
  json += lastSofar.battParams.valid ? "true" : "false";
  json += ",\"batt_param_poll_diag\":\"";
  json += lastSofar.battParams.pollDiag;
  json += "\",\"batt_param_reg_diag\":\"";
  json += lastSofar.battParams.regDiag;
  json += "\",\"batt_param_probe_progress\":\"";
  json += String(sofarBattParamOkCount(lastSofar.battParams));
  json += "/";
  json += String(sofarBattParamCount());
  json += "\",\"batt_param_poll_attempts\":";
  json += String(lastSofar.battParams.pollAttempts);
  json += ",\"batt_param_last_error\":\"";
  json += lastSofar.lastError;
  json += "\"";
  for (uint8_t i = 0; i < sofarBattParamCount(); i++) {
    const SofarBattParamMeta* m = sofarBattParamMeta(i);
    if (!m) {
      continue;
    }
    char regKey[24];
    snprintf(regKey, sizeof(regKey), "batt_param_reg_%04x", m->reg);
    json += ",\"";
    json += regKey;
    json += "\":\"";
    json += lastSofar.battParams.regWhy[i][0] ? lastSofar.battParams.regWhy[i] : "?";
    json += "\"";
    if (!lastSofar.battParams.regOk[i]) {
      continue;
    }
    json += ",\"batt_param_";
    json += m->key;
    json += "\":";
    const float scaled = sofarBattParamScaled(lastSofar.battParams, i);
    if (m->decimals == 0) {
      json += String(lroundf(scaled));
    } else {
      json += String(scaled, unsigned(m->decimals));
    }
  }
  if (lastSofar.faultValid) {
    char hex[48];
    snprintf(hex,
             sizeof(hex),
             "%04X,%04X,%04X,%04X,%04X",
             lastSofar.fault[0],
             lastSofar.fault[1],
             lastSofar.fault[2],
             lastSofar.fault[3],
             lastSofar.fault[4]);
    json += ",\"fault_raw\":\"";
    json += hex;
    json += "\"";
  }
  if (lastSofar.alertValid) {
    json += ",\"alert_raw\":";
    json += String(lastSofar.alert);
  }
  if (lastSofar.battFaultValid) {
    char hex[48];
    snprintf(hex,
             sizeof(hex),
             "%04X,%04X,%04X,%04X,%04X",
             lastSofar.battFault[0],
             lastSofar.battFault[1],
             lastSofar.battFault[2],
             lastSofar.battFault[3],
             lastSofar.battFault[4]);
    json += ",\"batt_fault_raw\":\"";
    json += hex;
    json += "\"";
  }
  json += "}";

  if (!mqtt.connected()) {
    Log.println("[mqtt] state skipped — not connected");
    return;
  }

  if (mqttPublish("state", json)) {
    Log.printf("[mqtt] state ok (%u bytes, sofar_cached=%d)\n",
               static_cast<unsigned>(json.length()),
               int(lastSofarOk));
  }
}

static bool startJsy() {
  jsy.setCallback([](Mycila::JSY::EventType event, const Mycila::JSY::Data& data) {
    if (event != Mycila::JSY::EventType::EVT_READ) {
      return;
    }
    portENTER_CRITICAL(&dataMux);
    jsyData = data;
    lastJsyReadMs = millis();
    portEXIT_CRITICAL(&dataMux);
  });

  Log.printf("[jsy] begin RX=%d TX=%d (async MycilaJSY)\n", JSY_RX_PIN, JSY_TX_PIN);
  jsy.begin(Serial2,
            JSY_RX_PIN,
            JSY_TX_PIN,
            Mycila::JSY::BaudRate::UNKNOWN,
            MYCILA_JSY_ADDRESS_BROADCAST,
            MYCILA_JSY_MK_194,
            true,
            0);

  delay(500);

  if (!jsy.isEnabled()) {
    Log.println("[jsy] failed to start — check wiring / 3V3 / TX-RX swap");
    return false;
  }

  Log.printf("[jsy] model=%s baud=%" PRIu32 " lastAddr=0x%02X\n",
             jsy.getModelName(),
             static_cast<uint32_t>(jsy.getBaudRate()),
             jsy.getLastAddress());

#if JSY_TARGET_BAUD == 38400
  if (jsy.getBaudRate() != Mycila::JSY::BaudRate::BAUD_38400) {
    Log.println("[jsy] switching to 38400 baud for ESS reactivity");
    jsy.end();
    jsy.begin(Serial2, JSY_RX_PIN, JSY_TX_PIN, false); // blocking session for config
    if (jsy.setBaudRate(Mycila::JSY::BaudRate::BAUD_38400)) {
      jsy.end();
      jsy.begin(Serial2,
                JSY_RX_PIN,
                JSY_TX_PIN,
                Mycila::JSY::BaudRate::BAUD_38400,
                MYCILA_JSY_ADDRESS_BROADCAST,
                MYCILA_JSY_MK_194,
                true,
                0);
      Log.printf("[jsy] now baud=%" PRIu32 "\n", static_cast<uint32_t>(jsy.getBaudRate()));
    } else {
      Log.println("[jsy] baud change failed — restarting async on detected rate");
      jsy.end();
      jsy.begin(Serial2,
                JSY_RX_PIN,
                JSY_TX_PIN,
                Mycila::JSY::BaudRate::UNKNOWN,
                MYCILA_JSY_ADDRESS_BROADCAST,
                MYCILA_JSY_MK_194,
                true,
                0);
    }
  }
#endif

  return true;
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Log.begin(LOG_BAUD);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  // If no serial monitor is attached, blocking USB TX stalls the ESS loop.
  Log.setTxTimeoutMs(0);
#endif
  delay(800);
  Log.println();
  Log.println("=== sofar_ess (ESP32-C3 + Sofar HW UART + MycilaJSY) ===");

  loadSettingsFromNvs();
  sofarBegin(Serial0);
  jsyReady = startJsy();
  digitalWrite(STATUS_LED_PIN, jsyReady ? HIGH : LOW);

  connectWifi();
#if OTA_ENABLE
  startOta();
#endif
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(4096); // state JSON ~2.5–3 kB with batt_param reg_diag fields
  mqtt.setKeepAlive(30);

  sofarHeartbeat();
  if (essEnabled) {
    sofarStandby();
    beginFollowGrace();
  } else {
    sofarAuto();
  }
}

void loop() {
#if OTA_ENABLE
  if (WiFi.status() == WL_CONNECTED) {
    if (!otaReady) {
      startOta();
    }
    ArduinoOTA.handle();
  } else {
    otaReady = false;
  }
#endif

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastWifi = 0;
    if (millis() - lastWifi > WIFI_RETRY_INTERVAL_MS) {
      lastWifi = millis();
      WiFi.reconnect();
    }
  } else {
    ensureMqtt();
    mqtt.loop();
  }

  const uint32_t now = millis();

  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    sofarHeartbeat();
  }

  if (now - lastEssMs >= ESS_LOOP_INTERVAL_MS) {
    lastEssMs = now;
    followRecoverTick();
    const Load2Snapshot snap = captureLoad2();
    if (snap.connected) {
      lastGridPowerW = snap.activePower;
      lastEnergyImportWh = snap.energyImportWh;
      lastEnergyExportWh = snap.energyExportWh;
      Log.printf("[jsy] load2 P=%.1fW V=%.1f E+=%.1fWh E-=%.1fWh\n",
                 snap.activePower,
                 snap.voltage,
                 snap.energyImportWh,
                 snap.energyExportWh);
      mqttPublish("ess/grid_power", String(snap.activePower, 1));
      if (essEnabled) {
        applyEss(snap.activePower);
      }
      digitalWrite(STATUS_LED_PIN, HIGH);
    } else {
      Log.println("[jsy] no fresh load2 data");
      digitalWrite(STATUS_LED_PIN, LOW);
    }

#if DUAL_BATT_ENABLE
    dualBattTick();
#endif
    manualChargeTick();

    // One Sofar register per ESS cycle (run → grid → batt → soc → …). Keeps
    // MQTT state fresh without a 4-read stall every 10 s.
#if DUAL_BATT_ENABLE
    // During SOC sync the state machine does dedicated SOC polls.
    if (dualBattState != DualBattState::SyncSoc) {
#endif
      if (sofarPollStatusField(lastSofar)) {
        lastSofarOk = true;
        checkChargeTargetSoc();
#if DUAL_BATT_ENABLE
        dualBattUpdateCacheFromLive();
#endif
      }
#if DUAL_BATT_ENABLE
    }
#endif
    epsTick();
    followTick();
  }

  energyMeterPublishTick();
  battParamPollTick();

  if (now - lastMqttStateMs >= MQTT_STATE_INTERVAL_MS) {
    lastMqttStateMs = now;
    if (!mqtt.connected()) {
      Log.println("[mqtt] state tick — not connected yet");
    }
    publishState();
    mqtt.loop();
  }

  delay(1);
}
