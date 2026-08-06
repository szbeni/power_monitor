#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <MycilaJSY.h>
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
static bool essChargeOnlyUser = ESS_CHARGE_ONLY;
static bool essSocProtectEnabled = ESS_SOC_PROTECT_ENABLE;
static uint8_t essSocProtectLow = ESS_SOC_PROTECT_LOW;
static uint8_t essSocProtectHyst = ESS_SOC_PROTECT_HYST;
static bool essSocProtectActive = false;
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
static uint32_t lastCmdMs = 0;
static int16_t lastCmdW = 0; // +discharge, -charge, 0=standby
static uint8_t chargeTargetSoc = 0; // 0 = no limit; manual charge stops at target → standby

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
static bool dualBattSelectedMatch = false;
static uint8_t dualBattReportedSelected = 0; // from selector MQTT, 0=unknown
static int16_t dualBattResumeCmdW = 0;
#endif

#if HA_MQTT_DISCOVERY
static bool haDiscoverySent = false;
#endif

static void resetEssIntegral() {
  essIntegral = 0.0f;
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

static void publishSocProtectParams() {
  mqttPublish("ess/soc_protect/enabled", essSocProtectEnabled ? "true" : "false", true);
  mqttPublish("ess/soc_protect/low", String(essSocProtectLow), true);
  mqttPublish("ess/soc_protect/hyst", String(essSocProtectHyst), true);
  mqttPublish("ess/soc_protect/active", essSocProtectActive ? "true" : "false", true);
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

// Bidirectional ESS → charge-only when SOC drops; resume at low + hysteresis.
static void updateEssSocProtect() {
  if (!essSocProtectEnabled || !essEnabled) {
    if (essSocProtectActive) {
      essSocProtectActive = false;
      publishSocProtectParams();
      mqttPublish("ess/effective_charge_only", essEffectiveChargeOnly() ? "true" : "false", true);
    }
    return;
  }
  if (!lastSofar.socValid) {
    return;
  }

  const uint8_t soc = uint8_t(lastSofar.batterySoc);
  const uint8_t resumeAt = uint8_t(min(100u, unsigned(essSocProtectLow) + unsigned(essSocProtectHyst)));

  if (!essSocProtectActive && soc < essSocProtectLow) {
    essSocProtectActive = true;
    if (essIntegral > 0.0f) {
      essIntegral = 0.0f;
    }
    Log.printf("[ess] SOC %u < %u — charge-only protect\n", soc, essSocProtectLow);
    publishSocProtectParams();
    mqttPublish("ess/effective_charge_only", "true", true);
  } else if (essSocProtectActive && soc >= resumeAt) {
    essSocProtectActive = false;
    Log.printf("[ess] SOC %u >= %u — bidirectional resumed\n", soc, resumeAt);
    publishSocProtectParams();
    mqttPublish("ess/effective_charge_only", essEffectiveChargeOnly() ? "true" : "false", true);
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

static void dualBattRequestBank(uint8_t bank) {
  if (!dualBattEnabled && dualBattForce == 0) {
    return;
  }
  if (bank == dualBattActive || bank == 0) {
    return;
  }
  const uint32_t now = millis();
  if (dualBattLastSwitchMs != 0 && (now - dualBattLastSwitchMs) < DUAL_BATT_COOLDOWN_MS) {
    Log.printf("[dual] cooldown %lu ms left\n",
               static_cast<unsigned long>(DUAL_BATT_COOLDOWN_MS - (now - dualBattLastSwitchMs)));
    return;
  }
  dualBattBeginSwitch(bank);
}

static void dualBattEvaluatePolicy() {
  if (!dualBattEnabled || dualBattBusy()) {
    return;
  }

  // Force override: keep requested bank connected.
  if (dualBattForce == 1 || dualBattForce == 2) {
    if (dualBattForce != dualBattActive) {
      dualBattRequestBank(dualBattForce);
    }
    return;
  }

  // Intent from last ESS/manual command. Near-zero hold does not switch banks.
  const bool wantDischarge = lastCmdW > int16_t(essHoldMinW);
  const bool wantCharge = lastCmdW < -int16_t(essHoldMinW);
  if (!wantDischarge && !wantCharge) {
    return;
  }

  const bool aKnown = dualBattSocAValid;
  const bool bKnown = dualBattSocBValid;
  const bool aEmpty = aKnown && dualBattSocA <= dualBattEmptySoc;
  const bool bEmpty = bKnown && dualBattSocB <= dualBattEmptySoc;
  const bool aFull = aKnown && dualBattSocA >= dualBattFullSoc;
  const bool bFull = bKnown && dualBattSocB >= dualBattFullSoc;

  if (wantDischarge) {
    if (dualBattActive == 1) {
      // A empty → try B unless we know B is also empty.
      if (aEmpty && (!bKnown || !bEmpty)) {
        dualBattRequestBank(2);
      }
    } else {
      // Prefer A whenever it may still have energy.
      if (!aKnown || !aEmpty) {
        dualBattRequestBank(1);
      }
    }
  } else if (wantCharge) {
    if (dualBattActive == 1) {
      if (aFull && (!bKnown || !bFull)) {
        dualBattRequestBank(2);
      }
    } else {
      if (!aKnown || !aFull) {
        dualBattRequestBank(1);
      }
    }
  }
}

static void dualBattTick() {
  const uint32_t now = millis();

  switch (dualBattState) {
    case DualBattState::Idle:
      dualBattEvaluatePolicy();
      break;

    case DualBattState::Standby:
      if (sofarStandby()) {
        lastCmdW = 0;
        lastCmdMs = now;
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

// Manual charge (ESS off): stop at chargeTargetSoc and go standby.
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
  float effectiveSoc = float(lastSofar.batterySoc);
#if DUAL_BATT_ENABLE
  const float combinedSoc = dualBattCombinedSoc();
  if (!isnan(combinedSoc)) {
    effectiveSoc = combinedSoc;
  }
#endif
  if (effectiveSoc < float(chargeTargetSoc)) {
    return;
  }

  const uint8_t hit = chargeTargetSoc;
  if (sofarStandby()) {
    lastCmdW = 0;
    lastCmdMs = millis();
    chargeTargetSoc = 0;
    publishChargeTargetSoc();
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
    {"battery_power", "Battery Power", "{{ value_json.battery_power_w }}", "W", "power", "measurement"},
    {"ess_command", "ESS Command", "{{ value_json.ess_command_w }}", "W", "power", "measurement"},
    {"battery_soc", "Battery SOC", "{{ value_json.battery_soc }}", "%", "battery", "measurement"},
    {"battery_a_soc", "Battery A SOC", "{{ value_json.battery_a_soc }}", "%", "battery", "measurement"},
    {"battery_b_soc", "Battery B SOC", "{{ value_json.battery_b_soc }}", "%", "battery", "measurement"},
    {"battery_combined_soc", "Battery Combined SOC", "{{ value_json.battery_combined_soc }}", "%", "battery", "measurement"},
    {"battery_active", "Battery Active", "{{ value_json.battery_active }}", nullptr, nullptr, nullptr},
    {"battery_dual_state", "Battery Dual State", "{{ value_json.battery_dual_state }}", nullptr, nullptr, nullptr},
    {"energy_import", "Energy Import", "{{ value_json.energy_import_wh }}", "Wh", "energy", "total_increasing"},
    {"energy_export", "Energy Export", "{{ value_json.energy_export_wh }}", "Wh", "energy", "total_increasing"},
    {"sofar_mode", "Sofar Mode", "{{ value_json.sofar_mode }}", nullptr, nullptr, nullptr},
    {"run_state", "Run State", "{{ value_json.run_state }}", nullptr, nullptr, nullptr},
    {"run_state_name", "Run State Name", "{{ value_json.run_state_name }}", nullptr, nullptr, nullptr},
    {"fault_message", "Sofar Fault", "{{ value_json.fault_message }}", nullptr, nullptr, nullptr},
    {"alert_message", "Sofar Alert", "{{ value_json.alert_message }}", nullptr, nullptr, nullptr},
    {"batt_fault_message", "Battery Fault", "{{ value_json.batt_fault_message }}", nullptr, nullptr, nullptr},
    {"sofar_last_error", "Sofar Last Error", "{{ value_json.sofar_last_error }}", nullptr, nullptr, nullptr},
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
    doc["value_template"] = "{{ value_json.sofar_ok }}";
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
    doc["value_template"] = "{{ value_json.fault_active }}";
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
      {"soc_protect_low", "ESS SOC Protect Low", "ess/soc_protect/low", "set/soc_protect_low", 1.0f, 100.0f, 1.0f},
      {"soc_protect_hyst", "ESS SOC Protect Hysteresis", "ess/soc_protect/hyst", "set/soc_protect_hyst", 0.0f, 30.0f, 1.0f},
#if DUAL_BATT_ENABLE
      {"dual_batt_empty", "Battery Empty SOC", "battery/empty", "set/dual_batt_empty", 0.0f, 100.0f, 1.0f},
      {"dual_batt_full", "Battery Full SOC", "battery/full", "set/dual_batt_full", 0.0f, 100.0f, 1.0f},
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
    struct {
      const char* objectId;
      const char* name;
      const char* commandTopic;
    } buttons[] = {
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

static void applyEss(float gridPowerW) {
  lastGridPowerW = gridPowerW;
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
  // Charge-only: allow charge (−) and hold, never discharge (+).
  const float uMax = chargeOnly ? 0.0f : maxW;
  const float uMin = -maxW;

  float uProbe = essKp * e + essKi * essIntegral;
  const bool saturated = (uProbe > uMax) || (uProbe < uMin);

  // Integrate only outside deadband and when not pushing further into saturation.
  if (fabsf(e) >= essDeadbandW && !saturated) {
    essIntegral += e * Ts;
    const float iLim = (essKi > 1e-6f) ? (maxW / essKi) : maxW;
    const float iMax = chargeOnly ? 0.0f : iLim;
    if (essIntegral > iMax) {
      essIntegral = iMax;
    } else if (essIntegral < -iLim) {
      essIntegral = -iLim;
    }
  }

  float u = essKp * e + essKi * essIntegral;
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

  // Hard clamp: charge-only never issues a discharge command.
  if (chargeOnly && targetW > 0) {
    targetW = 0;
  }

  // Near-zero: hold min charge/discharge instead of standby (Sofar relay chatter).
  if (targetW == 0) {
    const int16_t hold = int16_t(essHoldMinW);
    if (chargeOnly || lastCmdW < 0 || sofarLastMode() == SofarMode::Charge) {
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

  Log.printf("[ess] grid=%.1fW u=%.1f I=%.1f dt=%.2f kp=%.2f ki=%.2f cmd=%d (%s%s%s) ok=%d\n",
             gridPowerW,
             u,
             essIntegral,
             Ts,
             essKp,
             essKi,
             int(targetW),
             modeName(sofarLastMode()),
             chargeOnly ? " charge_only" : "",
             essSocProtectActive ? " soc_protect" : "",
             int(ok));
  mqttPublish("ess/grid_power", String(gridPowerW, 1));
  mqttPublish("ess/command_w", String(targetW));
  mqttPublish("ess/mode", modeName(sofarLastMode()));
  mqttPublish("ess/integral", String(essIntegral, 1));
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
    mqttPublish("ess/enabled", essEnabled ? "true" : "false", true);
    publishEssParams();
    return;
  }

  if (cmd == "charge_only") {
    essChargeOnlyUser = (msg != "false" && msg != "0" && msg != "off");
    if (essChargeOnlyUser && essIntegral > 0.0f) {
      essIntegral = 0.0f;
    }
    Log.printf("[ess] charge_only=%d\n", int(essChargeOnlyUser));
    publishEssParams();
    return;
  }

  if (cmd == "soc_protect" || cmd == "soc_protect_enabled") {
    essSocProtectEnabled = (msg != "false" && msg != "0" && msg != "off");
    if (!essSocProtectEnabled && essSocProtectActive) {
      essSocProtectActive = false;
    }
    Log.printf("[ess] soc_protect=%d\n", int(essSocProtectEnabled));
    publishEssParams();
    return;
  }

  if (cmd == "soc_protect_low" || cmd == "soc_low") {
    const int v = msg.toInt();
    if (v >= 1 && v <= 100) {
      essSocProtectLow = uint8_t(v);
      Log.printf("[ess] soc_protect_low=%u\n", essSocProtectLow);
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
      publishEssParams();
    } else {
      Log.println("[ess] soc_protect_hyst out of range (0..30)");
    }
    return;
  }

  if (cmd == "kp") {
    const float v = msg.toFloat();
    if (v >= 0.0f && v <= 5.0f) {
      essKp = v;
      Log.printf("[ess] kp=%.3f\n", essKp);
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
      publishChargeTargetSoc();
    } else {
      Log.println("[charge] target_soc out of range (0..100)");
    }
    return;
  }

#if DUAL_BATT_ENABLE
  if (cmd == "dual_batt" || cmd == "dual_batt_enabled") {
    dualBattEnabled = (msg != "false" && msg != "0" && msg != "off");
    Log.printf("[dual] enabled=%d\n", int(dualBattEnabled));
    publishDualBatt();
    return;
  }

  if (cmd == "dual_batt_empty" || cmd == "battery_empty") {
    const int v = msg.toInt();
    if (v >= 0 && v <= 100) {
      dualBattEmptySoc = uint8_t(v);
      Log.printf("[dual] empty=%u\n", dualBattEmptySoc);
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
      publishDualBatt();
    } else {
      Log.println("[dual] full out of range (0..100)");
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

  // Manual Sofar2mqtt-compatible overrides disable ESS until re-enabled
  const int value = msg.toInt();
  bool handled = false;

  if (cmd == "standby" && msg != "false") {
    essEnabled = false;
    resetEssIntegral();
    chargeTargetSoc = 0;
    publishChargeTargetSoc();
    handled = sofarStandby();
  } else if (cmd == "auto") {
    essEnabled = false;
    resetEssIntegral();
    if (msg == "true") {
      handled = sofarAuto();
    }
  } else if (cmd == "charge" && value > 0 && value <= MAX_POWER_W) {
    essEnabled = false;
    resetEssIntegral();
    handled = sofarCharge(uint16_t(value));
  } else if (cmd == "discharge" && value > 0 && value <= MAX_POWER_W) {
    essEnabled = false;
    resetEssIntegral();
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
  publishEssParams();
  publishChargeTargetSoc();
#if DUAL_BATT_ENABLE
  publishDualBatt();
#endif
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
  json += ",\"sofar_mode\":\"";
  json += modeName(sofarLastMode());
  json += "\",\"charge_target_soc\":";
  json += String(chargeTargetSoc);
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
  json += ",\"sofar_ok\":";
  json += lastSofarOk ? "true" : "false";
  json += ",\"sofar_last_error\":\"";
  json += lastSofar.lastError;
  json += "\"";

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
    json += ",\"battery_power_w\":";
    json += String(lastSofar.batteryPowerW);
    json += ",\"sofar_grid_raw\":";
    json += String(lastSofar.gridPowerRaw);
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

  sofarBegin(Serial0);
  jsyReady = startJsy();
  digitalWrite(STATUS_LED_PIN, jsyReady ? HIGH : LOW);

  connectWifi();
#if OTA_ENABLE
  startOta();
#endif
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(2048);
  mqtt.setKeepAlive(30);

  sofarHeartbeat();
  if (essEnabled) {
    sofarStandby();
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
      mqttPublish("ess/energy_import_wh", String(snap.energyImportWh, 1));
      mqttPublish("ess/energy_export_wh", String(snap.energyExportWh, 1));
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
  }

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
