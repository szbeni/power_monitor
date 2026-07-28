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
static float essKp = ESS_KP;
static float essKi = ESS_KI;
static float essDeadbandW = float(ESS_DEADBAND_W);
static float essMinDeltaW = float(ESS_MIN_DELTA_W);
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
#if HA_MQTT_DISCOVERY
static bool haDiscoverySent = false;
#endif

static void resetEssIntegral() {
  essIntegral = 0.0f;
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

static void publishEssParams() {
  mqttPublish("ess/kp", String(essKp, 3), true);
  mqttPublish("ess/ki", String(essKi, 3), true);
  mqttPublish("ess/deadband", String(essDeadbandW, 1), true);
  mqttPublish("ess/min_delta", String(essMinDeltaW, 1), true);
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
    {"energy_import", "Energy Import", "{{ value_json.energy_import_wh }}", "Wh", "energy", "total_increasing"},
    {"energy_export", "Energy Export", "{{ value_json.energy_export_wh }}", "Wh", "energy", "total_increasing"},
    {"sofar_mode", "Sofar Mode", "{{ value_json.sofar_mode }}", nullptr, nullptr, nullptr},
    {"run_state", "Run State", "{{ value_json.run_state }}", nullptr, nullptr, nullptr},
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

  haDiscoverySent = true;
  Log.println("[ha] MQTT discovery published");
}
#endif // HA_MQTT_DISCOVERY

static void applyEss(float gridPowerW) {
  lastGridPowerW = gridPowerW;

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

  float uProbe = essKp * e + essKi * essIntegral;
  const bool saturated = (uProbe > maxW) || (uProbe < -maxW);

  // Integrate only outside deadband and when not pushing further into saturation.
  if (fabsf(e) >= essDeadbandW && !saturated) {
    essIntegral += e * Ts;
    const float iLim = (essKi > 1e-6f) ? (maxW / essKi) : maxW;
    if (essIntegral > iLim) {
      essIntegral = iLim;
    } else if (essIntegral < -iLim) {
      essIntegral = -iLim;
    }
  }

  float u = essKp * e + essKi * essIntegral;
  if (u > maxW) {
    u = maxW;
  } else if (u < -maxW) {
    u = -maxW;
  }

  int16_t targetW = 0;
  if (u > essDeadbandW) {
    targetW = int16_t(u);
  } else if (u < -essDeadbandW) {
    targetW = int16_t(u);
  } else if (fabsf(e) < essDeadbandW) {
    // Flat grid and small command → idle; bleed I so we don't stick forever.
    targetW = 0;
    essIntegral *= 0.9f;
    if (fabsf(essIntegral) < 1.0f) {
      essIntegral = 0.0f;
    }
  } else {
    // e still large but u in deadband — hold off commanding tiny watts
    targetW = 0;
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
  } else if (targetW < 0) {
    ok = sofarCharge(uint16_t(-targetW));
  } else {
    ok = sofarStandby();
  }

  if (ok) {
    lastCmdW = targetW;
    lastCmdMs = now;
  }

  Log.printf("[ess] grid=%.1fW u=%.1f I=%.1f dt=%.2f kp=%.2f ki=%.2f cmd=%d (%s) ok=%d\n",
             gridPowerW,
             u,
             essIntegral,
             Ts,
             essKp,
             essKi,
             int(targetW),
             modeName(sofarLastMode()),
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
  const String cmd = t.substring(t.lastIndexOf('/') + 1);
  Log.printf("[mqtt] %s = %s\n", topic, msg.c_str());

  if (cmd == "ess") {
    essEnabled = (msg != "false" && msg != "0" && msg != "off");
    if (!essEnabled) {
      resetEssIntegral();
    }
    mqttPublish("ess/enabled", essEnabled ? "true" : "false", true);
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

  if (cmd == "reset_i" || cmd == "reset_integral") {
    resetEssIntegral();
    Log.println("[ess] integral reset");
    mqttPublish("ess/integral", "0.0");
    return;
  }

  // Manual Sofar2mqtt-compatible overrides disable ESS until re-enabled
  const int value = msg.toInt();
  bool handled = false;

  if (cmd == "standby" && msg != "false") {
    essEnabled = false;
    resetEssIntegral();
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
  mqttPublish("ess/enabled", essEnabled ? "true" : "false", true);
  publishEssParams();
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
  json += ",\"ess_integral\":";
  json += String(essIntegral, 1);
  json += ",\"sofar_mode\":\"";
  json += modeName(sofarLastMode());
  json += "\"";
  if (lastSofarOk) {
    json += ",\"run_state\":";
    json += String(lastSofar.runState);
    json += ",\"battery_soc\":";
    json += String(lastSofar.batterySoc);
    json += ",\"battery_power_w\":";
    json += String(lastSofar.batteryPowerW);
    json += ",\"sofar_grid_raw\":";
    json += String(lastSofar.gridPowerRaw);
  } else {
    json += ",\"sofar_ok\":false";
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

    // One Sofar register per ESS cycle (run → grid → batt → soc → …). Keeps
    // MQTT state fresh without a 4-read stall every 10 s.
    if (sofarPollStatusField(lastSofar)) {
      lastSofarOk = true;
    }
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
