#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <cmath>
#include <cstring>

#include "config.h"
#include "jsy.h"
#include "sofar.h"

static SoftwareSerial sofarBus(SOFAR_RX_PIN, SOFAR_TX_PIN);
static SoftwareSerial jsyBus(JSY_RX_PIN, JSY_TX_PIN);

static WiFiClient wifi;
static PubSubClient mqtt(wifi);

static bool essEnabled = ESS_ENABLE;
static float lastGridPowerW = NAN;
static float lastEnergyImportWh = NAN;
static float lastEnergyExportWh = NAN;
static uint32_t lastEssMs = 0;
static uint32_t lastHeartbeatMs = 0;
static uint32_t lastMqttStateMs = 0;
static uint32_t lastCmdMs = 0;
static int16_t lastCmdW = 0; // +discharge, -charge, 0=standby

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

static void mqttPublish(const char* suffix, const String& payload, bool retain = false) {
  if (!mqtt.connected()) {
    return;
  }
  String topic = String(DEVICE_NAME) + "/" + suffix;
  mqtt.publish(topic.c_str(), payload.c_str(), retain);
}

static void applyEss(float gridPowerW) {
  lastGridPowerW = gridPowerW;

  // Convention: >0 import from grid → discharge battery
  //             <0 export to grid   → charge battery
  int16_t targetW = 0;
  if (gridPowerW > ESS_DEADBAND_W) {
    const float w = gridPowerW > float(MAX_POWER_W) ? float(MAX_POWER_W) : gridPowerW;
    targetW = int16_t(w);
  } else if (gridPowerW < -ESS_DEADBAND_W) {
    const float w = (-gridPowerW) > float(MAX_POWER_W) ? float(MAX_POWER_W) : -gridPowerW;
    targetW = -int16_t(w);
  }

  const uint32_t now = millis();
  const bool changed = abs(int(targetW) - int(lastCmdW)) >= ESS_MIN_DELTA_W;
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

  Log.printf("[ess] grid=%.1fW cmd=%d (%s) ok=%d\n",
             gridPowerW,
             int(targetW),
             modeName(sofarLastMode()),
             int(ok));
  mqttPublish("ess/grid_power", String(gridPowerW, 1));
  mqttPublish("ess/command_w", String(targetW));
  mqttPublish("ess/mode", modeName(sofarLastMode()));
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
    mqttPublish("ess/enabled", essEnabled ? "true" : "false", true);
    return;
  }

  // Manual Sofar2mqtt-compatible overrides disable ESS until re-enabled
  const int value = msg.toInt();
  bool handled = false;

  if (cmd == "standby" && msg != "false") {
    essEnabled = false;
    handled = sofarStandby();
  } else if (cmd == "auto") {
    essEnabled = false;
    if (msg == "true") {
      handled = sofarAuto();
    }
  } else if (cmd == "charge" && value > 0 && value <= MAX_POWER_W) {
    essEnabled = false;
    handled = sofarCharge(uint16_t(value));
  } else if (cmd == "discharge" && value > 0 && value <= MAX_POWER_W) {
    essEnabled = false;
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
}

static void connectWifi() {
  if (strcmp(WIFI_SSID, "your-wifi-ssid") == 0) {
    Log.println("[wifi] edit include/secrets.h");
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Log.printf("[wifi] connecting to %s", WIFI_SSID);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    Log.print('.');
  }
  Log.println();
  if (WiFi.status() == WL_CONNECTED) {
    Log.printf("[wifi] OK %s\n", WiFi.localIP().toString().c_str());
  } else {
    Log.println("[wifi] failed");
  }
}

static void publishState() {
  SofarStatus st;
  const bool sofarOk = sofarReadStatus(st);
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
  json += ",\"sofar_mode\":\"";
  json += modeName(sofarLastMode());
  json += "\"";
  if (sofarOk) {
    json += ",\"run_state\":";
    json += String(st.runState);
    json += ",\"battery_soc\":";
    json += String(st.batterySoc);
    json += ",\"sofar_grid_raw\":";
    json += String(st.gridPowerRaw);
  }
  json += "}";
  mqttPublish("state", json);
}

void setup() {
  Log.begin(LOG_BAUD);
  delay(200);
  Log.println();
  Log.println("=== sofar_jsy_ess (NodeMCU + Sofar SoftSerial + JSY SoftSerial) ===");

  sofarBegin(sofarBus);
  jsyBegin(jsyBus);

  connectWifi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);

  sofarHeartbeat();
  if (essEnabled) {
    sofarStandby();
  } else {
    sofarAuto();
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastWifi = 0;
    if (millis() - lastWifi > 15000) {
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
    JsyLoad2 jsy;
    if (jsyReadLoad2(jsyBus, jsy)) {
      lastGridPowerW = jsy.activePower;
      lastEnergyImportWh = jsy.energyImportWh;
      lastEnergyExportWh = jsy.energyExportWh;
      Log.printf("[jsy] load2 P=%.1fW V=%.1f E+=%.1fWh E-=%.1fWh\n",
                 jsy.activePower,
                 jsy.voltage,
                 jsy.energyImportWh,
                 jsy.energyExportWh);
      mqttPublish("ess/grid_power", String(jsy.activePower, 1));
      mqttPublish("ess/energy_import_wh", String(jsy.energyImportWh, 1));
      mqttPublish("ess/energy_export_wh", String(jsy.energyExportWh, 1));
      if (essEnabled) {
        applyEss(jsy.activePower);
      }
    } else {
      Log.println("[jsy] read failed");
    }
  }

  if (now - lastMqttStateMs >= MQTT_STATE_INTERVAL_MS) {
    lastMqttStateMs = now;
    publishState();
  }

  delay(10);
}
