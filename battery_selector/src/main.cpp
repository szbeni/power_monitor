#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <cstring>

#include "config.h"

static const uint8_t kRelayPins[NUM_RELAYS] = {
    RELAY1_PIN,
    RELAY2_PIN,
    RELAY3_PIN,
    RELAY4_PIN,
};

static WiFiClient wifi;
static PubSubClient mqtt(wifi);

// 1 = Battery A (NC / de-energized), 2 = Battery B (NO / energized)
static uint8_t selectedBattery = DEFAULT_BATTERY;
static uint32_t lastReportMs = 0;
static uint32_t lastWifiRetryMs = 0;

static const char* batteryLabel(uint8_t battery) {
  return (battery == 2) ? HA_OPTION_B : HA_OPTION_A;
}

static bool mqttPublish(const char* suffix, const String& payload, bool retain = false) {
  if (!mqtt.connected()) {
    return false;
  }
  const String topic = String(DEVICE_NAME) + "/" + suffix;
  if (!mqtt.publish(topic.c_str(), payload.c_str(), retain)) {
    Log.printf("[mqtt] publish fail %s\n", topic.c_str());
    return false;
  }
  return true;
}

static void applyRelays(uint8_t battery) {
  // Battery A (1): de-energize → NC contacts
  // Battery B (2): energize active channels → NO contacts
  // Relay 4 is unused and always left de-energized.
  const bool energize = (battery == 2);

  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    const bool on = energize && (i < NUM_ACTIVE_RELAYS);
#if RELAY_ACTIVE_HIGH
    digitalWrite(kRelayPins[i], on ? HIGH : LOW);
#else
    digitalWrite(kRelayPins[i], on ? LOW : HIGH);
#endif
  }
  digitalWrite(STATUS_LED_PIN, energize ? HIGH : LOW);
}

static void publishSelected() {
  mqttPublish("selected", batteryLabel(selectedBattery), true);
}

static void setBattery(uint8_t battery) {
  if (battery < 1 || battery > NUM_BATTERIES) {
    Log.printf("[select] invalid battery %u (use 1=A or 2=B)\n", battery);
    return;
  }
  selectedBattery = battery;
  applyRelays(selectedBattery);
  Log.printf("[select] battery %u (%s)\n", selectedBattery, batteryLabel(selectedBattery));
  publishSelected();
  lastReportMs = millis();
}

static bool parseBatteryPayload(const String& msg, uint8_t& out) {
  String m = msg;
  m.trim();

  // Exact HA select option labels (case-sensitive match first)
  if (m == HA_OPTION_A) {
    out = 1;
    return true;
  }
  if (m == HA_OPTION_B) {
    out = 2;
    return true;
  }

  m.toLowerCase();

  if (m == "a" || m == "battery_a" || m == "batterya" || m == "battery a" || m == "1") {
    out = 1;
    return true;
  }
  if (m == "b" || m == "battery_b" || m == "batteryb" || m == "battery b" || m == "2") {
    out = 2;
    return true;
  }
  // "off" / "0" map to safe default: Battery A (de-energized / NC)
  if (m == "off" || m == "none" || m == "0" || m == "false") {
    out = 1;
    return true;
  }
  if (m.startsWith("battery")) {
    int n = 0;
    for (size_t i = 0; i < m.length(); i++) {
      if (m[i] >= '0' && m[i] <= '9') {
        n = n * 10 + (m[i] - '0');
      }
    }
    if (n >= 1 && n <= NUM_BATTERIES) {
      out = uint8_t(n);
      return true;
    }
    return false;
  }
  return false;
}

#if HA_MQTT_DISCOVERY
static void haFillDevice(JsonObject device) {
  device["identifiers"][0] = DEVICE_NAME;
  device["name"] = HA_DEVICE_NAME;
  device["manufacturer"] = "power_monitor";
  device["model"] = "ESP32 4-relay battery switch";
  device["sw_version"] = "battery_selector";
}

static void publishHaDiscovery() {
  if (!mqtt.connected()) {
    return;
  }

  const String stateTopic = String(DEVICE_NAME) + "/selected";
  const String commandTopic = String(DEVICE_NAME) + "/set/select";
  const String availTopic = String(DEVICE_NAME) + "/status";

  JsonDocument doc;
  doc["name"] = "Battery";
  char uniqueId[64];
  snprintf(uniqueId, sizeof(uniqueId), "%s_battery", DEVICE_NAME);
  doc["unique_id"] = uniqueId;
  doc["command_topic"] = commandTopic;
  doc["state_topic"] = stateTopic;
  doc["availability_topic"] = availTopic;
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["icon"] = "mdi:car-battery";
  JsonArray options = doc["options"].to<JsonArray>();
  options.add(HA_OPTION_A);
  options.add(HA_OPTION_B);
  haFillDevice(doc["device"].to<JsonObject>());

  String payload;
  serializeJson(doc, payload);

  char topic[160];
  snprintf(topic, sizeof(topic), "%s/select/%s/battery/config", HA_DISCOVERY_PREFIX, DEVICE_NAME);
  if (!mqtt.publish(topic, payload.c_str(), true)) {
    Log.printf("[ha] discovery fail (len=%u)\n", static_cast<unsigned>(payload.length()));
    return;
  }
  Log.println("[ha] MQTT discovery published (select/battery)");
}
#endif

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length + 1);
  for (unsigned i = 0; i < length; i++) {
    msg += char(payload[i]);
  }
  msg.trim();

  const String t(topic);
  const String cmd = t.substring(t.lastIndexOf('/') + 1);
  Log.printf("[mqtt] %s = %s\n", topic, msg.c_str());

  if (cmd == "select" || cmd == "battery" || cmd == "set") {
    uint8_t battery = 0;
    if (!parseBatteryPayload(msg, battery)) {
      Log.printf("[select] bad payload: %s (use Battery A/B, 1/2, or A/B)\n", msg.c_str());
      return;
    }
    setBattery(battery);
  }
}

static void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const uint32_t now = millis();
  if (lastWifiRetryMs != 0 && (now - lastWifiRetryMs) < WIFI_RETRY_INTERVAL_MS) {
    return;
  }
  lastWifiRetryMs = now;

  Log.printf("[wifi] connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_max_tx_power(WIFI_TX_POWER);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
    Log.print('.');
  }
  Log.println();

  if (WiFi.status() == WL_CONNECTED) {
    Log.printf("[wifi] connected %s\n", WiFi.localIP().toString().c_str());
  } else {
    Log.println("[wifi] connect failed");
  }
}

static void ensureMqtt() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) {
    return;
  }

  const String willTopic = String(DEVICE_NAME) + "/status";
  String clientId = String(DEVICE_NAME) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  Log.printf("[mqtt] connecting to %s:%d as %s\n", MQTT_HOST, MQTT_PORT, clientId.c_str());
  const bool ok = (MQTT_USER[0] != '\0')
                      ? mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD,
                                     willTopic.c_str(), 0, true, "offline")
                      : mqtt.connect(clientId.c_str(), willTopic.c_str(), 0, true, "offline");
  if (!ok) {
    Log.printf("[mqtt] connect failed rc=%d\n", mqtt.state());
    return;
  }

  Log.println("[mqtt] connected");
  mqtt.publish(willTopic.c_str(), "online", true);
  mqtt.subscribe((String(DEVICE_NAME) + "/set/#").c_str());
  publishSelected();
#if HA_MQTT_DISCOVERY
  publishHaDiscovery();
#endif
}

#if OTA_ENABLE
static void startOta() {
  ArduinoOTA.setHostname(DEVICE_NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { Log.println("[ota] start"); });
  ArduinoOTA.onEnd([]() { Log.println("\n[ota] end"); });
  ArduinoOTA.onError([](ota_error_t e) { Log.printf("[ota] error %u\n", e); });
  ArduinoOTA.begin();
  Log.printf("[ota] ready as '%s' @ %s\n", DEVICE_NAME, WiFi.localIP().toString().c_str());
}
#endif

void setup() {
  Log.begin(LOG_BAUD);
  delay(200);
  Log.println();
  Log.println("=== battery_selector ===");

  for (uint8_t i = 0; i < NUM_RELAYS; i++) {
    pinMode(kRelayPins[i], OUTPUT);
  }
  pinMode(STATUS_LED_PIN, OUTPUT);
  applyRelays(selectedBattery);

  connectWifi();
#if OTA_ENABLE
  if (WiFi.status() == WL_CONNECTED) {
    startOta();
  }
#endif

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(30);
  mqtt.setBufferSize(1024);
  ensureMqtt();

  lastReportMs = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
#if OTA_ENABLE
    if (WiFi.status() == WL_CONNECTED) {
      startOta();
    }
#endif
  }

  if (WiFi.status() == WL_CONNECTED) {
#if OTA_ENABLE
    ArduinoOTA.handle();
#endif
    ensureMqtt();
    if (mqtt.connected()) {
      mqtt.loop();
    }
  }

  const uint32_t now = millis();
  if ((now - lastReportMs) >= REPORT_INTERVAL_MS) {
    lastReportMs = now;
    if (mqtt.connected()) {
      publishSelected();
    }
    Log.printf("[report] selected=%u (%s) wifi=%s mqtt=%s\n",
               selectedBattery,
               batteryLabel(selectedBattery),
               WiFi.status() == WL_CONNECTED ? "up" : "down",
               mqtt.connected() ? "up" : "down");
  }
}
