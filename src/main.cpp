#include <Arduino.h>
#include <ArduinoJson.h>
#include <MycilaJSY.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <cmath>
#include <cstring>

#include "config.h"

#if ESS_UDP_ENABLE
#include <WiFiUdp.h>
#endif
#if ESS_MQTT_ENABLE
#include <MQTT.h>
#endif

// ESP32-C3 has only 2 UARTs; map Serial2 helpers like Mycila examples.
#ifndef SOC_UART_HP_NUM
#define SOC_UART_HP_NUM SOC_UART_NUM
#endif
#if SOC_UART_HP_NUM < 3
#define Serial2 Serial1
#endif

static Mycila::JSY jsy;
static Mycila::JSY::Data jsyData;
static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t readCount = 0;
static volatile uint32_t lastReadMs = 0;

#if ESS_UDP_ENABLE
static WiFiUDP udp;
static uint32_t udpSeq = 0;
static uint32_t udpOk = 0;
static uint32_t udpFail = 0;
#endif
#if ESS_MQTT_ENABLE
static WiFiClient wifiClient;
static MQTTClient mqtt(1536);
#endif
static WebServer server(HTTP_PORT);

static uint32_t lastPublishMs = 0;
static uint32_t lastEssTickMs = 0;
#if ESS_MQTT_ENABLE
static uint32_t lastMqttReconnectMs = 0;
static uint32_t lastHaPublishMs = 0;
static uint32_t mqttPublishOk = 0;
static uint32_t mqttPublishFail = 0;
static char mqttClientId[32] = MQTT_CLIENT_ID;
#if HA_MQTT_DISCOVERY
static bool haDiscoverySent = false;
#endif
#endif
static bool jsyReady = false;

// Set from WiFi events. AUTH_EXPIRE means the 4-way handshake failed — usually
// wrong password or WPA3-only AP. Waiting longer will not fix it.
static volatile uint8_t wifiLastDisconnectReason = 0;
static volatile bool wifiGotDisconnect = false;
static volatile bool wifiGotGotIp = false;

struct Load2Snapshot {
  float voltage = NAN;
  float current = NAN;
  float activePower = NAN;
  float apparentPower = NAN;
  float reactivePower = NAN;
  float powerFactor = NAN;
  float energyImported = NAN;
  float energyReturned = NAN;
  float frequency = NAN;
  bool connected = false;
  uint32_t ageMs = 0;
};

static Load2Snapshot captureLoad2() {
  Load2Snapshot s;
  Mycila::JSY::Data local;

  portENTER_CRITICAL(&dataMux);
  local = jsyData;
  const uint32_t stamp = lastReadMs;
  portEXIT_CRITICAL(&dataMux);

  const auto& ch = local.channel2();
  s.voltage = ch.voltage;
  s.current = ch.current;
  s.activePower = ch.activePower;
  s.apparentPower = ch.apparentPower;
  s.reactivePower = ch.reactivePower;
  s.powerFactor = ch.powerFactor;
  s.energyImported = ch.activeEnergyImported;
  s.energyReturned = ch.activeEnergyReturned;
  s.frequency = local.aggregate.frequency;
  s.connected = s.frequency > 0 && !std::isnan(s.activePower);
  s.ageMs = stamp ? (millis() - stamp) : UINT32_MAX;
  return s;
}

static void toJson(const Load2Snapshot& s, JsonDocument& doc) {
  doc["channel"] = ESS_CHANNEL;
  doc["connected"] = s.connected;
  doc["age_ms"] = s.ageMs;
  doc["frequency_hz"] = s.frequency;
  doc["voltage_v"] = s.voltage;
  doc["current_a"] = s.current;
  doc["power_w"] = s.activePower;
  doc["apparent_power_va"] = s.apparentPower;
  doc["reactive_power_var"] = s.reactivePower;
  doc["power_factor"] = s.powerFactor;
  doc["energy_imported_wh"] = s.energyImported;
  doc["energy_returned_wh"] = s.energyReturned;
  doc["ip"] = WiFi.isConnected() ? WiFi.localIP().toString() : "";
  // ESS convention helper: positive = import/consumption, negative = export/generation
  doc["ess_grid_power_w"] = s.activePower;
}

/**
 * ESS control hook — called every ESS_PUBLISH_INTERVAL_MS with fresh load2 power.
 * Plug your battery / inverter / relay logic here.
 *
 * @param powerW  channel2 active power (W). Sign depends on CT orientation:
 *                typically >0 import to loads, <0 export / reverse flow.
 */
static void onEssLoop(float powerW, const Load2Snapshot& snap) {
  (void)powerW;
  (void)snap;
  // Example skeleton:
  // if (powerW < -50)  { /* export: charge battery / divert */ }
  // if (powerW >  50)  { /* import: discharge battery */ }
}

#if ESS_UDP_ENABLE
static bool udpEnabled() {
  return ESS_UDP_HOST[0] != '\0';
}

static void udpSendPower(float powerW) {
  if (!udpEnabled() || !WiFi.isConnected() || std::isnan(powerW)) {
    udpFail++;
    return;
  }

  // ASCII: "<power_w> <seq> <millis>"
  char payload[48];
  const uint32_t seq = ++udpSeq;
  snprintf(payload,
           sizeof(payload),
           "%.3f %lu %lu",
           powerW,
           static_cast<unsigned long>(seq),
           static_cast<unsigned long>(millis()));

  if (!udp.beginPacket(ESS_UDP_HOST, ESS_UDP_PORT)) {
    udpFail++;
    return;
  }
  udp.write(reinterpret_cast<const uint8_t*>(payload), strlen(payload));
  if (!udp.endPacket()) {
    udpFail++;
    return;
  }
  udpOk++;
}
#endif // ESS_UDP_ENABLE

#if ESS_MQTT_ENABLE
static bool mqttEnabled() {
  return MQTT_HOST[0] != '\0';
}

static void mqttMakeClientId() {
  // Unique ID avoids broker kicking us when an old session or second flash shares the name.
  const uint64_t mac = ESP.getEfuseMac();
  snprintf(mqttClientId,
           sizeof(mqttClientId),
           "%s-%04x",
           MQTT_CLIENT_ID,
           static_cast<unsigned>((mac >> 32) & 0xffff));
}

static void mqttAvailTopic(char* out, size_t outLen) {
  snprintf(out, outLen, "%s/status", MQTT_TOPIC_ROOT);
}

static void mqttTeleTopic(char* out, size_t outLen) {
  snprintf(out, outLen, "%s/tele/SENSOR", MQTT_TOPIC_ROOT);
}

static void mqttPublishPower(float powerW) {
  if (!mqtt.connected() || std::isnan(powerW)) {
    mqttPublishFail++;
    return;
  }

  char topic[96];
  char payload[32];
  snprintf(topic, sizeof(topic), "%s/load2/power", MQTT_TOPIC_ROOT);
  dtostrf(powerW, 0, 3, payload);

  // qos=0, retain=false — ESS wants a live stream only.
  if (!mqtt.publish(topic, payload, false, 0)) {
    mqttPublishFail++;
    return;
  }
  mqttPublishOk++;
  mqtt.loop();
}

#if HA_MQTT_DISCOVERY
struct HaTeleSensor {
  const char* objectId;
  const char* name;
  const char* valueTemplate; // e.g. "{{ value_json.voltage_v }}"
  const char* unit;          // nullptr if none
  const char* deviceClass;   // nullptr if none
  const char* stateClass;
};

// All HA sensors read the same Tasmota-style tele/SENSOR JSON.
static const HaTeleSensor kHaTeleSensors[] = {
    {"power", "Power", "{{ value_json.power_w }}", "W", "power", "measurement"},
    {"voltage", "Voltage", "{{ value_json.voltage_v }}", "V", "voltage", "measurement"},
    {"current", "Current", "{{ value_json.current_a }}", "A", "current", "measurement"},
    {"power_factor", "Power Factor", "{{ value_json.power_factor }}", nullptr, "power_factor", "measurement"},
    {"frequency", "Frequency", "{{ value_json.frequency_hz }}", "Hz", "frequency", "measurement"},
    {"apparent_power", "Apparent Power", "{{ value_json.apparent_power_va }}", "VA", "apparent_power", "measurement"},
    {"reactive_power", "Reactive Power", "{{ value_json.reactive_power_var }}", "var", "reactive_power", "measurement"},
    {"energy_imported", "Energy Imported", "{{ value_json.energy_imported_wh }}", "Wh", "energy", "total_increasing"},
    {"energy_returned", "Energy Returned", "{{ value_json.energy_returned_wh }}", "Wh", "energy", "total_increasing"},
};

static void publishHaDiscovery() {
  if (!mqtt.connected()) {
    return;
  }

  char availTopic[96];
  char teleTopic[96];
  mqttAvailTopic(availTopic, sizeof(availTopic));
  mqttTeleTopic(teleTopic, sizeof(teleTopic));

  // Remove retained discovery left by earlier builds (fixed client id, per-metric topics).
  for (const HaTeleSensor& sensor : kHaTeleSensors) {
    char legacyTopic[160];
    snprintf(legacyTopic,
             sizeof(legacyTopic),
             "%s/sensor/%s/%s/config",
             HA_DISCOVERY_PREFIX,
             MQTT_CLIENT_ID,
             sensor.objectId);
    mqtt.publish(legacyTopic, "", true, 0);
    mqtt.loop();
  }

  for (const HaTeleSensor& sensor : kHaTeleSensors) {
    char configTopic[160];
    snprintf(configTopic,
             sizeof(configTopic),
             "%s/sensor/%s/%s/config",
             HA_DISCOVERY_PREFIX,
             mqttClientId,
             sensor.objectId);

    char uniqueId[96];
    snprintf(uniqueId, sizeof(uniqueId), "%s_%s", mqttClientId, sensor.objectId);

    JsonDocument doc;
    doc["name"] = sensor.name;
    doc["unique_id"] = uniqueId;
    doc["state_topic"] = teleTopic;
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
    doc["state_class"] = sensor.stateClass;

    JsonObject device = doc["device"].to<JsonObject>();
    device["identifiers"][0] = mqttClientId;
    device["name"] = HA_DEVICE_NAME;
    device["manufacturer"] = "JSY";
    device["model"] = "JSY-MK-194G";
    device["sw_version"] = "power_monitor";

    String payload;
    serializeJson(doc, payload);
    if (!mqtt.publish(configTopic, payload.c_str(), true, 0)) {
      Serial.printf("[ha] discovery failed: %s (len=%u)\n",
                    sensor.objectId,
                    static_cast<unsigned>(payload.length()));
    }
    mqtt.loop();
    delay(5);
  }
  haDiscoverySent = true;
  Serial.println("[ha] MQTT discovery published (tele/SENSOR)");
}

static void publishHaTele(const Load2Snapshot& s) {
  if (!mqtt.connected()) {
    return;
  }

  JsonDocument doc;
  toJson(s, doc);

  char teleTopic[96];
  mqttTeleTopic(teleTopic, sizeof(teleTopic));

  String payload;
  serializeJson(doc, payload);
  if (!mqtt.publish(teleTopic, payload.c_str(), true, 0)) {
    Serial.printf("[ha] tele publish failed (len=%u)\n", static_cast<unsigned>(payload.length()));
  }
  mqtt.loop();
}
#endif // HA_MQTT_DISCOVERY

static void ensureMqtt() {
  if (!mqttEnabled() || !WiFi.isConnected() || mqtt.connected()) {
    return;
  }
  if (millis() - lastMqttReconnectMs < 2000) {
    return;
  }
  lastMqttReconnectMs = millis();

  char availTopic[96];
  mqttAvailTopic(availTopic, sizeof(availTopic));
  mqtt.setWill(availTopic, "offline", true, 0);

  Serial.printf("[mqtt] connecting to %s:%d as %s ...\n", MQTT_HOST, MQTT_PORT, mqttClientId);
  bool ok;
  if (MQTT_USER[0] != '\0') {
    ok = mqtt.connect(mqttClientId, MQTT_USER, MQTT_PASSWORD);
  } else {
    ok = mqtt.connect(mqttClientId);
  }
  if (ok) {
    wifiClient.setNoDelay(true);
    mqtt.publish(availTopic, "online", true, 0);
    Serial.printf("[mqtt] connected (client=%s)\n", mqttClientId);
#if HA_MQTT_DISCOVERY
    haDiscoverySent = false;
    publishHaDiscovery();
    publishHaTele(captureLoad2());
    lastHaPublishMs = millis();
#endif
  } else {
    Serial.printf("[mqtt] failed\n");
  }
}
#endif // ESS_MQTT_ENABLE

static void publishEss(const Load2Snapshot& s) {
  onEssLoop(s.activePower, s);

  const uint32_t now = millis();
  const uint32_t dt = lastEssTickMs ? (now - lastEssTickMs) : 0;
  lastEssTickMs = now;

#if ESS_UDP_ENABLE
  udpSendPower(s.activePower);
#endif
#if ESS_MQTT_ENABLE
  mqttPublishPower(s.activePower);
#endif

  static uint32_t lastStatusMs = 0;
  if (now - lastStatusMs >= 2000) {
    lastStatusMs = now;
#if ESS_UDP_ENABLE && ESS_MQTT_ENABLE
    Serial.printf("[ess] P=%.3f W  dt=%lu ms  udp ok=%lu fail=%lu  mqtt=%s ok=%lu fail=%lu\n",
                  s.activePower,
                  static_cast<unsigned long>(dt),
                  static_cast<unsigned long>(udpOk),
                  static_cast<unsigned long>(udpFail),
                  mqtt.connected() ? "up" : "down",
                  static_cast<unsigned long>(mqttPublishOk),
                  static_cast<unsigned long>(mqttPublishFail));
#elif ESS_UDP_ENABLE
    Serial.printf("[ess] P=%.3f W  dt=%lu ms  udp ok=%lu fail=%lu\n",
                  s.activePower,
                  static_cast<unsigned long>(dt),
                  static_cast<unsigned long>(udpOk),
                  static_cast<unsigned long>(udpFail));
#elif ESS_MQTT_ENABLE
    Serial.printf("[ess] P=%.3f W  dt=%lu ms  mqtt=%s ok=%lu fail=%lu\n",
                  s.activePower,
                  static_cast<unsigned long>(dt),
                  mqtt.connected() ? "up" : "down",
                  static_cast<unsigned long>(mqttPublishOk),
                  static_cast<unsigned long>(mqttPublishFail));
#else
    Serial.printf("[ess] P=%.3f W  dt=%lu ms  age=%lu ms  reads=%lu\n",
                  s.activePower,
                  static_cast<unsigned long>(dt),
                  static_cast<unsigned long>(s.ageMs),
                  static_cast<unsigned long>(readCount));
#endif
  }
}

static void handleRoot() {
  const Load2Snapshot s = captureLoad2();
  char page[768];
  snprintf(page,
           sizeof(page),
           "<!DOCTYPE html><html><head><meta charset=utf-8>"
           "<meta http-equiv=refresh content=2>"
           "<title>JSY Load2</title></head><body>"
           "<h1>JSY-MK-194G Load2</h1>"
           "<p>Power: <b>%.1f W</b></p>"
           "<p>Voltage: %.1f V &nbsp; Current: %.3f A</p>"
           "<p>PF: %.3f &nbsp; Freq: %.2f Hz</p>"
           "<p>Connected: %s &nbsp; Age: %lu ms</p>"
           "<p><a href=/api/power>JSON API</a></p>"
           "</body></html>",
           s.activePower,
           s.voltage,
           s.current,
           s.powerFactor,
           s.frequency,
           s.connected ? "yes" : "no",
           static_cast<unsigned long>(s.ageMs));
  server.send(200, "text/html", page);
}

static void handleApiPower() {
  const Load2Snapshot s = captureLoad2();
  JsonDocument doc;
  toJson(s, doc);
  String payload;
  serializeJson(doc, payload);
  server.send(200, "application/json", payload);
}

static const char* wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return "IDLE";
    case WL_NO_SSID_AVAIL:
      return "NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:
      return "SCAN_COMPLETED";
    case WL_CONNECTED:
      return "CONNECTED";
    case WL_CONNECT_FAILED:
      return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

static const char* wifiReasonName(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_AUTH_EXPIRE:
      return "AUTH_EXPIRE (power droop on SuperMini, wrong password, or WPA3-only)";
    case WIFI_REASON_AUTH_FAIL:
      return "AUTH_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
      return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND:
      return "NO_AP_FOUND";
    case WIFI_REASON_ASSOC_FAIL:
      return "ASSOC_FAIL";
    case WIFI_REASON_CONNECTION_FAIL:
      return "CONNECTION_FAIL";
    case WIFI_REASON_BEACON_TIMEOUT:
      return "BEACON_TIMEOUT";
    case WIFI_REASON_AUTH_LEAVE:
      return "AUTH_LEAVE";
    case 36: // WIFI_REASON_STA_LEAVING on some cores
      return "STA_LEAVING";
    default:
      return "OTHER";
  }
}

// Stop this attempt (caller may retry after a pause / lower TX power).
static bool wifiReasonIsFatal(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_AUTH_EXPIRE:
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_NO_AP_FOUND:
    case WIFI_REASON_ASSOC_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
      return true;
    default:
      return false;
  }
}

static void wifiApplyRadioSettings() {
  WiFi.setSleep(false);
  // Disable modem PS — needed for a steady 250 ms MQTT stream.
  esp_wifi_set_ps(WIFI_PS_NONE);
  // SuperMini: onboard regulator often can't sustain default TX peaks.
  if (!WiFi.setTxPower(WIFI_TX_POWER)) {
    Serial.println("[wifi] setTxPower failed");
  } else {
    Serial.printf("[wifi] tx power set to enum=%d\n", static_cast<int>(WIFI_TX_POWER));
  }
}

static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifiGotGotIp = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      wifiGotDisconnect = true;
      wifiLastDisconnectReason = info.wifi_sta_disconnected.reason;
      break;
    default:
      break;
  }
}

static void scanWifiForTarget() {
  Serial.println("[wifi] scanning...");
  const int n = WiFi.scanNetworks(/*async=*/false, /*show_hidden=*/true);
  if (n <= 0) {
    Serial.println("[wifi] scan found 0 APs — check 2.4 GHz radio / antenna");
    return;
  }

  bool found = false;
  for (int i = 0; i < n; i++) {
    const String ssid = WiFi.SSID(i);
    const bool match = ssid == WIFI_SSID;
    if (match) {
      found = true;
    }
    if (match || i < 5) {
      Serial.printf("[wifi] %c %-24s ch=%2d rssi=%4d enc=%d\n",
                    match ? '*' : ' ',
                    ssid.c_str(),
                    WiFi.channel(i),
                    WiFi.RSSI(i),
                    static_cast<int>(WiFi.encryptionType(i)));
    }
  }
  WiFi.scanDelete();

  if (!found) {
    Serial.printf("[wifi] SSID '%s' not seen. Need 2.4 GHz (not 5 GHz-only).\n", WIFI_SSID);
  }
}

static void wifiStopClean() {
  // Arduino-ESP32 3.x: disconnect(wifioff, eraseap, timeout).
  // Do NOT pass disconnect(true, ...) — that turns the radio off (STA.end)
  // and the next call fails with ESP_ERR_WIFI_NOT_INIT.
  if (WiFi.getMode() == WIFI_OFF) {
    WiFi.mode(WIFI_STA);
    delay(50);
  }
  WiFi.disconnect(false /*wifioff*/, false /*eraseap*/, 500);
  delay(50);
}

static bool connectWifiOnce(uint32_t timeoutMs) {
  wifiGotDisconnect = false;
  wifiGotGotIp = false;
  wifiLastDisconnectReason = 0;

  if (WiFi.getMode() != WIFI_STA) {
    WiFi.mode(WIFI_STA);
    delay(50);
  }
  wifiStopClean();
  wifiApplyRadioSettings();
  WiFi.setHostname("jsy-ess-load2");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("[wifi] connecting to '%s' (pass len=%u)\n",
                WIFI_SSID,
                static_cast<unsigned>(strlen(WIFI_PASSWORD)));

  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (wifiGotGotIp || WiFi.status() == WL_CONNECTED) {
      Serial.printf("[wifi] OK ip=%s rssi=%d dBm\n",
                    WiFi.localIP().toString().c_str(),
                    WiFi.RSSI());
      return true;
    }

    if (wifiGotDisconnect) {
      const uint8_t reason = wifiLastDisconnectReason;
      wifiGotDisconnect = false;
      Serial.printf("[wifi] disconnect reason %u — %s\n", reason, wifiReasonName(reason));
      if (wifiReasonIsFatal(reason)) {
        // ESP would keep retrying AUTH_EXPIRE every ~1s; stop it ourselves.
        wifiStopClean();
        return false;
      }
    }
    delay(50);
  }

  Serial.printf("[wifi] timed out after %lu ms (status=%s)\n",
                static_cast<unsigned long>(timeoutMs),
                wifiStatusName(WiFi.status()));
  wifiStopClean();
  return false;
}

static void connectWifi() {
  if (WIFI_SSID[0] == '\0' || strcmp(WIFI_SSID, "your-wifi-ssid") == 0) {
    Serial.println("[wifi] SSID not configured — edit include/secrets.h");
    return;
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);
  delay(50);
  wifiApplyRadioSettings();

  scanWifiForTarget();

  for (int attempt = 1; attempt <= WIFI_BOOT_ATTEMPTS; attempt++) {
    Serial.printf("[wifi] attempt %d/%d\n", attempt, WIFI_BOOT_ATTEMPTS);
    if (connectWifiOnce(WIFI_CONNECT_TIMEOUT_MS)) {
      return;
    }
    // Brief pause lets the SuperMini LDO recover after a TX brownout.
    delay(500 * attempt);
  }

  Serial.println("[wifi] still down. On SuperMini, AUTH_EXPIRE is often power:");
  Serial.println("  - use a solid USB data port / short cable (not a weak hub)");
  Serial.println("  - add ~100uF on 3V3 near the board");
  Serial.println("  - don't feed JSY + ESP from a marginal 5V supply");
  Serial.println("  - also verify password / 2.4GHz / WPA2");
}

static bool startJsy() {
  jsy.setCallback([](Mycila::JSY::EventType event, const Mycila::JSY::Data& data) {
    if (event != Mycila::JSY::EventType::EVT_READ) {
      return;
    }
    portENTER_CRITICAL(&dataMux);
    jsyData = data;
    lastReadMs = millis();
    readCount++;
    portEXIT_CRITICAL(&dataMux);
  });

  // Detect baud/model, then run async reads on core 0 (C3 is single-core).
  Serial.printf("[jsy] begin RX=%d TX=%d (async)\n", JSY_RX_PIN, JSY_TX_PIN);
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
    Serial.println("[jsy] failed to start — check wiring / 5V / TX-RX swap");
    return false;
  }

  Serial.printf("[jsy] model=%s baud=%" PRIu32 " lastAddr=0x%02X\n",
                jsy.getModelName(),
                static_cast<uint32_t>(jsy.getBaudRate()),
                jsy.getLastAddress());

#if JSY_TARGET_BAUD == 38400
  if (jsy.getBaudRate() != Mycila::JSY::BaudRate::BAUD_38400) {
    Serial.println("[jsy] switching to 38400 baud for ESS reactivity");
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
      Serial.printf("[jsy] now baud=%" PRIu32 "\n", static_cast<uint32_t>(jsy.getBaudRate()));
    } else {
      Serial.println("[jsy] baud change failed — restarting async on detected rate");
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

  Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  // If no serial monitor is attached, blocking USB TX stalls the ESS loop.
  Serial.setTxTimeoutMs(0);
#endif
  delay(800);
  Serial.println();
  Serial.println("=== power_monitor: ESP32-C3 + JSY-MK-194G load2 / ESS ===");
  Serial.printf("[ess] tick every %d ms (udp=%s mqtt=%s)\n",
                ESS_PUBLISH_INTERVAL_MS,
                ESS_UDP_ENABLE ? "on" : "off",
                ESS_MQTT_ENABLE ? "on" : "off");

  connectWifi();

#if ESS_UDP_ENABLE
  if (udpEnabled()) {
    // Local ephemeral port; we only send.
    udp.begin(0);
    Serial.printf("[udp] ESS target %s:%d\n", ESS_UDP_HOST, ESS_UDP_PORT);
  }
#endif

#if ESS_MQTT_ENABLE
  if (mqttEnabled()) {
    mqttMakeClientId();
    // keepalive 10s, clean session, 1s socket timeout
    mqtt.setOptions(10, true, 1000);
    mqtt.begin(MQTT_HOST, MQTT_PORT, wifiClient);
    wifiClient.setNoDelay(true);
  }
#endif

  server.on("/", handleRoot);
  server.on("/api/power", handleApiPower);
  server.begin();
  Serial.printf("[http] http://%s/api/power\n",
                WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "0.0.0.0");

  jsyReady = startJsy();
  digitalWrite(STATUS_LED_PIN, jsyReady ? HIGH : LOW);
  lastPublishMs = millis();
}

void loop() {
  // Start countdown after boot connect attempt so we don't immediately retry.
  static uint32_t lastWifiTry = millis();
  const wl_status_t wifiSt = WiFi.status();

  if (wifiSt != WL_CONNECTED) {
    if (millis() - lastWifiTry > WIFI_RETRY_INTERVAL_MS) {
      lastWifiTry = millis();
      Serial.println("[wifi] background retry");
      connectWifiOnce(WIFI_CONNECT_TIMEOUT_MS);
    }
  } else {
#if ESS_MQTT_ENABLE
    ensureMqtt();
#endif
  }

#if ESS_MQTT_ENABLE
  if (mqtt.connected()) {
    mqtt.loop();
  }
#endif

  server.handleClient();

  const uint32_t now = millis();

  if (now - lastPublishMs >= ESS_PUBLISH_INTERVAL_MS) {
    // Advance by the interval (not wall-clock) so average rate stays accurate
    // even when a publish takes a few ms.
    lastPublishMs += ESS_PUBLISH_INTERVAL_MS;
    if (now - lastPublishMs >= ESS_PUBLISH_INTERVAL_MS) {
      lastPublishMs = now; // catch up after a long stall
    }
    const Load2Snapshot s = captureLoad2();
    publishEss(s);
#if ESS_MQTT_ENABLE
    if (mqtt.connected()) {
      mqtt.loop();
    }
#endif
    digitalWrite(STATUS_LED_PIN, s.connected ? HIGH : LOW);
  }

#if ESS_MQTT_ENABLE && HA_MQTT_DISCOVERY
  if (mqtt.connected() && (now - lastHaPublishMs >= HA_PUBLISH_INTERVAL_MS)) {
    lastHaPublishMs += HA_PUBLISH_INTERVAL_MS;
    if (now - lastHaPublishMs >= HA_PUBLISH_INTERVAL_MS) {
      lastHaPublishMs = now;
    }
    publishHaTele(captureLoad2());
  }
#endif

  delay(1);
}
