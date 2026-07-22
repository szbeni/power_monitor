#include <Arduino.h>
#include <ArduinoJson.h>
#include <MycilaJSY.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <cmath>
#include <cstring>

#include "config.h"

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

static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);
static WebServer server(HTTP_PORT);

static uint32_t lastPublishMs = 0;
static uint32_t lastSerialMs = 0;
static uint32_t lastMqttReconnectMs = 0;
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

static bool mqttEnabled() {
  return MQTT_HOST[0] != '\0';
}

static void mqttPublishFloat(const char* suffix, float value) {
  if (!mqtt.connected() || std::isnan(value)) {
    return;
  }
  char topic[96];
  char payload[32];
  snprintf(topic, sizeof(topic), "%s/load2/%s", MQTT_TOPIC_ROOT, suffix);
  dtostrf(value, 0, 3, payload);
  mqtt.publish(topic, payload, true);
}

static void publishEss(const Load2Snapshot& s) {
  onEssLoop(s.activePower, s);

  if (mqtt.connected() && s.connected) {
    mqttPublishFloat("power", s.activePower);
    mqttPublishFloat("voltage", s.voltage);
    mqttPublishFloat("current", s.current);
    mqttPublishFloat("power_factor", s.powerFactor);
    mqttPublishFloat("frequency", s.frequency);

    JsonDocument doc;
    toJson(s, doc);
    char topic[96];
    snprintf(topic, sizeof(topic), "%s/load2/json", MQTT_TOPIC_ROOT);
    String payload;
    serializeJson(doc, payload);
    mqtt.publish(topic, payload.c_str(), true);
  }
}

static void ensureMqtt() {
  if (!mqttEnabled() || !WiFi.isConnected() || mqtt.connected()) {
    return;
  }
  if (millis() - lastMqttReconnectMs < 3000) {
    return;
  }
  lastMqttReconnectMs = millis();

  Serial.printf("[mqtt] connecting to %s:%d ...\n", MQTT_HOST, MQTT_PORT);
  bool ok;
  if (MQTT_USER[0] != '\0') {
    ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
  } else {
    ok = mqtt.connect(MQTT_CLIENT_ID);
  }
  if (ok) {
    Serial.println("[mqtt] connected");
    char topic[96];
    snprintf(topic, sizeof(topic), "%s/status", MQTT_TOPIC_ROOT);
    mqtt.publish(topic, "online", true);
  } else {
    Serial.printf("[mqtt] failed, rc=%d\n", mqtt.state());
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
  delay(800);
  Serial.println();
  Serial.println("=== power_monitor: ESP32-C3 + JSY-MK-194G load2 / ESS ===");

  connectWifi();

  if (mqttEnabled()) {
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setBufferSize(512);
  }

  server.on("/", handleRoot);
  server.on("/api/power", handleApiPower);
  server.begin();
  Serial.printf("[http] http://%s/api/power\n",
                WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "0.0.0.0");

  jsyReady = startJsy();
  digitalWrite(STATUS_LED_PIN, jsyReady ? HIGH : LOW);
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
    ensureMqtt();
    mqtt.loop();
  }

  server.handleClient();

  const uint32_t now = millis();

  if (now - lastPublishMs >= ESS_PUBLISH_INTERVAL_MS) {
    lastPublishMs = now;
    const Load2Snapshot s = captureLoad2();
    publishEss(s);
    digitalWrite(STATUS_LED_PIN, s.connected ? HIGH : LOW);
  }

#if SERIAL_STATUS_INTERVAL_MS > 0
  if (now - lastSerialMs >= SERIAL_STATUS_INTERVAL_MS) {
    lastSerialMs = now;
    const Load2Snapshot s = captureLoad2();
    Serial.printf("[load2] P=%.1f W  V=%.1f V  I=%.3f A  PF=%.3f  f=%.2f Hz  age=%lums  reads=%lu  mqtt=%s\n",
                  s.activePower,
                  s.voltage,
                  s.current,
                  s.powerFactor,
                  s.frequency,
                  static_cast<unsigned long>(s.ageMs),
                  static_cast<unsigned long>(readCount),
                  mqtt.connected() ? "up" : "down");
  }
#endif

  delay(1);
}
