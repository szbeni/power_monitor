#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "secrets.h"

extern "C" {
#include "hlw811x.h"
#include "hlw811x_overrides.h"
}


// ============================================================
// CONFIGURATION
// ============================================================

// Device
#define DEVICE_ID       "hlw8112_01"
#define DEVICE_NAME     "HLW8112 Power Meter"


// OTA
#define OTA_HOSTNAME    "hlw8112-meter"


// ============================================================
// TIMING
// ============================================================

// Power advertised every 500 ms
static constexpr uint32_t POWER_INTERVAL_MS = 500;

// Full measurement interval
// Change this to 5000 for 5 seconds, for example.
static uint32_t FULL_INTERVAL_MS = 10000;


// Measurement polling
static constexpr uint32_t MEASUREMENT_INTERVAL_MS = 250;

// Backoff between MQTT connection attempts
static constexpr uint32_t MQTT_RETRY_INTERVAL_MS = 5000;


// ============================================================
// SPI PINS
// ============================================================

static constexpr int HLW_MOSI = 3;   // -> SDI
static constexpr int HLW_MISO = 5;   // <- SDO
static constexpr int HLW_SCK  = 1;   // -> SCLK
static constexpr int HLW_CS   = 7;   // -> SCSN


// SCLK must not exceed MCLK/4 (3.579545 MHz / 4 = 895 kHz).
static constexpr uint32_t HLW_SPI_HZ = 800000;

// The chip drives SDO on the rising edge and samples SDI on the falling
// edge, so the host shifts out on the leading edge: CPOL=0, CPHA=1.
static constexpr uint8_t HLW_SPI_MODE = SPI_MODE1;

#define WIFI_TX_POWER WIFI_POWER_11dBm


// ============================================================
// SPI
// ============================================================

SPIClass hlwSPI(FSPI);


// ============================================================
// MQTT
// ============================================================

WiFiClient wifiClient;

PubSubClient mqtt(wifiClient);


// ============================================================
// HLW8112 CONTEXT
// ============================================================

struct HLW_SPI_Context
{
    SPIClass* spi;
    int cs;
    uint32_t speed;

    // True while SCSN is held low waiting for the data phase of a read.
    bool frameOpen;
};

static HLW_SPI_Context hlwContext =
{
    &hlwSPI,
    HLW_CS,
    HLW_SPI_HZ,
    false
};


static struct hlw811x* meter = nullptr;


// ============================================================
// MEASUREMENT DATA
// ============================================================

struct Measurement
{
    float voltage;
    float current;
    float power;
    float energy;
    float frequency;
    float powerFactor;
    float phaseAngle;

    bool valid;
};

static Measurement measurement =
{
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    false
};


// ============================================================
// SPI FRAMING
// ============================================================
//
// The HLW8112 treats everything clocked while SCSN is low as one
// transaction, and resets its SPI block when SCSN rises. A register read is
// therefore a single frame: command byte out, then data bytes in. SCSN must
// stay asserted across both halves.
//

static void beginFrame(HLW_SPI_Context* c)
{
    if (c->frameOpen)
        return;

    c->spi->beginTransaction(
        SPISettings(
            c->speed,
            MSBFIRST,
            HLW_SPI_MODE
        )
    );

    digitalWrite(c->cs, LOW);

    c->frameOpen = true;
}


static void endFrame(HLW_SPI_Context* c)
{
    if (!c->frameOpen)
        return;

    digitalWrite(c->cs, HIGH);

    c->spi->endTransaction();

    c->frameOpen = false;
}


// ============================================================
// LOW LEVEL SPI WRITE
// ============================================================
//
// This function is called by libmcu/hlw811x.c
//

extern "C"
int hlw811x_ll_write(
    const uint8_t* data,
    size_t datalen,
    void* ctx
)
{
    if (!data || datalen == 0 || !ctx)
        return -1;

    HLW_SPI_Context* c =
        static_cast<HLW_SPI_Context*>(ctx);


    // Bit 7 of the command byte selects the direction. A read command is
    // only half a frame, so leave SCSN low for hlw811x_ll_read().
    bool isRead = (data[0] & 0x80u) == 0;


    // Discard any frame a previous read left open.
    endFrame(c);

    beginFrame(c);

    for (size_t i = 0; i < datalen; i++)
    {
        c->spi->transfer(data[i]);
    }

    if (!isRead)
    {
        endFrame(c);
    }

    return static_cast<int>(datalen);
}


// ============================================================
// LOW LEVEL SPI READ
// ============================================================
//
// The library sends the register address first.
// It then calls this function to obtain the response.
//

extern "C"
int hlw811x_ll_read(
    uint8_t* buf,
    size_t bufsize,
    void* ctx
)
{
    if (!buf || bufsize == 0 || !ctx)
        return -1;

    HLW_SPI_Context* c =
        static_cast<HLW_SPI_Context*>(ctx);


    // Without a preceding read command there is nothing to clock out.
    if (!c->frameOpen)
        return -1;

    for (size_t i = 0; i < bufsize; i++)
    {
        buf[i] = c->spi->transfer(0x00);
    }

    endFrame(c);

    return static_cast<int>(bufsize);
}


// ============================================================
// MQTT TOPIC
// ============================================================

String topic(const char* name)
{
    return String("homeassistant/") +
           DEVICE_ID +
           "/" +
           name;
}


// ============================================================
// MQTT PUBLISH
// ============================================================

void publishValue(
    const char* name,
    const String& value,
    bool retained = true
)
{
    String t = topic(name);

    mqtt.publish(
        t.c_str(),
        value.c_str(),
        retained
    );
}


// ============================================================
// HOME ASSISTANT DISCOVERY
// ============================================================

void publishSensorDiscovery(
    const char* sensorId,
    const char* name,
    const char* stateTopic,
    const char* unit,
    const char* deviceClass,
    const char* stateClass
)
{
    String configTopic =
        String("homeassistant/sensor/") +
        DEVICE_ID +
        "/" +
        sensorId +
        "/config";


    String payload;

    payload.reserve(700);

    payload += "{";

    payload += "\"name\":\"";
    payload += name;
    payload += "\",";

    payload += "\"unique_id\":\"";
    payload += DEVICE_ID;
    payload += "_";
    payload += sensorId;
    payload += "\",";

    payload += "\"state_topic\":\"";
    payload += stateTopic;
    payload += "\",";

    payload += "\"availability_topic\":\"";
    payload += topic("status");
    payload += "\",";

    if (strlen(unit) > 0)
    {
        payload += "\"unit_of_measurement\":\"";
        payload += unit;
        payload += "\",";
    }

    if (strlen(deviceClass) > 0)
    {
        payload += "\"device_class\":\"";
        payload += deviceClass;
        payload += "\",";
    }

    if (strlen(stateClass) > 0)
    {
        payload += "\"state_class\":\"";
        payload += stateClass;
        payload += "\",";
    }

    payload += "\"device\":{";

    payload += "\"identifiers\":[\"";
    payload += DEVICE_ID;
    payload += "\"],";

    payload += "\"name\":\"";
    payload += DEVICE_NAME;
    payload += "\",";

    payload += "\"manufacturer\":\"libmcu / HLW8112\",";

    payload += "\"model\":\"HLW8112 ESP32-C3\"";

    payload += "}";

    payload += "}";


    mqtt.publish(
        configTopic.c_str(),
        payload.c_str(),
        true
    );
}


// ============================================================
// MQTT DISCOVERY
// ============================================================

void publishDiscovery()
{
    publishSensorDiscovery(
        "voltage",
        "Voltage",
        topic("voltage").c_str(),
        "V",
        "voltage",
        "measurement"
    );


    publishSensorDiscovery(
        "current",
        "Current",
        topic("current").c_str(),
        "A",
        "current",
        "measurement"
    );


    publishSensorDiscovery(
        "power",
        "Power",
        topic("power").c_str(),
        "W",
        "power",
        "measurement"
    );


    publishSensorDiscovery(
        "energy",
        "Energy",
        topic("energy").c_str(),
        "kWh",
        "energy",
        "total_increasing"
    );


    publishSensorDiscovery(
        "frequency",
        "Frequency",
        topic("frequency").c_str(),
        "Hz",
        "frequency",
        "measurement"
    );


    publishSensorDiscovery(
        "power_factor",
        "Power Factor",
        topic("power_factor").c_str(),
        "",
        "power_factor",
        "measurement"
    );


    publishSensorDiscovery(
        "phase_angle",
        "Phase Angle",
        topic("phase_angle").c_str(),
        "°",
        "",
        "measurement"
    );


    mqtt.publish(
        topic("status").c_str(),
        "online",
        true
    );
}


// ============================================================
// READ HLW8112
// ============================================================

bool readMeter()
{
    if (!meter)
        return false;


    int32_t voltage_mV = 0;
    int32_t current_mA = 0;
    int32_t power_mW = 0;

    int32_t energy_Wh = 0;
    int32_t frequency_cHz = 0;
    int32_t pf_centi = 0;
    int32_t phase_centi = 0;


    hlw811x_error_t err;


    // Voltage
    err = hlw811x_get_rms(
        meter,
        HLW811X_CHANNEL_U,
        &voltage_mV
    );

    if (err != HLW811X_ERROR_NONE)
    {
        Serial.printf(
            "Voltage read error: %d\n",
            err
        );

        return false;
    }


    // Current
    err = hlw811x_get_rms(
        meter,
        HLW811X_CHANNEL_A,
        &current_mA
    );

    if (err != HLW811X_ERROR_NONE)
    {
        Serial.printf(
            "Current read error: %d\n",
            err
        );

        return false;
    }


    // Active power
    err = hlw811x_get_power(
        meter,
        HLW811X_CHANNEL_A,
        &power_mW
    );

    if (err != HLW811X_ERROR_NONE)
    {
        Serial.printf(
            "Power read error: %d\n",
            err
        );

        return false;
    }


    // Energy
    err = hlw811x_get_energy(
        meter,
        HLW811X_CHANNEL_A,
        &energy_Wh
    );

    if (err != HLW811X_ERROR_NONE)
    {
        Serial.printf(
            "Energy read error: %d\n",
            err
        );

        return false;
    }


    // Frequency
    err = hlw811x_get_frequency(
        meter,
        &frequency_cHz
    );

    if (err != HLW811X_ERROR_NONE)
    {
        Serial.printf(
            "Frequency read error: %d\n",
            err
        );

        // Don't fail the entire measurement
        frequency_cHz = 0;
    }


    // Power factor
    err = hlw811x_get_power_factor(
        meter,
        &pf_centi
    );

    if (err != HLW811X_ERROR_NONE)
    {
        Serial.printf(
            "PF read error: %d\n",
            err
        );

        pf_centi = 0;
    }


    // Phase angle
    err = hlw811x_get_phase_angle(
        meter,
        &phase_centi,
        HLW811X_LINE_FREQ_50HZ
    );

    if (err != HLW811X_ERROR_NONE)
    {
        phase_centi = 0;
    }


    // Convert library units
    measurement.voltage =
        voltage_mV / 1000.0f;

    measurement.current =
        current_mA / 1000.0f;

    measurement.power =
        power_mW / 1000.0f;

    measurement.energy =
        energy_Wh / 1000.0f;

    measurement.frequency =
        frequency_cHz / 100.0f;

    measurement.powerFactor =
        pf_centi / 100.0f;

    measurement.phaseAngle =
        phase_centi / 100.0f;

    measurement.valid = true;


    return true;
}


// ============================================================
// PRINT MEASUREMENT
// ============================================================

void printMeasurement()
{
    Serial.printf(
        "V=%.2f V  "
        "I=%.3f A  "
        "P=%.2f W  "
        "E=%.3f kWh  "
        "F=%.2f Hz  "
        "PF=%.2f %%  "
        "Angle=%.2f deg\n",

        measurement.voltage,
        measurement.current,
        measurement.power,
        measurement.energy,
        measurement.frequency,
        measurement.powerFactor,
        measurement.phaseAngle
    );
}


// ============================================================
// POWER MQTT
// ============================================================

void publishPower()
{
    if (!measurement.valid)
        return;

    publishValue(
        "power",
        String(measurement.power, 2),
        false
    );
}


// ============================================================
// FULL MQTT
// ============================================================

void publishFull()
{
    if (!measurement.valid)
        return;


    publishValue(
        "voltage",
        String(measurement.voltage, 2)
    );


    publishValue(
        "current",
        String(measurement.current, 3)
    );


    publishValue(
        "power",
        String(measurement.power, 2),
        false
    );


    publishValue(
        "energy",
        String(measurement.energy, 3)
    );


    publishValue(
        "frequency",
        String(measurement.frequency, 2)
    );


    publishValue(
        "power_factor",
        String(measurement.powerFactor, 2)
    );


    publishValue(
        "phase_angle",
        String(measurement.phaseAngle, 2)
    );
}


// ============================================================
// WIFI
// ============================================================

void connectWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return;


    Serial.println(
        "Connecting WiFi..."
    );


    WiFi.mode(WIFI_STA);

    WiFi.setSleep(false);

    if (!WiFi.setTxPower(WIFI_TX_POWER)) {
        Serial.println("[wifi] setTxPower failed");
    }

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );


    uint32_t start = millis();


    while (
        WiFi.status() != WL_CONNECTED &&
        millis() - start < 15000
    )
    {
        delay(250);

        Serial.print(".");
    }


    Serial.println();


    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print(
            "WiFi IP: "
        );

        Serial.println(
            WiFi.localIP()
        );
    }
    else
    {
        Serial.println(
            "WiFi connection failed"
        );
    }
}


// ============================================================
// MQTT CONNECT
// ============================================================

void connectMQTT()
{
    if (
        mqtt.connected() ||
        WiFi.status() != WL_CONNECTED
    )
    {
        return;
    }


    Serial.println(
        "Connecting MQTT..."
    );


    String clientId =
        String(DEVICE_ID) +
        "_" +
        String(
            (uint32_t)
            ESP.getEfuseMac(),
            HEX
        );


    // Retained "offline" last will so Home Assistant marks the entities
    // unavailable if this node drops off.
    if (
        mqtt.connect(
            clientId.c_str(),
            MQTT_USER,
            MQTT_PASSWORD,
            topic("status").c_str(),
            0,
            true,
            "offline"
        )
    )
    {
        Serial.println(
            "MQTT connected"
        );


        publishDiscovery();
    }
    else
    {
        Serial.printf(
            "MQTT failed: %d\n",
            mqtt.state()
        );
    }
}


// ============================================================
// OTA
// ============================================================

void setupOTA()
{
    // Reached again after every WiFi reconnect, but ArduinoOTA must only be
    // started once.
    static bool started = false;

    if (started)
        return;

    started = true;


    ArduinoOTA.setHostname(
        OTA_HOSTNAME
    );


    if (strlen(OTA_PASSWORD) > 0)
    {
        ArduinoOTA.setPassword(
            OTA_PASSWORD
        );
    }


    ArduinoOTA.onStart(
        []()
        {
            Serial.println(
                "OTA started"
            );
        }
    );


    ArduinoOTA.onEnd(
        []()
        {
            Serial.println(
                "\nOTA complete"
            );
        }
    );


    ArduinoOTA.onProgress(
        [](unsigned int progress,
           unsigned int total)
        {
            Serial.printf(
                "OTA %u%%\r",
                (progress * 100) / total
            );
        }
    );


    ArduinoOTA.onError(
        [](ota_error_t error)
        {
            Serial.printf(
                "OTA error: %u\n",
                error
            );
        }
    );


    ArduinoOTA.begin();


    Serial.print(
        "OTA hostname: "
    );

    Serial.println(
        OTA_HOSTNAME
    );
}


// ============================================================
// HLW8112 SETUP
// ============================================================

bool setupHLW8112()
{
    Serial.println(
        "Initialising HLW8112..."
    );


    pinMode(
        HLW_CS,
        OUTPUT
    );

    digitalWrite(
        HLW_CS,
        HIGH
    );


    // CS is driven manually in beginFrame()/endFrame(), so keep the SPI
    // driver from claiming it as a hardware SS pin.
    hlwSPI.begin(
        HLW_SCK,
        HLW_MISO,
        HLW_MOSI,
        -1
    );


    delay(20);


    meter =
        hlw811x_create(
            HLW811X_SPI,
            &hlwContext
        );


    if (!meter)
    {
        Serial.println(
            "ERROR: hlw811x_create() failed"
        );

        return false;
    }


    // HLW8112 requires at least 60 ms after reset.
    hlw811x_error_t err =
        hlw811x_reset(meter);


    Serial.printf(
        "HLW reset result: %d\n",
        err
    );


    delay(100);


    // --------------------------------------------------------
    // Read factory calibration
    // --------------------------------------------------------

    struct hlw811x_coeff coeff;


    err =
        hlw811x_read_coeff(
            meter,
            &coeff
        );


    Serial.printf(
        "Read coefficients: %d\n",
        err
    );


    if (err != HLW811X_ERROR_NONE)
    {
        Serial.println(
            "Could not read HLW8112 coefficients"
        );
    }


    // --------------------------------------------------------
    // IMPORTANT:
    //
    // These ratios depend on YOUR hardware.
    //
    // The defaults below are deliberately conservative.
    //
    // They must be calibrated for your voltage divider and
    // current shunt/CT.
    // --------------------------------------------------------

    struct hlw811x_resistor_ratio ratio =
    {
        .K1_A = 1.0f,
        .K1_B = 1.0f,
        .K2   = 1.0f
    };


    hlw811x_set_resistor_ratio(
        meter,
        &ratio
    );


    // --------------------------------------------------------
    // PGA
    // --------------------------------------------------------

    struct hlw811x_pga pga =
    {
        .A = HLW811X_PGA_GAIN_16,
        .B = HLW811X_PGA_GAIN_2,
        .U = HLW811X_PGA_GAIN_1
    };


    err =
        hlw811x_set_pga(
            meter,
            &pga
        );


    Serial.printf(
        "Set PGA: %d\n",
        err
    );


    // --------------------------------------------------------
    // Normal B channel
    // --------------------------------------------------------

    err =
        hlw811x_set_channel_b_mode(
            meter,
            HLW811X_B_MODE_NORMAL
        );


    Serial.printf(
        "B channel mode: %d\n",
        err
    );


    // --------------------------------------------------------
    // Active power
    // --------------------------------------------------------

    err =
        hlw811x_set_active_power_calc_mode(
            meter,
            HLW811X_ACTIVE_POWER_MODE_POS_NEG_ALGEBRAIC
        );


    Serial.printf(
        "Power mode: %d\n",
        err
    );


    // --------------------------------------------------------
    // AC RMS
    // --------------------------------------------------------

    err =
        hlw811x_set_rms_calc_mode(
            meter,
            HLW811X_RMS_MODE_AC
        );


    Serial.printf(
        "RMS mode: %d\n",
        err
    );


    // --------------------------------------------------------
    // Update frequency
    // --------------------------------------------------------

    err =
        hlw811x_set_data_update_frequency(
            meter,
            HLW811X_DATA_UPDATE_FREQ_HZ_3_4
        );


    Serial.printf(
        "Update frequency: %d\n",
        err
    );


    // --------------------------------------------------------
    // Zero crossing
    // --------------------------------------------------------

    err =
        hlw811x_set_zerocrossing_mode(
            meter,
            HLW811x_ZERO_CROSSING_MODE_POSITIVE
        );


    Serial.printf(
        "Zero crossing mode: %d\n",
        err
    );


    err =
        hlw811x_enable_waveform(
            meter
        );


    Serial.printf(
        "Waveform: %d\n",
        err
    );


    err =
        hlw811x_enable_zerocrossing(
            meter
        );


    Serial.printf(
        "Zero crossing: %d\n",
        err
    );


    // --------------------------------------------------------
    // Power factor
    // --------------------------------------------------------

    err =
        hlw811x_enable_power_factor(
            meter
        );


    Serial.printf(
        "Power factor: %d\n",
        err
    );


    // --------------------------------------------------------
    // Enable channels
    // --------------------------------------------------------

    err =
        hlw811x_enable_channel(
            meter,
            HLW811X_CHANNEL_ALL
        );


    Serial.printf(
        "Channels enabled: %d\n",
        err
    );


    // --------------------------------------------------------
    // Energy / pulse accumulation
    // --------------------------------------------------------

    err =
        hlw811x_enable_pulse(
            meter,
            HLW811X_CHANNEL_A
        );


    Serial.printf(
        "Energy accumulation: %d\n",
        err
    );


    // --------------------------------------------------------
    // Select channel A
    // --------------------------------------------------------

    err =
        hlw811x_select_channel(
            meter,
            HLW811X_CHANNEL_A
        );


    Serial.printf(
        "Channel A selected: %d\n",
        err
    );


    Serial.println(
        "HLW8112 initialisation complete"
    );


    return true;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);


    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "ESP32-C3 HLW8112 Power Meter"
    );
    Serial.println(
        "libmcu/hlw811x + SPI"
    );
    Serial.println(
        "================================"
    );


    Serial.println(
        "SPI pins:"
    );

    Serial.printf(
        "MOSI=%d MISO=%d SCK=%d CS=%d\n",
        HLW_MOSI,
        HLW_MISO,
        HLW_SCK,
        HLW_CS
    );


    // HLW8112
    if (!setupHLW8112())
    {
        Serial.println(
            "HLW8112 initialisation failed"
        );
    }


    // WiFi
    connectWiFi();


    // MQTT
    mqtt.setServer(
        MQTT_HOST,
        MQTT_PORT
    );

    // The discovery payloads are far larger than the 256 byte default.
    mqtt.setBufferSize(1024);


    // OTA
    if (
        WiFi.status() ==
        WL_CONNECTED
    )
    {
        setupOTA();
    }


    // Initial reading
    delay(1000);

    readMeter();

    printMeasurement();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    static uint32_t lastMeasurement = 0;
    static uint32_t lastPower = 0;
    static uint32_t lastFull = 0;

    uint32_t now = millis();


    // --------------------------------------------------------
    // WiFi
    // --------------------------------------------------------

    if (
        WiFi.status() != WL_CONNECTED
    )
    {
        connectWiFi();

        if (
            WiFi.status() ==
            WL_CONNECTED
        )
        {
            setupOTA();
        }
    }


    // --------------------------------------------------------
    // MQTT
    // --------------------------------------------------------

    // Each failed connect blocks for the socket timeout, so back off instead
    // of retrying on every pass.
    static uint32_t lastMqttAttempt = 0;

    if (
        WiFi.status() ==
        WL_CONNECTED &&
        !mqtt.connected() &&
        (lastMqttAttempt == 0 ||
         now - lastMqttAttempt >= MQTT_RETRY_INTERVAL_MS)
    )
    {
        lastMqttAttempt = now;

        connectMQTT();
    }


    mqtt.loop();


    // --------------------------------------------------------
    // OTA
    // --------------------------------------------------------

    if (
        WiFi.status() ==
        WL_CONNECTED
    )
    {
        ArduinoOTA.handle();
    }


    // --------------------------------------------------------
    // HLW8112 measurement
    // --------------------------------------------------------

    if (
        now - lastMeasurement >=
        MEASUREMENT_INTERVAL_MS
    )
    {
        lastMeasurement = now;

        if (readMeter())
        {
            printMeasurement();
        }
    }


    // --------------------------------------------------------
    // POWER EVERY 500ms
    // --------------------------------------------------------

    if (
        mqtt.connected() &&
        now - lastPower >=
        POWER_INTERVAL_MS
    )
    {
        lastPower = now;

        publishPower();
    }


    // --------------------------------------------------------
    // FULL DATA
    // --------------------------------------------------------

    if (
        mqtt.connected() &&
        now - lastFull >=
        FULL_INTERVAL_MS
    )
    {
        lastFull = now;

        publishFull();
    }


    delay(1);
}