# sofar2mqtt — standalone Sofar2mqtt

Vendored copy of [Sofar2mqtt](https://github.com/collin80/Sofar2mqtt) / local `~/repos/Sofar2mqtt` for ESP8266 NodeMCU.

This is the **original** Sofar RS485 ↔ MQTT bridge (OLED + battery_save). It does **not** include the JSY ESS loop — that is [`../sofar_ess/`](../sofar_ess/).

## Wiring

| NodeMCU | RS485 module |
|---------|----------------|
| D5 | DE + RE |
| D6 | RO (RX) |
| D7 | DI (TX) |
| D1/D2 | OLED I2C (optional SSD1306) |
| 3V3/5V + GND | as required |

## Configure

```bash
cd sofar2mqtt
cp include/secrets.h.example include/secrets.h
# edit WiFi / MQTT / DEVICE_NAME
```

Inverter type in `platformio.ini`:

- `-D INVERTER_HYBRID` (default)
- or `-D INVERTER_ME3000`

## Build / flash

```bash
./setup_and_build.sh
./setup_and_build.sh --upload /dev/ttyUSB0 --monitor
```

Upstream serial debug is **9600** baud.

## MQTT (unchanged from upstream)

- Publish: `Sofar2mqtt/state`
- Control (passive mode): `Sofar2mqtt/set/standby|auto|charge|discharge`

See `UPSTREAM_README.md` for full documentation and attribution.
