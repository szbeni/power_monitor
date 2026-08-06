# battery_selector — ESP32 dual-battery bank switch

PlatformIO firmware that selects **Battery A** or **Battery B** over MQTT and
drives the active relay channels together. Periodically publishes the current
selection.

Pins match the existing `battery_selector.yaml` ESPHome sketch.

## Behaviour

Two batteries, four relay channels:

| Selection | Relays 1–3 | Contact path |
| --------- | ---------- | ------------ |
| Battery A (default) | de-energized | **NC** |
| Battery B | energized | **NO** |

Relay 4 is unused and stays de-energized.

## Wiring (ESP32 DevKit)

| Function   | GPIO | Notes                          |
| ---------- | ---- | ------------------------------ |
| Relay 1    | 32   | Power                          |
| Relay 2    | 33   | CAN bus A                      |
| Relay 3    | 25   | CAN bus B                      |
| Relay 4    | 26   | Unused                         |
| Status LED | 23   | On when Battery B is selected  |
| GND        | —    | shared                         |

Default polarity is **active-high** (same as ESPHome gpio switches). For
active-low modules set `RELAY_ACTIVE_HIGH` to `0` in `include/config.h`.

## Configure

```bash
cd battery_selector
cp include/secrets.h.example include/secrets.h
# edit WiFi + MQTT
```

## Build / flash

```bash
cd battery_selector
./setup_and_build.sh
./setup_and_build.sh --upload /dev/ttyUSB0 --monitor
```

### OTA

```bash
./setup_and_build.sh --ota 192.168.x.x
```

## MQTT

Base topic = `DEVICE_NAME` (default `battery_selector`).

| Topic | Dir | Payload | Notes |
| ----- | --- | ------- | ----- |
| `…/set/select` | sub | `Battery A`/`Battery B`, `1`/`A`, `2`/`B`, `off` | Select bank; relays 1–3 update together (`off` → A) |
| `…/set/battery` | sub | same | Alias |
| `…/selected` | pub | `Battery A` or `Battery B` | Retained; also every `REPORT_INTERVAL_MS` (5 s) |
| `…/status` | pub | `online` / `offline` | LWT |

### Home Assistant

On MQTT connect the device publishes MQTT discovery for a **Select** entity
(`homeassistant/select/battery_selector/battery/config`) under device
**Battery Selector**. No YAML needed if MQTT discovery is enabled in HA.

Examples:

```bash
mosquitto_pub -t battery_selector/set/select -m "Battery A"
mosquitto_pub -t battery_selector/set/select -m 2
mosquitto_pub -t battery_selector/set/select -m B
mosquitto_sub -t battery_selector/selected -v
```
