# Power monitor — ESP32-C3 SuperMini + JSY-MK-194G

Firmware that reads **channel 2 (load2)** from a [JSY-MK-194G](https://www.jsypowermeter.com) via [MycilaJSY](https://github.com/mathieucarbou/MycilaJSY) and exposes it for an ESS control loop.

## Wiring

| ESP32-C3 SuperMini | JSY-MK-194G |
|--------------------|-------------|
| GPIO20 (RX)        | TX          |
| GPIO21 (TX)        | RX          |
| GND                | GND         |
| 5V                 | VCC         |

- Cross TX/RX and share GND.
- Put the grid / ESS-relevant conductor through **CT2 (channel 2)**.
- The C3 is **3.3 V logic only**. If the JSY UART TX is 5 V, use a level shifter into GPIO20.

Pins are set in `include/config.h` (`JSY_RX_PIN` / `JSY_TX_PIN`).

## Configure

```bash
cp include/secrets.h.example include/secrets.h
```

Edit `include/secrets.h`:

- `WIFI_SSID` / `WIFI_PASSWORD`
- `MQTT_HOST` (set to `""` to disable MQTT)

## Build & flash

```bash
./setup_and_build.sh              # install PlatformIO if needed + build
./setup_and_build.sh --upload     # build + flash (/dev/ttyACM0)
./setup_and_build.sh --upload --monitor
```

Or with PlatformIO directly:

```bash
pio run -t upload
pio device monitor
```

## Outputs

| Interface | Endpoint / topic | Notes |
|-----------|------------------|--------|
| Serial    | status every 2 s | ESS tick stats |
| HTTP      | `GET /api/power` | JSON snapshot |
| MQTT ESS  | `…/load2/power` | Float W every 250 ms (non-retained) |
| MQTT HA   | `…/tele/SENSOR` | Full JSON every 10 s + discovery |
| MQTT      | `…/status` | `online` / `offline` (LWT) |

HA auto-discovers load2 sensors from `tele/SENSOR` (`HA_MQTT_DISCOVERY`). ESS keeps the fast power topic.

```bash
./scripts/check_mqtt_interval.py --seconds 15
./scripts/forward_ess_to_plotjuggler.py   # MQTT ESS → PlotJuggler UDP :9870
```

PlotJuggler: Streaming → UDP Server (JSON) → port **9870**, then run the forwarder.

Sign of `load2/power` depends on CT orientation (typically positive = import, negative = export).

## ESS hook

Edit `onEssLoop()` in `src/main.cpp` to run local charge/discharge / divert logic every 250 ms (`ESS_PUBLISH_INTERVAL_MS`).

## Behaviour notes

- Async JSY reads on core 0 (ESP32-C3 is single-core).
- After connect, baud is raised to **38400** for ~330 ms change detection (Mycila recommendation).
- Model is forced to `JSY-MK-194`.
- WiFi TX power defaults to `WIFI_POWER_8_5dBm` — SuperMini LDOs often brown out at full TX and report `AUTH_EXPIRE`.

## SuperMini power tips

If you see `AUTH_EXPIRE` even with a correct password:

- Use a solid USB port / short cable (avoid weak hubs)
- Add ~100 µF on **3V3** near the board
- Don't starve the ESP by powering the JSY from a marginal shared 5 V rail
- Keep `WIFI_TX_POWER` low in `include/config.h` if the AP is nearby
