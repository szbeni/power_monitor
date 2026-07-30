# sofar_ess — ESP32-C3 Sofar + MycilaJSY ESS loop

ESP32-C3 SuperMini firmware that combines Sofar2mqtt-style Sofar RS485 control with a **JSY-MK-194G channel 2 (load2)** meter ([MycilaJSY](https://github.com/mathieucarbou/MycilaJSY)) and a closed-loop **active power** ESS controller.

## What it does

Every ~500 ms:

1. Sample **load2 active power** from MycilaJSY
2. If ESS enabled, run a **PI** on grid residual `e = P` (`u = Kp·e + Ki·∫e`), then charge/discharge/standby from `u`
3. Publish live ESS MQTT topics; `…/state` (~10 s) is **cache-only**. Sofar registers are refreshed one-per-ESS-cycle (run / grid / batt / SOC).

Defaults: `Kp=0.4`, `Ki=0.3` (per second), tunable live over MQTT. Integrator freezes in the deadband, uses measured `dt`, and has anti-windup at ±MAX.

**Charge-only mode** clamps the ESS command to ≤ 0 W: charge from export / excess, standby on import — never discharge. Toggle via MQTT or HA.

**SOC protect** (daytime ESS only, separate from overnight smart charging): when bidirectional ESS is running, SOC below the **low threshold** forces charge-only (no discharge); bidirectional resumes once SOC reaches **low + hysteresis** (default 20% / +5%). Tunable via MQTT or HA.

Inverter must be in **Passive Mode** (same requirement as Sofar2mqtt).

## Wiring (ESP32-C3 SuperMini)

| Function          | GPIO | Device                                       |
| ----------------- | ---- | -------------------------------------------- |
| Sofar DE/RE       | 4    | MAX485 DE+RE                                 |
| Sofar RX (UART0)  | 5    | MAX485 RO                                    |
| Sofar TX (UART0)  | 6    | MAX485 DI                                    |
| JSY RX (UART1)    | 20   | JSY TX                                       |
| JSY TX (UART1)    | 21   | JSY RX                                       |
| Debug             | USB  | USB-CDC serial monitor @ 115200              |
|                   | GND  | shared GND                                   |
|                   | 3V3  | JSY VCC (3.3 V OK; RS485 module as required) |

Grid / ESS CT → **JSY CT2 (channel 2)**.

> Sofar and JSY each get a hardware UART — no SoftSerial, no time-sharing.

## Configure

```bash
cd sofar_ess
cp include/secrets.h.example include/secrets.h
# edit WiFi + MQTT
```

Inverter type in `platformio.ini`:

- `-D INVERTER_HYBRID` (default, 3600 W max)
- or `-D INVERTER_ME3000` (3000 W max)

ESS tuning: `include/config.h` (`ESS_DEADBAND_W`, `ESS_LOOP_INTERVAL_MS`, …).

## Build / flash

```bash
cd sofar_ess
./setup_and_build.sh
./setup_and_build.sh --upload /dev/ttyACM0 --monitor
```

### OTA (ArduinoOTA)

After the device is on WiFi, flash over the LAN (no USB):

```bash
# set OTA_PASSWORD in include/secrets.h (default: ota_change_me)
# if you change it, also set upload_flags --auth=... in platformio.ini [env:esp32c3-ota]
./setup_and_build.sh --ota 192.168.x.x
# or:
pio run -e esp32c3-ota -t upload --upload-port 192.168.x.x
```

Serial shows `[ota] ready as 'sofaress' @ <ip>`. Hostname = `DEVICE_NAME`. Disable with `OTA_ENABLE 0` in `config.h`.

## MQTT

Base topic = `DEVICE_NAME` (default `sofaress`).

Sign convention for power values: **+ = discharge / import**, **− = charge / export**.

### Modes (Sofar passive control)

| Mode | What it does |
| ---- | ------------ |
| **standby** | Battery idle — neither charge nor discharge (~0 W). ESS uses this when grid residual is inside the deadband. |
| **charge** | Force charge at N watts (AC → battery). |
| **discharge** | Force discharge at N watts (battery → AC). |
| **auto** | Hand control back to the **inverter’s own logic** (not the ESS loop). Publishing `set/auto` also **disables ESS**. |

### Topics

| Topic | Dir | Description |
| ----- | --- | ----------- |
| `…/status` | pub | `online` / `offline` (LWT) — device availability |
| `…/state` | pub | JSON snapshot every ~10 s (see fields below) |
| `…/ess/grid_power` | pub | JSY load2 grid residual (W). +import, −export |
| `…/ess/battery_power` | pub | Sofar measured battery power (W). +discharge, −charge |
| `…/ess/energy_import_wh` | pub | JSY channel-2 energy imported from grid (Wh) |
| `…/ess/energy_export_wh` | pub | JSY channel-2 energy exported to grid (Wh) |
| `…/ess/command_w` | pub | Last ESS setpoint sent to Sofar (+discharge / −charge / 0=standby) |
| `…/ess/mode` | pub | Last Sofar mode string: `standby` / `charge` / `discharge` / `auto` / `unknown` |
| `…/ess/enabled` | pub | `true` if closed-loop ESS is running; `false` if manual-only |
| `…/ess/charge_only` | pub | User charge-only preference (`true` / `false`, retained) |
| `…/ess/effective_charge_only` | pub | Effective charge-only (user **or** SOC protect, retained) |
| `…/ess/soc_protect/enabled` | pub | SOC protect on/off (retained) |
| `…/ess/soc_protect/low` | pub | Low SOC threshold % — below this, ESS goes charge-only (retained) |
| `…/ess/soc_protect/hyst` | pub | Hysteresis % — bidirectional resumes at low + hyst (retained) |
| `…/ess/soc_protect/active` | pub | `true` when SOC protect has forced charge-only |
| `…/ess/kp` | pub | PI proportional gain (retained) |
| `…/ess/ki` | pub | PI integral gain in 1/s (retained) |
| `…/ess/deadband` | pub | ESS deadband watts (retained) |
| `…/ess/min_delta` | pub | Min command change watts (retained) |
| `…/ess/integral` | pub | Current integrator state |
| `…/set/ess` | sub | `true` / `false` — enable/disable; `charge_only` = enable + soak-only; `full` = enable + bidirectional |
| `…/set/charge_only` | sub | `true` / `false` — user charge-only preference (cmd ≤ 0) |
| `…/set/soc_protect` | sub | `true` / `false` — enable SOC-based charge-only protect |
| `…/set/soc_protect_low` | sub | int 1…100 — low SOC threshold % |
| `…/set/soc_protect_hyst` | sub | int 0…30 — hysteresis % above low before bidirectional resumes |
| `…/set/kp` | sub | float 0…5 — set Kp live |
| `…/set/ki` | sub | float 0…5 — set Ki live (1/s) |
| `…/set/deadband` | sub | float 0…500 — deadband watts |
| `…/set/min_delta` | sub | float 0…500 — min Sofar command delta watts |
| `…/set/reset_i` | sub | any — clear PI integrator |
| `…/set/standby` | sub | `true` — force standby (battery idle). **Disables ESS** |
| `…/set/auto` | sub | `true` — Sofar auto (inverter self-control). **Disables ESS** |
| `…/set/charge` | sub | watts (1…MAX) — force charge. **Disables ESS** |
| `…/set/charge_target_soc` | sub | int 0…100 — stop manual charge at this SOC and go standby (0 = no limit) |
| `…/charge/target_soc` | pub | Current charge target SOC % (retained; 0 = no limit) |
| `…/set/discharge` | sub | watts (1…MAX) — force discharge. **Disables ESS** |
| `…/response/<cmd>` | pub | `0` = OK after a manual set command |

Example:
```bash
mosquitto_pub -t sofaress/set/kp -m 0.4
mosquitto_pub -t sofaress/set/ki -m 0.3
mosquitto_pub -t sofaress/set/deadband -m 20
mosquitto_pub -t sofaress/set/min_delta -m 10
```

### `…/state` JSON fields

| Field | Meaning |
| ----- | ------- |
| `ess_enabled` | Closed-loop ESS on/off |
| `ess_charge_only` | User charge-only preference |
| `ess_effective_charge_only` | Effective charge-only (user or SOC protect) |
| `ess_soc_protect_enabled` / `ess_soc_protect_low` / `ess_soc_protect_hyst` | SOC protect config |
| `ess_soc_protect_active` | SOC protect currently forcing charge-only |
| `grid_power_w` | JSY residual grid power (W) |
| `energy_import_wh` / `energy_export_wh` | JSY energy totals |
| `ess_command_w` | Last commanded battery offset (W) |
| `ess_kp` / `ess_ki` | Live PI gains |
| `ess_deadband_w` / `ess_min_delta_w` | Live deadband / min command delta |
| `ess_integral` | Integrator state |
| `sofar_mode` | `standby` / `charge` / `discharge` / `auto` / `unknown` |
| `charge_target_soc` | Manual charge stop target % (0 = no limit) |
| `run_state` | Sofar run-state register (when RS485 OK) |
| `battery_soc` | Battery state of charge % |
| `battery_power_w` | Measured battery power (W) |
| `sofar_grid_raw` | Raw Sofar grid-power register |
| `sofar_ok` | Present as `false` if Sofar data never read successfully |

Manual `/set/charge|discharge|standby|auto` turns ESS off so you can take over; publish `…/set/ess` `true` to resume the loop (or `charge_only` to resume soak-only).

Example charge-only:
```bash
mosquitto_pub -t sofaress/set/ess -m charge_only
# or with ESS already on:
mosquitto_pub -t sofaress/set/charge_only -m true
```

## Home Assistant

With MQTT discovery enabled (`HA_MQTT_DISCOVERY=1`, default), device **Sofar ESS** appears when MQTT connects.

- Sensors: grid / battery power, ESS command, SOC, energy import/export, Sofar mode, run state  
- Switch: ESS enabled, ESS charge only, ESS SOC protect  
- Binary sensor: ESS SOC protect active  
- Numbers: ESS Kp, ESS Ki, Deadband, Min Delta, SOC protect low, SOC protect hysteresis  
- Buttons: Standby, Auto, Reset Integral  

See the MQTT section above for what each mode/topic means. Requires the HA MQTT integration (discovery prefix `homeassistant`). Re-flash / reconnect MQTT to refresh discovery.

### Dashboard + smart overnight charging

Repo files (copy into Home Assistant, do not expect the ESP to serve them):

| File | Purpose |
| ---- | ------- |
| [`ha-bashboard.yaml`](ha-bashboard.yaml) | Compact Energy Lovelace view (live + inverter + smart charge + history) |
| [`ha-smart-energy-package.yaml`](ha-smart-energy-package.yaml) | Helpers, sensors, scripts, and automations for forecast-based cheap-window charging |

**Forecast.Solar:** overnight decisions use day-ahead production. After midnight that is usually `sensor.energy_production_today` (falling back to `sensor.energy_production_tomorrow` if needed). Evenings preview tomorrow’s forecast on the dashboard.

**Cheap window:** 00:30–05:30 (9.5p/kWh). Outside that window the package restores full bidirectional ESS (`sofaress/set/ess` → `full`).

Setup:

1. Enable packages in `configuration.yaml`:
   ```yaml
   homeassistant:
     packages: !include_dir_named packages
   ```
2. Copy `ha-smart-energy-package.yaml` → `config/packages/smart_energy.yaml`
3. Check configuration and restart Home Assistant
4. Paste `ha-bashboard.yaml` as a Lovelace view (Raw configuration editor)
5. Set **Battery capacity**, **Learned daily demand**, and charge power on the dashboard
6. Keep **Automation enabled** off, run **Recalculate / apply now** — status should show `dry-run` and a Decision reason
7. When the target looks right, turn **Automation enabled** on

Behaviour (when automation is on):

- **00:30** — lock overnight target SOC from corrected forecast vs learned demand; charge if SOC is below target (firmware stops at target SOC), otherwise standby  
- **Reach target** — stop charging (standby) until the window ends — enforced in firmware via `set/charge_target_soc` and mirrored in HA  
- **05:30** — restore full ESS for the rest of the day  
- **Outside cheap window** — full bidirectional ESS whenever automation is enabled  
- **Enable mid-window** — re-evaluate and charge or hold  
- **HA restart** inside the window — re-evaluate after 30 s  
- **HA restart** outside the window — restore full ESS after 30 s  
- **SOC unavailable** — force standby  
- **~23:50** — blend learned demand from estimated house load energy  
- **~23:55** — nudge forecast correction from OpenDTU daily yield vs morning forecast  

Defaults: min SOC 20%, max overnight 80%, charge 2500 W. Formula:  
`target% = clamp(max(min%, (demand − forecast×correction) / capacity × 100), max%)` — sunny days stay near min; cloudy days raise the overnight SOC.

## PlotJuggler

Forward all `sofaress/#` MQTT values to PlotJuggler over UDP JSON:

```bash
# PlotJuggler: Streaming → UDP Server → JSON → port 9870
./scripts/forward_sofaress_to_plotjuggler.py
./scripts/forward_sofaress_to_plotjuggler.py --pj-port 9870 --include-t
```

Uses broker credentials from `sofar_ess/include/secrets.h` and `DEVICE_NAME` from config.
