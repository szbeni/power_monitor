# sofar_ess — ESP32-C3 Sofar + MycilaJSY ESS loop

ESP32-C3 SuperMini firmware that combines Sofar2mqtt-style Sofar RS485 control with a **JSY-MK-194G channel 2 (load2)** meter ([MycilaJSY](https://github.com/mathieucarbou/MycilaJSY)) and a closed-loop **active power** ESS controller.

## What it does

Every ~500 ms:

1. Sample **load2 active power** from MycilaJSY
2. If ESS enabled, run a **PI** on grid residual `e = P` (`u = Kp·e + Ki·∫e`), then charge/discharge from `u`
3. Publish live ESS MQTT topics; `…/state` (~5 s) is **cache-only**. Sofar registers are refreshed **one Modbus txn per ESS cycle** in round-robin (run / grid / batt / SOC / fault / alert / battFault / **PV strings** / **PV today**).

Defaults: `Kp=0.4`, `Ki=0.3` (per second), tunable live over MQTT. Integrator freezes in the deadband, uses measured `dt`, and has anti-windup at ±MAX.

**ESS near-zero hold:** while ESS is enabled, the PI deadband does **not** send Sofar `standby` (that was causing inverter relay chatter). Instead the firmware holds a small charge or discharge at `ESS_HOLD_MIN_W` (default 30 W) in the last polarity. Explicit `set/standby` and dual-battery bank switches still use real standby.

**Charge-only mode** clamps the ESS command to ≤ 0 W: charge from export / excess, hold charge on import — never discharge. Toggle via MQTT or HA.

**SOC protect** (daytime ESS only, separate from overnight smart charging): when bidirectional ESS is running, SOC below the **low threshold** forces charge-only (no discharge); bidirectional resumes once SOC reaches **low + hysteresis** (default 15% / +5%). Tunable via MQTT or HA.

**Dual battery** (with [`battery_selector`](../battery_selector/)): prefers bank **A** (10 kWh GTX5000 parallel) over **B** (5 kWh Fogstar). Tracks last-known SOC per bank; auto-switches when A is empty (discharge) or full (charge). Force a bank via `set/battery`. Empty/full defaults **10% / 98%**, MQTT/HA tunable. Bank changes: standby → MQTT select → settle → invalidate SOC → re-poll until confirmed.

The firmware publishes a capacity-weighted combined SOC:
`(A_SOC × 10 kWh + B_SOC × 5 kWh) / 15 kWh`. It is unavailable until both
banks have been observed. `smart_energy.yaml` uses the combined value and falls
back to the connected bank SOC during initial discovery.

**Sofar PV strings (hybrid):** Modbus block `0x0250–0x0255` (PV1/PV2 V×0.1, I×0.01 A, P×0.01 kW) and today generation `0x0218` (×0.01 kWh). Polled as low-priority round-robin phases (still one status txn per ESS tick). Sofar PV Total = PV1+PV2 watts. Per-string daily energy is **not** in this register map. HA also builds site totals from Hoymiles 1600 + Hoymiles 800 + Sofar PV.

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
| **standby** | Battery idle — neither charge nor discharge (~0 W). Used for manual `set/standby` and during dual-battery bank switches — **not** for ESS deadband. |
| **charge** | Force charge at N watts (AC → battery). ESS hold uses a small charge instead of standby near zero. |
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
| `…/ess/command_w` | pub | Last ESS setpoint sent to Sofar (+discharge / −charge; near-zero uses hold min, not 0) |
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
| `…/ess/hold_min` | pub | Near-zero hold watts instead of standby (retained) |
| `…/ess/integral` | pub | Current integrator state |
| `…/battery/active` | pub | Connected bank `A` / `B` (retained) |
| `…/battery/a/soc` | pub | Last-known bank A SOC % or `null` (retained) |
| `…/battery/b/soc` | pub | Last-known bank B SOC % or `null` (retained) |
| `…/battery/combined_soc` | pub | Capacity-weighted 15 kWh combined SOC, or `null` until both banks are known (retained) |
| `…/battery/dual_state` | pub | `idle` / `switching` / `settling` / `syncing` / `error` (retained) |
| `…/battery/force` | pub | `off` / `A` / `B` (retained) |
| `…/battery/force_select` | pub | HA select state: `Auto` / `Battery A` / `Battery B` (retained) |
| `…/battery/empty` | pub | Empty SOC threshold % (retained) |
| `…/battery/full` | pub | Full SOC threshold % (retained) |
| `…/battery/dual_enabled` | pub | Auto dual-battery policy on/off (retained) |
| `…/set/ess` | sub | `true` / `false` — enable/disable; `charge_only` = enable + soak-only; `full` = enable + bidirectional |
| `…/set/charge_only` | sub | `true` / `false` — user charge-only preference (cmd ≤ 0) |
| `…/set/soc_protect` | sub | `true` / `false` — enable SOC-based charge-only protect |
| `…/set/soc_protect_low` | sub | int 1…100 — low SOC threshold % |
| `…/set/soc_protect_hyst` | sub | int 0…30 — hysteresis % above low before bidirectional resumes |
| `…/set/kp` | sub | float 0…5 — set Kp live |
| `…/set/ki` | sub | float 0…5 — set Ki live (1/s) |
| `…/set/deadband` | sub | float 0…500 — deadband watts |
| `…/set/min_delta` | sub | float 0…500 — min Sofar command delta watts |
| `…/set/hold_min` | sub | int 1…MAX — ESS near-zero hold watts |
| `…/set/dual_batt` | sub | `true` / `false` — enable A-prefer auto bank switching |
| `…/set/dual_batt_empty` | sub | int 0…100 — empty threshold % (default 10) |
| `…/set/dual_batt_full` | sub | int 0…100 — full threshold % (default 98) |
| `…/set/battery` | sub | `Auto` / `Battery A` / `Battery B` / `A` / `B` — force bank or clear force |
| `…/set/reset_i` | sub | any — clear PI integrator |
| `…/set/standby` | sub | `true` — force standby (battery idle). **Disables ESS** |
| `…/set/auto` | sub | `true` — Sofar auto (inverter self-control). **Disables ESS** |
| `…/set/charge` | sub | watts (1…MAX) — force charge. **Disables ESS** |
| `…/set/charge_target_soc` | sub | int 0…100 — stop manual charge at this SOC and go standby (0 = no limit) |
| `…/charge/target_soc` | pub | Current charge target SOC % (retained; 0 = no limit) |
| `…/set/discharge` | sub | watts (1…MAX) — force discharge. **Disables ESS** |
| `…/response/<cmd>` | pub | `0` = OK after a manual set command |

Also publishes to `battery_selector/set/select` and subscribes to `battery_selector/selected`.

Example:
```bash
mosquitto_pub -t sofaress/set/kp -m 0.4
mosquitto_pub -t sofaress/set/ki -m 0.3
mosquitto_pub -t sofaress/set/deadband -m 20
mosquitto_pub -t sofaress/set/min_delta -m 10
mosquitto_pub -t sofaress/set/hold_min -m 30
mosquitto_pub -t sofaress/set/battery -m "Battery B"
mosquitto_pub -t sofaress/set/battery -m Auto
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
| `ess_deadband_w` / `ess_min_delta_w` / `ess_hold_min_w` | Live deadband / min command delta / near-zero hold |
| `ess_integral` | Integrator state |
| `sofar_mode` | `standby` / `charge` / `discharge` / `auto` / `unknown` |
| `charge_target_soc` | Manual charge stop target % (0 = no limit) |
| `battery_active` / `battery_force` / `battery_dual_state` | Dual-battery active bank, force, state machine |
| `battery_dual_enabled` / `battery_empty_soc` / `battery_full_soc` | Dual-battery policy config |
| `battery_a_soc` / `battery_b_soc` | Per-bank last-known SOC (`null` if never seen) |
| `battery_combined_soc` | Capacity-weighted combined SOC (`null` until both banks are known) |
| `battery_a_kwh` / `battery_b_kwh` | Configured capacities (10 / 5) |
| `pv1_power_w` / `pv1_voltage_v` / `pv1_current_a` | Sofar PV1 string (null until first PV poll) |
| `pv2_power_w` / `pv2_voltage_v` / `pv2_current_a` | Sofar PV2 string |
| `pv_total_w` | Sofar PV1 + PV2 power (W) |
| `pv_today_kwh` | Sofar inverter today generation (`0x0218`) |
| `sofar_ok` | `true` once any Sofar Modbus read succeeds; `false` = RS485 never OK |
| `sofar_last_error` | Last Modbus fail reason (`no-rx`, `timeout`, `crc`, `exception`, …) |
| `run_state` | Sofar run-state register (0–7) when RS485 OK |
| `run_state_name` | `wait` / `check` / `normal` / `discharge` / `fault` / `permanent_fault` / … |
| `fault_active` | `true` if run-state is fault/permanent or any named fault bit set |
| `fault_message` | Decoded inverter fault bits (regs `0x0201`–`0x0205`) |
| `fault_raw` | Five fault words as hex (`HHHH,HHHH,…`) |
| `alert_message` | Decoded inverter alerts (reg `0x022B`) |
| `alert_raw` | Raw alert register |
| `batt_fault_message` | Decoded battery fault bits (regs `0x023D`–`0x0241`) |
| `batt_fault_raw` | Five battery-fault words as hex |
| `battery_soc` | Battery state of charge % |
| `battery_power_w` | Measured battery power (W) |
| `sofar_grid_raw` | Raw Sofar grid-power register |

Manual `/set/charge|discharge|standby|auto` turns ESS off so you can take over; publish `…/set/ess` `true` to resume the loop (or `charge_only` to resume soak-only).

Example charge-only:
```bash
mosquitto_pub -t sofaress/set/ess -m charge_only
# or with ESS already on:
mosquitto_pub -t sofaress/set/charge_only -m true
```

## Home Assistant

With MQTT discovery enabled (`HA_MQTT_DISCOVERY=1`, default), device **Sofar ESS** appears when MQTT connects.

- Sensors: grid / battery power, ESS command, SOC (live + A/B + combined), Sofar PV1/PV2 (W/V/A) + PV total + PV today, active bank, dual state, energy import/export, Sofar mode, run state / name, fault / alert / battery-fault messages, Sofar last error  
- Binary sensors: Sofar OK (RS485), Sofar fault active, ESS SOC protect active  
- Switch: ESS enabled, ESS charge only, ESS SOC protect, Dual Battery Auto  
- Select: Battery Bank (`Auto` / `Battery A` / `Battery B`)  
- Numbers: ESS Kp, ESS Ki, Deadband, Min Delta, Hold Min, SOC protect low/hyst, Battery empty/full SOC  
- Buttons: Standby, Auto, Reset Integral  

HA package templates (in `smart_energy.yaml`): `sensor.site_solar_total_power` (Hoymiles 1600 + 800 + Sofar PV) and `sensor.site_solar_today_kwh`.

If **Sofar OK** stays off, check `sofar_last_error` (`no-rx` = wiring/DE/slave ID; `crc` = noise/baud; `exception` = bad register). Inverter **fault** run-state is separate — use **Sofar Fault** / **fault_raw** from the PDF fault-message registers.

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

- **00:30** — lock overnight target SOC; pick charge / standby / ESS from SOC vs target  
- **SOC < target** — force charge  
- **target … target+2%** — standby (grid only; no battery charge/discharge)  
- **SOC > target+2%** — full ESS  
- **05:30** — restore full ESS for the rest of the day  
- **Outside cheap window** — full bidirectional ESS whenever automation is enabled  
- **Enable mid-window** — re-evaluate and charge / hold / ESS  
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
