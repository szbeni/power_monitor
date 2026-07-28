# sofar_jsy_ess — NodeMCU Sofar + JSY ESS loop

ESP8266 NodeMCU firmware that combines Sofar2mqtt-style Sofar RS485 control with a **JSY-MK-194G channel 2 (load2)** meter and a closed-loop **active power** ESS controller.

Based on local `~/repos/Sofar2mqtt` and the JSY metering approach from this repo’s ESP32-C3 firmware.

## What it does

Every ~2 s:

1. Read **load2 active power** from the JSY (TTL Modbus)
2. If ESS enabled:
  - **Import** (P > deadband) → Sofar **discharge** at ~P watts  
  - **Export** (P < −deadband) → Sofar **charge** at ~|P| watts  
  - Near zero → Sofar **standby**
3. Publish state over MQTT

Inverter must be in **Passive Mode** (same requirement as Sofar2mqtt).

## Wiring (NodeMCU)


| Function          | NodeMCU | Device                                       |
| ----------------- | ------- | -------------------------------------------- |
| Sofar DE/RE       | D5      | MAX485 DE+RE                                 |
| Sofar RX          | D6      | MAX485 RO                                    |
| Sofar TX          | D7      | MAX485 DI                                    |
| JSY RX            | D1      | JSY TX                                       |
| JSY TX            | D2      | JSY RX                                       |
| Debug (USB UART0) | RX/TX   | serial monitor @ 115200                      |
|                   | GND     | shared GND                                   |
|                   | 3V3     | JSY VCC (3.3 V OK; RS485 module as required) |


Grid / ESS CT → **JSY CT2 (channel 2)**.

> Both Sofar and JSY use SoftSerial @ 9600. Only one listens at a time; the firmware alternates. ESS loop stays at **2 s**.



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
./setup_and_build.sh --upload /dev/ttyUSB0 --monitor
```



## MQTT

Base topic = `DEVICE_NAME` (default `SofarEss`).


| Topic              | Direction | Payload                                 |
| ------------------ | --------- | --------------------------------------- |
| `…/state`          | pub       | JSON status every 10 s                  |
| `…/ess/grid_power` | pub       | JSY load2 W                             |
| `…/ess/energy_import_wh` | pub | JSY ch2 imported energy (Wh)          |
| `…/ess/energy_export_wh` | pub | JSY ch2 exported energy (Wh)          |
| `…/ess/command_w`  | pub       | last ESS command (+discharge / −charge) |
| `…/ess/mode`       | pub       | `standby` / `charge` / `discharge`      |
| `…/ess/enabled`    | pub       | `true` / `false`                        |
| `…/set/ess`        | sub       | `true` / `false` — enable/disable loop  |
| `…/set/standby`    | sub       | `true` — manual (disables ESS)          |
| `…/set/auto`       | sub       | `true` — manual (disables ESS)          |
| `…/set/charge`     | sub       | watts — manual (disables ESS)           |
| `…/set/discharge`  | sub       | watts — manual (disables ESS)           |


Manual `/set/charge|discharge|…` commands turn ESS off so you can take over; publish `…/set/ess` `true` to resume the loop.