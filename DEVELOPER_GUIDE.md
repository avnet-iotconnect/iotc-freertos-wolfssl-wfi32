# Developer Guide

Build, flash, and debugging reference for the WFI32-IoT /IOTCONNECT demo. For first-time setup and
cloud configuration, start with the [README quickstart](README.md).

## Index

1. [Repository Layout](#1-repository-layout)
2. [Toolchain](#2-toolchain)
3. [Building](#3-building)
4. [Flashing](#4-flashing)
5. [Serial Consoles](#5-serial-consoles)
6. [Firmware Architecture](#6-firmware-architecture)
7. [I2C Bus Map](#7-i2c-bus-map)
8. [Click Board Support](#8-click-board-support)
9. [Hardware Constraints](#9-hardware-constraints)
10. [/IOTCONNECT Protocol Flow](#10-iotconnect-protocol-flow)
11. [Fixes in This Branch](#11-fixes-in-this-branch)

## 1. Repository Layout

```
firmware/
  iotc-wfi32-aws-demo.X/        MPLAB X project (config: aws_sdk_wfi32_iot_freertos)
  src/
    app.c                       Wi-Fi task, connect callback, power-save policy
    app_device.c/.h             On-board sensor task + shared I2C helpers (APP_SENSORS_*)
    app_iotconnect.c/.h         /IOTCONNECT session task (discovery -> identity -> MQTT)
    iotconnect.c/.h             MQTT glue: publish/subscribe, iotcl callbacks
    iotc_https.c / http_client.c HTTPS client used for discovery/identity
    sensors.c/.h                Click detection, reads, telemetry assembly
    clicks/                     One driver pair per supported Click board
    iotc_config.h               CPID / environment / MQTT tuning
    iotc-c-lib/                 Avnet IoTConnect C library (git submodule)
    config/aws_sdk_wfi32_iot_freertos/  Harmony-generated code (I2C, GPIO, TCP/IP, USB, ...)
    third_party/                FreeRTOS, wolfSSL, AWS IoT C SDK, paho
dashboards/                     Importable /IOTCONNECT dashboards
media/                          Images
wfi32demo_template.JSON         /IOTCONNECT device template (all attributes + commands)
```

## 2. Toolchain

| Component | Version |
|---|---|
| MPLAB X IDE | 6.25 |
| XC32 compiler | **4.60** |
| PIC32MZ-W_DFP device pack | **1.10.334** |
| Target device | PIC32MZ1025W104132 (WFI32E01PC module) |

The project was retargeted from XC32 4.45 / DFP 1.6.220 (which are no longer distributed) — if MPLAB
prompts about a missing compiler or pack, point it at the versions above; no source changes are needed.

Clone with `--recursive`: `firmware/src/iotc-c-lib` (and its cJSON) are git submodules.

## 3. Building

**IDE:** open `firmware/iotc-wfi32-aws-demo.X`, configuration `aws_sdk_wfi32_iot_freertos`, Build.

**Command line** (Windows paths shown; adjust versions to your install):

```
"C:\Program Files\Microchip\MPLABX\v6.25\mplab_platform\bin\prjMakefilesGenerator.bat" ^
    firmware\iotc-wfi32-aws-demo.X@aws_sdk_wfi32_iot_freertos

cd firmware\iotc-wfi32-aws-demo.X
"C:\Program Files\Microchip\MPLABX\v6.25\gnuBins\GnuWin32\bin\make.exe" ^
    -f nbproject\Makefile-aws_sdk_wfi32_iot_freertos.mk SUBPROJECTS= .build-conf
```

Output: `dist/aws_sdk_wfi32_iot_freertos/production/iotc-wfi32-aws-demo.X.production.hex`.

## 4. Flashing

Any of:

* **MPLAB X IDE** — Make and Program Device.
* **MPLAB IPE / ipecmd** — note `ipecmd` cannot select a PKoB4 by serial number; with more than one
  board attached it prompts interactively.
* **MDB** (scriptable, supports serial-number selection — the reliable choice with multiple boards):

  ```
  device PIC32MZ1025W104132
  hwtool pkob4 <sn>RYNxxxxxxxxx
  program path\to\iotc-wfi32-aws-demo.X.production.hex
  reset MCLR
  quit
  ```

  Run with `mdb.bat script.txt`. Find debugger serials with a script containing just `hwtool`.

> **Always power-cycle after flashing.** After a debugger-driven program or reset the application
> frequently remains halted until a true power-on reset. MCLR via the debugger is not sufficient.
> If MDB ever exits uncleanly and later reports the tool as reserved, delete
> `%USERPROFILE%\.mplabcomm\tool-reservations.dat` (with no MPLAB processes running).

## 5. Serial Consoles

Both are 115200 8N1. The board's internal USB hub exposes:

| Windows name | UART | Routed via | Content |
|---|---|---|---|
| USB Serial Device | UART1 | MCP2200 | **Application console**: `SYS_CONSOLE_PRINT`, command processor (`help`, `reboot`, `rssi`, `power_mode`, ...) |
| PICkit 4 On Board Virtual COM Port | UART3 | PKoB4 | `printf`/stdout (`xc32_monitor.c`) and the Wi-Fi driver's `MAC:` debug stream |

COM numbers follow USB port paths, not boards — replugging into a different port renumbers them.
The `MAC:` stream on UART3 is invaluable for Wi-Fi association debugging (auth/assoc/EAPOL states,
deauth reason codes, rate control, power-save state).

## 6. Firmware Architecture

FreeRTOS tasks, cooperating through flags in `app_common.h`:

1. **`app.c` (WLAN)** — brings up the Wi-Fi driver, connects using credentials from `WIFI.CFG`,
   reports IP/NTP readiness. On connect it selects the radio power-save policy — currently
   `POWERSAVE_RUN_MODE` (see [Hardware Constraints](#9-hardware-constraints)).
2. **`app_usb_msd.c`** — mounts the FAT filesystem, exports it as a USB drive, reads `WIFI.CFG` /
   `cloud.json`, extracts the ATECC certificate to the drive.
3. **`app_device.c`** — on-board MCP9808 + OPT3001 state machine; also owns the shared I2C helper
   functions (`APP_SENSORS_*`) used by every Click driver. Helpers submit a transfer to the `DRV_I2C`
   queue and **block until it completes** (`i2cWaitForTransfer`, polling
   `DRV_I2C_TransferStatusGet`) so callers can parse the receive buffer immediately on return.
4. **`sensors.c`** — `check_click_sensors()` probes for every supported Click at boot (with a bus
   scan printed to the console as a self-test), `read_click_sensors()` refreshes attached ones each
   loop, `add_sensor_data_to_telemetry()` adds their values to the outgoing message.
5. **`app_iotconnect.c` / `iotconnect.c`** — runs HTTP discovery and identity, opens the MQTT
   connection over wolfSSL (client auth via ATECC608B callbacks in `atmel.c`), publishes telemetry
   once per second, dispatches cloud-to-device commands, and restarts the whole session on error.

## 7. I2C Bus Map

| Bus | Owner | Devices |
|---|---|---|
| **I2C1** (`DRV_I2C` instance 0) | `app_device.c` helpers | On-board MCP9808 `0x18`, OPT3001 `0x44`, **and the mikroBUS socket** (SDA1/RPA5, SCL1/RPA4 — one shared bus, per board user guide Table 3-1) |
| **I2C2** | cryptoauthlib (direct plib binding) | ATECC608B secure element `0x6A` |

**Do not touch I2C2 from application code.** It belongs to cryptoauthlib; concurrent access corrupts
TLS operations and can wedge the bus. Everything Click-related lives on I2C1.

## 8. Click Board Support

Detection runs once in `check_click_sensors()` at boot:

1. A full I2C1 address sweep is printed (`I2C1 ack 0x..`) — `0x18` and `0x44` are the on-board
   sensors and act as a built-in positive control; if they are missing, distrust everything else.
2. Each supported Click address is probed 5 times (`hits out of 5`); real devices score 5/5.
3. Each Click driver's init/read is called; a device is marked present only if it returns sane data
   (serial number, calibration constants, or a plausible reading).

Supported Clicks and addresses are tabled in the [README](README.md#3-supported-click-boards).
Detection state gates both the periodic reads and which attributes get added to telemetry.

**Adding a Click:** put a driver pair in `clicks/` using the `APP_SENSORS_*` helpers (they block until
transfer completion — parse the receive buffer directly after the call), add the probe to
`check_click_sensors()`, the read to `read_click_sensors()`, the telemetry fields to
`add_sensor_data_to_telemetry()`, the attributes to `wfi32demo_template.JSON`, and the address to the
README table.

## 9. Hardware Constraints

**Power budget (most important).** The Debug USB port supplies at most **500 mA** for the whole board
(user guide §3.1). Wi-Fi TX bursts peak near 300 mA. A Shuttle with multiple Clicks can push the total
over budget: the observed failure mode is that 802.11 auth and association succeed but the WPA2
handshake fails repeatedly — the AP never accepts EAPOL M2 (corrupted by supply sag during TX),
retries M1 several times, then deauthenticates with reason 0x02, and the driver logs `tx tmo` /
internal errors. Remedies: high-current USB source, Li-Po battery on J101, fewer Clicks, and keep the
Click stack away from the PCB antenna at the top of the board.

**Radio power save.** The original demo enabled `POWERSAVE_WSM_MODE` on connect, which dozes with a
listen interval of 10 beacons (~1 s). That adds up to a second of latency to every inbound packet, so
multi-round-trip exchanges (TLS handshakes, QoS-1 PUBACKs) time out, and the AP's rate control decays
to 6 Mbps from the missed frames. This branch uses `POWERSAVE_RUN_MODE` (in `app.c`) — reliable, but
higher power draw. For battery deployments, revisit this trade-off rather than just switching WSM back
on: pair it with generous MQTT timeouts.

**Reset behaviour.** After any debugger-driven program/reset, the application typically stays halted
until a true power-on reset (USB unplug/replug). Plan scripted workflows around this.

**mikroBUS RST (RC12) / INT (RA13).** Harmony's `GPIO_Initialize` configures both as outputs.
`sensors.c` pulses RST before detection. Do **not** reconfigure RA13 as an input: on this board that
breaks Wi-Fi association (empirically verified; the working Azure RTOS sample also drives it).

## 10. /IOTCONNECT Protocol Flow

1. **Discovery** — `GET https://awsdiscovery.iotconnect.io/api/v2.1/dsdk/cpId/<CPID>/env/<ENV>`
   returns the account's identity base URL. CPID/ENV are compiled in `iotc_config.h`.
2. **Identity** — `GET <base>/uid/<DUID>` returns the MQTT broker host, client ID, and topic set.
   The DUID comes from `cloud.json` (`ClientID`) and must equal the ATECC608B serial.
3. **MQTT** — TLS 1.2 mutual auth on port 8883. The client certificate chain is read from the
   ATECC608B at runtime and ECDSA signing happens inside the secure element (wolfSSL crypto
   callbacks in `atmel.c`); there is no private key in flash.
4. **Telemetry** — published via iotc-c-lib to `$aws/rules/msg_d2c_rpt/<DUID>/2.1/0`; commands arrive
   on `iot/<DUID>/cmd`. Publishes are QoS 1 with retries; the synchronous publish timeout must exceed
   the whole retry schedule (see `IOTC_MQTT_TIMEOUT_MS` in `iotconnect.c`).

Session errors (TLS failure, MQTT disconnect, repeated publish failure) tear the session down and
restart from discovery after a delay.

## 11. Fixes in This Branch

Relative to the original `click-sensors-support` branch:

| Fix | File(s) | Symptom before |
|---|---|---|
| I2C helpers now block until transfer completion | `app_device.c` | Click drivers parsed the receive buffer before data arrived — every probe returned zeros, no Click ever detected |
| Click detection prints to the application console | `sensors.c` | Detection messages went to `printf`/UART3, buried in the `MAC:` debug stream |
| Boot-time I2C scan with on-board sensors as positive control | `sensors.c`, `app_device.c` | No way to tell "Click absent" from "bus broken" |
| mikroBUS RST pulsed before detection | `sensors.c` | RST left at its post-init state |
| Altitude 4 telemetry used a shadowed local struct | `sensors.c` | ALT4 published uninitialised stack values |
| Radio power save changed WSM → RUN mode | `app.c` | ~1 s inbound latency; TLS handshakes and QoS-1 publishes timed out; rate decayed to 6 Mbps |
| MQTT publish timeout raised 5 s → 30 s (beyond the QoS-1 retry schedule) | `iotconnect.c` | Publishes abandoned while retries were still in flight |
| Publish failures log payload and size | `iotconnect.c` | Failures gave no clue what was being sent |
| Toolchain retarget XC32 4.60 / DFP 1.10.334, stale project references removed | `configurations.xml` | Project did not load/build with current tools |

Known remaining issue (inherited): Click drivers share one receive buffer, and a failed read can leave
a previous device's bytes in it — with certain Click combinations a calibration-based detector
(Altitude 2, PHT) can false-positive on leftovers. The detectors should check the probe's transfer
status rather than inferring presence from buffer contents.
