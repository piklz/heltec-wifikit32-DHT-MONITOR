
## DHT + onboard battery monitor logger 
### timer based multiple readings (ntfy/mqtt) + web Portal for config

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg)]()
[![Status](https://img.shields.io/badge/Status-Production%20Ready-green.svg)]()
[![Power](https://img.shields.io/badge/Power-Ultra%20Low%20Efficiency-red.svg)]()
[![ReleaseFirmware sync](https://github.com/piklz/heltec-wifikit32-DHT-MONITOR/actions/workflows/release.yml/badge.svg?event=release)](https://github.com/piklz/heltec-wifikit32-DHT-MONITOR/actions/workflows/release.yml)

# Heltec WiFi Kit 32 V2 — Temperature & Humidity + Battery MQTT Monitor

A production-grade IoT firmware for the Heltec WiFi Kit 32 V2 (ESP32).  
Reads temperature and humidity from a DHT22, publishes over MQTT, and exposes a full web dashboard — all self-contained on the device.
Can be used portably/remotely, on battery only, for a long time taking measurements and pushing out the status's via wifi

<img width="320" height="166" alt="20260531_193225   w (Phone)" src="https://github.com/user-attachments/assets/490fc011-630f-4827-b02a-3f2033a771ec" />
<img width="200" height="166" alt="61WbISgpAeL _AC_SX679_" src="https://github.com/user-attachments/assets/143f00e8-3563-4b12-8b9c-85e698ee4aa7" />

# 🛰️ heltec-wifi-kit32 (v2)
---

## Hardware

| Item | Detail |
|---|---|
| Board | Heltec WiFi Kit 32 V2 / V2.1 |
| Sensor | DHT22 — data pin GPIO 2 |
| Display | Onboard SSD1306 OLED 128x64 |
| Battery monitor | GPIO 37 (VBAT voltage divider, always live) |
| Button | GPIO 0 (onboard PRG button) |

Wire the DHT22 data line to GPIO 2. The board's onboard 3.3 V rail powers the sensor; no additional components required.

---

## Features

**Sensing & publishing**
- DHT22 temperature and humidity with configurable read interval
- Trend indicators (up/down arrows) on both OLED and web dashboard — shows direction of change since last reading
- Threshold alerts via ntfy push notifications (high/low for both channels)
- MQTT publish to Standard broker, Adafruit IO, or Ubidots (selectable or all three simultaneously)

**Web dashboard**
- Live stats panel with colour-coded values and trend arrows
- Reading history chart (last 8 readings, persisted across deep sleep in RTC memory)
- Battery voltage, charge percentage, and power source detection (USB / charging / battery-only)
- All configuration via browser — no serial required after first flash

**OTA firmware updates**
- Manual upload via web UI with server-side validation: filename check, partition size guard, FW_VER marker scan, and downgrade warning
- GitHub manifest check — polls once per 24 hours, notifies via dashboard banner and ntfy when a newer version is available
- One-click install direct from GitHub release

**Power management**
- Configurable deep sleep with timer and button wake
- Stealth wake mode (silent, display off, sensor read + publish + sleep)
- Active wake mode (full OLED, LED, web server)
- Pre-sleep shutdown of WiFi, BT, OLED, Vext, DHT pin, and CPU frequency

**OLED screensaver**
- Auto-activates after configurable idle timeout
- Four presets: Bouncing Ball, Mario Bounce, Matrix Rain, DVD Logo bounce
- Preview any preset from the web UI without saving

**Reliability**
- Watchdog timer (30 s, reconfigured during OTA to 120 s)
- Boot counter (hybrid RTC + NVS, survives deep sleep)
- Battery warning and critical alerts with ntfy and MQTT
- Double-reset detection triggers WiFi reconfiguration portal

---

## Arduino IDE Setup

**Board package** — add this URL under *File > Preferences > Additional boards manager URLs*:

```
https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.5/package_heltec_esp32_index.json
```

Install **Heltec ESP32 Series Dev-boards** from Boards Manager.

**Partition scheme** — select `Minimal SPIFFS (1.9 MB app / 190 KB SPIFFS)` under *Tools > Partition Scheme*. The default scheme does not leave enough room for OTA.

**Required libraries** (install via Library Manager unless noted):

| Library | Version tested |
|---|---|
| Heltec ESP32 Dev-Boards | 2.1.6 |
| WiFiManager (tzapu) | 2.0.17 |
| PubSubClient | 2.8 |
| Adafruit Unified Sensor | 1.1.15 |
| DHT sensor library | 1.4.7 |
| ArduinoJson | 7.4.3 |
| OneButton | 2.6.2 |
| JLed | 4.15.0 |

---

## First Run

1. Flash the firmware via USB (Sketch > Upload).
2. On first boot the device starts a WiFi access point named **ESP32-Setup**.
3. Connect to that network from your phone or laptop, then navigate to `192.168.4.1`.
4. Enter your WiFi credentials. The device reboots and connects.
5. Find the device IP on your router's DHCP list, or watch the Serial Monitor at 115200 baud — it prints the IP on connect.
6. Open the IP in a browser to reach the dashboard.

> **Double-reset to reconfigure WiFi** — reset the board twice within 2 seconds to reopen the setup AP at any time.

---

## Configuration

All settings are saved to NVS (survives power loss) and are editable from the web UI at `/settings`.

**Key settings to review on first run:**

- *Device name* — used as MQTT client ID and dashboard title
- *MQTT broker, port, topic* — standard broker credentials
- *Sensor thresholds* — temperature and humidity high/low alert levels
- *Deep sleep interval* — set to 0 to disable; otherwise sets the wake period in minutes
- *ntfy server and topic* — for push notifications (ntfy.sh or self-hosted)
- *Battery calibration* — visit `/calibrate` with a multimeter reading to correct voltage display

---

## Button Actions

| Press | Action |
|---|---|
| Single click | Reset 10-minute deep-sleep inhibit timer |
| Double click | Trigger immediate sensor read and publish |
| Triple click | Pause / resume OLED frame scrolling |
| Hold 3 s | Initiate deep sleep immediately |
| Dismiss screensaver | Any press |

---

## MQTT Payload (Standard broker)

Published to the configured topic on each read:

```json
{
  "device": "heltechome_",
  "temperature": 24.3,
  "humidity": 58.1,
  "uptime": 3600,
  "boot_count": 42,
  "batt_v": "4.05",
  "batt_pct": 91,
  "power_src": "usb",
  "batt_status": "ok",
  "ota_available": false
}
```

---

## Battery Calibration

The onboard ADC reads VBAT via a fixed 390K / 10K voltage divider. Accuracy varies between boards.

1. Navigate to `/calibrate` in the web UI.
2. Enter the voltage shown on a multimeter connected to the battery terminals.
3. Save. The calibration factor is stored in NVS and survives sleep and reset.

Two factors are stored independently: one for USB/charging mode and one for battery-only mode, since the ADC reference shifts slightly between them.

---

## OTA Updates

**Manual upload** — go to `/update`, drag your `.ino.bin` file (not `.merged.bin`) onto the upload area. The page computes a CRC32 client-side and warns on downgrade. Server-side checks validate the filename, partition fit, and embedded firmware marker before writing a single byte.

**GitHub auto-update** — if a `manifest.json` is present in the repository's `firmware/` directory, the device checks it once per 24 hours. When a newer version is found, a banner appears on the dashboard with a one-click install option.

Manifest format:
```json
{
  "version": "5.31",
  "build_date": "2026-05-31",
  "binary": "https://github.com/.../firmware.bin",
  "size": 1245184,
  "crc32": "A1B2C3D4",
  "changelog": "Short description"
}
```

---

## License

MIT
