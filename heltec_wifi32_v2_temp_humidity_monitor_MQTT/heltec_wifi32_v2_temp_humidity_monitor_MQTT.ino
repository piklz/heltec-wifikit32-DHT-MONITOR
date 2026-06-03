/*
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║     Heltec WiFi Kit 32 V2 — Temperature & Humidity + BATTERY Monitor      ║
 * ║               Web Portal + MQTT-Enabled IoT System                        ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 *
 * ██████╗░██╗██╗░░██╗██╗░░░░░███████╗
 * ██╔══██╗██║██║░██╔╝██║░░░░░╚════██║
 * ██████╔╝██║█████═╝░██║░░░░░░░███╔═╝
 * ██╔═══╝░██║██╔═██╗░██║░░░░░██╔══╝░░
 * ██║░░░░░██║██║░╚██╗███████╗███████╗
 * ╚═╝░░░░░╚═╝╚═╝░░╚═╝╚══════╝╚══════╝ESP32
 *
 * Author:        piklz
 * GitHub:        heltec-wifikit32-DHT-MONITOR
 * Repository:    github.com/piklz/heltec-wifikit32-DHT-MONITOR
 * Version:       5.34
 * Last Updated:  2026-06-03
 * License:       MIT
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * DESCRIPTION
 * ─────────────────────────────────────────────────────────────────────────────
 * A complete ESP32 IoT system for the Heltec WiFi Kit 32 V2, featuring:
 *  • DHT22/DHT11 temperature & humidity sensing
 *  • MQTT publish/subscribe with PubSubClient
 *  • Dual-mode battery voltage calibration (USB + battery modes)
 *  • OTA firmware updates via GitHub manifest
 *  • Web-based dashboard & calibration interface
 *  • WiFi Manager for easy network configuration
 *  • Deep sleep support for low-power operation
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * CHANGELOG v5.34 — 2026-06-03
 * ─────────────────────────────────────────────────────────────────────────────
 *  - FIX CRITICAL: detectDoubleReset() now skipped on timer wakes. Previously
 *         every timer wake wrote the DRD flag; the next wake read it and opened
 *         the config portal, causing the LED to stick on and WiFi to stop.
 *  - FIX: clearDoubleReset() added to fast static-IP return path so the DRD
 *         flag is always cleared after a successful connection.
 *  - FIX: NTP sync wait (2s) added to fast static-IP path — previously
 *         configTime() was called but the function returned before
 *         getLocalTime() had a chance to complete, leaving ntpSynced=false
 *         and TIME/LAST READ showing --/-- --:--.
 *  - FIX: NTP retry added in readSensor() for cases where boot-time sync
 *         failed — ensures manual double-click reads always show a timestamp.
 *  - FIX: Date format in readSensor() corrected from US %m/%d to UK %d/%m.
 *  - FIX: Web page load no longer unconditionally resets the 10-min sleep
 *         window. Timer wakes keep their short window; only button wakes or
 *         already-extended windows get refreshed to 10 min.
 *  - OPT: Timer wake awake window reduced from 45s to 8s. Publish+flush
 *         completes in ~2-3s; 8s gives headroom without wasting battery.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * CHANGELOG v5.33 — 2026-06-03
 * ─────────────────────────────────────────────────────────────────────────────
 *  - NEW: Static IP caching — after first DHCP, IP/GW/subnet/DNS 
 *
 *
 */


// ── Core / ESP-IDF ────────────────────────────────────────────────────────────
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <esp_wifi.h>       // esp_wifi_stop() for guaranteed radio power-down
#include <esp_bt.h>         // esp_bt_controller_disable() for guaranteed BT off
#include <esp_ota_ops.h>    // esp_ota_get_running_partition() for running CRC32

// ── Display ───────────────────────────────────────────────────────────────────
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "HT_DisplayUi.h"

// ── WiFi / Networking ─────────────────────────────────────────────────────────
#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <Update.h>

// ── MQTT ──────────────────────────────────────────────────────────────────────
#include <PubSubClient.h>

// ── Sensor ────────────────────────────────────────────────────────────────────
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// ── Storage / Config ──────────────────────────────────────────────────────────
#include <Preferences.h>
#include <ArduinoJson.h>

// ── Button ────────────────────────────────────────────────────────────────────
#include <OneButton.h>

// ── LED ───────────────────────────────────────────────────────────────────────
#include <jled.h>
#include <HTTPClient.h>     // ntfy push notifications

// ── Double Reset Detector ─────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// BATTERY MONITOR — Heltec WiFi Kit 32 V2.1 hardware facts:
//   GPIO37 reads the VBAT voltage divider (390K + 10K, always live, no gate).
//   GPIO21 = Vext (OLED power rail) — NEVER toggle for battery reads.
//   BAT_CAL_CONST = real_V * 4095 / avg_raw  (adjust from your multimeter).
//   USB detection: ADC only ever sees VBAT. Charging raises it above ~4.05V.
//   USB-only / no battery = ADC floats; caught by variance check (isBatFloating).
// ─────────────────────────────────────────────────────────────────────────────
#define FW_VERSION            "5.34"   // keep in sync with VERSION comment at top
// This combines the text and macro into a single, permanent binary stamp
const char* fw_binary_signature = "FW_VER:" FW_VERSION;

// Firmware version marker — embedded in .rodata so the OTA page JS can find
// the version reliably by scanning for the "FW_VER:" prefix in the binary.
static const char FW_VER_MARKER[] = "FW_VER:" FW_VERSION;

// ── GitHub OTA Update Check ───────────────────────────────────────────────────
// manifest.json expected at MANIFEST_URL with fields:
//   { "version":"5.21", "build_date":"2026-05-20",
//     "binary":"https://.../.../firmware.bin",
//     "size":1048576, "crc32":"A1B2C3D4",
//     "changelog":"Short description of changes" }
// crc32 field is uppercase hex, 8 chars (CRC32/ISO-HDLC, same algo as JS client).
#define MANIFEST_URL          "https://raw.githubusercontent.com/piklz/heltec-wifikit32-DHT-MONITOR/main/firmware/manifest.json"
// Check at most once per day. Persisted in NVS so deep-sleep wakes don't reset the clock.
// Falls back to checking every wake if NTP isn't synced yet (epoch timestamp = 0).
#define OTA_CHECK_INTERVAL_SECS  86400UL   // 24 hours between remote manifest fetches
#define OTA_NTFY_COOLDOWN_MS     86400000UL   // max one ntfy per 24 h for same version

bool     otaUpdateAvailable = false;
bool     otaCheckDone       = false;   // set after first check this wake
bool     otaDismissed       = false;   // user clicked Dismiss on dashboard
static bool     uploadRejected      = false;   // set on any pre-flash check failure
static String   uploadRejectReason  = "";
static bool     uploadMarkerFound   = false;   // FW_VER: marker seen in binary
static uint32_t uploadBytesScanned  = 0;       // bytes searched for marker so far
String   otaNewVersion      = "";
String   otaDownloadUrl     = "";
String   otaCrc32Expected   = "";      // uppercase hex 8-char
String   runningCrc32       = "";      // CRC32 of the running sketch, computed once in setup()
uint32_t otaFileSize        = 0;
String   otaChangelog       = "";
String   otaBuildDate       = "";
unsigned long lastOtaCheck  = 0;       // millis() of last check this wake (for loop guard)
unsigned long lastOtaNtfy   = 0;       // millis() of last ntfy send THIS wake (in-session dedup)
// NVS epoch for cross-wake ntfy cooldown stored under "ota"/"ntfy_ep" (uint32 unix epoch)

// Survives deep sleep — set true when a newer version is confirmed available.
// Allows OLED update-available icon to persist across wake cycles without
// re-fetching the manifest on every single wake.
RTC_DATA_ATTR bool rtcOtaAvailable = false;

// ── CRC32 (ISO-HDLC / zlib polynomial 0xEDB88320) ────────────────────────────
// Table is built once in setup() by buildCrc32Table().
// Same algorithm as the client-side JS so computed values match exactly.
static uint32_t crc32Table[256];

void buildCrc32Table() {
  for (int n = 0; n < 256; n++) {
    uint32_t c = (uint32_t)n;
    for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320UL ^ (c >> 1) : c >> 1;
    crc32Table[n] = c;
  }
}

// Feed bytes into a running CRC32. Start with crc=0xFFFFFFFF,
// XOR final result with 0xFFFFFFFF to get the canonical value.
inline uint32_t crc32Feed(uint32_t crc, const uint8_t* buf, size_t len) {
  for (size_t i = 0; i < len; i++) crc = (crc >> 8) ^ crc32Table[(crc ^ buf[i]) & 0xFF];
  return crc;
}

#define BATTERY_PIN           37
// Battery voltage = analogReadMilliVolts(37) * VDIV_FACTOR / 1000
// VDIV_FACTOR = real_V * 1000 / analogReadMilliVolts(37)
// Calibrated: 3.92V real -> AMV~1463mV -> 3920/1463 = 2.68
// analogReadMilliVolts uses ESP32 eFuse ADC calibration = accurate across range.
// To recalibrate: visit /calibrate in web UI, enter multimeter reading.
#define VDIV_FACTOR_DEFAULT   2.68f
#define BAT_USB_THRESHOLD     4.08f  // USB floor on this hardware ~4.10-4.12V, battery max ~3.95V.
                                     // 4.08V sits cleanly in the ~150mV gap between them.
                                     // (Was 4.15V in v5.7 — too high, USB was never detected.)
// USB-only (no battery): GPIO37 VBAT divider with no battery sees a near-zero or very
// low floating voltage — typically < 0.5V through the divider (< ~200mV ADC).
// With battery+USB the ADC reads actual VBAT (>4.0V scaled) above BAT_USB_THRESHOLD.
// We detect USB-only by combining: (a) voltage implausibly below any real LiPo floor
// AND/OR (b) variance below 25mV indicating a truly floating/settled no-load signal.
#define BAT_FLOAT_VOLTAGE     2.8f   // Below this V, no real LiPo cell is present
#define BAT_USB_HYSTERESIS_MS 30000UL  // 30s instead of 15s
#define BATTERY_READ_MS       5000UL
#define BATTERY_WARN_PCT      20
#define BATTERY_CRIT_PCT       5

int   batteryVoltage    = 0;
float batteryVoltFloat  = 0.0f;
int   batteryPercentage = 0;
bool  isUSBPowered      = false;
bool  isBatFloating     = false;
bool  hadHighVoltage    = false;   // guard: hysteresis only fires after ≥1 genuine high-V read
unsigned long lastBatteryRead    = 0;
unsigned long lastHighVoltageTime = 0;
float learnedVFull    = 3.92f;  // battery-only max for this pack, adaptive
float vdivFactor      = VDIV_FACTOR_DEFAULT;  // USB / floating calibration factor — loaded from NVS "battery"/"vdiv"
float vdivFactorBat   = VDIV_FACTOR_DEFAULT;  // battery-only calibration factor  — loaded from NVS "battery"/"vdiv_bat"
                                               // Falls back to vdivFactor if never calibrated in battery mode.
float lastRawAvgMv    = 0.0f;  // raw ADC mV from last battery_read() or boot read
                                // used by /calibrate to avoid back-calc race when
                                // mode may change between read and form submit
bool  batteryWarnSent    = false;

// Sensor threshold alerts
float sensorTempHi     = 35.0f;   // °C high warn
float sensorTempLo     =  0.0f;   // °C low warn
float sensorHumHi      = 85.0f;   // % high warn
float sensorHumLo      = 10.0f;   // % low warn
bool  ntfy_on_sensor   = true;    // sensor threshold alerts enabled
bool  tempHiSent       = false;   // alert-sent flags (reset on recovery)
bool  tempLoSent       = false;
bool  humHiSent        = false;
bool  humLoSent        = false;

// XBM battery icons inlined from images.h (PROGMEM) ─────────────────────────
#define BAT_width     20
#define BAT_height     9
const unsigned char BAT_bits[] PROGMEM = {
  0xFC,0xFF,0x0F,0x04,0x00,0x08,0xF7,0xDE,0x0B,0xF1,0xDE,0x0B,
  0xF1,0xDE,0x0B,0xF1,0xDE,0x0B,0xF7,0xDE,0x0B,0x04,0x00,0x08,
  0xFC,0xFF,0x0F,
};
#define BATHALF_width  15
#define BATHALF_height  9
const unsigned char BATHALF_bits[] PROGMEM = {
  0xFF,0x0F,0x00,0x08,0xDE,0x0B,0xDE,0x0B,
  0xDE,0x0B,0xDE,0x0B,0xDE,0x0B,0x00,0x08,0xFF,0x0F,
};
#define BATLOW_width    5
#define BATLOW_height   9
const unsigned char BATLOW_bits[] PROGMEM = {
  0x0F,0x08,0x0B,0x0B,0x0B,0x0B,0x0B,0x08,0x0F,
};

// ─────────────────────────────────────────────────────────────────────────────
// NTFY PUSH NOTIFICATIONS — HTTP POST to ntfy.sh or self-hosted instance.
// ─────────────────────────────────────────────────────────────────────────────
// Verbosity levels (all saved in NVS under "ntfy" namespace):
//   ntfy_enabled      -- master switch; if false, nothing is sent
//   ntfy_on_batt      -- low/critical battery alerts (default ON; fires even if
//                        ntfy_on_publish is off -- battery safety is not optional)
//   ntfy_on_boot      -- boot/wake summary (same payload as first publish + context)
//   ntfy_on_sleep     -- "going to sleep" notification before each deep sleep
//   ntfy_on_publish   -- send full sensor summary on every periodic publish
// ─────────────────────────────────────────────────────────────────────────────
String ntfy_server       = "ntfy.sh";   // host only, no trailing slash
String ntfy_topic        = "";           // topic name
String ntfy_token        = "";           // Bearer token, blank = no auth
bool   ntfy_enabled      = false;        // master on/off
bool   ntfy_on_batt      = true;         // low/critical battery (default ON)
bool   ntfy_on_boot      = true;         // boot/wake summary   (default ON)
bool   ntfy_on_sleep     = false;        // pre-sleep notification
bool   ntfy_on_publish   = false;        // every periodic publish
String ntfy_last_msg     = "";           // last message sent (for display)
String ntfy_last_time    = "--:--";      // HH:MM of last send
unsigned long ntfy_last_millis = 0;

// ─────────────────────────────────────────────────────────────────────────────
// POWER SAVE — pre-sleep peripheral shutdown. Each ps_* flag controls one
// subsystem. ps_cpu_wake_mhz: active CPU freq (80/160/240 MHz). Saved in NVS.
// ─────────────────────────────────────────────────────────────────────────────
bool ps_wifi       = true;  // disconnect WiFi + esp_wifi_stop() before sleep
bool ps_bt         = true;  // btStop() + esp_bt_controller_disable() before sleep
bool ps_vext       = true;  // VextOFF() — cuts OLED + Vext-powered peripherals
bool ps_oled       = true;  // display.displayOff() before sleep
bool ps_dht        = true;  // float DHT pin (INPUT, no pull) before sleep
bool ps_cpu        = true;  // lower CPU to 10 MHz during brief pre-sleep window
uint32_t ps_cpu_wake_mhz = 240; // CPU MHz to use during the wake window

// Double-reset detection via NVS (survives hard resets).
// On boot: if "flag" already set from previous boot → double reset.
// Flag is cleared after WiFi connects (clearDoubleReset()).
#define DRD_TIMEOUT_MS 2000

// ─────────────────────────────────────────────────────────────────────────────
// Hardware pins & constants
// ─────────────────────────────────────────────────────────────────────────────
#define DHT_PIN          2
#define DHT_TYPE         DHT22
#define BUTTON_PIN       0     // Heltec V2 onboard button
#define LED_PIN          LED_BUILTIN
#define WDT_TIMEOUT_SEC  30    // Watchdog timeout — board resets if loop blocks longer

// ─────────────────────────────────────────────────────────────────────────────
// Deep sleep state  (RTC memory survives deep sleep)
// ─────────────────────────────────────────────────────────────────────────────
// ── Boot counter — hybrid RTC + NVS strategy ─────────────────────────────────
// RTC memory survives deep sleep but is wiped by power-loss and hard reset.
// NVS survives everything but has finite flash write endurance.
//
// Strategy: sleep wakes (timer/button) increment RTC only — zero NVS writes.
//           power-on / crash writes NVS — this IS the event worth tracking.
//
// bootCount (displayed everywhere) = nvsBootBase + rtcBootOffset
//   nvsBootBase   — last value committed to NVS (updated on power-on only)
//   rtcBootOffset — sleep-wake count since last power-on (wiped on power-loss)
//
// Flash writes drop from every boot to only on genuine power-on/crash,
// while the displayed counter still increments on every wake for continuity.
RTC_DATA_ATTR uint32_t rtcBootOffset      = 0;    // sleep-wake counter, wiped on power-loss
RTC_DATA_ATTR bool     rtcDeepSleepEnabled = false;

// ── Static IP cache (RTC) — populated after first DHCP, reused on timer wakes
// to skip DHCP negotiation and cut WiFi connect time from ~3s to ~400ms.
// All four fields stored as uint32_t (network byte order from IPAddress).
RTC_DATA_ATTR uint32_t rtcIp      = 0;
RTC_DATA_ATTR uint32_t rtcGw      = 0;
RTC_DATA_ATTR uint32_t rtcSn      = 0;
RTC_DATA_ATTR uint32_t rtcDns     = 0;
RTC_DATA_ATTR bool     rtcIpValid = false;  // true once first DHCP succeeded

// Stealth wake flush window — shorter than the normal 5 s to minimise wake time.
// MQTT TCP ACKs typically complete within 500-800 ms; 1500 ms gives headroom.
#define STEALTH_FLUSH_MS  1500UL
#define HIST_SIZE 8
RTC_DATA_ATTR float    rtcHistTemp[HIST_SIZE] = {};
RTC_DATA_ATTR float    rtcHistHum[HIST_SIZE]  = {};
RTC_DATA_ATTR uint32_t rtcHistTime[HIST_SIZE] = {};
RTC_DATA_ATTR uint8_t  rtcHistCount           = 0;
RTC_DATA_ATTR uint8_t  rtcHistHead            = 0;
int                    bootCount           = 0;    // combined display value, set in setup()

bool          deepSleepEnabled         = false;
uint32_t      deepSleepMinutes         = 10;
unsigned long deepSleepSeconds         = 600;
// Wake display mode: 1=Active (OLED+LED on during wake), 0=Stealth (silent, eco)
// Button-triggered wakes always show Active regardless of this setting.
uint8_t       wakeDisplayMode           = 1;   // default: Active
bool          stealthThisWake           = false; // set in setup() before ui.init()
// disableDeepSleepUntil is set at end of setup() — NEVER before WiFiManager
// webUiExpiresAt: unified 10-min web UI window. Any button press, page load,
// or settings save resets this. Deep sleep is blocked while millis() < webUiExpiresAt.
unsigned long webUiExpiresAt          = 0;
#define disableDeepSleepUntil webUiExpiresAt   // back-compat alias — existing code unchanged
unsigned long lastSensorPublishTime    = 0;
bool          triggerDeepSleepAfterPublish = false;

// ─────────────────────────────────────────────────────────────────────────────
// Display / UI
// ─────────────────────────────────────────────────────────────────────────────
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
DisplayUi ui(&display);

bool     portalActive      = false;  // true while WiFiManager AP is open
bool     ntpSynced         = false;
String   currentTimeStr    = "--:--";
String   currentDateStr    = "--/--";

// Display Care — OLED pixel exercise
bool          screenCleanActive   = false;
unsigned long screenCleanUntil    = 0;
unsigned long screenCleanToggle   = 0;
uint16_t      screenCleanDuration = 60;
uint8_t       screenCleanPreset   = 0;  // 0=Checkerboard 1=InvertRamp 2=Scanline
bool     scrollPaused      = false;  // toggled by 3-click
int      globalHoldSeconds = 0;      // >0: button hold overlay (countdown to sleep trigger)
int      globalSleepCountdown = 0;   // >0: imminent-sleep overlay (3..2..1 before sleep)
// ── OLED Screensaver ──────────────────────────────────────────────────────START
bool     ssaverEnabled     = false;  // master on/off
uint32_t ssaverTimeoutSecs = 60;     // idle seconds before activation
uint8_t  ssaverPreset      = 0;      // 0=Bouncing Ball  1=Mario  2=Matrix Rain  3=DVD Logo
bool     ssaverActive      = false;  // screensaver currently running
bool     ssaverPreview     = false;  // preview mode — auto-exits after 10 s
uint8_t  ssaverSavedPreset = 0;      // preset stored before a preview overrides it
unsigned long lastActivityMs  = 0;   // millis() of last user / display activity
unsigned long ssaverFrameMs   = 0;   // internal per-animation frame timer
unsigned long ssaverStartMs   = 0;   // millis() when screensaver activated
// Bouncing Ball state
static float bbX = 32, bbY = 16, bbVx = 1.7f, bbVy = 1.1f;
// Mario state
static float  mrX = 20, mrY = 30, mrVx = 1.5f, mrVy = 0.0f;
static uint8_t mrFrame = 0;
static unsigned long mrLastMoveMs = 0, mrLastFrameMs = 0;

// Matrix Rain state
#define MX_COLS    21    // 21 columns × 6 px = 126 px wide
#define MX_COL_PX   6   // pixels per column (matches ArialMT_Plain_10 glyph width)
#define MX_ROWS     8   // 64 px / 8 px per glyph = 8 character rows
static int16_t mxHead[MX_COLS];
static uint8_t mxSpeed[MX_COLS];
static uint8_t mxLen[MX_COLS];
// Per-cell characters — glyphs are randomised on init and mutate slowly each frame.
static char    mxChar[MX_COLS][MX_ROWS];
static bool    mxInited = false;
// Characters that visually resemble the Matrix katakana font at 6×8 pixels
static const char MX_CHARSET[] =
  "0123456789"    // digits
  "+-*=|:<>!?"    // angular/symmetric symbols
  "#@$%^&~"       // dense/complex glyphs
  "TZIJL"         // strong verticals / horizontals
  "/\\[]{}";      // bracket forms echo ｢ ｣ ｦ marks

// DVD Logo Bounce state
// Real 56×26 px bitmap traced from the DVD logo — drawXbm renders it directly.
// Bounce area: 72 px wide × 38 px tall on the 128×64 OLED.
#define DVD_LOGO_W  56
#define DVD_LOGO_H  26
static const uint8_t DVD_LOGO[] PROGMEM = {
  0xE0, 0xFF, 0xFF, 0x01, 0xF8, 0xFF, 0x03, 0xE0,
  0xFF, 0xFF, 0x03, 0xFC, 0xFF, 0x1F, 0xF0, 0xFF,
  0xFF, 0x03, 0xFE, 0xFF, 0x3F, 0x00, 0xE0, 0xFF,
  0x03, 0x3E, 0x00, 0x7F, 0x00, 0x80, 0xFF, 0x07,
  0x1F, 0x00, 0xFC, 0xF0, 0x01, 0xDF, 0x87, 0xDF,
  0x07, 0xFC, 0xF8, 0x01, 0xDF, 0xC7, 0xCF, 0x07,
  0xFC, 0xF8, 0x00, 0x9F, 0xCF, 0xC7, 0x07, 0xFC,
  0xF8, 0x80, 0x9F, 0xEF, 0xE3, 0x07, 0x7C, 0xF8,
  0x80, 0x8F, 0xFF, 0xE1, 0x07, 0x7E, 0xF8, 0xE0,
  0x0F, 0xFF, 0xE0, 0x03, 0x3F, 0xFC, 0xFE, 0x03,
  0xFF, 0xE0, 0xF7, 0x1F, 0xFC, 0xFF, 0x01, 0x7F,
  0xE0, 0xFF, 0x07, 0xFC, 0x7F, 0x00, 0x3E, 0xF0,
  0xFF, 0x01, 0xFC, 0x07, 0x00, 0x1E, 0xF0, 0x1F,
  0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x80, 0xFF, 0xFF,
  0xFF, 0xFF, 0x3F, 0x00, 0xFC, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x0F, 0x80, 0xFF,
  0xFF, 0x0F, 0xFF, 0xFF, 0x0F, 0x80, 0xFF, 0xFF,
  0x0F, 0xFC, 0xFF, 0xFF, 0xF8, 0xFF, 0xFF, 0x07,
  0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00,
  0xC0, 0xFF, 0xFF, 0x3F, 0x00, 0x00
};
static float dvdX = 10, dvdY = 10, dvdVx = 1.3f, dvdVy = 0.9f;
static bool  dvdCornerFlash = false;
static unsigned long dvdFlashMs = 0;
// ── OLED Screensaver ──────────────────────────────────────────────────────END

// ─────────────────────────────────────────────────────────────────────────────
// Sensor
// ─────────────────────────────────────────────────────────────────────────────
DHT_Unified dht(DHT_PIN, DHT_TYPE);
float temperature = 0;
float humidity    = 0;

// ── Trend tracking — previous reading for up/down arrow indicators ────────────
// Dead-bands filter ADC noise: arrows only appear on genuine movement.
// trendTemp/trendHumidity: -1=falling  0=stable  1=rising
#define TREND_DEAD_TEMP  0.3f   // °C  — ignore changes smaller than this
#define TREND_DEAD_HUM   0.5f   // %RH — ignore changes smaller than this
int8_t  trendTemp     = 0;
int8_t  trendHumidity = 0;

// ─────────────────────────────────────────────────────────────────────────────
// MQTT clients — one per platform
// ─────────────────────────────────────────────────────────────────────────────
WiFiClient   espClientStd, espClientAIO, espClientUBI;
PubSubClient mqttStd(espClientStd);
PubSubClient mqttAIO(espClientAIO);
PubSubClient mqttUBI(espClientUBI);

#define MQTT_BUFFER_SIZE 512
#define MQTT_KEEPALIVE   30

// ─────────────────────────────────────────────────────────────────────────────
// Web server & storage
// ─────────────────────────────────────────────────────────────────────────────
WebServer   server(80);
Preferences preferences;

// ─────────────────────────────────────────────────────────────────────────────
// Runtime config (loaded from NVS on boot)
// ─────────────────────────────────────────────────────────────────────────────
String mqtt_platform  = "standard";
String mqtt_server    = "";
String mqtt_port      = "1883";
String mqtt_user      = "";
String mqtt_pass      = "";
String mqtt_topic     = "esp32/sensor";
String device_name    = "ESP32-Sensor";
String aio_username   = "";
String aio_key        = "";
String ubidots_token  = "";
String ubidots_device = "";

unsigned long publishSeconds = 60;
unsigned long readInterval   = 60000UL;
unsigned long lastRead        = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Connection state
// ─────────────────────────────────────────────────────────────────────────────
bool wifiConnected         = false;
bool mqttStandardConnected = false;
bool mqttAdafruitConnected = false;
bool mqttUbidotsConnected  = false;

// ─────────────────────────────────────────────────────────────────────────────
// MQTT retry state
// ─────────────────────────────────────────────────────────────────────────────
unsigned long lastStandardRetry = 0, lastAdafruitRetry = 0, lastUbidotsRetry = 0;
unsigned long lastStdReset = 0,      lastAioReset = 0,      lastUbiReset = 0;
const unsigned long RETRY_INTERVAL       = 30000UL;
const unsigned long RETRY_RESET_INTERVAL = 300000UL;
const int           MAX_RETRIES          = 3;
int stdRetries = 0, aioRetries = 0, ubiRetries = 0;

// ─────────────────────────────────────────────────────────────────────────────
// WiFi reconnect state
// ─────────────────────────────────────────────────────────────────────────────
unsigned long lastWifiCheck            = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000UL;

// ─────────────────────────────────────────────────────────────────────────────
// Button & LED objects
// ─────────────────────────────────────────────────────────────────────────────
OneButton button(BUTTON_PIN, true, true);  // active-LOW, internal pull-up
JLed      led(LED_PIN);

// ─────────────────────────────────────────────────────────────────────────────
// Double reset detector
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// WiFiManager custom parameters
// ─────────────────────────────────────────────────────────────────────────────
WiFiManagerParameter p_platform ("platform",  "MQTT Platform (standard/adafruit/ubidots/all)", "standard", 20);
WiFiManagerParameter p_server   ("server",    "MQTT Server",         "",              40);
WiFiManagerParameter p_port     ("port",      "MQTT Port",           "1883",           6);
WiFiManagerParameter p_user     ("user",      "MQTT User",           "",              40);
WiFiManagerParameter p_pass     ("pass",      "MQTT Password",       "",              40);
WiFiManagerParameter p_topic    ("topic",     "MQTT Topic",          "esp32/sensor",  50);
WiFiManagerParameter p_name     ("name",      "Device Name",         "ESP32-Sensor",  30);
WiFiManagerParameter p_timer    ("pub_timer", "Publish every (sec)", "600",             5,
                                 "type='number' min='30' max='3600'");
WiFiManagerParameter h_aio      ("<h4 style='color:#64B5F6;border-bottom:2px solid #64B5F6;"
                                 "padding-bottom:5px;margin-top:10px'>Adafruit IO</h4>");
WiFiManagerParameter p_aio_user ("aio_user",  "AIO Username",        "",              40);
WiFiManagerParameter p_aio_key  ("aio_key",   "AIO Key",             "",              40);
WiFiManagerParameter h_ubi      ("<h4 style='color:#FF8A65;border-bottom:2px solid #FF8A65;"
                                 "padding-bottom:5px;margin-top:10px'>Ubidots</h4>");
WiFiManagerParameter p_ubi_tok  ("ubi_token", "Ubidots Token",       "",              60);
WiFiManagerParameter p_ubi_dev  ("ubi_device","Device Label",        "",              30);
WiFiManagerParameter h_sleep    ("<h4 style='color:#FF7043;border-bottom:2px solid #FF7043;"
                                 "padding-bottom:5px;margin-top:10px'>Deep Sleep</h4>");
WiFiManagerParameter p_ds_en    ("deep_enable","Enable Deep Sleep",  "0",              2,
                                 "type='checkbox'");
WiFiManagerParameter p_ds_min   ("deep_min",  "Wake every (min)",    "10",             5,
                                 "type='number' min='1' max='1440'");

// ═════════════════════════════════════════════════════════════════════════════
// UTILITY HELPERS
// ═════════════════════════════════════════════════════════════════════════════

// Returns human-readable uptime string e.g. "2d 3h 15m 42s"
String getUptime() {
  unsigned long t = millis() / 1000;
  int d = t / 86400, h = (t % 86400) / 3600, m = (t % 3600) / 60, s = t % 60;
  String u = "";
  if (d) u += String(d) + "d ";
  if (h || d) u += String(h) + "h ";
  if (m || h || d) u += String(m) + "m ";
  u += String(s) + "s";
  return u;
}

// Logs the hardware reset reason to Serial — first thing called in setup()
void printResetReason() {
  Serial.print("[BOOT] Reset reason: ");
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  Serial.println("Power on"); break;
    case ESP_RST_SW:       Serial.println("Software (ESP.restart)"); break;
    case ESP_RST_PANIC:    Serial.println("Exception/panic"); break;
    case ESP_RST_INT_WDT:  Serial.println("Interrupt watchdog"); break;
    case ESP_RST_TASK_WDT: Serial.println("Task watchdog — loop blocked!"); break;
    case ESP_RST_BROWNOUT: Serial.println("Brownout — check PSU"); break;
    case ESP_RST_DEEPSLEEP:Serial.println("Deep sleep wakeup"); break;
    default:               Serial.println("Other/unknown"); break;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// VEXT / OLED POWER
// ═════════════════════════════════════════════════════════════════════════════
void VextON()  { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW);  }
void VextOFF() { pinMode(Vext, OUTPUT); digitalWrite(Vext, HIGH); }

// ═════════════════════════════════════════════════════════════════════════════
// NVS — LOAD / SAVE
// ═════════════════════════════════════════════════════════════════════════════

// Load MQTT / device settings from "mqtt-config" namespace
void loadConfig() {
  preferences.begin("mqtt-config", true);
  mqtt_platform  = preferences.getString("platform",  "standard");
  mqtt_server    = preferences.getString("server",    "");
  mqtt_port      = preferences.getString("port",      "1883");
  mqtt_user      = preferences.getString("user",      "");
  mqtt_pass      = preferences.getString("pass",      "");
  mqtt_topic     = preferences.getString("topic",     "esp32/sensor");
  device_name    = preferences.getString("name",      "ESP32-Sensor");
  aio_username   = preferences.getString("aio_user",  "");
  aio_key        = preferences.getString("aio_key",   "");
  ubidots_token  = preferences.getString("ubi_token", "");
  ubidots_device = preferences.getString("ubi_device","");
  preferences.end();
}

// Load ntfy settings from "ntfy" namespace
void loadNtfyConfig() {
  preferences.begin("ntfy", true);
  ntfy_server     = preferences.getString("server",     "ntfy.sh");
  ntfy_topic      = preferences.getString("topic",      "");
  ntfy_token      = preferences.getString("token",      "");
  ntfy_enabled    = preferences.getBool  ("enabled",    false);
  ntfy_on_batt    = preferences.getBool  ("on_batt",    true);   // default ON
  ntfy_on_boot    = preferences.getBool  ("on_boot",    true);   // default ON
  ntfy_on_sleep   = preferences.getBool  ("on_sleep",   false);
  ntfy_on_publish = preferences.getBool  ("on_publish", false);
  ntfy_on_sensor  = preferences.getBool  ("on_sensor",  true);
  sensorTempHi    = preferences.getFloat ("t_hi",       35.0f);
  sensorTempLo    = preferences.getFloat ("t_lo",        0.0f);
  sensorHumHi     = preferences.getFloat ("h_hi",       85.0f);
  sensorHumLo     = preferences.getFloat ("h_lo",       10.0f);
  preferences.end();
}

// Save ntfy settings to "ntfy" namespace
void saveNtfyConfig() {
  preferences.begin("ntfy", false);
  preferences.putString("server",     ntfy_server);
  preferences.putString("topic",      ntfy_topic);
  preferences.putString("token",      ntfy_token);
  preferences.putBool  ("enabled",    ntfy_enabled);
  preferences.putBool  ("on_batt",    ntfy_on_batt);
  preferences.putBool  ("on_boot",    ntfy_on_boot);
  preferences.putBool  ("on_sleep",   ntfy_on_sleep);
  preferences.putBool  ("on_publish", ntfy_on_publish);
  preferences.putBool  ("on_sensor",  ntfy_on_sensor);
  preferences.putFloat ("t_hi",       sensorTempHi);
  preferences.putFloat ("t_lo",       sensorTempLo);
  preferences.putFloat ("h_hi",       sensorHumHi);
  preferences.putFloat ("h_lo",       sensorHumLo);
  preferences.end();
}

// Load deep-sleep and publish-interval settings from "deep" namespace
void loadDeepSleepConfig() {
  preferences.begin("deep", true);
  deepSleepEnabled = preferences.getBool  ("enable",    false);
  deepSleepMinutes = preferences.getUInt  ("minutes",   10);
  publishSeconds   = preferences.getULong ("pub_sec",   600);
  wakeDisplayMode  = preferences.getUChar ("wake_disp", 1);   // 1=Active, 0=Stealth
  preferences.end();

  // Clamp / validate
  if (wakeDisplayMode > 1) wakeDisplayMode = 1;
  deepSleepMinutes = max((uint32_t)1, deepSleepMinutes);
  deepSleepSeconds = deepSleepMinutes * 60UL;
  publishSeconds   = constrain(publishSeconds, 30UL, 3600UL);
  readInterval     = publishSeconds * 1000UL;

  // Mirror to RTC so wakeup path can check without reading NVS
  rtcDeepSleepEnabled = deepSleepEnabled;

  // Sync WiFiManager parameter display values
  p_ds_en.setValue(deepSleepEnabled ? "1" : "0", 2);
  p_ds_min.setValue(String(deepSleepMinutes).c_str(), 5);
  p_timer.setValue(String(publishSeconds).c_str(), 5);
}

// Save all MQTT settings in one clean begin/end block
void saveMqttConfig() {
  preferences.begin("mqtt-config", false);
  preferences.putString("platform",   mqtt_platform);
  preferences.putString("server",     mqtt_server);
  preferences.putString("port",       mqtt_port);
  preferences.putString("user",       mqtt_user);
  preferences.putString("pass",       mqtt_pass);
  preferences.putString("topic",      mqtt_topic);
  preferences.putString("name",       device_name);
  preferences.putString("aio_user",   aio_username);
  preferences.putString("aio_key",    aio_key);
  preferences.putString("ubi_token",  ubidots_token);
  preferences.putString("ubi_device", ubidots_device);
  preferences.end();
}

// Save deep-sleep settings — always to "deep" namespace to match loadDeepSleepConfig()
void saveDeepConfig() {
  preferences.begin("deep", false);
  preferences.putBool ("enable",    deepSleepEnabled);
  preferences.putUInt ("minutes",   deepSleepMinutes);
  preferences.putULong("pub_sec",   publishSeconds);
  preferences.putUChar("wake_disp", wakeDisplayMode);
  preferences.end();
  rtcDeepSleepEnabled = deepSleepEnabled;
}

// Load power-save flags from "pwrsave" namespace
void loadPowerSaveConfig() {
  preferences.begin("pwrsave", true);
  ps_wifi         = preferences.getBool ("ps_wifi",   true);
  ps_bt           = preferences.getBool ("ps_bt",     true);
  ps_vext         = preferences.getBool ("ps_vext",   true);
  ps_oled         = preferences.getBool ("ps_oled",   true);
  ps_dht          = preferences.getBool ("ps_dht",    true);
  ps_cpu          = preferences.getBool ("ps_cpu",    true);
  ps_cpu_wake_mhz = preferences.getUInt ("wake_mhz",  240);
  preferences.end();
  if (ps_cpu_wake_mhz != 80 && ps_cpu_wake_mhz != 160 && ps_cpu_wake_mhz != 240)
    ps_cpu_wake_mhz = 240;
}

// Load screensaver settings from "oled" namespace
void loadOledConfig() {
  preferences.begin("oled", true);
  ssaverEnabled     = preferences.getBool ("ss_en",      false);
  ssaverTimeoutSecs = preferences.getUInt ("ss_timeout", 60);
  ssaverPreset      = preferences.getUChar("ss_preset",  0);
  preferences.end();
  if (ssaverPreset > 3)          ssaverPreset      = 0;
  if (ssaverTimeoutSecs < 10)    ssaverTimeoutSecs = 10;
}

void saveOledConfig() {
  preferences.begin("oled", false);
  preferences.putBool ("ss_en",      ssaverEnabled);
  preferences.putUInt ("ss_timeout", ssaverTimeoutSecs);
  preferences.putUChar("ss_preset",  ssaverPreset);
  preferences.end();
}

// Save power-save flags to "pwrsave" namespace
void savePowerSaveConfig() {
  preferences.begin("pwrsave", false);
  preferences.putBool ("ps_wifi",  ps_wifi);
  preferences.putBool ("ps_bt",    ps_bt);
  preferences.putBool ("ps_vext",  ps_vext);
  preferences.putBool ("ps_oled",  ps_oled);
  preferences.putBool ("ps_dht",   ps_dht);
  preferences.putBool ("ps_cpu",   ps_cpu);
  preferences.putUInt ("wake_mhz", ps_cpu_wake_mhz);
  preferences.end();
}

// ═════════════════════════════════════════════════════════════════════════════
// DOUBLE RESET DETECTION — NVS based
// ═════════════════════════════════════════════════════════════════════════════
bool detectDoubleReset() {
  preferences.begin("drd", false);
  unsigned long now = millis(); (void)now;  // millis() is ~0 at this point
  bool flagSet = preferences.getBool("flag", false);
  if (flagSet) {
    // Flag was set on previous boot — this IS a double reset
    preferences.putBool("flag", false);  // clear so triple-reset doesn't re-trigger
    preferences.end();
    Serial.println("[DRD] Double reset detected — forcing config portal");
    return true;
  } else {
    // First reset — set flag; it will be cleared either by a second boot
    // detecting it, or by clearDoubleReset() after normal startup completes
    preferences.putBool("flag", true);
    preferences.end();
    // Schedule flag clear after DRD_TIMEOUT_MS using a simple millis check in loop
    return false;
  }
}

// Call after successful WiFi connect to cancel the DRD window
void clearDoubleReset() {
  preferences.begin("drd", false);
  preferences.putBool("flag", false);
  preferences.end();
}

// ═════════════════════════════════════════════════════════════════════════════
// BATTERY MONITOR — voltsToPercent + battery_read + checkBatteryAlerts
// ═════════════════════════════════════════════════════════════════════════════

// Piecewise LiPo discharge curve: breakpoints verified against real hardware.
// Voltage  : 4.20  4.10  4.00  3.90  3.80  3.75  3.70
// Percent  : 100    95    90    80    70    60    50
// Voltage  : 3.65  3.60  3.50  3.42  3.30  3.20  3.00
// Percent  :  40    30    20    10     5     2     0
int voltsToPercent(float v) {
  // USB/charging: always 100% (voltage above BAT_USB_THRESHOLD = charger active)
  if (isUSBPowered) return 100;
  // Battery-only: learnedVFull is the 100% reference (tracks real battery max ~3.9-4.0V).
  // Tolerance 0.01V: at 3.9V with learnedVFull=3.9V → 3.90 >= 3.89 → 100%. Correct.
  if (v >= learnedVFull - 0.01f) return 100;
  // 4.0V->learnedVFull range (very narrow on this battery — usually zero width)
  if (v >= 4.00f) return  95 + (int)(((v - 4.00f) / max(0.01f, learnedVFull - 4.00f)) *  5);
  if (v >= 3.90f) return  80 + (int)(((v - 3.90f) / 0.10f) * 10);
  if (v >= 3.80f) return  70 + (int)(((v - 3.80f) / 0.10f) * 10);
  if (v >= 3.75f) return  60 + (int)(((v - 3.75f) / 0.05f) * 10);
  if (v >= 3.70f) return  50 + (int)(((v - 3.70f) / 0.05f) * 10);
  if (v >= 3.65f) return  40 + (int)(((v - 3.65f) / 0.05f) * 10);
  if (v >= 3.60f) return  30 + (int)(((v - 3.60f) / 0.05f) * 10);
  if (v >= 3.50f) return  20 + (int)(((v - 3.50f) / 0.10f) * 10);
  if (v >= 3.42f) return  10 + (int)(((v - 3.42f) / 0.08f) * 10);
  if (v >= 3.30f) return   5 + (int)(((v - 3.30f) / 0.12f) *  5);
  if (v >= 3.20f) return   2 + (int)(((v - 3.20f) / 0.10f) *  3);
  if (v >= 3.00f) return       (int)(((v - 3.00f) / 0.20f) *  2);
  return 0;
}

// Uses analogReadMilliVolts() — internally uses ESP32 eFuse ADC calibration
// for accuracy across the full range. More reliable than raw analogRead().
// 16 samples with 100μs settle between = 1.6ms total, reduces noise to <5mV.
// vdivFactor loaded from NVS, adjustable via /calibrate web endpoint.
void battery_read() {
  if (millis() - lastBatteryRead < BATTERY_READ_MS) return;
  lastBatteryRead = millis();

  // 16 samples with ADC settle time — analogReadMilliVolts for eFuse calibration
  uint32_t sum = 0;
  uint16_t mn = 65535, mx = 0;
  for (int i = 0; i < 16; i++) {
    uint16_t s = analogReadMilliVolts(BATTERY_PIN);
    sum += s; if (s < mn) mn = s; if (s > mx) mx = s;
    delayMicroseconds(100);  // allow ADC input cap to settle
  }
  float avg_mv  = (float)sum / 16.0f;
  uint16_t vvar = mx - mn;
  lastRawAvgMv  = avg_mv;  // stash for /calibrate race-free back-calc

  // ── Two-pass source + calibration factor selection ────────────────────────
  // Pass 1: quick estimate using vdivFactor (USB-calibrated) to determine source.
  //   The USB/battery gap (>150mV between USB ~4.1V and max battery ~3.95V) is
  //   large enough that a 2% ADC error never crosses the 4.08V threshold.
  // Pass 2: re-apply mode-appropriate factor for the accurate final reading.
  float vEst = (avg_mv / 1000.0f) * vdivFactor;
  bool estUSB      = (vEst > BAT_USB_THRESHOLD) ||
                     (hadHighVoltage && millis() - lastHighVoltageTime < BAT_USB_HYSTERESIS_MS);
  bool estFloat    = (vEst < BAT_FLOAT_VOLTAGE);
  if (vEst > BAT_USB_THRESHOLD) estFloat = false;

  // Pick the right factor: battery-only mode uses its own NVS-calibrated value.
  float activeFactor = (!estUSB && !estFloat) ? vdivFactorBat : vdivFactor;
  float v       = (avg_mv / 1000.0f) * activeFactor;

  // ── Source detection ──────────────────────────────────────────────────────
  // isBatFloating: true when NO battery cell is connected.
  //   A floating/unloaded VBAT divider (no cell) reads near-zero through
  //   the voltage divider — well below any real LiPo floor of ~2.8V.
  //   LOW VARIANCE ALONE IS NOT RELIABLE: a stable full LiPo at 3.8V also
  //   has very low variance across 16 samples. Removed in v5.23.
  bool newFloating = (v < BAT_FLOAT_VOLTAGE);

  // isUSBPowered: true if voltage above USB threshold, OR within hysteresis
  //   window of the last high reading (handles charger throttle-back flicker).
  //   HYSTERESIS BUG FIX v5.23: lastHighVoltageTime initialised to 0 at boot.
  //   millis()-0 at first read (~3000ms) < 30000ms hysteresis → always fired.
  //   Fix: only apply hysteresis if hadHighVoltage is true (we've actually
  //   seen a voltage above BAT_USB_THRESHOLD at least once this session).
  if (v > BAT_USB_THRESHOLD) {
    lastHighVoltageTime = millis();
    hadHighVoltage = true;
  }
  bool newUSB = (v > BAT_USB_THRESHOLD) ||
                (hadHighVoltage && millis() - lastHighVoltageTime < BAT_USB_HYSTERESIS_MS);

  // A reading above threshold cannot be floating simultaneously.
  if (v > BAT_USB_THRESHOLD) newFloating = false;

  isBatFloating = newFloating;
  isUSBPowered  = newUSB;

  // Adaptive full-voltage (NVS-persisted).
  // Only update on battery-only power, capped below USB threshold.
  if (!isUSBPowered && !isBatFloating && v > learnedVFull && v < BAT_USB_THRESHOLD) {
    learnedVFull = v;
    Preferences p; p.begin("battery", false);
    p.putFloat("vFull", learnedVFull); p.end();
  }

  batteryVoltFloat  = v * 1000.0f;   // store as millivolts (e.g. 4105.0 = 4.105V)
  batteryVoltage    = (int)batteryVoltFloat;
  batteryPercentage = constrain(voltsToPercent(v), 0, 100);

  // Determine display source string — three distinct states
  const char* srcStr = isBatFloating ? "USB/ONLY"
                     : isUSBPowered  ? "USB/CHG"
                     :                 "BAT";
  Serial.printf("[BAT] amv=%.0f var=%u V=%.3fV %d%% %s (factor=%.4f)\n",
                avg_mv, vvar, v, batteryPercentage, srcStr, activeFactor);
}

// Sends an HTTP POST notification to ntfy.sh or a self-hosted ntfy server.
// title    — notification title shown on the device
// message  — body text (keep ASCII-safe, no degree symbols or emoji in body)
// priority — 1=min 2=low 3=default 4=high 5=urgent
// tags     — comma-separated ntfy tag names e.g. "warning,battery"
void sendNtfy(const String &title, const String &message,
              int priority = 3, const String &tags = "") {
  if (!ntfy_enabled || ntfy_topic.length() == 0) return;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[NTFY] No WiFi (status=" + String(WiFi.status()) + ") — skipping");
    return;
  }
  Serial.println("[NTFY] Sending: " + title + " -> " + ntfy_server + "/" + ntfy_topic);

  // Build URL — add https:// scheme if none supplied
  String url = ntfy_server;
  if (!url.startsWith("http://") && !url.startsWith("https://"))
    url = "https://" + url;
  url += "/" + ntfy_topic;

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "text/plain");
  http.addHeader("Title",    title);
  http.addHeader("Priority", String(priority));
  if (tags.length())       http.addHeader("Tags", tags);
  if (ntfy_token.length()) http.addHeader("Authorization", "Bearer " + ntfy_token);

  int code = http.POST(message);
  http.end();

  if (code > 0) {
    Serial.printf("[NTFY] Sent OK: %s (HTTP %d)\n", title.c_str(), code);
    // Store last message preview (ASCII only) and timestamp
    ntfy_last_msg = title + ": " + message;
    if (ntfy_last_msg.length() > 30) ntfy_last_msg = ntfy_last_msg.substring(0, 30);
    String safePreview = "";
    for (char ch : ntfy_last_msg) { if ((unsigned char)ch < 128) safePreview += ch; }
    ntfy_last_msg = safePreview;
    unsigned long t = millis() / 1000;
    char buf[6]; snprintf(buf, sizeof(buf), "%02d:%02d", (int)(t/3600)%24, (int)(t%3600)/60);
    ntfy_last_time   = String(buf);
    ntfy_last_millis = millis();
  } else {
    Serial.printf("[NTFY] Failed HTTP: %d\n", code);
  }
}

// Returns the canonical power source string used in all ntfy / MQTT outputs.
// Three states: "USB/ONLY" (no cell), "USB/CHG" (cell present + charging), "Battery"
inline String powerSrcStr() {
  return isBatFloating ? "USB/ONLY"
       : isUSBPowered  ? "USB/CHG"
       :                 "Battery";
}
// Short lowercase form used in MQTT JSON fields ("usb_only","usb","battery")
inline String powerSrcJson() {
  return isBatFloating ? "usb_only"
       : isUSBPowered  ? "usb"
       :                 "battery";
}
// Rough estimate of deep-sleep cycles remaining given current battery percent.
// Uses a simple linear model: each wake cycle drains ~(wakeMah + sleepMah).
//   wakeMah  = active current draw during the ~45s wake window @ 80mA avg
//   sleepMah = sleep current (Heltec V2 ~0.5mA) for deepSleepMinutes
// This is a ballpark — useful for "a few sleeps left" type warnings.
// Returns -1 if deep sleep is disabled or estimate is not useful.
int estSleepsRemaining() {
  if (!deepSleepEnabled || deepSleepMinutes == 0) return -1;
  if (batteryPercentage <= 0) return 0;
  // LiPo capacity estimate: assume 500mAh (conservative for small pack)
  // Remaining mAh = 500 * (pct/100)
  float remainMah = 5.0f * (float)batteryPercentage;  // 500mAh * pct/100
  // Wake cost: ~80mA for ~45s = 80 * (45/3600) = 1.0 mAh per wake
  float wakeMah   = 1.0f;
  // Sleep cost: ~0.5mA * sleep_minutes/60
  float sleepMah  = 0.5f * ((float)deepSleepMinutes / 60.0f);
  float perCycle  = wakeMah + sleepMah;
  if (perCycle < 0.01f) return -1;
  return (int)(remainMah / perCycle);
}

// publishBatteryStatus() — broadcast battery alert state to all MQTT brokers.
// Called from checkBatteryAlerts() on LOW or CRITICAL transitions.
// Separate from publishSensorData() so it fires even when ntfy_on_publish=false.
void publishBatteryStatus(const String &level) {
  // "level" is "low", "critical", or "ok"
  float v = batteryVoltFloat / 1000.0f;
  // Standard MQTT: dedicated /status topic + batt_status field
  if ((mqtt_platform == "standard" || mqtt_platform == "all") && mqttStd.connected()) {
    mqttStd.publish((mqtt_topic + "/status").c_str(), ("battery_" + level).c_str());
    JsonDocument doc;
    doc["event"]      = "battery_" + level;
    doc["device"]     = device_name;
    doc["batt_v"]     = serialized(String(v, 2));
    doc["batt_pct"]   = batteryPercentage;
    doc["power_src"]  = powerSrcJson();
    int est = estSleepsRemaining();
    if (est >= 0) doc["est_sleeps"] = est;
    String p; serializeJson(doc, p);
    mqttStd.publish((mqtt_topic + "/battery").c_str(), p.c_str());
    mqttStd.loop();
  }
  // Adafruit IO: battery-status feed
  if ((mqtt_platform == "adafruit" || mqtt_platform == "all") && mqttAIO.connected()) {
    mqttAIO.publish((aio_username + "/feeds/battery-status").c_str(), level.c_str());
    mqttAIO.publish((aio_username + "/feeds/battery-percent").c_str(),
                    String(batteryPercentage).c_str());
    mqttAIO.loop();
  }
  // Ubidots: battery_status numeric (0=ok,1=low,2=critical) + battery_pct
  if ((mqtt_platform == "ubidots" || mqtt_platform == "all") && mqttUBI.connected()) {
    JsonDocument doc;
    doc["battery_status"] = (level == "critical") ? 2 : (level == "low") ? 1 : 0;
    doc["battery_pct"]    = batteryPercentage;
    doc["battery_v"]      = serialized(String(v, 2));
    String p; serializeJson(doc, p);
    String topic = "/v1.6/devices/" + (ubidots_device.length() ? ubidots_device : device_name);
    mqttUBI.publish(topic.c_str(), p.c_str());
    mqttUBI.loop();
  }
  Serial.println("[BAT] Published battery_" + level + " to MQTT");
}

// batteryWarnSent prevents repeated alerts during the same wake cycle.
void checkBatteryAlerts() {
  // Guard: skip if battery has never been read (batteryVoltFloat stays 0.0
  // until the first battery_read() completes). Use voltage not percentage
  // because 0% is a valid (though critical) reading at ~3.2V.
  if (batteryVoltFloat < 100.0f) return;  // not yet read
  if (isUSBPowered)    return;  // USB/charging -- no battery alerts
  if (isBatFloating)   return;  // ADC floating (USB, no battery?) -- no alerts

  float v   = batteryVoltFloat / 1000.0f;
  int   est = estSleepsRemaining();
  String estStr = (est >= 0) ? ("~" + String(est) + " sleeps left") : "";

  // ── CRITICAL ──────────────────────────────────────────────────────────────
  // Force sleep with alerts -- but only after 2 minutes uptime.
  // This prevents immediate forced sleep when USB-powered with a flat battery,
  // giving the user time to reach the web UI to disable deep sleep or charge.
  if (batteryPercentage <= BATTERY_CRIT_PCT) {
    if (millis() > 120000UL) {
      Serial.println("[BAT] Critical -- forced sleep (uptime > 2min)");
      // MQTT -- fires regardless of ntfy settings (battery safety)
      publishBatteryStatus("critical");
      // ntfy -- gated on ntfy_enabled + ntfy_on_batt (default true)
      if (ntfy_enabled && ntfy_on_batt && ntfy_topic.length()) {
        String msg = device_name + " battery CRITICAL: " +
                     String(batteryPercentage) + "% (" + String(v, 2) + "V)\n"
                     "Entering deep sleep now.";
        if (estStr.length()) msg += "\n" + estStr;
        sendNtfy("!! Battery Critical !!", msg, 5, "warning,battery,skull");
      }
      delay(300);
      goToDeepSleep();
    } else {
      Serial.println("[BAT] Critical but suppressed -- within 2min boot window (charge battery!)");
    }
    return;  // don't fall through to low-warning path
  }

  // ── LOW WARNING ───────────────────────────────────────────────────────────
  // Send once per wake cycle. ntfy_on_batt fires independently of ntfy_on_publish.
  if (!batteryWarnSent && batteryPercentage <= BATTERY_WARN_PCT) {
    batteryWarnSent = true;
    // MQTT -- always published (useful for HA automations, no ntfy dependency)
    publishBatteryStatus("low");
    // ntfy -- gated on ntfy_enabled + ntfy_on_batt (default true)
    if (ntfy_enabled && ntfy_on_batt && ntfy_topic.length()) {
      String msg = device_name + " battery LOW: " +
                   String(batteryPercentage) + "% (" + String(v, 2) + "V)";
      if (estStr.length()) msg += "\n" + estStr;
      sendNtfy("Battery Low", msg, 4, "warning,battery");
    }
  }
  // Reset warn flag when battery recovers (charging)
  if (batteryPercentage > BATTERY_WARN_PCT + 5) {
    if (batteryWarnSent) {
      batteryWarnSent = false;
      publishBatteryStatus("ok");  // MQTT recovery notification
    }
  }
}

// Check temp/humidity thresholds and send ntfy alerts on breach.
// Called from loop() after each successful sensor read.
// Hysteresis: 1°C / 2% deadband before re-arming so alerts don't spam.
void checkSensorAlerts() {
  if (!ntfy_enabled || !ntfy_on_sensor || !ntfy_topic.length()) return;
  if (temperature == 0.0f && humidity == 0.0f) return;  // no reading yet

  // ── TEMPERATURE HIGH ─────────────────────────────────────────────────
  if (!tempHiSent && temperature >= sensorTempHi) {
    tempHiSent = true;
    String msg = device_name + "\n"
      "Temperature HIGH: " + String(temperature,1) + "°C\n"
      "Threshold: " + String(sensorTempHi,1) + "°C\n"
      "Humidity: " + String(humidity,1) + "%";
    sendNtfy("🌡️ Temp High: " + device_name, msg, 4, "thermometer,warning");
    Serial.printf("[SENSOR] Temp HIGH alert: %.1f°C\n", temperature);
  } else if (tempHiSent && temperature < sensorTempHi - 1.0f) {
    tempHiSent = false;  // re-arm after 1°C recovery
  }

  // ── TEMPERATURE LOW ──────────────────────────────────────────────────
  if (!tempLoSent && temperature <= sensorTempLo) {
    tempLoSent = true;
    String msg = device_name + "\n"
      "Temperature LOW: " + String(temperature,1) + "°C\n"
      "Threshold: " + String(sensorTempLo,1) + "°C\n"
      "Humidity: " + String(humidity,1) + "%";
    sendNtfy("🌡️ Temp Low: " + device_name, msg, 4, "thermometer,snowflake");
    Serial.printf("[SENSOR] Temp LOW alert: %.1f°C\n", temperature);
  } else if (tempLoSent && temperature > sensorTempLo + 1.0f) {
    tempLoSent = false;
  }

  // ── HUMIDITY HIGH ─────────────────────────────────────────────────────
  if (!humHiSent && humidity >= sensorHumHi) {
    humHiSent = true;
    String msg = device_name + "\n"
      "Humidity HIGH: " + String(humidity,1) + "%\n"
      "Threshold: " + String(sensorHumHi,1) + "%\n"
      "Temp: " + String(temperature,1) + "°C";
    sendNtfy("💧 Humidity High: " + device_name, msg, 3, "droplet,warning");
    Serial.printf("[SENSOR] Humidity HIGH alert: %.1f%%\n", humidity);
  } else if (humHiSent && humidity < sensorHumHi - 2.0f) {
    humHiSent = false;
  }

  // ── HUMIDITY LOW ──────────────────────────────────────────────────────
  if (!humLoSent && humidity <= sensorHumLo) {
    humLoSent = true;
    String msg = device_name + "\n"
      "Humidity LOW: " + String(humidity,1) + "%\n"
      "Threshold: " + String(sensorHumLo,1) + "%\n"
      "Temp: " + String(temperature,1) + "°C";
    sendNtfy("💧 Humidity Low: " + device_name, msg, 3, "droplet,desert_island");
    Serial.printf("[SENSOR] Humidity LOW alert: %.1f%%\n", humidity);
  } else if (humLoSent && humidity > sensorHumLo + 2.0f) {
    humLoSent = false;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// DEEP SLEEP + POWER SAVE
// ═════════════════════════════════════════════════════════════════════════════
// Sleep-imminent overlay — full centred display, called from goToDeepSleep()
// just before powerDownPeripherals(). Counts 3..2..1 so the user can see the
// device is about to sleep rather than just going blank unexpectedly.
// Reuses the same style as drawHoldOverlay so both overlays feel consistent.
void drawSleepOverlay(int secsLeft) {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 4,  "GOING TO SLEEP");
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 18, String(secsLeft) + "s");
  display.setFont(ArialMT_Plain_10);
  // Show sleep duration if known
  if (deepSleepEnabled) {
    display.drawString(64, 38, "Wake in " + String(deepSleepMinutes) + " min");
  }
  display.drawString(64, 52, "Hold btn to wake early");
  display.display();
}

void powerDownPeripherals() {
  Serial.println(F("[PWRDN] Shutdown..."));
  if (mqttStd.connected()) {
    mqttStd.publish((mqtt_topic+"/status").c_str(), "sleeping");
    mqttStd.loop(); delay(80);
  }
  // ntfy sleep notification -- gated on ntfy_on_sleep (default OFF)
  if (ntfy_enabled && ntfy_on_sleep && ntfy_topic.length()) {
    float v = batteryVoltFloat / 1000.0f;
    String msg = device_name + "\n"
                 "Sleeping for " + String(deepSleepMinutes) + " min\n"
                 "Batt: " + String(v, 2) + "V  " + String(batteryPercentage) + "%  Src: " + powerSrcStr();
    int est = estSleepsRemaining();
    if (est >= 0) msg += "\n~" + String(est) + " sleeps remaining";
    sendNtfy("Sleep: " + device_name, msg, 1, "zzz,crescent_moon");
  }
  mqttStd.disconnect(); mqttAIO.disconnect(); mqttUBI.disconnect();
  if (ps_wifi) { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(100); esp_wifi_stop(); }
  else         { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(50); }
  if (ps_bt)   { btStop(); if (esp_bt_controller_get_status()!=ESP_BT_CONTROLLER_STATUS_IDLE) esp_bt_controller_disable(); }
  if (ps_oled) { display.displayOff(); }
  if (ps_vext) { VextOFF(); }
  if (ps_dht)  { pinMode(DHT_PIN, INPUT); }
  led.Off().Update();
  if (ps_cpu)  { setCpuFrequencyMhz(10); }
  Serial.println(F("[PWRDN] Done."));
  delay(ps_cpu ? 50 : 10);
}

// powerUpPeripherals() — called at the very top of setup() on every wakeup.
// Restores CPU speed, Vext, and DHT pin direction before the rest of setup()
// re-initialises the OLED and WiFi normally. Safe to call even if the
// corresponding ps_* flag is false (ops are idempotent).
void powerUpPeripherals() {
  // ── Restore CPU frequency first — everything else is faster at full speed ─
  if (ps_cpu_wake_mhz == 80 || ps_cpu_wake_mhz == 160 || ps_cpu_wake_mhz == 240)
    setCpuFrequencyMhz(ps_cpu_wake_mhz);
  else
    setCpuFrequencyMhz(240);
  Serial.printf("[PWRUP] CPU -> %d MHz\n", getCpuFrequencyMhz());

  // ── Vext rail on — OLED and Vext peripherals need power before init ───────
  VextON();
  delay(20);  // allow rail to stabilise

  // ── DHT pin: restore to normal output-capable mode (DHT lib sets it) ──────
  // Just ensure it's not left floating — DHT.begin() in setup() handles the rest
  pinMode(DHT_PIN, INPUT_PULLUP);

  // ── OLED: re-initialise and turn on after sleep or first boot ─────────────
  // SSD1306Wire needs init() + displayOn() every time Vext was cut or on power-on.
  // ui.init() calls display.init() internally, but displayOn() is separate —
  // without it the panel stays blank even though data is being written to the buffer.
  display.init();
  display.clear();
  // displayOn() called later in setup() after stealthThisWake is known

  Serial.println("[PWRUP] Peripherals restored");
}

// goToDeepSleep() — public entry point for all sleep triggers.
// Shows a 3-second OLED countdown overlay so the user knows the device is
// about to sleep, then powers down peripherals and enters deep sleep.
void goToDeepSleep() {
  Serial.println("[SLEEP] Entering deep sleep for " + String(deepSleepMinutes) + " min");

  // ── 3-second sleep countdown on OLED ────────────────────────────────────
  // Skipped in Stealth timer wakes — the display is off and we don't want to
  // turn it on just to count down. Button wakes are Active so they get the countdown.
  if (!stealthThisWake) {
    for (int i = 3; i >= 1; i--) {
      drawSleepOverlay(i);
      unsigned long t = millis();
      while (millis() - t < 1000) {
        esp_task_wdt_reset();
        led.Update();
        delay(20);
      }
    }
    drawSleepOverlay(0);
    delay(200);
  }

  powerDownPeripherals();

  esp_sleep_enable_timer_wakeup((uint64_t)deepSleepSeconds * 1000000ULL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // button wakes immediately

  Serial.println("[SLEEP] Going now. Button or timer will wake.");
  esp_deep_sleep_start();
  // execution stops here; resumes at setup() on next wakeup
}

// ═════════════════════════════════════════════════════════════════════════════
// OLED DISPLAY FRAMES
// Each frame function draws one screen of the rotating display.
// All frames check globalHoldSeconds first and show a sleep-countdown overlay
// so the user always gets feedback regardless of which frame is showing.
// ═════════════════════════════════════════════════════════════════════════════

// Shared overlay — drawn at top of every frame when button is held
void drawHoldOverlay(ScreenDisplay *d, int16_t x, int16_t y) {
  d->setFont(ArialMT_Plain_10);
  d->setTextAlignment(TEXT_ALIGN_CENTER);
  d->drawString(64 + x, 0 + y,  "HOLD TO SLEEP");
  d->setFont(ArialMT_Plain_16);
  d->drawString(64 + x, 14 + y, String(globalHoldSeconds) + "s");
}

// Frame 1 — live sensor readings
// ── Trend triangle — draws a small ▲ or ▼ at (tx, ty) using three drawLines.
// Size: 5 px wide × 4 px tall. trend > 0 = up, < 0 = down, 0 = nothing drawn.
void drawTrendTriangle(ScreenDisplay *d, int16_t tx, int16_t ty, int8_t trend) {
  if (trend == 0) return;
  if (trend > 0) {
    // ▲ up: apex at top-centre, base at bottom
    d->drawLine(tx + 2, ty,     tx,     ty + 4);  // left slope
    d->drawLine(tx + 2, ty,     tx + 4, ty + 4);  // right slope
    d->drawLine(tx,     ty + 4, tx + 4, ty + 4);  // base
  } else {
    // ▼ down: base at top, apex at bottom-centre
    d->drawLine(tx,     ty,     tx + 4, ty    );  // top base
    d->drawLine(tx,     ty,     tx + 2, ty + 4);  // left slope
    d->drawLine(tx + 4, ty,     tx + 2, ty + 4);  // right slope
  }
}

void drawFrame1(ScreenDisplay *d, DisplayUiState *s, int16_t x, int16_t y) {
  if (globalHoldSeconds > 0) { drawHoldOverlay(d, x, y); return; }
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_10);
  d->drawString(0 + x, 0 + y,   "SENSOR DATA");
  d->drawString(100 + x, 0 + y, wifiConnected ? "WiFi" : "----");
  d->drawLine(0 + x, 12 + y, 128, 12 + y);
  d->setFont(ArialMT_Plain_16);
  d->drawString(0 + x, 16 + y, "Temp:");
  d->drawString(60 + x, 16 + y, !isnan(temperature) ? String(temperature, 1) + "C" : "---");
  // Trend triangle sits to the right of the value, vertically centred on the 16px row
  drawTrendTriangle(d, 113 + x, 20 + y, trendTemp);
  d->drawString(0 + x, 38 + y, "Humid:");
  d->drawString(60 + x, 38 + y, !isnan(humidity)    ? String(humidity, 1)    + "%" : "---");
  drawTrendTriangle(d, 113 + x, 42 + y, trendHumidity);
}

// Frame 2 — system info (IP, SSID, RSSI, uptime, sleep countdown)
void drawFrame2(ScreenDisplay *d, DisplayUiState *s, int16_t x, int16_t y) {
  if (globalHoldSeconds > 0) { drawHoldOverlay(d, x, y); return; }
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_10);
  d->drawString(0 + x, 0 + y, "SYSTEM INFO");
  d->drawLine(0 + x, 12 + y, 128, 12 + y);
  d->drawString(0 + x, 13 + y, device_name);
  if (wifiConnected) {
    String ssid = WiFi.SSID();
    if (ssid.length() > 16) ssid = ssid.substring(0, 13) + "...";
    d->drawString(0 + x, 23 + y, "WiFi: " + ssid);
    d->drawString(0 + x, 33 + y, WiFi.localIP().toString());
    d->drawString(0 + x, 43 + y, "RSSI: " + String(WiFi.RSSI()) + "dBm");
  } else {
    d->drawString(0 + x, 23 + y, "WiFi: Disconnected");
    d->drawString(0 + x, 33 + y, "AP: ESP32-Setup");
  }
  // Bottom line: uptime or web UI countdown if window is active
  String bot = "Up: " + getUptime();
  if (webUiExpiresAt > millis()) {
    unsigned long msLeft = webUiExpiresAt - millis();
    if (msLeft < 120000UL) {
      // Under 2 min — show mm:ss for urgency
      unsigned int sLeft = msLeft / 1000;
      char buf[16];
      snprintf(buf, sizeof(buf), "WebUI: %u:%02u left", sLeft / 60, sLeft % 60);
      bot = String(buf);
    } else {
      int minsLeft = msLeft / 60000;
      bot = "WebUI: " + String(minsLeft) + "m left";
    }
  }
  d->drawString(0 + x, 53 + y, bot);
}

// Frame 3 — MQTT broker status
void drawFrame3(ScreenDisplay *d, DisplayUiState *s, int16_t x, int16_t y) {
  if (globalHoldSeconds > 0) { drawHoldOverlay(d, x, y); return; }
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_10);
  d->drawString(0 + x, 0 + y, "MQTT STATUS");
  d->drawLine(0 + x, 12 + y, 128, 12 + y);
  String plat = mqtt_platform; plat.toUpperCase();
  d->drawString(0 + x, 13 + y, "Mode: " + plat);
  if (mqtt_platform == "all" || mqtt_platform == "standard")
    d->drawString(0 + x, 25 + y, "Std: " + String(mqttStandardConnected ? "OK" : "X"));
  if (mqtt_platform == "all" || mqtt_platform == "adafruit")
    d->drawString(0 + x, 35 + y, "AIO: " + String(mqttAdafruitConnected ? "OK" : "X"));
  if (mqtt_platform == "all" || mqtt_platform == "ubidots")
    d->drawString(0 + x, 45 + y, "Ubi: " + String(mqttUbidotsConnected  ? "OK" : "X"));
  if (deepSleepEnabled)
    d->drawString(0 + x, 53 + y, "Sleep: " + String(deepSleepMinutes) + "min");
}

// Frame 4 — WiFi AP / portal info (shown whenever portal is open)
// Mirrors the layout style of other frames so it feels consistent.
void drawFrame4(ScreenDisplay *d, DisplayUiState *s, int16_t x, int16_t y) {
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_10);
  d->drawString(0 + x, 0 + y, "WIFI SETUP");
  d->drawLine(0 + x, 12 + y, 128, 12 + y);

  d->drawString(0 + x, 14 + y, "1. Join WiFi network:");
  d->setFont(ArialMT_Plain_16);
  d->drawString(4 + x, 24 + y, "ESP32-Setup");

  d->setFont(ArialMT_Plain_10);
  d->drawString(0 + x, 42 + y, "2. Open browser:");
  d->drawString(4 + x, 52 + y, "192.168.4.1");
}

// Active frame set — updated in setup() after we know if portal is needed
// Frame 5 — Battery status only (clean, no overlap)
// Layout (128×64):
//   Row  0-11 : Header "BATTERY" + icon top-right
//   Row 12    : separator line
//   Row 14-24 : Source + voltage  e.g. "USB/CHG  4.05V"
//   Row 26-36 : Percent text      e.g. "Charge:  87%"
//   Row 40-48 : Bar outline (64px wide × 8px tall)
//   Row 50-62 : Status note       e.g. "No battery?" or blank
void drawFrame5(ScreenDisplay *d, DisplayUiState *s, int16_t x, int16_t y) {
  if (globalHoldSeconds > 0) { drawHoldOverlay(d, x, y); return; }

  // ── Header ────────────────────────────────────────────────────────────────
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->setFont(ArialMT_Plain_10);
  d->drawString(0 + x, 0 + y, "BATTERY");
  d->drawLine(0 + x, 12 + y, 128 + x, 12 + y);

  // Battery icon — top-right, clear of header text
  // Only draw when we have a valid reading
  if (batteryVoltFloat >= 100.0f) {
    if (batteryPercentage > 60) {
      d->drawXbm(107 + x, 1 + y, BAT_width,     BAT_height,     BAT_bits);
    } else if (batteryPercentage > 20) {
      d->drawXbm(112 + x, 1 + y, BATHALF_width, BATHALF_height, BATHALF_bits);
    } else {
      d->drawXbm(122 + x, 1 + y, BATLOW_width,  BATLOW_height,  BATLOW_bits);
    }
  }

  // ── Source + voltage on one line ─────────────────────────────────────────
  float v = batteryVoltFloat / 1000.0f;
  d->setFont(ArialMT_Plain_10);
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  if (batteryVoltFloat < 100.0f) {
    // Not yet read
    d->drawString(0 + x, 15 + y, "Reading...");
  } else {
    String src = isUSBPowered ? "USB/CHG" : (isBatFloating ? "FLOATING" : "BATTERY");
    d->drawString(0 + x, 15 + y, src);
    d->setTextAlignment(TEXT_ALIGN_RIGHT);
    d->drawString(128 + x, 15 + y, String(v, 2) + "V");
    d->setTextAlignment(TEXT_ALIGN_LEFT);
  }

  // ── Percentage line ───────────────────────────────────────────────────────
  if (batteryVoltFloat >= 100.0f) {
    if (isBatFloating) {
      d->drawString(0 + x, 27 + y, "No battery detected");
    } else if (isUSBPowered) {
      d->drawString(0 + x, 27 + y, "Charging / USB");
      d->setTextAlignment(TEXT_ALIGN_RIGHT);
      d->drawString(128 + x, 27 + y, "100%");
      d->setTextAlignment(TEXT_ALIGN_LEFT);
    } else {
      d->drawString(0 + x, 27 + y, "Charge:");
      d->setTextAlignment(TEXT_ALIGN_RIGHT);
      d->drawString(128 + x, 27 + y, String(batteryPercentage) + "%");
      d->setTextAlignment(TEXT_ALIGN_LEFT);
    }
  }

  // ── Progress bar (full width, 8px tall) ──────────────────────────────────
  // Drawn at y=40 so nothing overlaps it
  if (batteryVoltFloat >= 100.0f && !isBatFloating) {
    int pct = isUSBPowered ? 100 : batteryPercentage;
    int barW = (pct * 122) / 100;   // 122px inner width (2px margin each side)
    d->drawRect(0 + x, 40 + y, 124, 8);
    if (barW > 0) d->fillRect(2 + x, 42 + y, barW, 4);
  }

  // ── Low-battery warning note ──────────────────────────────────────────────
  if (!isUSBPowered && !isBatFloating && batteryVoltFloat >= 100.0f) {
    if (batteryPercentage <= BATTERY_CRIT_PCT) {
      d->setFont(ArialMT_Plain_10);
      d->drawString(0 + x, 52 + y, "!! CRITICAL !!");
    } else if (batteryPercentage <= BATTERY_WARN_PCT) {
      d->setFont(ArialMT_Plain_10);
      d->drawString(0 + x, 52 + y, "Low battery");
    }
  }
}

// ── OTA-available overlay ─────────────────────────────────────────────────────
// Drawn on top of every normal frame when rtcOtaAvailable is true.
// A filled up-arrow triangle + "FW" label, starting at x≈68 — just after the
// widest frame header ("SYSTEM INFO" / "MQTT STATUS" at ~66px in 10pt).
// Consistent, unambiguous, and never overlaps any frame header text.
void drawOtaAvailableOverlay(ScreenDisplay *d, DisplayUiState *s) {
  if (!rtcOtaAvailable) return;
  // Filled up-pointing triangle: tip at (71,0), base at y=3, width 7px
  //   row 0: 1px  x=71
  //   row 1: 3px  x=70..72
  //   row 2: 5px  x=69..73
  //   row 3: 7px  x=68..74  (base)
  d->drawHorizontalLine(71, 0, 1);
  d->drawHorizontalLine(70, 1, 3);
  d->drawHorizontalLine(69, 2, 5);
  d->drawHorizontalLine(68, 3, 7);
  // "FW" label immediately right of the arrow
  d->setFont(ArialMT_Plain_10);
  d->setTextAlignment(TEXT_ALIGN_LEFT);
  d->drawString(77, 0, "FW");
}

OverlayCallback overlays[] = { drawOtaAvailableOverlay };

FrameCallback frames3[] = { drawFrame1, drawFrame2, drawFrame3 };
FrameCallback frames4[] = { drawFrame1, drawFrame2, drawFrame3, drawFrame4 };
FrameCallback frames5[] = { drawFrame1, drawFrame2, drawFrame3, drawFrame5 };

// ═════════════════════════════════════════════════════════════════════════════
// WiFiManager SAVE CALLBACK
// ═════════════════════════════════════════════════════════════════════════════
void saveConfigCallback() {
  mqtt_platform  = p_platform.getValue();
  mqtt_server    = p_server.getValue();
  mqtt_port      = p_port.getValue();
  mqtt_user      = p_user.getValue();
  mqtt_pass      = p_pass.getValue();
  mqtt_topic     = p_topic.getValue();
  device_name    = p_name.getValue();
  aio_username   = p_aio_user.getValue();
  aio_key        = p_aio_key.getValue();
  ubidots_token  = p_ubi_tok.getValue();
  ubidots_device = p_ubi_dev.getValue();

  publishSeconds = constrain((unsigned long)String(p_timer.getValue()).toInt(), 30UL, 3600UL);
  readInterval   = publishSeconds * 1000UL;

  String dsVal = String(p_ds_en.getValue());
  deepSleepEnabled = (dsVal == "1" || dsVal == "on");
  deepSleepMinutes = (uint32_t)max(1L, (long)String(p_ds_min.getValue()).toInt());
  deepSleepSeconds = deepSleepMinutes * 60UL;

  saveMqttConfig();
  saveDeepConfig();
}

// ═════════════════════════════════════════════════════════════════════════════
// WiFiManager SETUP
// NOTE: intentionally called BEFORE WDT is started — portal can block 3 min.
// ═════════════════════════════════════════════════════════════════════════════
void setupWiFiManager(bool forcePortal) {
  // ── Fast static-IP reconnect for stealth timer wakes ─────────────────────
  // Skips WiFiManager and DHCP entirely — cuts connect time from ~3s to ~400ms.
  // Falls back to full DHCP path below if static connect fails within 3 s.
  if (stealthThisWake && rtcIpValid && !forcePortal) {
    WiFi.mode(WIFI_STA);
    WiFi.config(IPAddress(rtcIp), IPAddress(rtcGw), IPAddress(rtcSn), IPAddress(rtcDns));
    WiFi.begin();  // reconnects to last saved SSID/password
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 3000) {
      delay(50); esp_task_wdt_reset();
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      portalActive  = false;
      ui.setFrames(frames5, 5);
      Serial.printf("[WiFi] Fast static connect: %s (%.0f ms)\n",
                    WiFi.localIP().toString().c_str(), (float)(millis() - t));
      configTime(0, 0, "uk.pool.ntp.org", "time.google.com");
      setenv("TZ", "GMT0BST,M3.5.0/1,M10.5.0", 1); tzset();
      loadConfig();  // ensure globals populated even if we skipped normal path
      // Clear DRD flag — fast path skips the normal clearDoubleReset() call
      clearDoubleReset();
      // Wait up to 2s for NTP — same budget as normal DHCP path
      struct tm _nti;
      if (getLocalTime(&_nti, 2000)) {
        char _tb[6], _db[6];
        strftime(_tb, sizeof(_tb), "%H:%M", &_nti);
        strftime(_db, sizeof(_db), "%d/%m", &_nti);
        currentTimeStr = String(_tb);
        currentDateStr = String(_db);
        ntpSynced = true;
        Serial.println("[NTP] Synced (fast path): " + currentDateStr + " " + currentTimeStr);
      } else {
        Serial.println(F("[NTP] Sync pending — will retry on read"));
      }
      return;        // ← skip full WiFiManager block
    }
    // Static connect failed — clear cached IP and fall through to normal DHCP
    Serial.println("[WiFi] Static connect failed — falling back to DHCP");
    rtcIpValid = false;
    WiFi.disconnect(true);
    delay(100);
  }

  WiFiManager wm;
  loadConfig();  // populate globals before setting parameter defaults

  p_platform.setValue(mqtt_platform.c_str(), 20);
  p_server.setValue  (mqtt_server.c_str(),   40);
  p_port.setValue    (mqtt_port.c_str(),       6);
  p_user.setValue    (mqtt_user.c_str(),      40);
  p_pass.setValue    (mqtt_pass.c_str(),      40);
  p_topic.setValue   (mqtt_topic.c_str(),     50);
  p_name.setValue    (device_name.c_str(),    30);
  p_aio_user.setValue(aio_username.c_str(),   40);
  p_aio_key.setValue (aio_key.c_str(),        40);
  p_ubi_tok.setValue (ubidots_token.c_str(),  60);
  p_ubi_dev.setValue (ubidots_device.c_str(), 30);

  wm.addParameter(&p_platform); wm.addParameter(&p_server);
  wm.addParameter(&p_port);     wm.addParameter(&p_user);
  wm.addParameter(&p_pass);     wm.addParameter(&p_topic);
  wm.addParameter(&p_name);     wm.addParameter(&p_timer);
  wm.addParameter(&h_aio);      wm.addParameter(&p_aio_user);
  wm.addParameter(&p_aio_key);  wm.addParameter(&h_ubi);
  wm.addParameter(&p_ubi_tok);  wm.addParameter(&p_ubi_dev);
  wm.addParameter(&h_sleep);    wm.addParameter(&p_ds_en);
  wm.addParameter(&p_ds_min);

  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setConfigPortalTimeout(180);

  // Called by WiFiManager the moment the AP/portal opens (both auto and forced).
  // Switches OLED to portal-info frame and sets portalActive flag immediately.
  wm.setAPCallback([](WiFiManager* wm) {
    portalActive = true;
    ui.setFrames(frames4, 4);
    led.Blink(100, 100).Forever().Update();  // fast blink = portal open
    Serial.println(F("[WiFi] Portal open -- connect to ESP32-Setup then browse 192.168.4.1"));
  });

  // Fast blink during WiFi connect — suppressed for Stealth timer wakes
  if (!stealthThisWake) {
    led.Blink(150, 150).Forever().Update();
  }

  bool connected = false;
  if (forcePortal) {
    Serial.println("[WiFi] Double-reset detected — forcing config portal");
    connected = wm.startConfigPortal("ESP32-Setup");
  } else {
    connected = wm.autoConnect("ESP32-Setup");
  }

  portalActive = false;

  if (!connected) {
    Serial.println("[WiFi] Connection failed — restarting");
    delay(1000);
    ESP.restart();
  }

  // ── Cache IP config in RTC for fast static-IP reconnect on next timer wake ──
  rtcIp      = (uint32_t)WiFi.localIP();
  rtcGw      = (uint32_t)WiFi.gatewayIP();
  rtcSn      = (uint32_t)WiFi.subnetMask();
  rtcDns     = (uint32_t)WiFi.dnsIP();
  rtcIpValid = true;
  Serial.printf("[WiFi] IP cached for static reuse: %s\n", WiFi.localIP().toString().c_str());

  // Switch to full 5-frame set now that WiFi is connected
  ui.setFrames(frames5, 5);
  wifiConnected = true;
  Serial.println("[WiFi] Connected: " + WiFi.localIP().toString());
  //pool.ntp.org for global server
  configTime(0, 0, "uk.pool.ntp.org", "time.google.com");
  // set it to uk british summer time rules here
  setenv("TZ", "GMT0BST,M3.5.0/1,M10.5.0", 1) ; tzset(); 
  struct tm _nti;
  if (getLocalTime(&_nti, 2000)) {
    char _tb[6], _db[6];
    strftime(_tb, sizeof(_tb), "%H:%M", &_nti);
    strftime(_db, sizeof(_db), "%d/%m", &_nti);
    currentTimeStr = String(_tb);
    currentDateStr = String(_db);
    ntpSynced = true;
    Serial.println("[NTP] Synced: " + currentDateStr + " " + currentTimeStr);
  } else { Serial.println(F("[NTP] Sync failed")); }
  // Clear the DRD flag now that we've connected successfully —
  // prevents a normal reboot after this from being mistaken as double-reset
  clearDoubleReset();
}

// ═════════════════════════════════════════════════════════════════════════════
// WiFi RECONNECT — called every WIFI_CHECK_INTERVAL ms from loop()
// Resets MQTT state and sets lastRead=0 so a publish fires immediately.
// ═════════════════════════════════════════════════════════════════════════════
void checkWiFi() {
  if (millis() - lastWifiCheck < WIFI_CHECK_INTERVAL) return;
  lastWifiCheck = millis();
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("[WiFi] Lost — reconnecting...");
  wifiConnected = false;
  WiFi.disconnect(); WiFi.reconnect();

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500); esp_task_wdt_reset(); Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("[WiFi] Reconnected: " + WiFi.localIP().toString());
    mqttStd.disconnect(); mqttAIO.disconnect(); mqttUBI.disconnect();
    mqttStandardConnected = mqttAdafruitConnected = mqttUbidotsConnected = false;
    stdRetries = aioRetries = ubiRetries = 0;
    lastRead = 0;  // publish immediately after reconnect
  } else {
    Serial.println("[WiFi] Reconnect failed — retrying in 30s");
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// MQTT CLIENTS SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setupMQTTClients() {
  mqttStd.setBufferSize(MQTT_BUFFER_SIZE); mqttStd.setKeepAlive(MQTT_KEEPALIVE);
  mqttAIO.setBufferSize(MQTT_BUFFER_SIZE); mqttAIO.setKeepAlive(MQTT_KEEPALIVE);
  mqttUBI.setBufferSize(MQTT_BUFFER_SIZE); mqttUBI.setKeepAlive(MQTT_KEEPALIVE);
}

// Connects a single PubSubClient; returns true on success
bool connectMQTT(PubSubClient &c, const String &host, int port,
                 const String &user, const String &pass, const String &id) {
  if (c.connected()) return true;
  c.setServer(host.c_str(), port);
  return user.length() ? c.connect(id.c_str(), user.c_str(), pass.c_str())
                       : c.connect(id.c_str());
}

// Attempts (re)connection to all configured brokers.
// Retry counters reset after RETRY_RESET_INTERVAL.
void reconnectMQTT() {
  unsigned long now = millis();

  // ── Standard ──────────────────────────────────────────────────────────────
  if ((mqtt_platform == "standard" || mqtt_platform == "all") && mqtt_server.length()) {
    if (stdRetries >= MAX_RETRIES && now - lastStdReset > RETRY_RESET_INTERVAL)
      { stdRetries = 0; lastStdReset = now; }
    if (!mqttStandardConnected && stdRetries < MAX_RETRIES && now - lastStandardRetry > RETRY_INTERVAL) {
      Serial.print("[MQTT-Std] Connecting...");
      String id = device_name + "-std-" + String(random(0xffff), HEX);
      if (connectMQTT(mqttStd, mqtt_server, mqtt_port.toInt(), mqtt_user, mqtt_pass, id)) {
        mqttStandardConnected = true; stdRetries = 0; lastRead = 0;
        mqttStd.publish((mqtt_topic + "/status").c_str(), "awake");
        Serial.println("OK");
      } else { stdRetries++; lastStandardRetry = now; Serial.println("fail " + String(stdRetries)); }
    }
    if (!mqttStd.connected() && mqttStandardConnected)
      { mqttStandardConnected = false; Serial.println("[MQTT-Std] Dropped"); }
  }

  // ── Adafruit IO ───────────────────────────────────────────────────────────
  if ((mqtt_platform == "adafruit" || mqtt_platform == "all") && aio_username.length() && aio_key.length()) {
    if (aioRetries >= MAX_RETRIES && now - lastAioReset > RETRY_RESET_INTERVAL)
      { aioRetries = 0; lastAioReset = now; }
    if (!mqttAdafruitConnected && aioRetries < MAX_RETRIES && now - lastAdafruitRetry > RETRY_INTERVAL) {
      Serial.print("[MQTT-AIO] Connecting...");
      String id = device_name + "-aio-" + String(random(0xffff), HEX);
      if (connectMQTT(mqttAIO, "io.adafruit.com", 1883, aio_username, aio_key, id)) {
        mqttAdafruitConnected = true; aioRetries = 0; lastRead = 0; Serial.println("OK");
      } else { aioRetries++; lastAdafruitRetry = now; Serial.println("fail " + String(aioRetries)); }
    }
    if (!mqttAIO.connected() && mqttAdafruitConnected)
      { mqttAdafruitConnected = false; Serial.println("[MQTT-AIO] Dropped"); }
  }

  // ── Ubidots ───────────────────────────────────────────────────────────────
  if ((mqtt_platform == "ubidots" || mqtt_platform == "all") && ubidots_token.length()) {
    if (ubiRetries >= MAX_RETRIES && now - lastUbiReset > RETRY_RESET_INTERVAL)
      { ubiRetries = 0; lastUbiReset = now; }
    if (!mqttUbidotsConnected && ubiRetries < MAX_RETRIES && now - lastUbidotsRetry > RETRY_INTERVAL) {
      Serial.print("[MQTT-Ubi] Connecting...");
      String id = device_name + "-ubi-" + String(random(0xffff), HEX);
      if (connectMQTT(mqttUBI, "industrial.api.ubidots.com", 1883, ubidots_token, "", id)) {
        mqttUbidotsConnected = true; ubiRetries = 0; lastRead = 0; Serial.println("OK");
      } else { ubiRetries++; lastUbidotsRetry = now; Serial.println("fail " + String(ubiRetries)); }
    }
    if (!mqttUBI.connected() && mqttUbidotsConnected)
      { mqttUbidotsConnected = false; Serial.println("[MQTT-Ubi] Dropped"); }
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// SENSOR READ + PUBLISH
// ═════════════════════════════════════════════════════════════════════════════
void publishSensorData() {
  if (!mqttStd.connected()) mqttStandardConnected = false;
  if (!mqttAIO.connected()) mqttAdafruitConnected = false;
  if (!mqttUBI.connected()) mqttUbidotsConnected  = false;

  float battV = batteryVoltFloat / 1000.0f;
  JsonDocument doc;

  // Standard MQTT — full JSON payload including battery
  if ((mqtt_platform == "standard" || mqtt_platform == "all") && mqttStd.connected()) {
    doc["device"]      = device_name;
    doc["temperature"] = temperature;
    doc["humidity"]    = humidity;
    doc["uptime"]      = millis() / 1000;
    doc["boot_count"]  = bootCount;
    doc["batt_v"]      = serialized(String(battV, 2));
    doc["batt_pct"]    = batteryPercentage;
    doc["power_src"]   = powerSrcJson();
    doc["batt_status"] = (batteryPercentage <= BATTERY_CRIT_PCT) ? "critical"
                       : (batteryPercentage <= BATTERY_WARN_PCT)  ? "low" : "ok";
    doc["ota_available"] = otaUpdateAvailable;   // true = newer FW confirmed on GitHub
    String p; serializeJson(doc, p);
    if (!mqttStd.publish(mqtt_topic.c_str(), p.c_str()))
      Serial.println("[MQTT-Std] Publish failed — will retry next interval");
    mqttStd.loop();  // flush send buffer immediately after publish
  }

  // Adafruit IO — temperature, humidity, battery feeds
  if ((mqtt_platform == "adafruit" || mqtt_platform == "all") && mqttAIO.connected()) {
    mqttAIO.publish((aio_username + "/feeds/temperature").c_str(),
                    String(temperature, 1).c_str());
    mqttAIO.publish((aio_username + "/feeds/humidity").c_str(),
                    String(humidity, 1).c_str());
    mqttAIO.publish((aio_username + "/feeds/battery-voltage").c_str(),
                    String(battV, 2).c_str());
    mqttAIO.publish((aio_username + "/feeds/battery-percent").c_str(),
                    String(batteryPercentage).c_str());
    mqttAIO.publish((aio_username + "/feeds/power-source").c_str(),
                    powerSrcJson().c_str());
    mqttAIO.loop();
  }

  // Ubidots — temperature, humidity, battery
  if ((mqtt_platform == "ubidots" || mqtt_platform == "all") && mqttUBI.connected()) {
    doc.clear();
    doc["temperature"] = temperature;
    doc["humidity"]    = humidity;
    doc["battery_v"]   = serialized(String(battV, 2));
    doc["battery_pct"] = batteryPercentage;
    doc["power_src"]   = powerSrcJson();  // "usb_only","usb","battery"
    String p; serializeJson(doc, p);
    mqttUBI.publish(("/v1.6/devices/" +
                    (ubidots_device.length() ? ubidots_device : device_name)).c_str(),
                    p.c_str());
    mqttUBI.loop();
  }

  // Optional ntfy on each publish — ASCII safe, no degree symbol
  if (ntfy_enabled && ntfy_on_publish) {
    float battV2 = batteryVoltFloat / 1000.0f;
    String msg = device_name + "\n"
                 "Temp: " + String(temperature, 1) + "C  Hum: " + String(humidity, 1) + "%\n"
                 "Batt: " + String(battV2, 2) + "V  " + String(batteryPercentage) + "%  Src: " + powerSrcStr();
    sendNtfy("Sensor: " + device_name, msg, 2, "thermometer");
  }
}

// One-time boot/wake summary to all brokers and ntfy.
void publishBootSummary() {
  float v = batteryVoltFloat / 1000.0f;
  String reason = "";
  switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_EXT0:  reason = "button"; break;
    case ESP_SLEEP_WAKEUP_TIMER: reason = "timer";  break;
    default:                     reason = "power-on"; break;
  }

  // Standard MQTT -- full JSON summary
  if ((mqtt_platform == "standard" || mqtt_platform == "all") && mqttStd.connected()) {
    JsonDocument doc;
    doc["event"]        = "boot";
    doc["device"]       = device_name;
    doc["version"]      = FW_VERSION;
    doc["boot_count"]   = bootCount;
    doc["sleep_wakes"]  = (uint32_t)rtcBootOffset;
    doc["wakeup"]       = reason;
    doc["wake_mode"]    = (wakeDisplayMode == 0) ? "stealth" : "active";
    doc["ip"]           = WiFi.localIP().toString();
    doc["batt_v"]       = serialized(String(v, 2));
    doc["batt_pct"]     = batteryPercentage;
    doc["power_src"]    = powerSrcJson();
    doc["batt_status"]  = (batteryPercentage <= BATTERY_CRIT_PCT) ? "critical"
                        : (batteryPercentage <= BATTERY_WARN_PCT)  ? "low" : "ok";
    doc["ota_available"] = otaUpdateAvailable;   // true = newer FW confirmed on GitHub
    String p; serializeJson(doc, p);
    mqttStd.publish((mqtt_topic + "/boot").c_str(), p.c_str());
    mqttStd.loop();
  }

  // Adafruit IO — battery feeds + battery-status + power-source on boot
  if ((mqtt_platform == "adafruit" || mqtt_platform == "all") && mqttAdafruitConnected) {
    mqttAIO.publish((aio_username + "/feeds/battery-voltage").c_str(),  String(v, 2).c_str());
    mqttAIO.publish((aio_username + "/feeds/battery-percent").c_str(),  String(batteryPercentage).c_str());
    String bs = (batteryPercentage <= BATTERY_CRIT_PCT) ? "critical"
              : (batteryPercentage <= BATTERY_WARN_PCT)  ? "low" : "ok";
    mqttAIO.publish((aio_username + "/feeds/battery-status").c_str(), bs.c_str());
    mqttAIO.publish((aio_username + "/feeds/power-source").c_str(),   powerSrcJson().c_str());
    mqttAIO.loop();
  }

  // Ubidots -- boot context with battery_status + power_src (0=ok,1=low,2=critical)
  if ((mqtt_platform == "ubidots" || mqtt_platform == "all") && mqttUbidotsConnected) {
    JsonDocument doc;
    doc["battery_v"]      = serialized(String(v, 2));
    doc["battery_pct"]    = batteryPercentage;
    doc["battery_status"] = (batteryPercentage <= BATTERY_CRIT_PCT) ? 2
                          : (batteryPercentage <= BATTERY_WARN_PCT)  ? 1 : 0;
    doc["power_src"]      = powerSrcJson();   // "usb_only" | "usb" | "battery"
    doc["boot_count"]     = bootCount;
    doc["ota_available"]  = otaUpdateAvailable;
    String p; serializeJson(doc, p);
    String topic = "/v1.6/devices/" + (ubidots_device.length() ? ubidots_device : device_name);
    mqttUBI.publish(topic.c_str(), p.c_str());
    mqttUBI.loop();
  }

  // ntfy boot/wake notification
  Serial.printf("[NTFY-BOOT] enabled=%d on_boot=%d topic='%s' wifi=%d\n",
                ntfy_enabled, ntfy_on_boot, ntfy_topic.c_str(), WiFi.status() == WL_CONNECTED);
  if (ntfy_enabled && ntfy_on_boot && ntfy_topic.length()) {
    // Title reflects wakeup reason
    String ntfyTitle;
    String wakeIcon;
    if (reason == "button")   { ntfyTitle = "Wake (button)"; wakeIcon = "hand"; }
    else if (reason == "timer") { ntfyTitle = "Wake (timer)";  wakeIcon = "alarm_clock"; }
    else                        { ntfyTitle = "Boot";          wakeIcon = "electric_plug"; }

    // Power source icon — three states
    String srcIcon = isBatFloating ? "plug"
                   : isUSBPowered  ? "zap"
                   :                 "battery";
    String modeStr = (wakeDisplayMode == 0) ? "Stealth" : "Active";

    String msg = device_name + "\n"
                 "Temp: " + String(temperature, 1) + "C  Hum: " + String(humidity, 1) + "%\n"
                 "Batt: " + String(v, 2) + "V  " + String(batteryPercentage) + "%  Src: " + powerSrcStr() + "\n"
                 "Wake: " + reason + "  Mode: " + modeStr + "  Boot#" + String(bootCount) + "\n"
                 "IP: " + WiFi.localIP().toString() + "  v" + FW_VERSION;
    int est = estSleepsRemaining();
    if (est >= 0) msg += "\n~" + String(est) + " sleeps remaining";
    sendNtfy(ntfyTitle + ": " + device_name, msg, 2, wakeIcon + "," + srcIcon);
  } else {
    if (!ntfy_enabled)                 Serial.println("[NTFY-BOOT] Skipped -- ntfy disabled");
    if (ntfy_enabled && !ntfy_on_boot) Serial.println("[NTFY-BOOT] Skipped -- on_boot disabled");
    if (!ntfy_topic.length())          Serial.println("[NTFY-BOOT] Skipped -- topic empty");
  }

  Serial.println("[BOOT] Summary published");
}

// Reads DHT22, logs to Serial, publishes, sets sleep trigger flag
void readSensor() {
  sensors_event_t ev;
  bool ok = true;
  float newTemp = NAN, newHum = NAN;
  dht.temperature().getEvent(&ev);
  if (isnan(ev.temperature)) ok = false; else newTemp = ev.temperature;
  dht.humidity().getEvent(&ev);
  if (isnan(ev.relative_humidity)) ok = false; else newHum = ev.relative_humidity;

  if (!ok) {
    Serial.println("[DHT] Read failed — sleep trigger still set");
    lastSensorPublishTime = millis();
    triggerDeepSleepAfterPublish = true;
    return;
  }

  // ── Trend: compare new vs previous reading, honouring dead-band ─────────────
  if (!isnan(temperature) && temperature != 0.0f) {
    float dt = newTemp - temperature;
    trendTemp = (dt >  TREND_DEAD_TEMP) ?  1 :
                (dt < -TREND_DEAD_TEMP) ? -1 : 0;
  }
  if (!isnan(humidity) && humidity != 0.0f) {
    float dh = newHum - humidity;
    trendHumidity = (dh >  TREND_DEAD_HUM) ?  1 :
                    (dh < -TREND_DEAD_HUM) ? -1 : 0;
  }

  temperature = newTemp;
  humidity    = newHum;
  Serial.printf("[DHT] %.1f°C  %.1f%%\n", temperature, humidity);
  {
    struct tm ti; time_t now = 0;
    if (getLocalTime(&ti, 0)) now = mktime(&ti);
    rtcHistTemp[rtcHistHead] = temperature;
    rtcHistHum[rtcHistHead]  = humidity;
    rtcHistTime[rtcHistHead] = (uint32_t)now;
    rtcHistHead = (rtcHistHead + 1) % HIST_SIZE;
    if (rtcHistCount < HIST_SIZE) rtcHistCount++;
  }
  publishSensorData();
  lastSensorPublishTime = millis();
  triggerDeepSleepAfterPublish = true;

  //     Dblclick manual trigger also updates web ui last read stat  
  // === UPDATE DASHBOARD LAST READ TIME ===
  // If NTP didn't sync at boot (common on fast static-IP wakes where NTP
  // resolves after setup() returns), retry once now — we've been up a few
  // seconds and DNS has had time to respond.
  if (!ntpSynced) {
    struct tm _ntr;
    if (getLocalTime(&_ntr, 1500)) {
      ntpSynced = true;
      Serial.println(F("[NTP] Synced on read retry"));
    }
  }
  if (ntpSynced) {
      time_t now = time(nullptr);
      struct tm* t = localtime(&now);
      char dateBuf[6], timeBuf[6];
      strftime(dateBuf, sizeof(dateBuf), "%d/%m", t);   // UK format dd/mm
      strftime(timeBuf, sizeof(timeBuf), "%H:%M", t);
      currentDateStr = dateBuf;
      currentTimeStr = timeBuf;
  }

    Serial.println(F("[SENSOR] Read + Publish complete"));
    // === UPDATE DASHBOARD LAST READ TIME ===
}

// ═════════════════════════════════════════════════════════════════════════════
// OTA / WEB SERVER
// ═════════════════════════════════════════════════════════════════════════════
// Shared response page — icon, message, animated shrink bar, auto-redirect + back button

// ═════════════════════════════════════════════════════════════════════════════
// SHARED CSS — stored in PROGMEM, streamed once per page, never in RAM
// ═════════════════════════════════════════════════════════════════════════════
static const char COMMON_CSS[] PROGMEM =
  "*{margin:0;padding:0;box-sizing:border-box}"
  "body{font-family:-apple-system,sans-serif;background:#1a1a1a;color:#e0e0e0;padding:20px}"
  ".c{max-width:700px;margin:0 auto;background:#2d2d2d;padding:24px;border-radius:12px;box-shadow:0 8px 32px rgba(0,0,0,.4)}"
  "h1{color:#64B5F6;border-bottom:2px solid #424242;padding-bottom:12px;margin-bottom:20px}"
  ".card{background:#1f1f1f;padding:16px;border-radius:8px;margin:12px 0;border:1px solid #424242}"
  ".card h2{color:#9e9e9e;font-size:14px;margin-bottom:12px;text-transform:uppercase;letter-spacing:1px}"
  ".card p{margin:6px 0}"
  ".btn{background:#64B5F6;color:#000;padding:12px 24px;border:none;border-radius:6px;text-decoration:none;"
        "display:inline-block;margin:6px;font-weight:600;cursor:pointer;transition:all .2s}"
  ".btn:hover{background:#42A5F5}"
  ".ota{background:#FF8A65}.ota:hover{background:#FF7043}"
  ".danger{background:#ef5350}.danger:hover{background:#e53935}"
  ".dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:8px}"
  ".on{background:#66bb6a}.off{background:#ef5350}"
  "label{display:block;margin:12px 0 6px;font-weight:600;color:#9e9e9e;font-size:13px}"
  "input,select{width:100%;padding:10px;border:1px solid #424242;border-radius:6px;background:#1a1a1a;color:#e0e0e0;font-size:14px}"
  "input:focus,select:focus{outline:none;border-color:#64B5F6}"
  ".sec{background:#1f1f1f;padding:20px;border-radius:8px;margin:16px 0;border:1px solid #424242}"
  ".sec h2{color:#9e9e9e;font-size:16px;margin-bottom:16px;border-bottom:1px solid #424242;padding-bottom:8px}"
  ".info{background:#1a237e;padding:12px;border-radius:6px;margin:12px 0;font-size:12px;color:#90caf9;border:1px solid #283593}"
  ".cb{display:flex;align-items:center;gap:8px;cursor:pointer;margin:8px 0}.cb input{width:auto}"
  ".back{background:#757575;color:#fff !important}.back:hover{background:#616161}";
// Return standard head string for buffered page sends
static String pageHead(const String& title) {
  return String(F("<!DOCTYPE html><html><head><title>")) + title +
    F("</title><meta charset='UTF-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'><style>") +
    FPSTR(COMMON_CSS) +
    F("</style></head><body><div class='c'>"
      // ── Web UI countdown banner — shown only when deep sleep is enabled ──────
      "<div id='_wucb' style='display:none;text-align:center;padding:5px 10px;margin-bottom:6px;"
      "border-radius:6px;font-size:12px;font-weight:600;background:#1f1f1f;color:#888'>"
      "&#x23F1; Web UI active: <span id='_wuct'>--:--</span></div>"
      "<script>"
      "(function(){"
      "var _s=0,_ds=false;"
      "function fmt(s){if(s<=0)return'00:00';"
      "var m=Math.floor(s/60),r=s%60;"
      "return(m<10?'0':'')+m+':'+(r<10?'0':'')+r;}"
      "function render(){"
      "var b=document.getElementById('_wucb');"
      "var t=document.getElementById('_wuct');"
      "if(!b||!t)return;"
      "if(!_ds){b.style.display='none';return;}"
      "b.style.display='block';"
      "t.textContent=fmt(_s);"
      "if(_s<=30){b.style.background='#3b1a1a';b.style.color='#ef9a9a';}"
      "else if(_s<=120){b.style.background='#2b2000';b.style.color='#ffe082';}"
      "else{b.style.background='#1f1f1f';b.style.color='#888';}}"
      "function poll(){"
      "fetch('/api/status').then(function(r){return r.json();}).then(function(d){"
      "_ds=!!d.deep_sleep_enabled;"
      "_s=parseInt(d.webui_secs_left)||0;"
      "render();"
      "}).catch(function(){});}"
      "setInterval(function(){if(_s>0)_s--;render();},1000);"
      "poll();setInterval(poll,30000);"
      "})();"
      "</script>");
}
static String pageFoot() { return F("</div></body></html>"); }

// Send a complete buffered page — no chunked transfer, browser gets it all at once
static void sendPage(const String& html) {
  server.send(200, "text/html", html);
}

// ═════════════════════════════════════════════════════════════════════════════
// OTA / WEB SERVER
// ═════════════════════════════════════════════════════════════════════════════
String actionPage(const String& icon,const String& msg,const String& sub,int secs,const String& dest="/"){
  String s=String(secs);
  return String(F("<!DOCTYPE html><html><head><meta charset='UTF-8'><style>"
    "*{margin:0;padding:0;box-sizing:border-box}"
    "body{font-family:-apple-system,sans-serif;background:#1a1a1a;color:#e0e0e0;"
    "min-height:100vh;display:flex;align-items:center;justify-content:center}"
    ".box{background:#2d2d2d;border-radius:12px;padding:36px 32px;max-width:400px;"
    "width:90%;text-align:center;box-shadow:0 8px 32px rgba(0,0,0,.4)}"
    ".ic{font-size:48px;margin-bottom:16px}h2{color:#64B5F6;margin-bottom:8px}"
    "p{color:#9e9e9e;font-size:14px;margin-bottom:20px}"
    ".bw{background:#1f1f1f;border-radius:6px;overflow:hidden;height:6px;margin-bottom:16px}"
    ".bar{height:6px;background:#64B5F6}"
    "a{display:inline-block;background:#64B5F6;color:#000;padding:10px 22px;"
    "border-radius:6px;text-decoration:none;font-weight:600;font-size:14px}"
    "</style>"))
    +"<script>var _t="+s+",_d='"+dest+"';"
    "var _b=document.querySelector('.bar');"
    "var _i=setInterval(function(){"
    "_t--;if(_b)_b.style.width=((_t/"+s+")*100)+'%';"
    "if(_t<=0){clearInterval(_i);location.href=_d;}},1000);</script>"
    "<body><div class='box'><div class='ic'>"+icon+"</div>"
    "<h2>"+msg+"</h2><p>"+sub+"</p>"
    "<div class='bw'><div class='bar' style='width:100%;transition:width 1s linear'></div></div>"
    "<a href='"+dest+"'>&#8592; Back now</a>"
    "</div></body></html>";
}

const char OTA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><title>OTA Update</title>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<style>*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,sans-serif;background:#1a1a1a;color:#e0e0e0;min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px}
.container{background:#2d2d2d;border-radius:12px;box-shadow:0 8px 32px rgba(0,0,0,.4);max-width:600px;width:100%;padding:32px}
h1{color:#64B5F6;margin-bottom:4px;font-size:24px}.subtitle{color:#9e9e9e;margin-bottom:6px;font-size:13px}
.fw-row{display:flex;align-items:center;gap:10px;margin-bottom:20px;background:#1f1f1f;padding:10px 14px;border-radius:8px;border:1px solid #333}
.fw-label{font-size:12px;color:#9e9e9e;text-transform:uppercase;letter-spacing:.5px}
.fw-val{font-size:16px;font-weight:700;color:#64B5F6;letter-spacing:.5px}
.arrow{font-size:18px;color:#555;flex-shrink:0}
.fw-new{color:#4db6ac}
.drop-area{border:3px dashed #555;border-radius:8px;padding:36px 20px;text-align:center;cursor:pointer;background:#1f1f1f;transition:all .3s;margin:16px 0}
.drop-area.drag-over{background:#1e2a2a;border-color:#64B5F6}
.drop-area p{color:#9e9e9e;margin:6px 0}
.file-input{display:none}
.file-info{background:#1f1f1f;border:1px solid #333;border-radius:8px;padding:12px 14px;margin:8px 0;min-height:36px;display:none}
.file-info.visible{display:block}
.file-row{display:flex;justify-content:space-between;align-items:center;gap:8px;font-size:13px}
.file-name{color:#64B5F6;font-weight:600;word-break:break-all}
.file-meta{color:#9e9e9e;font-size:12px;margin-top:4px}
.warn-merged{color:#ef9a9a;font-size:12px;margin-top:6px;background:#2a1515;padding:6px 10px;border-radius:4px;border:1px solid #5a1a1a}
.crc-row{font-size:12px;margin-top:6px;color:#9e9e9e}
.crc-val{color:#80cbc4;font-family:monospace}
.upload-btn{background:#64B5F6;color:#000;padding:14px 28px;border:none;border-radius:6px;cursor:pointer;font-size:15px;font-weight:600;width:100%;margin-top:12px;transition:all .2s}
.upload-btn:hover{background:#42A5F5}.upload-btn:disabled{background:#424242;color:#757575;cursor:not-allowed}
.progress-container{margin:20px 0;display:none}
.progress{width:100%;height:28px;background:#1f1f1f;border-radius:14px;overflow:hidden;margin-bottom:8px}
.progress-bar{height:100%;background:linear-gradient(90deg,#64B5F6,#42A5F5);transition:width .3s;display:flex;align-items:center;justify-content:center;color:#000;font-weight:600;font-size:13px;min-width:40px}
.progress-text{text-align:center;color:#9e9e9e;font-size:12px}
.status{padding:16px;border-radius:8px;margin:16px 0;display:none;font-weight:500;line-height:1.6}
.success{background:#1b5e20;color:#a5d6a7;border:1px solid #2e7d32}
.error{background:#b71c1c;color:#ef9a9a;border:1px solid #c62828}
.info-box{background:#1a237e;padding:14px;border-radius:8px;margin:16px 0;font-size:13px;color:#90caf9;border:1px solid #283593}
.steps{list-style:none;padding:0;margin:8px 0}
.steps li{padding:3px 0;font-size:13px;color:#90caf9}
.steps li::before{content:"→ ";color:#64B5F6}
.spinner{border:3px solid #424242;border-top:3px solid #64B5F6;border-radius:50%;width:40px;height:40px;animation:spin 1s linear infinite;margin:20px auto;display:none}
@keyframes spin{to{transform:rotate(360deg)}}
.back-btn{display:inline-block;margin-top:16px;color:#64B5F6;text-decoration:none;font-weight:600;font-size:14px}
.countdown{font-size:13px;color:#9e9e9e;text-align:center;margin-top:8px}
</style></head>
<body><div class='container'>
<h1>&#x1F680; Firmware Update</h1>
<div class='subtitle'>%DEVICE_NAME%</div>
<div class='fw-row'>
  <div><div class='fw-label'>Current Version</div><div class='fw-val'>v%FW_VERSION% <span style='font-size:11px;color:#546e7a'>[%RUNNING_CRC32%]</span></div></div>
  <div class='arrow'>&#x2192;</div>
  <div><div class='fw-label'>New Version</div><div class='fw-val fw-new' id='nv'>—</div></div>
  <div style='flex:1'></div>
  <div style='text-align:right'><div class='fw-label'>File Size</div><div class='fw-val' id='fsz' style='font-size:13px;color:#9e9e9e'>—</div></div>
</div>
<div class='info-box'>
  <strong>&#x26A0;&#xFE0F; How to build the correct file:</strong>
  <ul class='steps'>
    <li>Arduino IDE: Sketch &rarr; Export Compiled Binary &rarr; use <code>.ino.bin</code></li>
    <li>Do NOT use <code>.merged.bin</code> (includes bootloader — will fail Upload)</li>
    <li>Version is read from the <code>FW_VERSION</code> define embedded in the binary</li>
    <li>CRC32 is computed client-side before upload as a sanity check</li>
  </ul>
</div>
%REMOTE_INFO%
<form id='f'>
  <div class='drop-area' id='da'>
    <p>&#x1F4C1; Drag &amp; drop <strong>.ino.bin</strong> here</p>
    <p style='font-size:12px'>or click to browse</p>
    <input type='file' id='fi' accept='.bin' class='file-input'>
  </div>
  <div class='file-info' id='fi-info'>
    <div class='file-row'><span class='file-name' id='fname'>—</span></div>
    <div class='file-meta' id='fmeta'>—</div>
    <div class='crc-row'>CRC32: <span class='crc-val' id='fcrc'>computing...</span></div>
    <div class='warn-merged' id='fwarn' style='display:none'>
      &#x274C; Looks like a <strong>merged</strong> binary (.merged. in filename). This includes
      the bootloader and WILL fail. Export Compiled Binary from Arduino IDE instead.
    </div>
  </div>
  <button type='submit' class='upload-btn' id='ub' disabled>Upload Firmware</button>
</form>
<div class='progress-container' id='pc'>
  <div class='progress'><div class='progress-bar' id='pb'>0%</div></div>
  <div class='progress-text' id='pt'>Preparing upload...</div>
</div>
<div class='spinner' id='sp'></div>
<div class='status' id='st'></div>
<div class='countdown' id='cd'></div>
<div class='info-box' style='margin-top:16px'>
  <strong>&#x2601; GitHub Auto-Update</strong><br>
  <span style='font-size:12px'>Device checks <a href='https://github.com/piklz/heltec-wifikit32-DHT-MONITOR/tree/main/firmware' target='_blank' style='color:#64B5F6'>firmware repo</a> every hour for new releases.<br>
  When a new version is found, a banner appears on the dashboard and a notification is sent via MQTT and ntfy.<br>
  You can also install directly from GitHub via the dashboard banner, or use manual drag-drop above.</span><br><br>
  <button onclick="checkNow()" class='upload-btn' style='width:auto;padding:8px 18px;font-size:13px;margin:0'>&#x1F50D; Check Now</button>
  <span id='check-result' style='font-size:12px;color:#9e9e9e;margin-left:12px'></span>
  <div id='install-banner' style='display:none'></div>
</div>
<a href='/' class='back-btn'>&#x2190; Back to Dashboard</a>
</div>
<script>
function checkNow(){
  var el=document.getElementById('check-result');
  var ib=document.getElementById('install-banner');
  el.textContent='Checking...';
  fetch('/ota_check').then(r=>r.json()).then(function(d){
    if(d.update_available){
      el.innerHTML='<span style="color:#4db6ac">&#x2705; v'+d.available+' available &mdash; see below</span>';
      // Grab current file CRC for comparison
      var fcrcEl=document.getElementById('fcrc');
      var fileCrcVal=fcrcEl?fcrcEl.textContent:'';
      var crcMatch=(fileCrcVal.length===10&&d.crc32.length===8&&
                    fileCrcVal.replace('0x','').toUpperCase()===d.crc32.toUpperCase());
      var crcHtml='<span style="color:#9e9e9e">load a file above to verify</span>';
      if(fileCrcVal&&fileCrcVal!=='computing...'){
        crcHtml='<span style="color:'+(crcMatch?'#66bb6a':'#ef9a9a')+'">'+fileCrcVal+
                (crcMatch?' &#x2714; matches manifest':' &#x274C; mismatch!')+'</span>';
      }
      ib.innerHTML=
        '<div style="background:#1a3a3a;border:2px solid #4db6ac;border-radius:8px;padding:14px;margin-top:12px">'
        +'<div style="color:#4db6ac;font-weight:700;font-size:15px;margin-bottom:8px">&#x1F310; GitHub release: v'+d.available+'</div>'
        +(d.build_date?'<div style="color:#80cbc4;font-size:12px;margin-bottom:6px">Released: '+d.build_date+'</div>':'')
        +(d.changelog?'<div style="color:#b2dfdb;font-size:13px;margin-bottom:8px">'+d.changelog+'</div>':'')
        +'<div style="background:#0d2626;border-radius:4px;padding:8px 12px;font-family:monospace;font-size:12px;margin-bottom:12px">'
        +'<div style="margin-bottom:4px"><span style="color:#546e7a">Manifest CRC32 : </span>'
        +'<span style="color:#4db6ac">'+d.crc32+'</span>'
        +'<span style="color:#546e7a;font-size:11px;margin-left:8px">(from GitHub manifest.json)</span></div>'
        +'<div style="margin-bottom:4px"><span style="color:#546e7a">File CRC32     : </span>'+crcHtml+'</div>'
        +'<div><span style="color:#546e7a">Binary size    : </span>'
        +'<span style="color:#80cbc4">'+(d.size>0?(d.size/1024).toFixed(1)+' KB':'unknown')+'</span></div>'
        +'</div>'
        +'<div style="display:flex;gap:10px;flex-wrap:wrap;align-items:center">'
        +'<a href="/ota_install" style="background:#4db6ac;color:#000;padding:10px 22px;border-radius:6px;font-weight:700;font-size:14px;text-decoration:none">&#x26A1; Install from GitHub</a>'
        +'<span style="color:#9e9e9e;font-size:12px">OLED shows live progress &mdash; do not power off</span>'
        +'</div>'
        +'</div>';
      ib.style.display='block';
    } else {
      var crcMismatch = d.crc32.length===8 && d.running_crc32.length===8
                        && d.crc32.toUpperCase() !== d.running_crc32.toUpperCase();
      if (crcMismatch) {
        el.innerHTML='<span style="color:#ff8a65">&#x26A0; v'+d.current+' &mdash; CRC mismatch</span>';
        ib.innerHTML=
          '<div style="background:#2d1f0e;border:2px solid #ff8a65;border-radius:8px;padding:14px;margin-top:12px">'
          +'<div style="color:#ff8a65;font-weight:700;font-size:14px;margin-bottom:6px">&#x26A0; Same version, different binary</div>'
          +'<div style="color:#9e9e9e;font-size:12px;margin-bottom:8px">Running firmware CRC32 does not match the GitHub release. '
          +'The binary may have been rebuilt without a version bump.</div>'
          +(d.changelog?'<div style="background:#1a1200;border-radius:4px;padding:8px 10px;font-size:12px;color:#ffe082;margin-bottom:10px">'
          +'<span style="color:#546e7a">Changelog: </span>'+d.changelog+'</div>':'')
          +'<div style="background:#1a1200;border-radius:4px;padding:8px 12px;font-family:monospace;font-size:12px;margin-bottom:12px">'
          +'<div><span style="color:#546e7a">Running CRC32 : </span><span style="color:#ef9a9a">'+d.running_crc32+'</span></div>'
          +'<div><span style="color:#546e7a">GitHub CRC32  : </span><span style="color:#4db6ac">'+d.crc32+'</span></div>'
          +'</div>'
          +'<a href="/ota_reinstall" style="background:#ff8a65;color:#000;padding:10px 22px;border-radius:6px;'
          +'font-weight:700;font-size:14px;text-decoration:none">&#x1F504; Re-flash from GitHub</a>'
          +'</div>';
        ib.style.display='block';
      } else {
        el.innerHTML='<span style="color:#66bb6a">&#x2714; Up to date (v'+d.current+')</span>';
        ib.style.display='none';
      }
    }
  }).catch(function(){el.innerHTML='<span style="color:#ef9a9a">&#x274C; Check failed &mdash; is WiFi connected?</span>';});
}
// CRC32 table
var CURRENT_VER="%FW_VERSION%";
var crcTable=(function(){var t=[];for(var n=0;n<256;n++){var c=n;for(var k=0;k<8;k++)c=(c&1)?0xEDB88320^(c>>>1):c>>>1;t[n]=c;}return t;})();
function crc32(buf){var crc=0xFFFFFFFF;var u8=new Uint8Array(buf);for(var i=0;i<u8.length;i++)crc=(crc>>>8)^crcTable[(crc^u8[i])&0xFF];return((crc^0xFFFFFFFF)>>>0).toString(16).toUpperCase().padStart(8,'0');}

// Extract firmware version from binary reliably using a strict marker signature
function sniffVersion(buf) {
  var u8 = new Uint8Array(buf);
  var marker = 'FW_VER:';
  
  // Strict, fast loop looking ONLY for the literal marker string
  for (var i = 0; i < u8.length - marker.length - 8; i++) {
    var match = true;
    for (var j = 0; j < marker.length; j++) {
      if (u8[i + j] !== marker.charCodeAt(j)) {
        match = false;
        break;
      }
    }
    
    // Found the exact signature block! Extract the version characters
    if (match) {
      var ver = '';
      // Read the raw bytes immediately following the marker
      for (var k = i + marker.length; k < i + marker.length + 8; k++) {
        var charCode = u8[k];
        // Break instantly if we hit a null terminator, space, or non-printable character
        if (charCode === 0 || charCode < 33 || charCode > 126) break;
        ver += String.fromCharCode(charCode);
      }
      
      // Clean up the extraction and verify it fits standard versioning format
      ver = ver.trim();
      if (/^\d+\.\d+/.test(ver)) {
        return ver;
      }
    }
  }
  
  return 'unknown';
}

var fi=document.getElementById('fi'),da=document.getElementById('da'),
    ub=document.getElementById('ub'),pc=document.getElementById('pc'),
    pb=document.getElementById('pb'),pt=document.getElementById('pt'),
    sp=document.getElementById('sp'),st=document.getElementById('st'),
    cd=document.getElementById('cd'),nv=document.getElementById('nv'),
    fsz=document.getElementById('fsz'),fiInfo=document.getElementById('fi-info'),
    fname=document.getElementById('fname'),fmeta=document.getElementById('fmeta'),
    fcrc=document.getElementById('fcrc'),fwarn=document.getElementById('fwarn'),
    f=document.getElementById('f');

da.onclick=()=>fi.click();
['dragenter','dragover','dragleave','drop'].forEach(e=>da.addEventListener(e,ev=>{ev.preventDefault();ev.stopPropagation();}));
['dragenter','dragover'].forEach(e=>da.addEventListener(e,()=>da.classList.add('drag-over')));
['dragleave','drop'].forEach(e=>da.addEventListener(e,()=>da.classList.remove('drag-over')));
da.addEventListener('drop',e=>{fi.files=e.dataTransfer.files;onFile();});
fi.onchange=onFile;

function onFile(){
  var x=fi.files[0];if(!x)return;
  fname.textContent=x.name;
  fmeta.textContent=(x.size/1024).toFixed(1)+' KB  •  '+new Date(x.lastModified).toLocaleString();
  fsz.textContent=(x.size/1024).toFixed(1)+' KB';
  fiInfo.classList.add('visible');
  fwarn.style.display=x.name.toLowerCase().includes('.merged.')?'block':'none';
  fcrc.textContent='computing...';
  ub.disabled=true;
  var r=new FileReader();
  r.onload=function(e){
    var crc=crc32(e.target.result);
    fcrc.textContent='0x'+crc;
    var ver=sniffVersion(e.target.result);
    nv.textContent='v'+ver;
    ub.disabled=x.name.toLowerCase().includes('.merged.');
  };
  r.readAsArrayBuffer(x);
}

f.onsubmit=async function(e){
  e.preventDefault();
  var x=fi.files[0];
  if(!x){alert('Select a firmware file first.');return;}
  if(!x.name.endsWith('.bin')){alert('File must end in .bin');return;}
  if(x.name.toLowerCase().includes('.merged.')){alert('Do not use merged binaries. Export Compiled Binary from Arduino IDE instead.');return;}
  // ── Downgrade warning ──────────────────────────────────────────────────────
  var sniffedVer = nv.textContent.replace(/^v/,'').trim();
  if(sniffedVer && sniffedVer!=='unknown' && sniffedVer!=='—'){
    var cv=CURRENT_VER.split('.').map(Number), sv=sniffedVer.split('.').map(Number);
    var isDowngrade=(sv[0]<cv[0])||(sv[0]===cv[0]&&sv[1]<cv[1]);
    if(isDowngrade && !confirm(
      '⚠️ Downgrade detected\n\n'+
      'You are flashing v'+sniffedVer+' over the current v'+CURRENT_VER+'.\n\n'+
      'This is fine for development/testing.\nContinue?'
    )) return;
  }
  var fd=new FormData();fd.append('update',x);
  ub.disabled=true;pc.style.display='block';st.style.display='none';cd.textContent='';
  var xhr=new XMLHttpRequest(),t0=Date.now();
  xhr.upload.onprogress=function(e){
    if(e.lengthComputable){
      var p=Math.round(e.loaded/e.total*100);
      var spd=(e.loaded/1024)/Math.max(0.1,(Date.now()-t0)/1000);
      pb.style.width=p+'%';pb.textContent=p+'%';
      pt.textContent='Uploading: '+(e.loaded/1024).toFixed(0)+' / '+(e.total/1024).toFixed(0)+' KB  ('+spd.toFixed(1)+' KB/s)';
    }
  };
  xhr.onload=function(){
    pc.style.display='none';
    if(xhr.status===200&&xhr.responseText.indexOf('OK')!==-1){
      sp.style.display='block';
      st.className='status success';
      st.innerHTML='&#x2705; <strong>Update Successful!</strong><br>'
        +'Device is rebooting and applying the new firmware.<br>'
        +'OLED will show confirmation. Reconnecting in 20s&hellip;';
      st.style.display='block';
      var countdown=20;
      var iv=setInterval(function(){
        countdown--;cd.textContent='Redirecting in '+countdown+'s…';
        if(countdown<=0){clearInterval(iv);cd.textContent='';location.href='/';}
      },1000);
    } else {
      st.className='status error';
      st.innerHTML='&#x274C; <strong>Update Failed</strong><br>'
        +'Server returned: '+xhr.responseText+'<br>'
        +'Check that you are using the <strong>.ino.bin</strong> (not .merged.bin) file.';
      st.style.display='block';ub.disabled=false;
    }
  };
  xhr.onerror=function(){
    pc.style.display='none';
    st.className='status error';
    st.innerHTML='&#x274C; <strong>Upload Error</strong><br>'
      +'Connection lost during transfer. The device may have rebooted.<br>'
      +'Wait 20s then <a href="/" style="color:#ef9a9a">check the dashboard</a>.';
    st.style.display='block';ub.disabled=false;
  };
  xhr.open('POST','/update',true);xhr.send(fd);
};
</script></body></html>)rawliteral";


// ═════════════════════════════════════════════════════════════════════════════
// GITHUB OTA UPDATE CHECK
// ═════════════════════════════════════════════════════════════════════════════

// Compare two "major.minor" version strings. Returns true if newVer > current.
bool isVersionNewer(const String& current, const String& newVer) {
  int cMaj = 0, cMin = 0, nMaj = 0, nMin = 0;
  sscanf(current.c_str(), "%d.%d", &cMaj, &cMin);
  sscanf(newVer.c_str(),  "%d.%d", &nMaj, &nMin);
  return (nMaj > cMaj) || (nMaj == cMaj && nMin > cMin);
}

// Fetch manifest.json from GitHub, parse it, and update otaUpdate* globals.
// Sends MQTT + ntfy if a newer version is found (respects cooldown).
//
// SLEEP-AWARE INTERVAL:
//   millis() resets to 0 on every deep-sleep wake, so a millis-based interval
//   is meaningless — it would fire on every wake. Instead we persist the last
//   check time as a Unix epoch uint32 in NVS ("ota"/"last_chk").
//   If NTP hasn't synced yet (time() returns <100000), we skip the interval
//   check and always run — this ensures the first post-boot check runs promptly.
//   After a successful check the epoch is saved to NVS.
//   In loop() we also guard with lastOtaCheck (millis) so we don't call this
//   more than once per wake even if the NTP condition would allow it.
void checkOtaManifest(bool force) {
  if (!wifiConnected) return;

  // ── millis guard: don't call more than once per wake cycle ───────────────
  if (!force && lastOtaCheck > 0) return;   // already checked this wake
  lastOtaCheck = millis();                   // mark as checked for this wake

  // ── NVS epoch guard: at most once per OTA_CHECK_INTERVAL_SECS ─────────────
  if (!force) {
    time_t now = time(nullptr);
    if (now > 100000UL) {
      // NTP synced — load last check epoch from NVS
      preferences.begin("ota", true);
      uint32_t lastChkEpoch = preferences.getUInt("last_chk", 0);
      preferences.end();
      if (lastChkEpoch > 0 && (uint32_t)now - lastChkEpoch < OTA_CHECK_INTERVAL_SECS) {
        Serial.printf("[OTA] Next check in %lus — skipping\n",
          (unsigned long)(OTA_CHECK_INTERVAL_SECS - ((uint32_t)now - lastChkEpoch)));
        // Still sync runtime flag from RTC in case it was set a previous wake
        otaUpdateAvailable = rtcOtaAvailable;
        otaCheckDone = true;
        return;
      }
    }
    // If now <= 100000 (no NTP) fall through and check anyway
  }

  Serial.println(F("[OTA] Checking manifest: " MANIFEST_URL));

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(8000);
  http.begin(MANIFEST_URL);
  http.addHeader("Accept", "*/*"); 
  int code = http.GET();

  if (code != HTTP_CODE_OK) {
    Serial.printf("[OTA] Manifest fetch failed: HTTP %d\n", code);
    http.end();
    otaCheckDone = true;
    return;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("[OTA] Manifest parse error: %s\n", err.c_str());
    otaCheckDone = true;
    return;
  }

  String remoteVer   = doc["version"]    | "";
  String binaryUrl   = doc["binary"]     | "";
  String crc32Hex    = doc["crc32"]      | "";
  uint32_t fileSize  = doc["size"]       | (uint32_t)0;
  String changelog   = doc["changelog"]  | "";
  String buildDate   = doc["build_date"] | "";

  crc32Hex.toUpperCase();

  Serial.printf("[OTA] Manifest: remote v%s  current v%s\n",
                remoteVer.c_str(), FW_VERSION);

  if (remoteVer.length() == 0 || binaryUrl.length() == 0) {
    Serial.println(F("[OTA] Manifest missing required fields"));
    otaCheckDone = true;
    return;
  }

  if (isVersionNewer(FW_VERSION, remoteVer)) {
    bool newAlert = (remoteVer != otaNewVersion);
    otaUpdateAvailable = true;
    rtcOtaAvailable    = true;   // persists across deep sleep for OLED icon
    otaDismissed       = false;
    otaNewVersion      = remoteVer;
    otaDownloadUrl     = binaryUrl;
    otaCrc32Expected   = crc32Hex;
    otaFileSize        = fileSize;
    otaChangelog       = changelog;
    otaBuildDate       = buildDate;
    Serial.printf("[OTA] Update available: v%s -> v%s\n", FW_VERSION, remoteVer.c_str());

    // ── ntfy cooldown: NVS-persisted epoch so it survives deep sleep ────────
    // millis()-based lastOtaNtfy resets to 0 on every wake — can't use it for
    // cross-wake dedup. Use "ota"/"ntfy_ep" (uint32 unix epoch) instead.
    // Falls back to allowing the send if NTP hasn't synced yet.
    bool ntfyCooldownOk = false;
    {
      time_t nowNtfy = time(nullptr);
      if (nowNtfy > 100000UL) {
        preferences.begin("ota", true);
        uint32_t ntfyEp = preferences.getUInt("ntfy_ep", 0);
        preferences.end();
        ntfyCooldownOk = (ntfyEp == 0 ||
                          (uint32_t)nowNtfy - ntfyEp >= (uint32_t)(OTA_NTFY_COOLDOWN_MS / 1000UL));
      } else {
        // No NTP yet — use in-session millis guard as fallback
        ntfyCooldownOk = (millis() - lastOtaNtfy > OTA_NTFY_COOLDOWN_MS);
      }
    }

    // ── MQTT update notification ──────────────────────────────────────────────
    if ((mqtt_platform == "standard" || mqtt_platform == "all") && mqttStd.connected()) {
      JsonDocument nd;
      nd["event"]      = "ota_available";
      nd["device"]     = device_name;
      nd["current"]    = FW_VERSION;
      nd["available"]  = remoteVer;
      nd["size"]       = fileSize;
      nd["crc32"]      = crc32Hex;
      nd["changelog"]  = changelog;
      nd["url"]        = binaryUrl;
      String p; serializeJson(nd, p);
      mqttStd.publish((mqtt_topic + "/ota").c_str(), p.c_str());
      mqttStd.loop();
      Serial.println(F("[OTA] MQTT ota notification published"));
    }

    // ── ntfy update notification (once per version per 24 h) ─────────────────
    if (ntfy_enabled && ntfy_topic.length() && ntfyCooldownOk) {
      String msg = "Device: " + device_name + "\n"
                   "Current:   v" + String(FW_VERSION) + "\n"
                   "Available: v" + remoteVer;
      if (buildDate.length())  msg += "  (" + buildDate + ")";
      if (changelog.length())  msg += "\n" + changelog;
      if (fileSize > 0) msg += "\n" + String(fileSize / 1024) + " KB";
      msg += "\nInstall via web UI → OTA";
      sendNtfy("Firmware Update Available", msg, 3, "rocket,inbox_tray");
      lastOtaNtfy = millis();   // in-session dedup
      // Persist epoch so cooldown survives deep sleep
      {
        time_t nowEp = time(nullptr);
        if (nowEp > 100000UL) {
          preferences.begin("ota", false);
          preferences.putUInt("ntfy_ep", (uint32_t)nowEp);
          preferences.end();
          Serial.printf("[OTA] ntfy epoch saved: %lu\n", (unsigned long)nowEp);
        }
      }
    }
  } else {
    Serial.println(F("[OTA] Firmware is up to date"));
    otaUpdateAvailable = false;
    rtcOtaAvailable    = false;  // clear sleep-persistent flag
    otaNewVersion      = "";
  }

  // ── Persist check epoch to NVS so the interval survives deep sleep ─────────
  {
    time_t now = time(nullptr);
    if (now > 100000UL) {
      preferences.begin("ota", false);
      preferences.putUInt("last_chk", (uint32_t)now);
      preferences.end();
      Serial.printf("[OTA] Check epoch saved: %lu\n", (unsigned long)now);
    }
  }
  otaCheckDone = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Streaming OTA install from a URL (e.g. GitHub raw binary).
// Downloads the binary, verifies CRC32 against expectedCrc32 (uppercase hex),
// and applies the update. Returns true on success.
// Blocks the main loop — call only from a web handler or button ISR context.
// OLED shows live progress; watchdog is fed throughout.
// ─────────────────────────────────────────────────────────────────────────────
bool installOtaFromUrl(const String& url, const String& expectedCrc32, uint32_t knownSize) {
  Serial.println("[OTA] Installing from URL: " + url);

  if (!stealthThisWake) {
    display.displayOn(); display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 0,  "OTA GITHUB INSTALL");
    display.drawLine(0, 12, 128, 12);
    display.drawString(64, 24, "Connecting...");
    display.drawString(64, 40, "Do not power off");
    display.display();
  }

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(30000);
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[OTA] Download failed: HTTP %d\n", httpCode);
    http.end();
    return false;
  }

  int contentLen = http.getSize();
  uint32_t totalExpected = (knownSize > 0) ? knownSize : (contentLen > 0 ? (uint32_t)contentLen : 0);

  if (!Update.begin(totalExpected > 0 ? totalExpected : UPDATE_SIZE_UNKNOWN)) {
    Serial.print(F("[OTA] Update.begin failed: "));
    Update.printError(Serial);
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[512];
  uint32_t received  = 0;
  uint32_t crcState  = 0xFFFFFFFF;
  unsigned long lastOledUpdate = 0;

  while (http.connected()) {
    esp_task_wdt_reset();
    size_t avail = stream->available();
    if (avail == 0) { delay(1); continue; }

    size_t chunk = stream->readBytes(buf, min(avail, sizeof(buf)));
    if (chunk == 0) break;

    // Write to flash
    if (Update.write(buf, chunk) != chunk) {
      Serial.print(F("[OTA] Write error: "));
      Update.printError(Serial);
      http.end();
      return false;
    }

    // Accumulate CRC32
    crcState = crc32Feed(crcState, buf, chunk);
    received += chunk;

    // OLED progress (throttled to ~400 ms)
    if (!stealthThisWake && millis() - lastOledUpdate > 400) {
      lastOledUpdate = millis();
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.drawString(64, 0, "OTA GITHUB INSTALL");
      display.drawLine(0, 12, 128, 12);
      display.drawString(64, 16, String(received / 1024) + " / " +
        (totalExpected > 0 ? String(totalExpected / 1024) : "?") + " KB");
      if (totalExpected > 0) {
        int pct = (int)((received * 100UL) / totalExpected);
        display.drawRect(0, 30, 124, 8);
        display.fillRect(2, 32, (pct * 120) / 100, 4);
        display.drawString(64, 42, String(pct) + "%  Flashing...");
      } else {
        display.drawString(64, 38, "Flashing...");
      }
      display.drawString(64, 54, "Do not power off");
      display.display();
    }

    if (totalExpected > 0 && received >= totalExpected) break;
  }
  http.end();

  uint32_t crcFinal = crcState ^ 0xFFFFFFFF;
  char crcHex[9]; snprintf(crcHex, sizeof(crcHex), "%08X", crcFinal);
  Serial.printf("[OTA] Downloaded %u bytes, CRC32: %s\n", received, crcHex);

  // ── CRC32 verification ────────────────────────────────────────────────────
  if (expectedCrc32.length() == 8) {
    String expectedUp = expectedCrc32; expectedUp.toUpperCase();
    if (String(crcHex) != expectedUp) {
      Serial.printf("[OTA] CRC32 MISMATCH! got %s expected %s\n",
                    crcHex, expectedUp.c_str());
      Update.abort();
      if (!stealthThisWake) {
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 10, "OTA FAILED");
        display.drawString(64, 26, "CRC32 MISMATCH");
        display.drawString(64, 42, "Flash aborted");
        display.display(); delay(4000);
      }
      return false;
    }
    Serial.println(F("[OTA] CRC32 verified OK"));
  } else {
    Serial.println(F("[OTA] No expected CRC32 — skipping verification"));
  }

  if (!Update.end(true)) {
    Serial.print(F("[OTA] Update.end failed: "));
    Update.printError(Serial);
    return false;
  }

  Serial.println(F("[OTA] Install complete — rebooting"));
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// showOtaBootSplash() — called once in setup() when NVS "ota/updated" is set.
// Shows a temporary OLED frame: "FIRMWARE UPDATED / vX.XX → vY.YY / Boot#N"
// for ~8 s, then returns so normal setup continues. Clears the NVS flag.
// ─────────────────────────────────────────────────────────────────────────────
void showOtaBootSplash(const String& prevVer) {
  if (stealthThisWake) {
    Serial.println(F("[OTA-SPLASH] Stealth wake — skipping OLED splash"));
    return;
  }
  // Stop the normal UI frame rotation so it doesn't overwrite our splash.
  // We re-enable it by returning to loop() where ui.update() continues.
  display.displayOn();
  display.setTextAlignment(TEXT_ALIGN_CENTER);

  Serial.printf("[OTA-SPLASH] Showing: v%s -> v%s  Boot#%d\n",
    prevVer.c_str(), FW_VERSION, bootCount);

  for (int i = 8; i >= 0; i--) {
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(64, 0, "FIRMWARE UPDATED");
    display.drawLine(0, 12, 128, 12);

    display.setFont(ArialMT_Plain_16);
    // Keep arrow short — long strings overflow 128px at 16pt
    String arrow = "v" + prevVer + F(" > v") + String(FW_VERSION);
    display.drawString(64, 16, arrow);

    display.setFont(ArialMT_Plain_10);
    display.drawString(64, 36, "Boot #" + String(bootCount));

    // Countdown bar drains left-to-right
    display.drawRect(0, 48, 124, 8);
    display.fillRect(2, 50, (i * 120) / 8, 4);
    if (i > 0) display.drawString(64, 56, String(i) + "s");
    display.display();

    if (i > 0) {
      unsigned long t = millis();
      while (millis() - t < 1000) {
        esp_task_wdt_reset();
        button.tick();
        led.Update();
        delay(20);
      }
    }
  }
  Serial.println(F("[OTA-SPLASH] Done"));
}

// ── Forward declarations — screensaver reset functions called inside lambdas
// in setupOTA() which precedes their definitions. Arduino's auto-prototype
// generator does not forward-declare functions referenced inside lambdas.
void ssResetBall();
void ssResetMario();

void setupOTA() {
  // ── Dashboard ──────────────────────────────────────────────────────────────
  server.on("/", HTTP_GET, []() {
    // Only extend the sleep window to 10 min if this was a button wake
    // (window already > 5 min). Timer wakes keep their short window so
    // the browser syncs down from /api/status rather than overriding it.
    if (webUiExpiresAt > millis() + 5UL * 60UL * 1000UL) {
      webUiExpiresAt = millis() + 10UL * 60UL * 1000UL;  // refresh button-wake window
    }
    String h = pageHead(device_name);

    // System
    h += F("<h1>&#x1F4DF; ");
    h += device_name;
    h += F("</h1>"
      // ── GitHub / project link panel ──────────────────────────────────────
      "<div class='card' style='padding:10px 16px;margin-bottom:4px;"
      "display:flex;align-items:center;gap:10px;flex-wrap:wrap'>"
      "<span style='color:#e57373;font-size:16px'>&#x2665;</span>"
      "<a href='https://github.com/piklz/heltec-wifikit32-DHT-MONITOR'"
      " target='_blank'"
      " style='color:#64B5F6;font-size:13px;font-weight:600;text-decoration:none;"
      "letter-spacing:.2px'>"
      "HELTEC ESP32 Temp / Humidity MQTT Monitor</a>"
      "<span style='color:#616161;font-size:12px;margin-left:auto'>v");
    h += FW_VERSION;
    h += F("</span></div>");

    // ── OTA update-available banner (TOP of dashboard) ──────────────────────
    // Shown immediately below the header card when a newer FW is confirmed.
    if (otaUpdateAvailable && !otaDismissed) {
      h += F("<div style='"
        "background:#1a3a3a;border:2px solid #4db6ac;border-radius:10px;"
        "padding:14px 18px;margin-bottom:16px;"
        "display:flex;align-items:flex-start;gap:14px;flex-wrap:wrap'>"
        "<span style='font-size:26px;flex-shrink:0'>&#x1F680;</span>"
        "<div style='flex:1;min-width:180px'>"
        "<div style='color:#4db6ac;font-weight:700;font-size:16px;margin-bottom:4px'>"
        "Firmware update available &mdash; v");
      h += otaNewVersion;
      h += F("</div>");
      if (otaBuildDate.length()) {
        h += F("<div style='color:#80cbc4;font-size:12px'>Released: ");
        h += otaBuildDate;
        h += F("</div>");
      }
      if (otaChangelog.length()) {
        h += F("<div style='color:#b2dfdb;font-size:13px;margin-top:4px'>");
        h += otaChangelog;
        h += F("</div>");
      }
      if (otaCrc32Expected.length()) {
        h += F("<div style='color:#546e7a;font-size:11px;margin-top:4px;"
          "font-family:monospace'>Manifest CRC32: ");
        h += otaCrc32Expected;
        h += F("</div>");
      }
      h += F("<div style='margin-top:10px;display:flex;gap:10px;flex-wrap:wrap'>");
      if (otaDownloadUrl.length()) {
        h += F("<a href='/ota_install' style='"
          "background:#4db6ac;color:#000;padding:8px 16px;border-radius:6px;"
          "font-weight:700;font-size:13px;text-decoration:none'>"
          "&#x26A1; Install from GitHub</a>");
      }
      h += F("<a href='/update' style='"
        "background:#37474f;color:#e0e0e0;padding:8px 16px;border-radius:6px;"
        "font-weight:600;font-size:13px;text-decoration:none'>"
        "&#x1F4C1; Upload manually</a>"
        "<a href='/ota_dismiss' style='"
        "color:#546e7a;font-size:12px;padding:8px 6px;text-decoration:none'>"
        "Dismiss</a>"
        "</div></div></div>");
    }

    // ── Live Stats panel ───────────────────────────────────────────────────
    h += F("<div class='card'>"
      "<h2>&#x1F4CA; Live Stats</h2>"
      "<div style='display:grid;grid-template-columns:1fr 1fr;gap:8px 16px'>");

    // Temperature with threshold badge + trend arrow
    h += F("<div><span style='color:#9e9e9e;font-size:12px'>TEMPERATURE</span><br>"
      "<span style='font-size:26px;font-weight:600;color:");
    if      (temperature >= sensorTempHi) h += F("#ef5350");
    else if (temperature <= sensorTempLo) h += F("#42a5f5");
    else                                   h += F("#ffffff");
    h += F("'>");
    h += String(temperature,1);
    h += F("</span><span style='color:#9e9e9e'>&nbsp;&deg;C</span>");
    // Trend arrow — colour: red=rising, blue=falling, hidden when stable
    if (trendTemp != 0) {
      h += F("<span style='font-size:18px;font-weight:700;margin-left:5px;color:");
      h += (trendTemp > 0) ? F("#ef5350") : F("#42a5f5");
      h += F("'>");
      h += (trendTemp > 0) ? F("&#x2191;") : F("&#x2193;");
      h += F("</span>");
    }
    h += F("<br><span style='font-size:11px;color:#666'>");
    h += String(sensorTempLo,1) + "&deg;&#x2193; &nbsp; " + String(sensorTempHi,1) + "&deg;&#x2191;";
    h += F("</span></div>");

    // Humidity with threshold badge + trend arrow
    h += F("<div><span style='color:#9e9e9e;font-size:12px'>HUMIDITY</span><br>"
      "<span style='font-size:26px;font-weight:600;color:");
    if      (humidity >= sensorHumHi) h += F("#ef5350");
    else if (humidity <= sensorHumLo) h += F("#42a5f5");
    else                               h += F("#ffffff");
    h += F("'>");
    h += String(humidity,1);
    h += F("</span><span style='color:#9e9e9e'>&nbsp;%</span>");
    // Trend arrow — colour: blue=rising (more humid), orange=falling (drying)
    if (trendHumidity != 0) {
      h += F("<span style='font-size:18px;font-weight:700;margin-left:5px;color:");
      h += (trendHumidity > 0) ? F("#42a5f5") : F("#ff9800");
      h += F("'>");
      h += (trendHumidity > 0) ? F("&#x2191;") : F("&#x2193;");
      h += F("</span>");
    }
    h += F("<br><span style='font-size:11px;color:#666'>");
    h += String(sensorHumLo,1) + "%&#x2193; &nbsp; " + String(sensorHumHi,1) + "%&#x2191;";
    h += F("</span></div>");

    // Battery
    h += F("<div><span style='color:#9e9e9e;font-size:12px'>BATTERY</span><br>"
      "<span style='font-size:20px;font-weight:600'>");
    h += String(batteryPercentage);
    h += F("%</span><span style='color:#9e9e9e;font-size:13px'> &nbsp;");
    h += String(batteryVoltFloat/1000.0f, 2);
    h += F("V</span><br><span style='font-size:12px'>");
    if (isUSBPowered)
      h += F("<span style='color:#66bb6a'>&#x26A1; USB / Charging</span>");
    else
      h += F("<span style='color:#ffa726'>&#x1F50B; Battery</span>");
    h += F("</span></div>");

    // Time + last reading
    h += F("<div><span style='color:#9e9e9e;font-size:12px'>TIME / LAST READ</span><br>"
      "<span style='font-size:18px;font-weight:600;color:#64B5F6'>");
    if (ntpSynced) {
      h += currentDateStr + " " + currentTimeStr;
    } else {
      h += F("--/-- --:--"); 
    }
    h += F("</span><br><span style='color:#9e9e9e;font-size:12px'>"
      "Up: ");
    h += getUptime();
    h += F(" &nbsp;&#x23F1;</span></div>");

    // WiFi
    h += F("</div>");  // close grid
    h += F("<div style='margin-top:10px;padding-top:8px;border-top:1px solid #333;"
      "font-size:12px;color:#9e9e9e;display:flex;gap:16px;flex-wrap:wrap'>");
    h += F("<span>&#x1F4F6; ");
    h += WiFi.SSID();
    h += F(" &nbsp;");
    h += String(WiFi.RSSI());
    h += F("dBm</span><span>&#x1F5A5; ");
    h += WiFi.localIP().toString();
    h += F("</span><span>v");
    h += FW_VERSION;
    h += F(" &nbsp;Boot#");
    h += String(bootCount);
    h += F("</span></div></div>");

    // Sensor alerts status strip (only if any threshold is currently breached)
    if (tempHiSent || tempLoSent || humHiSent || humLoSent) {
      h += F("<div class='card' style='border-left:3px solid #ef5350;padding-left:12px'>"
        "<h2 style='color:#ef5350'>&#x26A0; Active Alerts</h2>");
      if (tempHiSent) { h += F("<p>&#x1F321;&#xFE0F; Temp HIGH: "); h += String(temperature,1); h += F("&deg;C &ge; "); h += String(sensorTempHi,1); h += F("&deg;C</p>"); }
      if (tempLoSent) { h += F("<p>&#x1F321;&#xFE0F; Temp LOW: ");  h += String(temperature,1); h += F("&deg;C &le; "); h += String(sensorTempLo,1); h += F("&deg;C</p>"); }
      if (humHiSent)  { h += F("<p>&#x1F4A7; Hum HIGH: ");  h += String(humidity,1);    h += F("% &ge; ");  h += String(sensorHumHi,1);  h += F("%</p>"); }
      if (humLoSent)  { h += F("<p>&#x1F4A7; Hum LOW: ");   h += String(humidity,1);    h += F("% &le; ");  h += String(sensorHumLo,1);  h += F("%</p>"); }
      h += F("</div>");
    }

    // Sensor

    // ── Reading History Chart ─────────────────────────────────────────────────
    if (rtcHistCount >= 2) {
      uint8_t count = rtcHistCount;
      float temps[HIST_SIZE], hums[HIST_SIZE];
      String labels[HIST_SIZE];
      for (uint8_t i = 0; i < count; i++) {
        uint8_t ridx = (rtcHistHead - count + i + HIST_SIZE) % HIST_SIZE;
        temps[i] = rtcHistTemp[ridx]; hums[i] = rtcHistHum[ridx];
        if (rtcHistTime[ridx] > 0) {
          time_t t = (time_t)rtcHistTime[ridx];
          struct tm* ti = localtime(&t);
          char tb[6]; strftime(tb, sizeof(tb), "%H:%M", ti);
          labels[i] = String(tb);
        } else { labels[i] = "#" + String(i + 1); }
      }
      String tArr="[", hArr="[", lArr="[";
      for (uint8_t i = 0; i < count; i++) {
        if (i) { tArr+=","; hArr+=","; lArr+=","; }
        tArr += String(temps[i], 1);
        hArr += String(hums[i], 1);
        lArr += "\"" + labels[i] + "\"";
      }
      tArr+="]"; hArr+="]"; lArr+="]";
      h += F("<div class='card'><h2>&#x1F4C8; History</h2>"
        "<canvas id='hc' height='160'></canvas>"
        "<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js'></script>"
        "<script>");
      h += "new Chart(document.getElementById('hc'),{type:'line',data:{labels:" + lArr + ",";
      h += "datasets:[";
      h += "{label:'Temp \u00b0C',data:" + tArr + ",borderColor:'#ff9800',"
           "backgroundColor:'rgba(255,152,0,0.07)',borderWidth:2,"
           "pointRadius:4,pointBackgroundColor:'#ff9800',tension:0.35,yAxisID:'yT'},";
      h += "{label:'Humidity %',data:" + hArr + ",borderColor:'#42a5f5',"
           "backgroundColor:'rgba(66,165,245,0.07)',borderWidth:2,"
           "pointRadius:4,pointBackgroundColor:'#42a5f5',tension:0.35,yAxisID:'yH'}";
      h += "]},options:{responsive:true,interaction:{mode:'index',intersect:false},";
      h += "plugins:{legend:{labels:{color:'#ccc',font:{size:12}}},";
      h += "tooltip:{callbacks:{label:ctx=>ctx.dataset.label+': '+ctx.parsed.y.toFixed(1)}}},";
      h += "scales:{";
      h += "x:{ticks:{color:'#9e9e9e',font:{size:11}},grid:{color:'#2a2a2a'},";
      h += "title:{display:true,text:'Time',color:'#777',font:{size:11}}},";
      h += "yT:{type:'linear',position:'left',ticks:{color:'#ff9800',font:{size:11}},";
      h += "grid:{color:'#2a2a2a'},title:{display:true,text:'\u00b0C',color:'#ff9800'}},";
      h += "yH:{type:'linear',position:'right',ticks:{color:'#42a5f5',font:{size:11}},";
      h += "grid:{drawOnChartArea:false},title:{display:true,text:'%',color:'#42a5f5'}}";
      h += "}}})</script></div>";
    }

    h += F("<div class='card'><h2>Sensor</h2>"
      "<p><strong>Temperature:</strong> ");
    h += String(temperature,1);
    h += F(" &deg;C</p><p><strong>Humidity:</strong> ");
    h += String(humidity,1);
    h += F(" %</p></div>");

    // MQTT
    h += F("<div class='card'><h2>MQTT</h2><p><strong>Platform:</strong> ");
    String plat=mqtt_platform; plat.toUpperCase();
    h += plat;
    h += F("</p>");
    if(mqtt_platform=="all"||mqtt_platform=="standard"){
      h += F("<p><span class='dot ");
      h += mqttStandardConnected?F("on"):F("off");
      h += F("'></span>Standard</p>");
    }
    if(mqtt_platform=="all"||mqtt_platform=="adafruit"){
      h += F("<p><span class='dot ");
      h += mqttAdafruitConnected?F("on"):F("off");
      h += F("'></span>Adafruit IO</p>");
    }
    if(mqtt_platform=="all"||mqtt_platform=="ubidots"){
      h += F("<p><span class='dot ");
      h += mqttUbidotsConnected?F("on"):F("off");
      h += F("'></span>Ubidots</p>");
    }
    h += F("</div>");

    // Deep sleep
    if(deepSleepEnabled){
      h += F("<div class='card'><h2>Deep Sleep</h2><p><strong>Interval:</strong> ");
      h += String(deepSleepMinutes);
      h += F(" min</p><p><strong>Wake mode:</strong> ");
      h += (wakeDisplayMode == 0)
        ? F("<span style='background:#37474f;color:#cfd8dc;padding:2px 8px;border-radius:4px'>&#x1F50D; Stealth</span>")
        : F("<span style='background:#1565c0;color:#fff;padding:2px 8px;border-radius:4px'>&#x1F4E1; Active</span>");
      h += F("</p><p style='color:#9e9e9e;font-size:13px'>"
        "Single click = 10 min awake. Hold 3s = sleep now.</p></div>");
    }

    // Battery
    h += F("<div class='card'><h2>Battery</h2><p><strong>Power:</strong> ");
    if(isUSBPowered)
      h += F("<span style='background:#388e3c;color:#fff;padding:2px 8px;border-radius:4px'>USB / Charging</span>");
    else
      h += F("<span style='background:#1565c0;color:#fff;padding:2px 8px;border-radius:4px'>Battery</span>");
    h += F("</p><p><strong>Voltage:</strong> ");
    h += String(batteryVoltFloat/1000.0f,3);
    h += F("V &nbsp;<a href='/calibrate' style='color:#64B5F6;font-size:12px'>[calibrate]</a></p>"
      "<p><strong>Charge:</strong> ");
    h += String(batteryPercentage);
    h += F(" %</p></div>");

    // ntfy
    h += F("<div class='card'><h2>ntfy</h2><p><strong>Status:</strong> ");
    h += ntfy_enabled?F("Enabled"):F("Disabled");
    if(ntfy_enabled){
      h += F("</p><p><strong>Server:</strong> ");
      h += ntfy_server;
      h += F("</p><p><strong>Topic:</strong> ");
      h += ntfy_topic;
      // Verbosity summary
      h += F("</p><p><strong>Alerts:</strong> ");
      String flags="";
      if(ntfy_on_batt)   flags+="Battery ";
      if(ntfy_on_boot)   flags+="Boot ";
      if(ntfy_on_sleep)  flags+="Sleep ";
      if(ntfy_on_publish)flags+="Publish ";
      if(!flags.length())flags="None";
      h += flags;
      if(ntfy_last_millis>0){
        String sm=""; for(char ch:ntfy_last_msg){if((unsigned char)ch<128)sm+=ch;}
        h += F("</p><p><strong>Last sent:</strong> ");
        h += ntfy_last_time+" - "+sm;
      }
    }
    h += F("</p></div>");

    // Power save
    h += F("<div class='card'><h2>&#x26A1; Power Save</h2><p><strong>CPU wake:</strong> ");
    h += String(ps_cpu_wake_mhz);
    h += F(" MHz (current: ");
    h += String(getCpuFrequencyMhz());
    h += F(" MHz)</p><p><strong>Pre-sleep shutdown:</strong> ");
    String pa="";
    if(ps_wifi)pa+=F("WiFi ");if(ps_bt)pa+=F("BT ");if(ps_vext)pa+=F("Vext ");
    if(ps_oled)pa+=F("OLED ");if(ps_dht)pa+=F("DHT ");if(ps_cpu)pa+=F("CPU ");
    if(!pa.length())pa=F("None");
    h += pa;
    h += F("</p></div>");

    // Buttons row (OTA button only shown when no update banner is at top)
    h += F("<div style='margin-top:24px;text-align:center'>");
    if (!otaUpdateAvailable || otaDismissed) {
      h += F("<a href='/update' class='btn ota'>&#x1F680; OTA</a>");
    }
    h += F("<a href='/settings' class='btn'>&#x2699;&#xFE0F; Settings</a>"
      "<a href='/wifi' class='btn'>&#x1F4E1; WiFi</a>"
      "<a href='/reset' class='btn danger'>&#x1F504; Reset</a>"
      "</div>");
    sendPage(h + pageFoot());
  });

  // ── OTA upload page ────────────────────────────────────────────────────────
  server.on("/update", HTTP_GET, []() {
    String h = OTA_HTML;
    h.replace("%DEVICE_NAME%", device_name);
    h.replace("%FW_VERSION%",  FW_VERSION);
    h.replace("%RUNNING_CRC32%", runningCrc32.length() ? runningCrc32 : "—");

    // Inject remote firmware info block — shown when manifest check has result
    String remoteBlock = "";
    if (otaUpdateAvailable && !otaDismissed) {
      remoteBlock =
        "<div style='background:#1a3a3a;border:1px solid #4db6ac;border-radius:8px;"
        "padding:12px 14px;margin:14px 0'>"
        "<div style='color:#4db6ac;font-weight:700;font-size:14px'>&#x1F310; "
        "GitHub release: v" + otaNewVersion + "</div>";
      if (otaBuildDate.length())
        remoteBlock += "<div style='color:#80cbc4;font-size:12px'>Released: "
                       + otaBuildDate + "</div>";
      if (otaChangelog.length())
        remoteBlock += "<div style='color:#b2dfdb;font-size:12px;margin-top:4px'>"
                       + otaChangelog + "</div>";
      remoteBlock +=
        "<div style='margin-top:8px;padding:6px 10px;background:#0d2626;"
        "border-radius:4px;font-family:monospace;font-size:12px'>"
        "<span style='color:#546e7a'>Manifest CRC32: </span>"
        "<span style='color:#4db6ac'>" + (otaCrc32Expected.length()
          ? otaCrc32Expected : "not provided") + "</span>"
        "<span style='color:#546e7a;margin-left:12px;font-size:11px'>"
        "&#x2191; compare this with the file CRC32 computed below</span></div>"
        "</div>";
    } else if (!otaCheckDone) {
      remoteBlock =
        "<div style='background:#1f1f1f;border:1px solid #333;border-radius:8px;"
        "padding:10px 14px;margin:14px 0;font-size:13px;color:#9e9e9e'>"
        "&#x1F50D; GitHub check not yet run this session. "
        "<a href='/ota_check' style='color:#64B5F6'>Check now</a></div>";
    } else {
      remoteBlock =
        "<div style='background:#1b3a1b;border:1px solid #2e7d32;border-radius:8px;"
        "padding:10px 14px;margin:14px 0;font-size:13px;color:#81c784'>"
        "&#x2705; v" + String(FW_VERSION) + " is the latest version"
        + (otaCrc32Expected.length()
            ? "<div style='margin-top:6px;font-size:11px;color:#546e7a'>"
              "Running CRC32: <span style='color:#4db6ac'>"
              + (runningCrc32.length() ? runningCrc32 : "—") +
              "</span>&nbsp;&nbsp;|&nbsp;&nbsp;"
              "GitHub CRC32: <span style='color:"
              + (runningCrc32.length() && runningCrc32 == otaCrc32Expected
                  ? "#4db6ac'>&#x2714; " : "#ff8a65'>&#x26A0; ")
              + otaCrc32Expected + "</span></div>"
            : "")
        + "</div>";
    }
    h.replace("%REMOTE_INFO%", remoteBlock);
    server.send(200, F("text/html"), h);
  });
  server.on("/update", HTTP_POST,
    // Completion handler — fires after all file chunks have been written
    []() {
      esp_task_wdt_reset();
      bool ok = !Update.hasError() && !uploadRejected;
      server.sendHeader(F("Connection"), F("close"));
      String response = ok ? String("OK")
        : String("FAIL: ") + (uploadRejected ? uploadRejectReason : String("flash write error"));
      server.send(200, F("text/plain"), response);

      // Restore normal WDT timeout (was boosted to 120s during upload)
      esp_task_wdt_config_t normalCfg = {
        .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true
      };
      esp_task_wdt_reconfigure(&normalCfg);
      esp_task_wdt_reset();

      // ── OLED confirmation — stays on screen 12s then device reboots ────────
      if (!stealthThisWake) {
        display.displayOn();
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 0,  ok ? "FIRMWARE UPDATED" : "UPDATE FAILED");
        display.drawLine(0, 12, 128, 12);
        if (ok) {
          display.setFont(ArialMT_Plain_16);
          display.drawString(64, 16, "v" + String(FW_VERSION));
          display.setFont(ArialMT_Plain_10);
          display.drawString(64, 36, "Rebooting now...");
          // Animated countdown 10..1 on last line
          for (int i = 10; i >= 1; i--) {
            display.clear();
            display.setFont(ArialMT_Plain_10);
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.drawString(64, 0,  "FIRMWARE UPDATED");
            display.drawLine(0, 12, 128, 12);
            display.setFont(ArialMT_Plain_16);
            display.drawString(64, 16, "v" + String(FW_VERSION));
            display.setFont(ArialMT_Plain_10);
            display.drawString(64, 36, "Successfully installed");
            // Progress bar based on countdown
            int bw = (i * 122) / 10;
            display.drawRect(0, 50, 124, 8);
            display.fillRect(2, 52, bw, 4);
            display.drawString(64, 54, String(i) + "s");
            display.display();
            delay(1000);
          }
        } else {
          display.setFont(ArialMT_Plain_10);
          String reason = uploadRejected ? uploadRejectReason : "Flash write error";
          // Wrap reason at 21 chars per line for the 128px OLED
          display.drawString(64, 16, reason.length() > 21 ? reason.substring(0,21) : reason);
          if (reason.length() > 21)
            display.drawString(64, 28, reason.substring(21, min((int)reason.length(), 42)));
          display.drawString(64, 44, "Update rejected");
          display.display();
          delay(5000);
        }
      } else {
        delay(1000);
      }
      // Save previous version to NVS so the post-reboot boot splash knows
      // what to show. Flag is cleared by showOtaBootSplash() in setup().
      if (ok) {
        preferences.begin("ota", false);
        preferences.putBool  ("updated",  true);
        preferences.putString("prev_ver", FW_VERSION);
        preferences.end();
        // Clear update-available flag — it's being applied right now
        rtcOtaAvailable    = false;
        otaUpdateAvailable = false;
      }
      ESP.restart();
    },
    // Upload handler — called for each HTTP chunk during the file transfer
    []() {
      HTTPUpload &u = server.upload();
      static uint32_t uploadCrcState     = 0;
      static uint32_t uploadExpectedSize  = 0;


      if (u.status == UPLOAD_FILE_START) {
        // Reset all per-upload state
        uploadCrcState    = 0xFFFFFFFF;
        uploadRejected    = false;
        uploadRejectReason= "";
        uploadMarkerFound = false;
        uploadBytesScanned= 0;
        Serial.printf("[OTA] Start: %s\n", u.filename.c_str());

        // ── Boost WDT to 120s for the duration of the upload ────────────────
        esp_task_wdt_config_t otaCfg = {
          .timeout_ms     = 120000,
          .idle_core_mask = 0,
          .trigger_panic  = true
        };
        esp_task_wdt_reconfigure(&otaCfg);
        esp_task_wdt_reset();

        // Capture Content-Length so WRITE handler can show a % progress bar.
        uploadExpectedSize = (uint32_t)server.header("Content-Length").toInt();

        // ── Filename validation ──────────────────────────────────────────────
        String fn = u.filename; fn.toLowerCase();
        if (!u.filename.endsWith(".bin")) {
          Serial.println(F("[OTA] Rejected: filename must end in .bin"));
          uploadRejected     = true;
          uploadRejectReason = "Filename must end in .bin";
        } else if (fn.indexOf(".merged.") >= 0) {
          Serial.println(F("[OTA] Rejected: merged binary not allowed"));
          uploadRejected     = true;
          uploadRejectReason = "Merged binary (.merged.) not allowed — use .ino.bin";
        }

        // ── Partition size guard ─────────────────────────────────────────────
        // Pass the known file size to Update.begin() so the framework rejects
        // oversized binaries before a single byte is written to flash.
        // Falls back to UPDATE_SIZE_UNKNOWN only if Content-Length was absent.
        uint32_t beginSize = (uploadExpectedSize > 0 && uploadExpectedSize < 0xFFFFFF00U)
                           ? uploadExpectedSize : UPDATE_SIZE_UNKNOWN;
        if (!uploadRejected) {
          if (!Update.begin(beginSize)) {
            Serial.print(F("[OTA] Update.begin failed: "));
            Update.printError(Serial);
            uploadRejected     = true;
            uploadRejectReason = beginSize != UPDATE_SIZE_UNKNOWN
              ? "Binary too large for OTA partition"
              : "Update.begin() failed — check flash layout";
          }
        }
        if (!stealthThisWake) {
          display.displayOn();
          display.clear();
          display.setFont(ArialMT_Plain_10);
          display.setTextAlignment(TEXT_ALIGN_CENTER);
          display.drawString(64, 0,  "OTA UPLOADING");
          display.drawLine(0, 12, 128, 12);
          display.drawString(64, 24, u.filename.length() > 20
            ? u.filename.substring(0, 17) + "..."
            : u.filename);
          display.drawString(64, 40, "Do not power off");
          display.display();
        }

      } else if (u.status == UPLOAD_FILE_WRITE) {
        // ── Feed WDT unconditionally before every flash write ────────────────
        esp_task_wdt_reset();

        // ── Early-reject: previous check failed, abort and drain ─────────────
        if (uploadRejected) {
          Update.abort();
          return;  // drain remaining HTTP chunks without writing
        }

        // ── FW_VER marker scan (first 64 KB only) ────────────────────────────
        // FW_VER_MARKER ("FW_VER:5.29") lives in .rodata, always within first
        // 64 KB of the binary.  If we exhaust 64 KB without finding it the
        // binary is either corrupt or from a completely different project.
        if (!uploadMarkerFound && uploadBytesScanned < 65536) {
          const uint8_t* b  = u.buf;
          const size_t   sz = u.currentSize;
          const char*    mk = "FW_VER:";
          const size_t   ml = 7;
          for (size_t i = 0; i + ml <= sz && !uploadMarkerFound; i++) {
            bool hit = true;
            for (size_t j = 0; j < ml && hit; j++)
              hit = (b[i+j] == (uint8_t)mk[j]);
            if (hit) uploadMarkerFound = true;
          }
          uploadBytesScanned += u.currentSize;
          if (!uploadMarkerFound && uploadBytesScanned >= 65536) {
            Serial.println(F("[OTA] Rejected: FW_VER marker not found in first 64 KB"));
            Update.abort();
            uploadRejected     = true;
            uploadRejectReason = "Not a valid firmware for this device (FW_VER marker missing)";
            return;
          }
        }

        if (Update.write(u.buf, u.currentSize) != u.currentSize) {
          Update.printError(Serial);
        }
        uploadCrcState = crc32Feed(uploadCrcState, u.buf, u.currentSize);

        static uint32_t lastOledUpdate = 0;
        if (!stealthThisWake && millis() - lastOledUpdate > 400) {
          lastOledUpdate = millis();
          display.clear();
          display.setFont(ArialMT_Plain_10);
          display.setTextAlignment(TEXT_ALIGN_CENTER);
          display.drawString(64, 0,  "OTA UPLOADING");
          display.drawLine(0, 12, 128, 12);
          display.drawString(64, 16, String(u.totalSize / 1024) + " / " +
            (uploadExpectedSize > 0 ? String(uploadExpectedSize / 1024) : "?") + " KB");
          if (uploadExpectedSize > 0) {
            int pct = (int)((u.totalSize * 100UL) / uploadExpectedSize);
            pct = min(pct, 100);
            display.drawRect(0, 30, 124, 8);
            display.fillRect(2, 32, (pct * 120) / 100, 4);
            display.drawString(64, 42, String(pct) + "%  Flashing...");
          } else {
            static uint8_t dotIdx = 0; dotIdx = (dotIdx + 1) % 4;
            display.drawString(64, 36, "Flashing" + String(dotIdx, '.'));
          }
          display.drawString(64, 54, "Do not power off");
          display.display();
        }

      } else if (u.status == UPLOAD_FILE_END) {
        esp_task_wdt_reset();
        uint32_t finalCrc = uploadCrcState ^ 0xFFFFFFFF;
        char crcHex[9]; snprintf(crcHex, sizeof(crcHex), "%08X", finalCrc);
        if (Update.end(true)) {
          Serial.printf("[OTA] Success: %u bytes  CRC32: %s\n", u.totalSize, crcHex);
        } else {
          Update.printError(Serial);
        }
        // ── Restore normal WDT timeout now that flash write is done ─────────
        esp_task_wdt_config_t normalCfg = {
          .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
          .idle_core_mask = 0,
          .trigger_panic  = true
        };
        esp_task_wdt_reconfigure(&normalCfg);
        esp_task_wdt_reset();
      }
    }
  );

  // ── Settings page ──────────────────────────────────────────────────────────
  server.on("/settings", HTTP_GET, []() {
    String h = pageHead(F("Settings"));
    h += F("<h1>&#x2699;&#xFE0F; Configuration</h1>"
      "<form method='POST' action='/save_settings'>"
      "<div class='sec'><h2>Device</h2>"
      "<label>Name</label><input name='device_name' value='");
    h += device_name;
    h += F("' required>"
      "<label>Platform</label><select name='mqtt_platform'>");
    const char* plats[]={"standard","adafruit","ubidots","all"};
    const char* platL[]={"Standard","Adafruit IO","Ubidots","ALL"};
    for(int i=0;i<4;i++){
      h += F("<option value='");
      h += plats[i];
      h += F("'");
      if(mqtt_platform==plats[i])h += F(" selected");
      h += F(">");
      h += platL[i];
      h += F("</option>");
    }
    h += F("</select></div>"
      "<div class='sec'><h2>Publish Interval</h2>"
      "<label>Publish every (seconds)</label>"
      "<input type='number' name='pub_timer' value='");
    h += String(publishSeconds);
    h += F("' min='30' max='3600'></div>"
      "<div class='sec'><h2>Standard MQTT</h2>"
      "<div class='info'>Local broker e.g. Mosquitto</div>"
      "<label>Server</label><input name='mqtt_server' value='");
    h += mqtt_server;
    h += F("' placeholder='192.168.1.x'>"
      "<label>Port</label><input type='number' name='mqtt_port' value='");
    h += mqtt_port;
    h += F("'><label>User</label><input name='mqtt_user' value='");
    h += mqtt_user;
    h += F("'><label>Password</label><input type='password' name='mqtt_pass' value='");
    h += mqtt_pass;
    h += F("'><label>Topic</label><input name='mqtt_topic' value='");
    h += mqtt_topic;
    h += F("'></div>"
      "<div class='sec'><h2>Adafruit IO</h2>"
      "<label>Username</label><input name='aio_user' value='");
    h += aio_username;
    h += F("'><label>Key</label><input type='password' name='aio_key' value='");
    h += aio_key;
    h += F("'></div>"
      "<div class='sec'><h2>Ubidots</h2>"
      "<label>Token</label><input type='password' name='ubi_token' value='");
    h += ubidots_token;
    h += F("'><label>Device Label</label><input name='ubi_device' value='");
    h += ubidots_device;
    h += F("'></div>"
      "<div class='sec'><h2>Deep Sleep</h2>"
      "<div class='info'>Board wakes&rarr;reads&rarr;publishes&rarr;sleeps.<br>"
      "Single click=10min awake. Hold 3s=sleep now.</div>"
      "<label style='display:flex;align-items:center;gap:10px;cursor:pointer'>"
      "<input type='checkbox' name='deep_enable' style='width:auto'");
    if(deepSleepEnabled)h += F(" checked");
    h += F("> Enable Deep Sleep</label>"
      "<label>Wake interval (minutes)</label>"
      "<input type='number' name='deep_min' value='");
    h += String(deepSleepMinutes);
    h += F("' min='1' max='1440'>"
      "<div style='margin-top:16px;border-top:1px solid #424242;padding-top:12px'>"
      "<div style='font-weight:600;color:#9e9e9e;font-size:13px;text-transform:uppercase;"
      "letter-spacing:1px;margin-bottom:8px'>Wake Display Mode</div>"
      "<div class='info' style='margin-bottom:10px'>"
      "Controls OLED &amp; LED during the wake/read/publish window.<br>"
      "<em>Button wakes always use Active regardless of this setting.</em>"
      "</div>"
      "<label class='cb'><input type='radio' name='wake_disp' value='1'");
    if (wakeDisplayMode == 1) h += F(" checked");
    h += F("> <strong>&#x1F4E1; Active</strong>"
      " <span style='color:#9e9e9e;font-size:12px'>"
      "OLED on &amp; LED breathing &mdash; full feedback while awake (default)</span></label>"
      "<label class='cb'><input type='radio' name='wake_disp' value='0'");
    if (wakeDisplayMode == 0) h += F(" checked");
    h += F("> <strong>&#x1F50D; Stealth</strong>"
      " <span style='color:#9e9e9e;font-size:12px'>"
      "OLED &amp; LED off &mdash; silent reads, maximum battery life</span></label>"
      "</div>"   // close wake-mode inner div
      "</div>");  // close Deep Sleep sec div
    h += F("<div class='sec'><h2>ntfy Notifications</h2>"
      "<div class='info'>Push alerts via <a href='https://ntfy.sh' target='_blank' "
      "style='color:#64B5F6'>ntfy.sh</a> or self-hosted.<br>"
      "Battery alerts fire independently &mdash; they don't require "
      "the publish setting to be on.</div>"
      "<label class='cb'><input type='checkbox' name='ntfy_enabled' style='width:auto'");
    if(ntfy_enabled)h += F(" checked");
    h += F("><strong>Enable ntfy</strong> (master on/off)</label>"
      "<div style='margin:10px 0 4px;font-size:12px;color:#9e9e9e;"
      "text-transform:uppercase;letter-spacing:1px'>What to send:</div>"
      "<label class='cb'><input type='checkbox' name='ntfy_on_batt' style='width:auto'");
    if(ntfy_on_batt)h += F(" checked");
    h += F("> Battery low / critical"
      " <span style='color:#ef9a9a;font-size:12px'>(recommended ON)</span></label>"
      "<label class='cb'><input type='checkbox' name='ntfy_on_boot' style='width:auto'");
    if(ntfy_on_boot)h += F(" checked");
    h += F("> Boot &amp; wake summary"
      " <span style='color:#9e9e9e;font-size:12px'>(mirrors MQTT boot payload)</span></label>"
      "<label class='cb'><input type='checkbox' name='ntfy_on_sleep' style='width:auto'");
    if(ntfy_on_sleep)h += F(" checked");
    h += F("> Going-to-sleep notification"
      " <span style='color:#9e9e9e;font-size:12px'>(one message per sleep cycle)</span></label>"
      "<label class='cb'><input type='checkbox' name='ntfy_on_publish' style='width:auto'");
    if(ntfy_on_publish)h += F(" checked");
    h += F("> Every periodic publish"
      " <span style='color:#9e9e9e;font-size:12px'>(verbose)</span></label>"
      "<label>Server</label><input name='ntfy_server' value='");
    h += ntfy_server;
    h += F("' placeholder='ntfy.sh'>"
      "<label>Topic</label><input name='ntfy_topic' value='");
    h += ntfy_topic;
    h += F("' placeholder='my-sensor-alerts'>"
      "<label>Bearer Token (optional)</label>"
      "<input type='password' name='ntfy_token' value='");
    h += ntfy_token;
    h += F("' placeholder='leave blank for public topics'></div>"
      "<div class='sec'><h2>&#x1F321;&#xFE0F; Sensor Alerts</h2>"
      "<div class='info'>Send ntfy alerts when temperature or humidity crosses these limits. Requires ntfy to be enabled above. "
      "1&#176;C / 2% hysteresis prevents spam.</div>"
      "<label class='cb'><input type='checkbox' name='ntfy_on_sensor' style='width:auto'");
    if(ntfy_on_sensor)h+=F(" checked");
    h+=F("> <strong>Enable sensor alerts</strong></label>"
      "<div class='row' style='gap:12px;flex-wrap:wrap;margin-top:8px'>"
      "<div style='flex:1;min-width:120px'>"
      "<label style='color:#ff9800'>&#x1F321; Temp High (&deg;C)</label>"
      "<input type='number' name='t_hi' step='0.5' value='");
    h+=String(sensorTempHi,1);
    h+=F("'>"
      "</div><div style='flex:1;min-width:120px'>"
      "<label style='color:#42a5f5'>&#x1F321; Temp Low (&deg;C)</label>"
      "<input type='number' name='t_lo' step='0.5' value='");
    h+=String(sensorTempLo,1);
    h+=F("'>"
      "</div><div style='flex:1;min-width:120px'>"
      "<label style='color:#ff9800'>&#x1F4A7; Hum High (%)</label>"
      "<input type='number' name='h_hi' step='1' min='0' max='100' value='");
    h+=String(sensorHumHi,1);
    h+=F("'>"
      "</div><div style='flex:1;min-width:120px'>"
      "<label style='color:#42a5f5'>&#x1F4A7; Hum Low (%)</label>"
      "<input type='number' name='h_lo' step='1' min='0' max='100' value='");
    h+=String(sensorHumLo,1);
    h+=F("'>"
      "</div></div></div>"
      "<div class='sec'><h2>&#x26A1; Power Save</h2>"
      "<div class='info'>Shutdown hardware before deep sleep.</div>");
    // Power save checkboxes
    struct { const char* n; const char* lbl; bool val; } ps[]={
      {"ps_wifi","WiFi OFF (~20-80mA)",ps_wifi},{"ps_bt","BT OFF (~10mA)",ps_bt},
      {"ps_oled","OLED displayOff",ps_oled},{"ps_vext","Vext rail OFF (~4mA)",ps_vext},
      {"ps_dht","Float DHT pin",ps_dht},{"ps_cpu","CPU 10MHz pre-sleep",ps_cpu}
    };
    for(auto& p:ps){
      h += F("<label class='cb'><input type='checkbox' name='");
      h += p.n;
      h += F("'");
      if(p.val)h += F(" checked");
      h += F("> ");
      h += p.lbl;
      h += F("</label>");
    }
    h += F("<label>Wake CPU MHz</label><select name='ps_cpu_wake_mhz'>");
    int mhzOpts[]={80,160,240};
    for(int m:mhzOpts){
      h += F("<option value='");
      h += String(m);
      if(ps_cpu_wake_mhz==(uint32_t)m)h += F("' selected>");
      else h += F("'>");
      h += String(m);
      h += F("MHz</option>");
    }
    h += F("</select></div>");

    // ── Display Care ──────────────────────────────────────────────────────────
    h += F("<div class='sec'><h2>&#x1F4FA; Display Care</h2>"
      "<div class='info'>Exercises all pixels to counteract OLED burn-in. "
      "Save settings first to apply your pattern/duration choice, "
      "then press Run Now.</div>"
      "<label>Pattern</label>"
      "<select name='scrn_preset'><option value='0'");
    if (screenCleanPreset==0) h+=F(" selected");
    h+=F(">Checkerboard Shift (recommended)</option>"
         "<option value='1'");
    if (screenCleanPreset==1) h+=F(" selected");
    h+=F(">Invert Ramp (sweep + invert)</option>"
         "<option value='2'");
    if (screenCleanPreset==2) h+=F(" selected");
    h+=F(">Scanline Sweep (rolling line)</option>"
         "<option value='3'");
    if (screenCleanPreset==3) h+=F(" selected");
    h+=F(">Full Bright Pulse (all pixels + 1s flash)</option></select>"
      "<label>Duration (seconds)</label>"
      "<div class='row'><input type='number' name='scrn_dur' value='");
    h+=String(screenCleanDuration);
    h+=F("' min='10' max='600'></div>");

    // Run Now button — now INSIDE the Display Care panel
    h += F("<div style='margin-top:12px'>");
    h += F("<a href='/screen_clean' class='btn'>&#x25B6; Run Now</a>");
    if (screenCleanActive) {
      int sl = max(0L,(long)(screenCleanUntil-millis())/1000L);
      h += F("<a href='/screen_cancel' class='btn' style='background:#c62828;margin-left:8px'>"
            "&#x23F9; Cancel (");
      h += String(sl);
      h += F("s)</a>");
    }
    h += F("</div></div>");  // close Display Care sec

    // ── OLED Screensaver section ─────────────────────────────────────────────
    h += F("<div class='sec'><h2>&#x2728; OLED Screensaver</h2>"
      "<div class='info'>Animated overlay after idle. Button press dismisses it.</div>"
      "<label class='cb'><input type='checkbox' name='ss_en'");
    if (ssaverEnabled) h += F(" checked");
    h += F("> <strong>Enable screensaver</strong></label>"

    "<label>Idle timeout</label>"
    "<select name='ss_timeout'>");
    struct { uint32_t v; const char* lbl; } ssTO[] = {
      {30,"30 seconds"},{60,"1 minute"},{300,"5 minutes"},
      {600,"10 minutes"},{1800,"30 minutes"},{3600,"1 hour"}
    };
    for (auto& t : ssTO) {
      h += F("<option value='"); h += String(t.v);
      if (ssaverTimeoutSecs == t.v) h += F("' selected>"); else h += F("'>");
      h += t.lbl; h += F("</option>");
    }
    h += F("</select>"

    "<label>Animation preset</label>"
    "<select name='ss_preset'>"
    "<option value='0'"); if (ssaverPreset==0) h+=F(" selected"); h+=F(">&#x26AA; Bouncing Ball</option>"
    "<option value='1'"); if (ssaverPreset==1) h+=F(" selected"); h+=F(">&#x1F344; Mario Bounce</option>"
    "<option value='2'"); if (ssaverPreset==2) h+=F(" selected"); h+=F(">&#x1F7E9; Matrix Rain</option>"
    "<option value='3'"); if (ssaverPreset==3) h+=F(" selected"); h+=F(">&#x1F4BF; DVD Bounce</option>"    
    "</select>"

    "<div style='margin:12px 0 8px 0'>"
    "<button type='button' class='btn' style='font-size:13px;background:#4db6ac'"
    " onclick=\"window.location='/ssaver_preview?p='+document.querySelector('[name=ss_preset]').value\">"
    "&#x25B6; Preview Selected Preset</button>"
    "<span style='display:block;margin-top:6px;font-size:11px;color:#9e9e9e'>"
    "Runs the chosen preset for 10 s &mdash; no save needed. "
    "Press device button to dismiss early.</span>"
    "</div></div>");
    h += F("<button type='submit' class='btn'>&#x1F4BE; Save</button>"
      "<a href='/' class='btn back'>&#x2190; Back</a>"
      "<a href='/reset_bootcount'"
      " onclick=\"return confirm('Reset boot counter to 1?')\""
      " class='btn' style='background:#546E7A;margin:6px'>"
      "&#x1F522; Reset Boot#</a>"
      "</form>");
    sendPage(h + pageFoot());
  });

  // ── Save settings ──────────────────────────────────────────────────────────
  server.on("/save_settings", HTTP_POST, []() {
    if(server.hasArg("device_name"))   device_name   =server.arg("device_name");
    if(server.hasArg("mqtt_platform")) mqtt_platform =server.arg("mqtt_platform");
    if(server.hasArg("mqtt_server"))   mqtt_server   =server.arg("mqtt_server");
    if(server.hasArg("mqtt_port"))     mqtt_port     =server.arg("mqtt_port");
    if(server.hasArg("mqtt_user"))     mqtt_user     =server.arg("mqtt_user");
    if(server.hasArg("mqtt_pass"))     mqtt_pass     =server.arg("mqtt_pass");
    if(server.hasArg("mqtt_topic"))    mqtt_topic    =server.arg("mqtt_topic");
    if(server.hasArg("aio_user"))      aio_username  =server.arg("aio_user");
    if(server.hasArg("aio_key"))       aio_key       =server.arg("aio_key");
    if(server.hasArg("ubi_token"))     ubidots_token =server.arg("ubi_token");
    if(server.hasArg("ubi_device"))    ubidots_device=server.arg("ubi_device");
    if(server.hasArg("pub_timer")){
      publishSeconds=constrain((unsigned long)server.arg("pub_timer").toInt(),30UL,3600UL);
      readInterval=publishSeconds*1000UL;
    }
    deepSleepEnabled=server.hasArg("deep_enable");
    if(server.hasArg("deep_min"))deepSleepMinutes=(uint32_t)max(1L,(long)server.arg("deep_min").toInt());
    deepSleepSeconds=deepSleepMinutes*60UL;
    if(server.hasArg("wake_disp")){uint8_t wd=(uint8_t)server.arg("wake_disp").toInt();wakeDisplayMode=(wd<=1)?wd:1;}
    if(server.hasArg("scrn_preset")){uint8_t sp=(uint8_t)server.arg("scrn_preset").toInt();if(sp<=3)screenCleanPreset=sp;}
    if(server.hasArg("scrn_dur")){int sd=server.arg("scrn_dur").toInt();if(sd>=10&&sd<=600)screenCleanDuration=(uint16_t)sd;}
    ssaverEnabled = server.hasArg("ss_en");
    if (server.hasArg("ss_timeout")) {
      uint32_t st = (uint32_t)server.arg("ss_timeout").toInt();
      if (st >= 10) ssaverTimeoutSecs = st;
    }
    if (server.hasArg("ss_preset")) {
      uint8_t sp = (uint8_t)server.arg("ss_preset").toInt();
      if (sp <= 3) ssaverPreset = sp;
    }
    // Force reload to make sure variables are in sync
    saveOledConfig();
    loadOledConfig();   // <--- Important: reload immediately

    ntfy_enabled   =server.hasArg("ntfy_enabled");
    ntfy_on_batt   =server.hasArg("ntfy_on_batt");
    ntfy_on_boot   =server.hasArg("ntfy_on_boot");
    ntfy_on_sleep  =server.hasArg("ntfy_on_sleep");
    ntfy_on_publish=server.hasArg("ntfy_on_publish");
    ntfy_on_sensor =server.hasArg("ntfy_on_sensor");
    if(server.hasArg("t_hi"))sensorTempHi=server.arg("t_hi").toFloat();
    if(server.hasArg("t_lo"))sensorTempLo=server.arg("t_lo").toFloat();
    if(server.hasArg("h_hi"))sensorHumHi =server.arg("h_hi").toFloat();
    if(server.hasArg("h_lo"))sensorHumLo =server.arg("h_lo").toFloat();
    // Reset sent-flags so new thresholds take effect immediately
    tempHiSent=tempLoSent=humHiSent=humLoSent=false;
    if(server.hasArg("ntfy_server")&&server.arg("ntfy_server").length())ntfy_server=server.arg("ntfy_server");
    if(server.hasArg("ntfy_topic")) ntfy_topic=server.arg("ntfy_topic");
    if(server.hasArg("ntfy_token")) ntfy_token=server.arg("ntfy_token");
    ps_wifi=server.hasArg("ps_wifi"); ps_bt  =server.hasArg("ps_bt");
    ps_vext=server.hasArg("ps_vext"); ps_oled=server.hasArg("ps_oled");
    ps_dht =server.hasArg("ps_dht");  ps_cpu =server.hasArg("ps_cpu");
    if(server.hasArg("ps_cpu_wake_mhz")){
      uint32_t mhz=(uint32_t)server.arg("ps_cpu_wake_mhz").toInt();
      if(mhz==80||mhz==160||mhz==240){ps_cpu_wake_mhz=mhz;setCpuFrequencyMhz(mhz);}
    }
    saveMqttConfig(); saveDeepConfig(); saveNtfyConfig(); savePowerSaveConfig(); saveOledConfig();
    mqttStd.disconnect(); mqttAIO.disconnect(); mqttUBI.disconnect();
    mqttStandardConnected=mqttAdafruitConnected=mqttUbidotsConnected=false;
    stdRetries=aioRetries=ubiRetries=0;
    server.send(200,F("text/html"),actionPage("&#x2705;","Settings Saved","Reconnecting to MQTT...",4));
  });

  // ── WiFi reset page ────────────────────────────────────────────────────────
  server.on("/wifi", HTTP_GET, []() {
    String h = pageHead(F("WiFi"));
    h += F("<h1>&#x1F4E1; WiFi</h1>"
      "<div class='card'><p>Current: <strong>");
    h += WiFi.SSID();
    h += F("</strong> (");
    h += String(WiFi.RSSI());
    h += F(" dBm)</p></div>"
      "<div style='text-align:center;margin-top:20px'>"
      "<a href='/reset_wifi' class='btn danger'>Reset WiFi</a>"
      "<a href='/' class='btn' style='background:#757575'>Cancel</a>"
      "</div>");
    sendPage(h + pageFoot());
  });
  server.on("/reset_wifi", HTTP_GET, []() {
    server.send(200,F("text/html"),actionPage("&#x1F4E1;","Resetting WiFi","Connect to ESP32-Setup AP then browse to 192.168.4.1",5));
    delay(1000); WiFiManager wm; wm.resetSettings(); ESP.restart();
  });
  server.on("/reset", HTTP_GET, []() {
    server.send(200,F("text/html"),actionPage("&#x1F504;","Restarting","Please wait...",3));
    delay(1000); ESP.restart();
  });
  server.on("/reset_bootcount", HTTP_GET, []() {
    // Reset both NVS base and RTC offset — counter starts from 1 on next power-on
    rtcBootOffset = 0;
    bootCount     = 1;
    preferences.begin("sys", false);
    preferences.putInt("bootcount", 1);
    preferences.end();
    Serial.println(F("[BOOT] Boot count reset to 1 via web UI"));
    server.send(200, F("text/html"),
      actionPage("&#x1F522;", "Boot Count Reset", "Counter restarted from 1", 3));
  });

  // /screen_clean — start pixel exercise
  server.on("/screen_clean", HTTP_GET, []() {
    disableDeepSleepUntil = millis() + ((unsigned long)screenCleanDuration + 15) * 1000UL;
    screenCleanUntil  = millis() + (unsigned long)screenCleanDuration * 1000UL;
    screenCleanToggle = 0;
    screenCleanActive = true;
    display.normalDisplay();  // ensure clean state before starting
    display.displayOn();
    const char* names[] = {"Checkerboard Shift","Invert Ramp","Scanline Sweep","Full Bright Pulse"};
    Serial.printf("[OLED] Pixel exercise: %s %ds\n",
      names[min((int)screenCleanPreset,3)], screenCleanDuration);
    server.send(200, F("text/html"),
      actionPage("&#x1F4FA;", "Pixel Exercise Running",
        String(names[min((int)screenCleanPreset,3)]) + " &mdash; " +
        String(screenCleanDuration) + "s. Restores automatically.<br><br>"
        "<a href='/screen_cancel' style='color:#ef5350;font-weight:600'>"
        "&#x23F9; Cancel now</a>",
        screenCleanDuration + 4));
  });

  // /screen_cancel — abort pixel exercise immediately
  server.on("/screen_cancel", HTTP_GET, []() {
    screenCleanActive = false;
    display.normalDisplay();
    display.clear();
    display.displayOn();
    Serial.println(F("[OLED] Pixel exercise cancelled"));
    server.send(200, F("text/html"),
      actionPage("&#x2714;", "Exercise Cancelled", "Display restored.", 3));
  });

  // /ssaver_preview — force start current or selected preset
  server.on("/ssaver_preview", HTTP_GET, []() {
    uint8_t p = ssaverPreset;  // default to saved
    if (server.hasArg("p")) {
      int pv = server.arg("p").toInt();
      if (pv >= 0 && pv <= 3) p = (uint8_t)pv;
    }

    ssaverSavedPreset = ssaverPreset;
    ssaverPreset      = p;
    ssaverActive      = true;
    ssaverPreview     = true;
    ssaverStartMs     = millis();
    ssaverFrameMs     = 0;
    mxInited          = false;
    
    ssResetBall();
    ssResetMario();

    display.displayOn();
    display.clear();
    disableDeepSleepUntil = millis() + 20000UL;

    const char* names[] = { "Bouncing Ball", "Mario Bounce", "Matrix Rain", "DVD Logo" };
    Serial.printf("[SAVER] Preview started: %s\n", names[p]);

    server.send(200, F("text/html"),
      actionPage("&#x2728;", "Screensaver Preview",
        String(names[p]) + " running for 10 seconds.<br>"
        "Press device button to dismiss early.",
        12, "/settings"));
  });

  // ── /api/status — lightweight JSON polled by web UI for live countdown ──────
  // READ-ONLY — does NOT reset webUiExpiresAt. Only button presses and page
  // loads reset the timer so the countdown is accurate.
  server.on("/api/status", HTTP_GET, []() {
    long secsLeft = webUiExpiresAt > millis() ? (long)((webUiExpiresAt - millis()) / 1000) : 0;
    String j = "{\"webui_secs_left\":" + String(secsLeft) + ","
               "\"deep_sleep_enabled\":" + (deepSleepEnabled ? "true" : "false") + ","
               "\"uptime\":" + String(millis() / 1000) + "}";
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, F("application/json"), j);
  });

  // ── /ota_check — trigger immediate manifest fetch, return JSON status ─────────
  server.on("/ota_check", HTTP_GET, []() {
  checkOtaManifest(true);  // force=true bypasses the interval guard
  
  // Build JSON — include build_date so OTA page manifest card is complete
  String j = "{\"current\":\"" + String(FW_VERSION) + "\","
             "\"available\":\"" + otaNewVersion + "\","
             "\"update_available\":" + (otaUpdateAvailable ? "true" : "false") + ","
             "\"changelog\":\"" + otaChangelog + "\","
             "\"build_date\":\"" + otaBuildDate + "\","
             "\"size\":" + String(otaFileSize) + ","
             "\"crc32\":\"" + otaCrc32Expected + "\","
             "\"running_crc32\":\"" + runningCrc32 + "\"}";
             
  server.send(200, F("application/json"), j);
  });

  // ── /ota_reinstall — re-flash same version from GitHub (CRC mismatch recovery)
  // Skips the otaUpdateAvailable version guard — only requires a valid download URL.
  server.on("/ota_reinstall", HTTP_GET, []() {
    if (otaDownloadUrl.length() == 0) {
      server.send(400, F("text/plain"), F("No download URL — run Check Updates first."));
      return;
    }
    String h = String(F("<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Re-flashing</title>"))
      + F("<meta http-equiv='refresh' content='60;url=/'>"
        "<style>body{background:#1a1a1a;color:#e0e0e0;font-family:sans-serif;"
        "display:flex;align-items:center;justify-content:center;min-height:100vh}"
        ".b{background:#2d2d2d;border-radius:12px;padding:36px;text-align:center;max-width:400px}"
        "h2{color:#ff8a65}p{color:#9e9e9e;margin:8px 0}</style></head>"
        "<body><div class='b'>"
        "<div style='font-size:48px'>&#x1F504;</div>"
        "<h2>Re-flashing v")
      + String(FW_VERSION)
      + F("</h2><p>CRC mismatch detected &mdash; downloading clean binary from GitHub.</p>"
        "<p>OLED shows live progress. Device will reboot when done.</p>"
        "<p style='color:#ef9a9a;font-size:12px'>Do not power off during install.</p>"
        "</div></body></html>");
    server.send(200, F("text/html"), h);
    delay(200);
    bool ok = installOtaFromUrl(otaDownloadUrl, otaCrc32Expected, otaFileSize);
    if (ok) {
      Serial.println(F("[OTA] Re-flash complete — rebooting"));
      delay(500); ESP.restart();
    } else {
      Serial.println(F("[OTA] Re-flash FAILED"));
    }
  });

  // ── /ota_dismiss — user dismissed the update banner ───────────────────────
  server.on("/ota_dismiss", HTTP_GET, []() {
    otaDismissed = true;
    server.sendHeader("Location", "/"); server.send(302, "text/plain", "");
  });

  // ── /ota_install — stream binary from GitHub and apply OTA ────────────────
  server.on("/ota_install", HTTP_GET, []() {
    if (!otaUpdateAvailable || otaDownloadUrl.length() == 0) {
      server.send(400, F("text/plain"), F("No update available. Run /ota_check first."));
      return;
    }
    // Send the response page immediately — the actual install blocks for up to 60 s
    String h = String(F("<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Installing Update</title>"))
      + F("<meta http-equiv='refresh' content='60;url=/'>"
        "<style>body{background:#1a1a1a;color:#e0e0e0;font-family:sans-serif;"
        "display:flex;align-items:center;justify-content:center;min-height:100vh}"
        ".b{background:#2d2d2d;border-radius:12px;padding:36px;text-align:center;max-width:400px}"
        "h2{color:#64B5F6}p{color:#9e9e9e;margin:8px 0}</style></head>"
        "<body><div class='b'>"
        "<div style='font-size:48px'>&#x1F4E5;</div>"
        "<h2>Installing v")
      + otaNewVersion
      + F("</h2><p>Downloading and flashing from GitHub...</p>"
        "<p>OLED shows live progress. Device will reboot when done.</p>"
        "<p style='color:#ef9a9a;font-size:12px'>Do not power off during install.</p>"
        "</div></body></html>");
    server.send(200, F("text/html"), h);

    delay(200);  // allow HTTP response to flush before blocking

    bool ok = installOtaFromUrl(otaDownloadUrl, otaCrc32Expected, otaFileSize);

    if (ok) {
      // Save prev_ver to NVS so boot splash can show what changed
      preferences.begin("ota", false);
      preferences.putBool  ("updated",  true);
      preferences.putString("prev_ver", FW_VERSION);
      preferences.end();
      Serial.println(F("[OTA] GitHub install OK — rebooting"));
    } else {
      Serial.println(F("[OTA] GitHub install FAILED"));
      // Show error on OLED a few seconds before returning to normal
      if (!stealthThisWake) {
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 20, "INSTALL FAILED");
        display.drawString(64, 36, "Check Serial log");
        display.display(); delay(4000);
        display.displayOff();
      }
      return;  // don't restart on failure
    }
    delay(500);
    ESP.restart();
  });

  // /calibrate
  server.on("/calibrate", HTTP_GET, []() {
    String rv = server.arg("v");

    // ── SAVE: real voltage supplied ─────────────────────────────────────────
    if (rv.length() > 0) {
      // Guard: battery must have been read at least once
      if (lastRawAvgMv < 1.0f) {
        String h = pageHead(F("Calibrate Battery"));
        h += F("<h1>&#x1F3AF; Battery Calibration</h1>"
          "<div class='card' style='border-left:4px solid #ef5350'>"
          "<h2 style='color:#ef5350'>&#x26A0; Reading not ready</h2>"
          "<p>The ADC has not completed a battery read yet. "
          "Wait a few seconds after boot, then try again.</p>"
          "<a href='/calibrate' style='color:#64B5F6'>&#x21BA; Retry</a>"
          "</div>");
        sendPage(h + pageFoot());
        return;
      }
      float realV = rv.toFloat();
      if (realV > 3.0f && realV < 5.5f) {
        // Decide which NVS key to write based on CURRENT detected source.
        // USB/floating → high-voltage (USB) path → "vdiv"
        // Battery only → battery-only path         → "vdiv_bat"
        bool calOnBattery = (!isUSBPowered && !isBatFloating);
        // Use raw ADC mV stored at last battery_read() — avoids race where
        // batteryVoltFloat was produced by a different factor if mode changed.
        float newFactor = realV / (lastRawAvgMv / 1000.0f);
        preferences.begin("battery", false);
        if (calOnBattery) {
          vdivFactorBat = newFactor;
          preferences.putFloat("vdiv_bat", vdivFactorBat);
          // Only reset learnedVFull in battery mode — USB cal must not corrupt it
          preferences.putFloat("vFull", 3.92f);
          learnedVFull = 3.92f;
          Serial.printf("[CAL] Battery factor: vdivFactorBat=%.4f\n", vdivFactorBat);
        } else {
          vdivFactor = newFactor;
          preferences.putFloat("vdiv", vdivFactor);
          // learnedVFull intentionally NOT reset here — USB cal must not corrupt it
          Serial.printf("[CAL] USB factor: vdivFactor=%.4f\n", vdivFactor);
        }
        preferences.end();
        String savedMode = calOnBattery ? "Battery-only factor" : "USB / charging factor";
        String nextMode  = calOnBattery ? "USB" : "battery-only";
        String nextInstr = calOnBattery
          ? "Plug in USB, revisit /calibrate."
          : "Unplug USB, run on battery only, revisit /calibrate.";
        // Redirect to /calibrate so the user lands on step 2 immediately
        server.send(200, F("text/html"),
          actionPage("&#x1F3AF;", "Calibration Saved",
            savedMode + ": <strong>" + String(newFactor, 4) + "</strong>"
            " &nbsp;&rarr;&nbsp; reads " + String(realV, 3) + "V"
            "<br><span style='font-size:13px;color:#9e9e9e'>Next: " + nextInstr +
            "</span><br><a href='/calibrate' style='font-size:13px'>"
            "&#x21B3; Calibrate " + nextMode + " now</a>",
            6, "/calibrate"));
        return;
      }
      // Voltage out of range — show error
      String h = pageHead(F("Calibrate Battery"));
      h += F("<h1>&#x1F3AF; Battery Calibration</h1>"
        "<div class='card' style='border-left:4px solid #ef5350'>"
        "<h2 style='color:#ef5350'>&#x26A0; Invalid voltage</h2>"
        "<p>Value must be between 3.000 V and 5.500 V. "
        "Check your multimeter reading and try again.</p>"
        "<a href='/calibrate' style='color:#64B5F6'>&#x2190; Back</a>"
        "</div>");
      sendPage(h + pageFoot());
      return;
    }

    // ── DISPLAY form ────────────────────────────────────────────────────────
    float curV = batteryVoltFloat / 1000.0f;
    bool onBattery = (!isUSBPowered && !isBatFloating);
    // "calibrated" = factor has been explicitly saved (differs from compile-time default)
    bool usbCalDone = (fabsf(vdivFactor    - VDIV_FACTOR_DEFAULT) > 0.0001f);
    bool batCalDone = (fabsf(vdivFactorBat - VDIV_FACTOR_DEFAULT) > 0.0001f)
                   || (fabsf(vdivFactorBat - vdivFactor)           > 0.0001f);

    String h = pageHead(F("Calibrate Battery"));
    h += F("<h1>&#x1F3AF; Battery Calibration</h1>");

    // Current mode banner
    h += F("<div class='card' style='border-left:4px solid ");
    if (isBatFloating)
      h += F("#546e7a'><h2>&#x26A0; USB-only / no battery</h2>"
             "<p style='color:#9e9e9e'>No battery cell detected. "
             "There is nothing to calibrate for this state — connect a battery.</p>");
    else if (onBattery)
      h += F("#ffa726'><h2>&#x1F50B; Battery-only mode</h2>");
    else
      h += F("#66bb6a'><h2>&#x26A1; USB / Charging mode</h2>");
    if (!isBatFloating) {
      h += F("<p>ESP32 reads: <strong>");
      h += String(curV, 3);
      h += F("V</strong> &nbsp;Active factor: <strong>");
      h += String(onBattery ? vdivFactorBat : vdivFactor, 4);
      h += F("</strong></p>");
    }
    h += F("</div>");

    // Step progress table
    h += F("<div class='card'><h2>Calibration Progress</h2>"
      "<table style='width:100%;border-collapse:collapse;font-size:14px'>"
      "<tr style='color:#9e9e9e'>"
      "<td style='padding:6px 4px'>Step</td>"
      "<td style='padding:6px 4px'>Mode</td>"
      "<td style='padding:6px 4px'>Factor (NVS)</td>"
      "<td style='padding:6px 4px'>Status</td></tr>"
      "<tr style='border-top:1px solid #333'>"
      "<td style='padding:6px 4px;font-weight:700'>1</td>"
      "<td style='padding:6px 4px'>&#x26A1; USB / Charging</td>"
      "<td style='padding:6px 4px;font-family:monospace'>");
    h += String(vdivFactor, 4);
    h += F("</td><td style='padding:6px 4px'>");
    h += usbCalDone
      ? F("<span style='color:#66bb6a'>&#x2705; calibrated</span>")
      : F("<span style='color:#9e9e9e'>&#x23F3; not yet set</span>");
    h += F("</td></tr>"
      "<tr style='border-top:1px solid #333'>"
      "<td style='padding:6px 4px;font-weight:700'>2</td>"
      "<td style='padding:6px 4px'>&#x1F50B; Battery only</td>"
      "<td style='padding:6px 4px;font-family:monospace'>");
    h += String(vdivFactorBat, 4);
    h += F("</td><td style='padding:6px 4px'>");
    h += batCalDone
      ? F("<span style='color:#66bb6a'>&#x2705; calibrated</span>")
      : F("<span style='color:#ffa726'>&#x23F3; using USB factor</span>");
    h += F("</td></tr></table></div>");

    // How-to
    h += F("<div class='info'><strong>Two-step calibration — do both for accuracy:</strong>"
      "<ol style='padding-left:18px;margin-top:8px;font-size:13px;line-height:1.7'>"
      "<li><strong>Step 1 (USB):</strong> Plug in USB. Measure battery+ terminal "
      "to GND with a multimeter. Enter the reading below and save.</li>"
      "<li><strong>Step 2 (Battery):</strong> Unplug USB, run on battery only. "
      "Measure again. Enter the reading below and save.</li>"
      "<li>Code auto-selects the right factor each reading. Both must be done "
      "for full accuracy.</li>"
      "</ol></div>");

    // Input form — only show when a battery read is available and mode is known
    if (!isBatFloating) {
      h += F("<div class='card'><h2>Enter multimeter reading "
        "<span style='font-weight:400;color:#9e9e9e;font-size:14px'>"
        "(saves </span>");
      h += onBattery
        ? F("<span style='color:#ffa726'>battery factor &#x2014; step 2</span>")
        : F("<span style='color:#66bb6a'>USB factor &#x2014; step 1</span>");
      h += F("<span style='font-weight:400;color:#9e9e9e;font-size:14px'>)"
        "</span></h2>"
        "<form method=GET action=/calibrate style='margin-top:8px'>"
        "<div style='display:flex;gap:8px;align-items:center'>"
        "<input name=v type=number step=0.001 min=3.0 max=5.5 placeholder='e.g. ");
      h += onBattery ? F("3.770") : F("4.100");
      h += F("' required style='max-width:160px'>"
        "<button type=submit class='btn' style='margin:0'>&#x1F4BE; Save</button>"
        "</div>"
        "<p style='color:#9e9e9e;font-size:12px;margin-top:8px'>"
        "Measure NOW while the device is in this mode. "
        "Saves to NVS immediately. Affects all future readings in this mode.</p>"
        "</form></div>");
    }
    h += F("<a href='/' style='display:inline-block;margin-top:16px;color:#64B5F6'>"
      "&#x2190; Dashboard</a>");
    sendPage(h + pageFoot());
  });

  static const char* collectHdrs[] = {"Content-Length"};
  server.collectHeaders(collectHdrs, 1);
  server.begin();
  Serial.println(F("[Web] Server started"));
  Serial.print(F("[Web] http://")); Serial.println(WiFi.localIP().toString());
}

// ── Forward declarations — screensaver reset functions called inside lambdas
// in setupOTA() which precedes their definitions. Arduino's auto-prototype
// generator does not forward-declare functions referenced inside lambdas.
void ssResetBall();
void ssResetMario();

void setup() {
  Serial.begin(115200);
  printResetReason();

  // ── CRC32 lookup table — built once, used by OTA upload + GitHub install ──
  buildCrc32Table();
  // Compute CRC32 of the running sketch from flash — done once at boot.
  // Uses the same ISO-HDLC polynomial as the OTA upload checker and manifest.
  {
    const esp_partition_t* part = esp_ota_get_running_partition();
    if (part) {
      uint32_t crc       = 0xFFFFFFFF;
      uint8_t  buf[256];
      size_t   remaining = ESP.getSketchSize();
      size_t   offset    = 0;
      while (remaining > 0) {
        size_t chunk = min(remaining, sizeof(buf));
        esp_partition_read(part, offset, buf, chunk);
        crc = crc32Feed(crc, buf, chunk);
        offset    += chunk;
        remaining -= chunk;
      }
      crc ^= 0xFFFFFFFF;
      char hex[9];
      snprintf(hex, sizeof(hex), "%08X", crc);
      runningCrc32 = String(hex);
      Serial.printf("[BOOT] Running firmware CRC32: %s\n", hex);
    }
  }

  //To guarantee the compiler doesn't throw this variable away during optimization
  Serial.println(fw_binary_signature);

  // ── Power-save: restore peripherals shut down before previous sleep ────────
  loadPowerSaveConfig();
  powerUpPeripherals();
  // Sync runtime OTA-available flag from RTC memory.
  // rtcOtaAvailable survives deep sleep so we know an update was found on a
  // previous wake even if we skip the manifest check this cycle.
  otaUpdateAvailable = rtcOtaAvailable;

  // ── Boot counter — RTC + NVS hybrid ─────────────────────────────────────────
  // Wakeup cause must be read BEFORE setup() does anything that resets it.
  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  bool wokeByButton = (wakeupCause == ESP_SLEEP_WAKEUP_EXT0);
  bool wokeByTimer  = (wakeupCause == ESP_SLEEP_WAKEUP_TIMER);
  bool wokeFromSleep = wokeByButton || wokeByTimer;

  if (wokeFromSleep) {
    // Sleep wake: just increment RTC counter — no NVS write, no flash wear
    rtcBootOffset++;
    preferences.begin("sys", true);   // read-only
    uint32_t nvsBase = (uint32_t)preferences.getInt("bootcount", 0);
    preferences.end();
    bootCount = (int)(nvsBase + rtcBootOffset);
    Serial.printf("[BOOT] Sleep wake — count %d (NVS base + RTC offset %u, no flash write)\n",
                  bootCount, rtcBootOffset);
  } else {
    // Power-on or crash: RTC was wiped, write NVS — this is the event worth tracking
    rtcBootOffset = 0;
    preferences.begin("sys", false);
    uint32_t nvsBase = (uint32_t)preferences.getInt("bootcount", 0) + 1;
    preferences.putInt("bootcount", (int)nvsBase);
    preferences.end();
    bootCount = (int)nvsBase;
    Serial.printf("[BOOT] Power-on/crash — count %d written to NVS\n", bootCount);
  }
  batteryWarnSent = false;
  if (wokeByButton) Serial.println(F("[BOOT] Woke by button"));
  if (wokeByTimer)  Serial.println(F("[BOOT] Woke by timer"));

  // Load deep sleep config NOW — wakeDisplayMode must be known before ui.init()
  // because ui.init() sends the SSD1306 init sequence which turns the display on.
  // We need to call displayOff() immediately after for Stealth timer wakes.
  loadDeepSleepConfig();
  stealthThisWake = (wakeDisplayMode == 0) && wokeByTimer;

  // Hardware init — Vext already on from powerUpPeripherals(); just set button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // OLED + UI init
  ui.setTargetFPS(30);
  ui.setIndicatorPosition(BOTTOM);
  ui.setIndicatorDirection(LEFT_RIGHT);
  ui.setFrameAnimation(SLIDE_LEFT);
  ui.setFrames(frames3, 3);
  ui.init();
  ui.setOverlays(overlays, 1);  // OTA-available badge overlay, drawn on every frame
  // ui.init() enables the display as part of SSD1306 hardware init.
  // Kill it immediately if this is a Stealth timer wake.
  if (stealthThisWake) display.displayOff();

  analogReadResolution(12);
  dht.begin();

  // Single click: extend awake window 10 min
  button.attachClick([]() {
    disableDeepSleepUntil = millis() + 10UL * 60UL * 1000UL;
    Serial.println(F("[BTN] Click -- awake 10 min"));
  });

  // Double-click: immediate sensor read + publish on demand
  button.attachDoubleClick([]() {
    Serial.println(F("[BTN] Double-click -- manual read+publish"));
    disableDeepSleepUntil = millis() + 10UL * 60UL * 1000UL;  // stay awake
    // Show a brief "Reading..." overlay on current frame
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 10, "Reading...");
    display.setFont(ArialMT_Plain_10);
    display.drawString(64, 34, "Manual trigger");
    display.display();
    delay(400);  // brief visual feedback
    readSensor();  // reads DHT + publishes + updates history
    Serial.println(F("[BTN] Manual read complete"));
  });

  // Triple-click: pause/resume OLED auto-scroll
  button.attachMultiClick([]() {
    if (button.getNumberClicks() == 3) {
      scrollPaused = !scrollPaused;
      if (scrollPaused) ui.disableAutoTransition();
      else              ui.enableAutoTransition();
      Serial.println("[BTN] Scroll " + String(scrollPaused ? "PAUSED" : "RESUMED"));
    }
  });

  // Long press START: begin countdown display (feedback only)
  button.attachLongPressStart([]() {
    button.setPressTicks(3000);
    Serial.println(F("[BTN] Hold started"));
  });

  // Long press STOP (released after 3 s): trigger deep sleep ONLY if enabled.
  // Does NOT force-enable deep sleep — the web UI setting is the authority.
  // If sleep is disabled, logs a message so the user knows why nothing happened.
  button.setPressTicks(3000);
  button.attachLongPressStop([]() {
    if (!deepSleepEnabled) {
      Serial.println(F("[BTN] Hold ignored -- deep sleep is DISABLED in settings"));
      return;
    }
    Serial.println(F("[BTN] 3s hold -- sleeping now"));
    goToDeepSleep();
  });

  // ── ntfy config ──────────────────────────────────────────────────────────────
  loadNtfyConfig();
 

  // ── Double reset check — BEFORE WiFiManager ────────────────────────────────
  // SKIP on timer wakes: deep sleep resets the ESP32 on every wake, so DRD
  // would fire on every second timer wake and force the config portal open.
  // Only meaningful on power-on or button wakes where a human double-reset
  // is actually possible.
  bool forcePortal = (!wokeByTimer) && detectDoubleReset();

  // ── WiFiManager (blocks here until connected or portal times out) ──────────
  // WDT is NOT started yet — portal can legitimately take 3 minutes.
  setupWiFiManager(forcePortal);

  // ── Oled Ssaver config ───────────────────────────────────────────────────────
  loadOledConfig(); // ensure latest saved preset is loaded

  // ── Apply display state now that WiFi is up ──────────────────────────────────
  // stealthThisWake was computed before ui.init() above.
  // For Active/button/power-on: turn display on now.
  // For Stealth timer wake: display is already off (killed after ui.init).
  if (stealthThisWake) {
    led.Off().Update();
    Serial.println(F("[DISP] Stealth -- OLED+LED off for this timer wake"));
  } else {
    display.displayOn();
    lastActivityMs = millis();
  }

  // ── WDT: safe to start now that WiFi is connected ─────────────────────────
  // Core 3.x initialises TWDT itself before user code runs, so calling
  // esp_task_wdt_init() again throws "TWDT already initialized". Instead
  // reconfigure the existing instance with our timeout, then subscribe.
  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_reconfigure(&wdtCfg);  // update timeout on existing WDT
  // Subscribe main task immediately after reconfigure — must happen before
  // any call to esp_task_wdt_reset() (including inside checkWiFi / loop)
  esp_task_wdt_add(NULL);
  Serial.println(F("[WDT] Configured (30s)"));

  // ── MQTT + OTA ─────────────────────────────────────────────────────────────
  setupMQTTClients();
  setupOTA();
  reconnectMQTT();

  // ── Wait up to 10 s for MQTT broker before first publish ─────────────────
  {
    unsigned long waitStart = millis();
    bool connected = false;
    while (millis() - waitStart < 10000 && !connected) {
      esp_task_wdt_reset();
      // Try each configured platform directly (bypasses retry-interval guard)
      if ((mqtt_platform == "standard" || mqtt_platform == "all") && mqtt_server.length() && !mqttStd.connected()) {
        String id = device_name + "-std-" + String(random(0xffff), HEX);
        connectMQTT(mqttStd, mqtt_server, mqtt_port.toInt(), mqtt_user, mqtt_pass, id);
        if (mqttStd.connected()) { mqttStandardConnected = true; stdRetries = 0; }
      }
      if ((mqtt_platform == "adafruit" || mqtt_platform == "all") && aio_username.length() && !mqttAIO.connected()) {
        String id = device_name + "-aio-" + String(random(0xffff), HEX);
        connectMQTT(mqttAIO, "io.adafruit.com", 1883, aio_username, aio_key, id);
        if (mqttAIO.connected()) { mqttAdafruitConnected = true; aioRetries = 0; }
      }
      if ((mqtt_platform == "ubidots" || mqtt_platform == "all") && ubidots_token.length() && !mqttUBI.connected()) {
        String id = device_name + "-ubi-" + String(random(0xffff), HEX);
        connectMQTT(mqttUBI, "industrial.api.ubidots.com", 1883, ubidots_token, "", id);
        if (mqttUBI.connected()) { mqttUbidotsConnected = true; ubiRetries = 0; }
      }
      connected = mqttStd.connected() || mqttAIO.connected() || mqttUBI.connected();
      if (!connected) delay(500);
    }
    if (connected)
      Serial.println("[MQTT] Broker ready in " + String(millis() - waitStart) + "ms");
    else
      Serial.println(F("[MQTT] No broker after 10s -- publish skipped this cycle"));
  }

  // ── Blocking battery read before first publish ────────────────────────────
  {
    { Preferences p; p.begin("battery", true);
      learnedVFull  = p.getFloat("vFull",   3.92f);
      vdivFactor    = p.getFloat("vdiv",    VDIV_FACTOR_DEFAULT);
      vdivFactorBat = p.getFloat("vdiv_bat", vdivFactor);  // fallback: USB factor if never calibrated on battery
      p.end(); }
    Serial.printf("[CAL] vdivFactor=%.4f  vdivFactorBat=%.4f  learnedVFull=%.3f\n",
                  vdivFactor, vdivFactorBat, learnedVFull);

    // Allow ADC to settle after power-on / wake transients before sampling
    delay(50);

    uint32_t bsum = 0; uint16_t bmn=65535, bmx=0;
    for (int i = 0; i < 16; i++) {
      uint16_t s = analogReadMilliVolts(BATTERY_PIN);
      bsum += s; if(s<bmn)bmn=s; if(s>bmx)bmx=s;
      delayMicroseconds(500);  // slightly longer spacing vs runtime (100µs) for settle
    }
    // Drop the single highest and lowest samples to reject ADC glitches,
    // then average the remaining 14. This is the main reason boot reads
    // previously spiked above BAT_USB_THRESHOLD and falsely reported "usb".
    bsum -= bmn; bsum -= bmx;
    float bavg = (float)bsum / 14.0f;
    uint16_t bvar = bmx - bmn;
    lastRawAvgMv = bavg;  // stash for /calibrate race-free back-calc

    // Boot battery detection — same two-pass logic as runtime battery_read() v5.25.
    // Pass 1: estimate with vdivFactor (USB-calibrated) to determine source.
    float bvEst       = (bavg / 1000.0f) * vdivFactor;
    bool bootFloating = (bvEst < BAT_FLOAT_VOLTAGE);
    if (bvEst > BAT_USB_THRESHOLD) {
      lastHighVoltageTime = millis();
      hadHighVoltage = true;
      bootFloating = false;
    }
    bool bootUSB = (bvEst > BAT_USB_THRESHOLD) ||
                   (hadHighVoltage && millis() - lastHighVoltageTime < BAT_USB_HYSTERESIS_MS);

    // Pass 2: re-apply mode-appropriate factor for accurate final voltage.
    float bootFactor = (!bootUSB && !bootFloating) ? vdivFactorBat : vdivFactor;
    float bv         = (bavg / 1000.0f) * bootFactor;

    isBatFloating = bootFloating;
    isUSBPowered  = bootUSB;

    // Only update learnedVFull on battery-only, capped below USB threshold
    if (!isUSBPowered && !isBatFloating && bv > learnedVFull && bv < BAT_USB_THRESHOLD) {
      learnedVFull = bv;
      Preferences p; p.begin("battery", false);
      p.putFloat("vFull", learnedVFull); p.end();
    }
    batteryVoltFloat  = bv * 1000.0f;
    batteryVoltage    = (int)batteryVoltFloat;
    batteryPercentage = constrain(voltsToPercent(bv), 0, 100);
    lastBatteryRead   = millis();

    const char* bootSrcStr = isBatFloating ? "USB/ONLY"
                           : isUSBPowered  ? "USB/CHG"
                           :                 "BAT";
    Serial.printf("[BAT-BOOT] raw=%.0f var=%u V=%.3f %d%% %s factor=%.4f learnedFull=%.3f\n",
                  bavg, bvar, bv, batteryPercentage, bootSrcStr, bootFactor, learnedVFull);
  }

  

  // ── Post-OTA boot splash — check NVS for "just updated" flag ────────────────
  {
    preferences.begin("ota", false);
    bool justUpdated = preferences.getBool  ("updated",  false);
    String prevVer   = preferences.getString("prev_ver", "");
    if (justUpdated) {
      preferences.putBool("updated", false);  // clear — shown only once
    }
    preferences.end();

    Serial.printf("[OTA] Boot check: justUpdated=%d prevVer='%s' FW='%s'\n",
      justUpdated, prevVer.c_str(), FW_VERSION);

    if (justUpdated && prevVer.length()) {
      // Clear the runtime OTA-available flag — we just installed it
      rtcOtaAvailable    = false;
      otaUpdateAvailable = false;
      showOtaBootSplash(prevVer);

      // MQTT: publish update-applied event
      if ((mqtt_platform == "standard" || mqtt_platform == "all") && mqttStd.connected()) {
        JsonDocument ud;
        ud["event"]     = "ota_applied";
        ud["device"]    = device_name;
        ud["from"]      = prevVer;
        ud["to"]        = FW_VERSION;
        ud["boot"]      = bootCount;
        String up; serializeJson(ud, up);
        mqttStd.publish((mqtt_topic + "/ota").c_str(), up.c_str());
        mqttStd.loop();
      }

      // ntfy: update-applied notification
      if (ntfy_enabled && ntfy_on_boot && ntfy_topic.length()) {
        sendNtfy("Firmware Updated: " + device_name,"v" + prevVer + " -> v" + String(FW_VERSION) + "" "Boot #" + String(bootCount) + "  IP: " + WiFi.localIP().toString(), 3, "white_check_mark,rocket");
      }
    }
  }

  // ── GitHub OTA update check on first boot/wake ────────────────────────────
  // checkOtaManifest() is also called periodically from loop().
  // Only check if WiFi connected and not in a very short wake window.
  checkOtaManifest(false);

  // Initial sensor read/publish
  readSensor();


  // Boot summary: one-time publish to MQTT + ntfy with battery + context
  publishBootSummary();

  
  // ── Set awake window AFTER all setup() blocking is complete ───────────────
  if (wokeByButton) {
    disableDeepSleepUntil = millis() + 10UL * 60UL * 1000UL;
    Serial.println(F("[BOOT] Button wake -- awake 10 min"));
  } else {
    // Timer wake: publish+flush takes ~2-3s; 8s gives comfortable headroom.
    disableDeepSleepUntil = millis() + 8000UL;
    Serial.println(F("[BOOT] Timer/power wake -- awake 8s then sleep"));
  }

  // ── LED: slow breathing = running normally (suppressed in Stealth timer wakes) ──
  if (!stealthThisWake) {
    led.Breathe(2000).Forever().Update();
  }

  Serial.println("[BOOT] Ready — " + WiFi.localIP().toString());
}

// ═════════════════════════════════════════════════════════════════════════════
// OLED SCREENSAVER ANIMATIONS
// ═════════════════════════════════════════════════════════════════════════════

// ── Preset 0 — Bouncing Ball ──────────────────────────────────────────────────
// Single dot with dot-product (angle-of-incidence = angle-of-reflection) physics.
void ssResetBall() {
  bbX  = 32 + random(0, 64); bbY  = 16 + random(0, 32);
  bbVx = 1.2f + random(0, 100) / 180.0f;
  bbVy = 0.8f + random(0, 100) / 180.0f;
  if (random(2)) bbVx = -bbVx;
  if (random(2)) bbVy = -bbVy;
}

void ssDrawBouncingBall() {
  if (millis() - ssaverFrameMs < 38) return;   // ~26 fps
  ssaverFrameMs = millis();

  bbX += bbVx;
  bbY += bbVy;

  // Reflect velocity component perpendicular to the wall (dot-product style)
  if (bbX < 2.0f)   { bbX = 2.0f;   bbVx =  fabsf(bbVx); }
  if (bbX > 125.0f) { bbX = 125.0f; bbVx = -fabsf(bbVx); }
  if (bbY < 2.0f)   { bbY = 2.0f;   bbVy =  fabsf(bbVy); }
  if (bbY > 61.0f)  { bbY = 61.0f;  bbVy = -fabsf(bbVy); }

  int bx = (int)bbX, by = (int)bbY;
  display.clear();
  // Filled 3×3 core + cross arms for visibility
  for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++)
      display.setPixel(bx + dx, by + dy);
  display.setPixel(bx,     by - 2);
  display.setPixel(bx,     by + 2);
  display.setPixel(bx - 2, by);
  display.setPixel(bx + 2, by);
  display.drawRect(0, 0, 128, 64);  // visible room border
  display.display();
}

// ── Preset 1 — Mario Bounce ───────────────────────────────────────────────────
// Gravity arc + wall bounces. 9×11 sprite, 2 walk frames, horizontally mirrored
// when moving left. Sprite is drawn pixel-by-pixel so no bitmap library needed.
//
// Sprite layout (9 cols wide, MARIO_W=9):
//   bit (MARIO_W-1-col) of each row word = pixel at that column.
//   Rows as 9-bit patterns — comments show the pixel map (.=off, X=on).
#define MARIO_W 9
#define MARIO_H 11
static const uint16_t MARIO_FRAMES[2][MARIO_H] PROGMEM = {
  { // Frame 0 — stand / left foot forward
    0b011100000,  // .XXX.....
    0b011111000,  // .XXXXX...
    0b111111100,  // XXXXXXX..
    0b011111000,  // .XXXXX...
    0b011111000,  // .XXXXX...
    0b111111100,  // XXXXXXX..
    0b011111000,  // .XXXXX...
    0b010001000,  // .X...X...  legs apart
    0b010001000,  // .X...X...
    0b110001100,  // XX...XX..  feet
    0b000000000
  },
  { // Frame 1 — right foot forward
    0b011100000,
    0b011111000,
    0b111111100,
    0b011111000,
    0b011111000,
    0b111111100,
    0b011111000,
    0b110000100,  // XX....X..  legs shifted
    0b011001000,  // .XX..X...
    0b011001100,  // .XX..XX..  feet shifted
    0b000000000
  }
};

void ssResetMario() {
  mrX = 20; mrY = 30;
  mrVx = 1.4f + random(0, 80) / 200.0f;
  mrVy = -3.8f;
  mrFrame = 0;
}

void ssDrawMario() {
  const float GRAVITY    =  0.22f;
  const float BOUNCE_VY  = -4.5f;
  const float FLOOR      =  64.0f - MARIO_H - 1;

  if (millis() - mrLastMoveMs < 28) return;   // ~35 fps physics
  mrLastMoveMs = millis();

  mrVy += GRAVITY;
  mrX  += mrVx;
  mrY  += mrVy;

  if (mrY >= FLOOR)           { mrY = FLOOR;           mrVy = BOUNCE_VY;         }
  if (mrY < 0)                { mrY = 0;               mrVy = fabsf(mrVy)*0.5f;  }
  if (mrX < 0)                { mrX = 0;               mrVx =  fabsf(mrVx);      }
  if (mrX > 128 - MARIO_W)    { mrX = 128 - MARIO_W;  mrVx = -fabsf(mrVx);      }

  // Alternate walk frame every 150 ms
  if (millis() - mrLastFrameMs > 150) {
    mrLastFrameMs = millis();
    mrFrame = (mrFrame + 1) & 1;
  }

  display.clear();

  int ox = (int)mrX, oy = (int)mrY;
  bool flipX = (mrVx < 0);   // mirror sprite when moving left

  for (int row = 0; row < MARIO_H; row++) {
    uint16_t bits = pgm_read_word(&MARIO_FRAMES[mrFrame][row]);
    for (int col = 0; col < MARIO_W; col++) {
      int srcCol = flipX ? (MARIO_W - 1 - col) : col;
      if (bits & (1u << (MARIO_W - 1 - srcCol))) {
        int px = ox + col, py = oy + row;
        if (px >= 0 && px < 128 && py >= 0 && py < 64)
          display.setPixel(px, py);
      }
    }
  }
  display.display();
}

// ── Preset 2 — Matrix Rain ────────────────────────────────────────────────────
// 21 falling columns of actual rendered characters, mimicking the half-width
// katakana glyphs used in the film's title sequence.
// ArialMT_Plain_10 = ~6 px wide × 8 px tall → 21 cols × 8 rows fills 128×64.
// Head glyph re-randomises every frame (bright flicker).
// Trail cells mutate one random glyph per frame (the subtle swap from the film).
// Tail fading is faked by skipping alternate rows deeper in the trail.

static const uint8_t MX_CHARSET_LEN = sizeof(MX_CHARSET) - 1;

static char mxRandChar() {
  return MX_CHARSET[random(MX_CHARSET_LEN)];
}

void ssInitMatrix() {
  for (int c = 0; c < MX_COLS; c++) {
    mxHead[c]  = -(int16_t)random(10, 90);
    mxSpeed[c] =  1 + (uint8_t)random(0, 3);
    mxLen[c]   =  4 + (uint8_t)random(0, 5);
    for (int r = 0; r < MX_ROWS; r++) mxChar[c][r] = mxRandChar();
  }
  mxInited = true;
}

void ssDrawMatrix() {
  if (!mxInited) ssInitMatrix();
  if (millis() - ssaverFrameMs < 80) return;  // ~12 fps — readable character pace
  ssaverFrameMs = millis();

  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  for (int c = 0; c < MX_COLS; c++) {
    int headPx  = (int)mxHead[c];
    int headRow = headPx / 8;

    // Mutate one random trail cell each frame
    mxChar[c][(int16_t)(random(MX_ROWS))] = mxRandChar();

    // Re-randomise head glyph every frame for bright flicker
    mxChar[c][(headRow + MX_ROWS) % MX_ROWS] = mxRandChar();

    for (int t = 0; t < (int)mxLen[c]; t++) {
      int row = headRow - t;
      if (row < 0 || row >= MX_ROWS) continue;

      // Skip alternate rows deep in tail → faked dimming
      if (t >= 3 && (t % 2 != 0)) continue;
      if (t >= (int)mxLen[c] - 1 && (row + c) % 2 != 0) continue;

      char buf[2] = { mxChar[c][row % MX_ROWS], '\0' };
      display.drawString(c * MX_COL_PX, row * 8, buf);
    }

    mxHead[c] += mxSpeed[c];

    if ((mxHead[c] / 8) - (int)mxLen[c] >= MX_ROWS) {
      mxHead[c]  = -(int16_t)(random(5, 64));
      mxSpeed[c] =  1 + (uint8_t)random(0, 3);
      mxLen[c]   =  4 + (uint8_t)random(0, 5);
      for (int r = 0; r < MX_ROWS; r++) mxChar[c][r] = mxRandChar();
    }
  }

  display.display();
}
// ── Preset 3 — DVD Logo Bounce ────────────────────────────────────────────────
// Faithful recreation of the classic 2000s DVD player idle screensaver.
// A "DVD" badge (text + border rect) bounces perfectly around the screen.
// Corner hit: brief hardware invert flash — the moment everyone waits for.

void ssResetDvd() {
  dvdX  = 5.0f + random(0, 60);
  dvdY  = 2.0f + random(0, 20);
  dvdVx = 1.1f + random(0, 80) / 160.0f;
  dvdVy = 0.7f + random(0, 80) / 160.0f;
  if (random(2)) dvdVx = -dvdVx;
  if (random(2)) dvdVy = -dvdVy;
  dvdCornerFlash = false;
}

void ssDrawDvd() {
  if (millis() - ssaverFrameMs < 40) return;   // ~25 fps
  ssaverFrameMs = millis();

  const float MAX_X = 128.0f - DVD_LOGO_W;
  const float MAX_Y =  64.0f - DVD_LOGO_H;

  bool hitX = false, hitY = false;

  dvdX += dvdVx;
  dvdY += dvdVy;

  if (dvdX <= 0.0f)  { dvdX = 0.0f;  dvdVx =  fabsf(dvdVx); hitX = true; }
  if (dvdX >= MAX_X) { dvdX = MAX_X; dvdVx = -fabsf(dvdVx); hitX = true; }
  if (dvdY <= 0.0f)  { dvdY = 0.0f;  dvdVy =  fabsf(dvdVy); hitY = true; }
  if (dvdY >= MAX_Y) { dvdY = MAX_Y; dvdVy = -fabsf(dvdVy); hitY = true; }

  // Corner hit — the legendary moment — brief hardware invert flash
  if (hitX && hitY) {
    dvdCornerFlash = true;
    dvdFlashMs     = millis();
    display.invertDisplay();
    Serial.println(F("[SAVER] DVD corner hit!"));
  }
  if (dvdCornerFlash && millis() - dvdFlashMs > 150) {
    dvdCornerFlash = false;
    display.normalDisplay();
  }

  display.clear();
  display.drawXbm((int)dvdX, (int)dvdY, DVD_LOGO_W, DVD_LOGO_H, DVD_LOGO);
  display.display();
}

// ── Screensaver dispatcher ────────────────────────────────────────────────────
// Call every loop() iteration. Activates after idle timeout; any button press
// (via ssaverResetActivity()) dismisses it instantly.
void runScreensaver() {
  // Allow preview to run even when ssaverEnabled is false; only block
  // auto-activation.  ssaverPreview is only true during an explicit preview
  // triggered from the web UI, so this is safe.
  if ((!ssaverEnabled && !ssaverPreview) || stealthThisWake || screenCleanActive) {
    if (ssaverActive) {
      ssaverActive = false;
      display.normalDisplay();
      display.clear();
      display.displayOn();
    }
    return;
  }

  unsigned long idleMs = millis() - lastActivityMs;

  // Activate if idle long enough
  if (!ssaverActive && idleMs >= ssaverTimeoutSecs * 1000UL) {
    ssaverActive  = true;
    ssaverPreview = false;
    ssaverStartMs = millis();
    ssaverFrameMs = 0;
    mxInited      = false;
    
    ssResetBall();
    ssResetMario();
    ssResetDvd();

    display.displayOn();
    Serial.printf("[SAVER] Auto-activated preset %u after %lu ms idle\n", 
                  ssaverPreset, idleMs);
  }

  if (!ssaverActive) return;

  // Preview timeout
  if (ssaverPreview && millis() - ssaverStartMs > 10000UL) {
    ssaverActive = false;
    ssaverPreview = false;
    ssaverPreset = ssaverSavedPreset;
    display.normalDisplay();
    display.clear();
    lastActivityMs = millis();
    Serial.println(F("[SAVER] Preview ended"));
    return;
  }

  // Draw current animation
  switch (ssaverPreset) {
    case 1:  ssDrawMario();         break;
    case 2:  ssDrawMatrix();        break;
    case 3:  ssDrawDvd();           break;
    default: ssDrawBouncingBall();  break;
  }
}

void ssaverResetActivity() {
  lastActivityMs = millis();
  if (ssaverActive) {
    ssaverActive = false;
    ssaverPreview = false;
    display.normalDisplay();
    display.clear();
    display.displayOn();
    Serial.println(F("[SAVER] Dismissed by button"));
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// LOOP
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  esp_task_wdt_reset();  // feed watchdog — must be called regularly
  button.tick();         // process all button events via OneButton
  led.Update();          // advance JLed animation state

  // ── Button activity detection for screensaver ─────────────────────────────
  static bool lastBtnState = HIGH;
  bool currentBtnState = digitalRead(BUTTON_PIN);
  if (currentBtnState == LOW || button.getNumberClicks() > 0) {
    ssaverResetActivity();
  }
  lastBtnState = currentBtnState;

  // ── Hold countdown display (globalHoldSeconds drives OLED overlay) ─────────
  // Uses OneButton's internal press duration — no competing digitalRead needed
  static unsigned long holdStart = 0;
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (holdStart == 0) holdStart = millis();
    unsigned long held = millis() - holdStart;
    if (held >= 3000)      globalHoldSeconds = 0;
    else if (held > 100)   globalHoldSeconds = max(0, (int)(3 - held / 1000));
  } else {
    holdStart = 0;
    globalHoldSeconds = 0;
  }

  // ── Pixel-exercise screensaver ──────────────────────────────────────────────
  if (screenCleanActive) {
    if (millis() > screenCleanUntil) {
      // Cycle complete — normalDisplay() essential to undo any invertDisplay()
      screenCleanActive = false;
      display.normalDisplay();
      display.clear();
      display.displayOn();
      Serial.println(F("[OLED] Pixel exercise complete"));
    } else {
      unsigned long elapsed = millis() - (screenCleanUntil - (unsigned long)screenCleanDuration * 1000UL);
      if (screenCleanPreset == 0) {
        // Checkerboard Shift: alternating polarity every 500ms — equal pixel exercise
        if (millis() - screenCleanToggle > 500UL) {
          screenCleanToggle = millis();
          static bool ckInv = false; ckInv = !ckInv;
          display.clear();
          for (int cy = 0; cy < 64; cy++)
            for (int cx = 0; cx < 128; cx++)
              if (((cx + cy) % 2 == 0) != ckInv) display.setPixel(cx, cy);
          display.display();
        }
      } else if (screenCleanPreset == 1) {
        // Invert Ramp: sweeping fill + full invert mid-cycle
        if (millis() - screenCleanToggle > 80UL) {
          screenCleanToggle = millis();
          static uint8_t rampRow = 0;
          display.clear();
          for (int ry = 0; ry < rampRow; ry++)
            for (int rx = 0; rx < 128; rx++) display.setPixel(rx, ry);
          display.display();
          rampRow++;
          if (rampRow >= 64) {
            rampRow = 0;
            // Toggle hardware invert on each sweep completion
            static bool inverted = false; inverted = !inverted;
            inverted ? display.invertDisplay() : display.normalDisplay();
          }
        }
      } else if (screenCleanPreset == 2) {
        // Scanline Sweep: rolling bright line; inverts panel mid-duration
        if (millis() - screenCleanToggle > 60UL) {
          screenCleanToggle = millis();
          static int scanY = 0;
          bool invPhase = elapsed > ((unsigned long)screenCleanDuration * 500UL);
          if (invPhase) display.invertDisplay(); else display.normalDisplay();
          display.clear();
          for (int rx = 0; rx < 128; rx++) display.setPixel(rx, scanY);
          display.display();
          scanY = (scanY + 1) % 64;
        }
      } else {
        // Preset 3 — Full Bright Pulse: fill all pixels white then flash on/off every 1s.
        // Exercises every pixel at maximum brightness, ideal for burn-in recovery.
        if (millis() - screenCleanToggle > 2000UL) {
          screenCleanToggle = millis();
          static bool inv = false; inv = !inv;
          inv ? display.invertDisplay() : display.normalDisplay();
        }
      }
    }
  }

  // ── OLED Screensaver ──────────────────────────────────────────────────────
  runScreensaver();
  
  // ── Normal UI Update + Activity Timer ─────────────────────────────────────
  int budget = 10;   // default delay when screensaver is active
  
  if (!stealthThisWake && !screenCleanActive && !ssaverActive) {
    budget = ui.update();
    // NOTE: do NOT reset lastActivityMs here — ui.update() returns >= 0 on every
    // frame, so resetting here would permanently prevent the screensaver from
    // ever firing.  Idle timer is only reset by real user events (button press,
    // ssaverResetActivity) and once on boot / display-on in setup().
  }

  // Extra safety: any button press resets idle timer
  if (button.getNumberClicks() > 0) {
    lastActivityMs = millis();
  }
  

  // ── WiFi health check ──────────────────────────────────────────────────────
  checkWiFi();
  wifiConnected = (WiFi.status() == WL_CONNECTED);

  // ── MQTT keep-alive + reconnect ────────────────────────────────────────────
  if (wifiConnected) {
    reconnectMQTT();
    mqttStd.loop(); mqttAIO.loop(); mqttUBI.loop();
  }

  // ── Periodic sensor read ───────────────────────────────────────────────────
  if (millis() - lastRead >= readInterval) {
    lastRead = millis();
    readSensor();
  }

  // ── Battery monitor (non-blocking, every 5 s) ──────────────────────────────
  battery_read();
  checkBatteryAlerts();
  checkSensorAlerts();

  // ── GitHub OTA manifest check (periodic, non-blocking) ─────────────────────
  // Only runs if interval has elapsed — safe to call every loop iteration.
  if (wifiConnected) checkOtaManifest(false);

  // ── Web server ─────────────────────────────────────────────────────────────
  server.handleClient();

  // ── Deep sleep trigger ─────────────────────────────────────────────────────
  if (deepSleepEnabled && triggerDeepSleepAfterPublish) {
    bool windowExpired  = (millis() > webUiExpiresAt);
    // Stealth timer wakes use a shorter flush window to minimise total wake time.
    unsigned long flushMs = stealthThisWake ? STEALTH_FLUSH_MS : 5000UL;
    bool flushComplete  = (millis() - lastSensorPublishTime > flushMs);
    if (windowExpired && flushComplete) {
      // Final MQTT flush before sleeping
      unsigned long t = millis();
      while (millis() - t < 1000) {
        mqttStd.loop(); mqttAIO.loop(); mqttUBI.loop();
        esp_task_wdt_reset(); delay(10);
      }
      triggerDeepSleepAfterPublish = false;
      goToDeepSleep();
    }
  }

  if (budget > 0) delay(budget);
}