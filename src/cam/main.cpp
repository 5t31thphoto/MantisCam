// ============================================================
//  MantisCam camera firmware  v1.1.0
//  One source for classic ESP32-CAM boards and ESP32-S3 boards (XIAO Sense).
//
//  Built once PER BOARD: the board is chosen at compile time with -DCAM_<ID>
//  (one PlatformIO env per board, e.g. cam-xiao_s3). No runtime pin guessing.
//
//  Boot sequence
//    1. Crash bookkeeping (RTC memory survives resets) -> safe mode after 3
//       crashes in a row, so a bad module/cable can never loop forever.
//    2. Apply settings the web flasher wrote into the "mcfg" partition.
//    3. Camera init with this board's pins (once).
//    4. Hotspot (always on, for the Core2) + optional home Wi-Fi, web servers.
//
//  Endpoints (port 80):  /  /status  /controls  /control  /snapshot
//                        /config  /restart  /update
//  MJPEG stream (port 8081):  /stream
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_partition.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "mcfg.h"

#ifndef FEATURE_CAMERA
#define FEATURE_CAMERA 0
#endif
#ifndef FEATURE_NAT
#define FEATURE_NAT 0
#endif
#ifndef FEATURE_OTA
#define FEATURE_OTA 0
#endif
#ifndef FEATURE_MDNS
#define FEATURE_MDNS 0
#endif
#ifndef FEATURE_LED
#define FEATURE_LED 0
#endif
#ifndef FEATURE_MOTION
#define FEATURE_MOTION 0
#endif

#if FEATURE_CAMERA
#include "esp_camera.h"
#include "img_converters.h"
#endif
#if FEATURE_OTA
#include <Update.h>
#endif
#if FEATURE_MDNS
#include <ESPmDNS.h>
#endif
#if FEATURE_NAT
#include "lwip/lwip_napt.h"
#endif

#define FW_VERSION "1.1.0"
#define LED_CHANNEL 7

#ifdef CONFIG_ESP_TASK_WDT_TIMEOUT_S
#define TWDT_DEFAULT_S CONFIG_ESP_TASK_WDT_TIMEOUT_S
#else
#define TWDT_DEFAULT_S 5
#endif

// Defaults - must match what the Core2 looks for out of the box.
#define DEF_AP_SSID  "Mantis_1_Hotspot"
#define DEF_AP_PASS  "mantis33"
#define DEF_NAME     "Mantis Cam"
#define DEF_ADMIN    "admin1234"

static constexpr uint32_t STABLE_AFTER_MS   = 60000;  // uptime that clears the crash counter
static constexpr uint32_t SAFE_MODE_CRASHES = 3;

Preferences prefs;
String cfgStSsid, cfgStPass, cfgApSsid, cfgApPass, cfgName, cfgAdmin;
String cfgFramesize;
int  cfgQuality = 12, cfgBright = 0;
bool cfgVflip = false, cfgHmirror = false;
int  g_baseV = 0, g_baseH = 0;   // sensor setting that gives an upright image

httpd_handle_t web_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

volatile bool  g_streaming = false;
volatile float g_fps       = 0.0f;
volatile bool  g_motion    = false;
uint32_t g_motionAt = 0;
size_t   g_lastLen  = 0;
bool g_cameraOk  = false;
bool g_natOn     = false;
bool g_staOk     = false;
bool g_safeMode  = false;
bool g_cfgApplied = false;
int  g_ledPin    = -1;
bool g_ledInv    = false;
const char *g_camState = "off";       // off | ok | init_failed | safe_mode | disabled
char g_resetReason[16] = "?";

// ---- Survives soft resets and panics (not power-off). ----
#define RTC_MAGIC 0x4D43414Du
RTC_NOINIT_ATTR static uint32_t r_magic;
RTC_NOINIT_ATTR static uint32_t r_badBoots;     // consecutive crashes before reaching "stable"
RTC_NOINIT_ATTR static int32_t  r_camInit;      // 1 while esp_camera_init() is running
static void resetBootState();

// ------------------------------------------------ board (compile time) ----
// Pin order: PWDN RESET XCLK SIOD SIOC Y9 Y8 Y7 Y6 Y5 Y4 Y3 Y2 VSYNC HREF PCLK
// Each env in platformio.ini defines exactly one CAM_<ID>. The ids match the
// board list in website/index.html (tools/check_consistency.py enforces it).
#if defined(CAM_XIAO_S3)
  #define CAM_ID "xiao_s3"
  #define CAM_NAME "Seeed XIAO ESP32S3 Sense"
  #define CAM_PINS {-1, -1, 10, 40, 39, 48, 11, 12, 14, 16, 18, 17, 15, 38, 47, 13}
  #define CAM_LED 21
  #define CAM_LED_INV 1
  #define CAM_CHIP_S3 1
#elif defined(CAM_S3_EYE)
  #define CAM_ID "s3_eye"
  #define CAM_NAME "ESP32-S3-EYE / Freenove S3 / AI-Thinker S3"
  #define CAM_PINS {-1, -1, 15, 4, 5, 16, 17, 18, 12, 10, 8, 9, 11, 6, 7, 13}
  #define CAM_LED -1
  #define CAM_BASE_VFLIP 1        // Espressif example flips the S3-EYE
  #define CAM_CHIP_S3 1
#elif defined(CAM_NULLLAB_S3)
  #define CAM_ID "nulllab_s3"
  #define CAM_NAME "nulllab ESP32-S3-CAM"
  #define CAM_PINS {-1, -1, 15, 4, 5, 16, 17, 18, 12, 10, 8, 9, 11, 6, 7, 13}
  #define CAM_LED 3
  #define CAM_CHIP_S3 1
#elif defined(CAM_FIREBEETLE_S3)
  #define CAM_ID "firebeetle_s3"
  #define CAM_NAME "DFRobot FireBeetle 2 ESP32-S3"
  #define CAM_PINS {-1, -1, 45, 1, 2, 48, 46, 8, 7, 4, 41, 40, 39, 6, 42, 5}
  #define CAM_LED -1
  #define CAM_CHIP_S3 1
#elif defined(CAM_DFR_AICAM_S3)
  #define CAM_ID "dfr_aicam_s3"
  #define CAM_NAME "DFRobot ESP32-S3 AI CAM"
  #define CAM_PINS {-1, -1, 5, 8, 9, 4, 6, 7, 14, 17, 21, 18, 16, 1, 2, 15}
  #define CAM_LED 3
  #define CAM_CHIP_S3 1
#elif defined(CAM_S3_CAM_LCD)
  #define CAM_ID "s3_cam_lcd"
  #define CAM_NAME "ESP32-S3-CAM (LCD variant)"
  #define CAM_PINS {-1, -1, 40, 17, 18, 39, 41, 42, 12, 3, 14, 47, 13, 21, 38, 11}
  #define CAM_LED -1
  #define CAM_CHIP_S3 1
#elif defined(CAM_AI_THINKER)
  #define CAM_ID "ai_thinker"
  #define CAM_NAME "AI-Thinker ESP32-CAM"
  #define CAM_PINS {32, -1, 0, 26, 27, 35, 34, 39, 36, 21, 19, 18, 5, 25, 23, 22}
  #define CAM_LED 4
#elif defined(CAM_ESP_EYE)
  #define CAM_ID "esp_eye"
  #define CAM_NAME "ESP-EYE"
  #define CAM_PINS {-1, -1, 4, 18, 23, 36, 37, 38, 39, 35, 14, 13, 34, 5, 27, 25}
  #define CAM_LED 22
#elif defined(CAM_WROVER_KIT)
  #define CAM_ID "wrover_kit"
  #define CAM_NAME "ESP32 WROVER-KIT / Freenove WROVER"
  #define CAM_PINS {-1, -1, 21, 26, 27, 35, 34, 39, 36, 19, 18, 5, 4, 25, 23, 22}
  #define CAM_LED -1
#elif defined(CAM_M5_PSRAM)
  #define CAM_ID "m5_psram"
  #define CAM_NAME "M5Stack Camera (PSRAM / Model A)"
  #define CAM_PINS {-1, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 32, 22, 26, 21}
  #define CAM_LED -1
#elif defined(CAM_M5_V2)
  #define CAM_ID "m5_v2"
  #define CAM_NAME "M5Stack Camera (V2 / Model B)"
  #define CAM_PINS {-1, 15, 27, 22, 23, 19, 36, 18, 39, 5, 34, 35, 32, 25, 26, 21}
  #define CAM_LED -1
#elif defined(CAM_M5_WIDE)
  #define CAM_ID "m5_wide"
  #define CAM_NAME "M5Stack Wide"
  #define CAM_PINS {-1, 15, 27, 22, 23, 19, 36, 18, 39, 5, 34, 35, 32, 25, 26, 21}
  #define CAM_LED 2
  #define CAM_BASE_VFLIP 1        // Espressif example: vflip + hmirror
  #define CAM_BASE_HMIRROR 1
#elif defined(CAM_M5_NOPSRAM)
  #define CAM_ID "m5_nopsram"
  #define CAM_NAME "M5Stack ESP32-CAM (no PSRAM)"
  #define CAM_PINS {-1, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 17, 22, 26, 21}
  #define CAM_LED -1
  #define CAM_BASE_VFLIP 1        // Espressif example: vflip + hmirror
  #define CAM_BASE_HMIRROR 1
#elif defined(CAM_TTGO_TJOURNAL)
  #define CAM_ID "ttgo_tjournal"
  #define CAM_NAME "TTGO T-Journal"
  #define CAM_PINS {0, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 17, 22, 26, 21}
  #define CAM_LED -1
#else
  #error "No camera board selected: build one of the cam-<board> envs in platformio.ini"
#endif
#ifndef CAM_LED_INV
#define CAM_LED_INV 0
#endif
#ifndef CAM_BASE_VFLIP
#define CAM_BASE_VFLIP 0
#endif
#ifndef CAM_BASE_HMIRROR
#define CAM_BASE_HMIRROR 0
#endif
// Sources for the pin tables:
//   Espressif arduino-esp32 CameraWebServer/camera_pins.h (classic boards,
//   ESP32S3_EYE, ESP32S3_CAM_LCD, DFRobot_FireBeetle2_ESP32S3),
//   Seeed wiki "Camera Usage in XIAO ESP32S3 Sense" (GPIO table),
//   DFRobot wiki DFR1154 "Timed camera" example (ESP32-S3 AI CAM).
#ifndef CAM_CHIP_S3
#define CAM_CHIP_S3 0
#endif
#if CAM_CHIP_S3 && !CONFIG_IDF_TARGET_ESP32S3
#error "This board is ESP32-S3: build it with an ESP32-S3 board env"
#endif
#if !CAM_CHIP_S3 && !CONFIG_IDF_TARGET_ESP32
#error "This board is classic ESP32: build it with an ESP32 board env"
#endif
static const int8_t CAM_PIN[16] = CAM_PINS;

// ------------------------------------------------ settings (NVS) ----
static bool validApPass(const String &s) { return s.length() >= 8 && s.length() <= 63; }
static bool validSsid(const String &s)   { return s.length() >= 1 && s.length() <= 32; }

static void loadConfig() {
  cfgStSsid    = prefs.getString("st_ssid", "");
  cfgStPass    = prefs.getString("st_pass", "");
  cfgApSsid    = prefs.getString("ap_ssid", DEF_AP_SSID);
  cfgApPass    = prefs.getString("ap_pass", DEF_AP_PASS);
  cfgName      = prefs.getString("name",    DEF_NAME);
  cfgAdmin     = prefs.getString("admin",   DEF_ADMIN);
  cfgFramesize = prefs.getString("framesize", "VGA");
  cfgQuality   = prefs.getInt("quality", 12);
  cfgBright    = prefs.getInt("brightness", 0);
  cfgVflip     = prefs.getBool("vflip", false);
  cfgHmirror   = prefs.getBool("hmirror", false);
  if (!validSsid(cfgApSsid)) cfgApSsid = DEF_AP_SSID;
  if (!validApPass(cfgApPass)) cfgApPass = DEF_AP_PASS;
  if (!cfgAdmin.length()) cfgAdmin = DEF_ADMIN;
}

static void resetBootState() {
  r_magic = RTC_MAGIC;
  r_badBoots = 0;
  r_camInit = 0;
}

// Copy one string field from the flasher JSON into NVS, with validation.
static void cfgPutStr(JsonDocument &doc, const char *key, bool (*ok)(const String &)) {
  JsonVariant v = doc[key];
  if (!v.is<const char *>()) return;
  String s = v.as<const char *>();
  if (ok && !ok(s)) {
    Serial.printf("[CFG] '%s' rejected (invalid value), keeping previous\n", key);
    return;
  }
  prefs.putString(key, s);
}
static bool validName(const String &s)  { return s.length() >= 1 && s.length() <= 31; }
static bool validAdmin(const String &s) { return s.length() >= 8 && s.length() <= 63; }
static bool validStSsid(const String &s){ return s.length() <= 32; }  // "" = no home Wi-Fi
static bool validStPass(const String &s){ return s.length() == 0 || (s.length() >= 8 && s.length() <= 63); }
static bool validAny(const String &s)   { return s.length() <= 48; }

// Settings written by the web flasher at install time.
static void applyFlashConfig() {
  const esp_partition_t *part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t) 0x40, "mcfg");
  if (!part) {
    Serial.println("[CFG] no 'mcfg' partition - flashed with an old image? using stored settings");
    return;
  }
  uint8_t *buf = (uint8_t *) malloc(MCFG_SIZE);
  if (!buf) return;
  if (esp_partition_read(part, 0, buf, MCFG_SIZE) != ESP_OK) { free(buf); return; }

  mcfg_header_t h;
  const uint8_t *json = NULL;
  mcfg_result_t r = mcfg_parse(buf, MCFG_SIZE, &h, &json);
  if (r == MCFG_EMPTY) { free(buf); return; }
  if (r != MCFG_OK) {
    Serial.printf("[CFG] flasher settings ignored: %s\n", mcfg_result_str(r));
    free(buf);
    return;
  }

  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, (const char *) json, h.len);
  if (e) {
    Serial.printf("[CFG] flasher settings JSON error: %s\n", e.c_str());
    free(buf);
    return;
  }

  if (doc["factory"] | false) prefs.clear();
  cfgPutStr(doc, "name",    validName);
  cfgPutStr(doc, "ap_ssid", validSsid);
  cfgPutStr(doc, "ap_pass", validApPass);
  cfgPutStr(doc, "st_ssid", validStSsid);
  cfgPutStr(doc, "st_pass", validStPass);
  cfgPutStr(doc, "admin",   validAdmin);
  cfgPutStr(doc, "framesize", validAny);
  if (doc["quality"].is<int>())    prefs.putInt("quality",    constrain(doc["quality"].as<int>(), 6, 40));
  if (doc["brightness"].is<int>()) prefs.putInt("brightness", constrain(doc["brightness"].as<int>(), -2, 2));
  if (doc["vflip"].is<bool>())     prefs.putBool("vflip",   doc["vflip"].as<bool>());
  if (doc["hmirror"].is<bool>())   prefs.putBool("hmirror", doc["hmirror"].as<bool>());
  prefs.putULong("cfg_nonce", h.nonce);

  // Wipe the sector so passwords don't linger outside NVS.
  esp_partition_erase_range(part, 0, MCFG_SIZE);
  free(buf);

  resetBootState();
  g_cfgApplied = true;
  Serial.printf("[CFG] applied flasher settings (id %08lx)\n", (unsigned long) h.nonce);
}

static bool checkAuth(httpd_req_t *req) {
  char hdr[65] = {0};
  if (httpd_req_get_hdr_value_str(req, "X-Admin-Pass", hdr, sizeof(hdr) - 1) == ESP_OK
      && cfgAdmin == String(hdr)) return true;
  httpd_resp_set_status(req, "401 Unauthorized");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, "unauthorized");
  return false;
}

static void urlDecode(char *s) {
  char *o = s;
  while (*s) {
    if (*s == '+') { *o++ = ' '; s++; }
    else if (*s == '%' && s[1] && s[2]) {
      char hex[3] = { s[1], s[2], 0 };
      *o++ = (char) strtol(hex, NULL, 16);
      s += 3;
    } else *o++ = *s++;
  }
  *o = 0;
}

// ------------------------------------------------ crash bookkeeping -
static void bootBookkeeping() {
  esp_reset_reason_t rr = esp_reset_reason();
  const char *rs = "other";
  switch (rr) {
    case ESP_RST_POWERON:  rs = "power_on"; break;
    case ESP_RST_EXT:      rs = "ext_pin";  break;
    case ESP_RST_SW:       rs = "software"; break;
    case ESP_RST_PANIC:    rs = "panic";    break;
    case ESP_RST_INT_WDT:  rs = "int_wdt";  break;
    case ESP_RST_TASK_WDT: rs = "task_wdt"; break;
    case ESP_RST_WDT:      rs = "wdt";      break;
    case ESP_RST_BROWNOUT: rs = "brownout"; break;
    case ESP_RST_DEEPSLEEP:rs = "deepsleep";break;
    default: break;
  }
  strncpy(g_resetReason, rs, sizeof(g_resetReason) - 1);

  // Power-on or the RESET button = the user asking for a fresh start.
  if (r_magic != RTC_MAGIC || rr == ESP_RST_POWERON || rr == ESP_RST_EXT) resetBootState();

  bool crashed = rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT ||
                 rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT;
  if (crashed || rr == ESP_RST_BROWNOUT) r_badBoots++;
  if (crashed && r_camInit == 1)
    Serial.println("[BOOT] last boot crashed inside camera init - check the camera module/ribbon");
  r_camInit = 0;
  g_safeMode = r_badBoots >= SAFE_MODE_CRASHES;

  Serial.printf("[BOOT] reset=%s  crashes-in-a-row=%lu%s\n", rs,
                (unsigned long) r_badBoots, g_safeMode ? "  -> SAFE MODE (camera off)" : "");
  if (rr == ESP_RST_BROWNOUT)
    Serial.println("[BOOT] brownout: use a better USB cable/port or a 5V 1A+ supply");
}

// ------------------------------------------------ camera ----------
#if FEATURE_CAMERA
static const char *sensorName() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return "none";
  switch (s->id.PID) {
    case OV2640_PID: return "OV2640";
    case OV3660_PID: return "OV3660";
    case OV5640_PID: return "OV5640";
    case OV7725_PID: return "OV7725";
    case GC2145_PID: return "GC2145";
    default:         return "unknown";
  }
}

static framesize_t fsFromName(const char *n) {
  if (!strcmp(n, "QQVGA")) return FRAMESIZE_QQVGA;
  if (!strcmp(n, "QVGA"))  return FRAMESIZE_QVGA;
  if (!strcmp(n, "CIF"))   return FRAMESIZE_CIF;
  if (!strcmp(n, "VGA"))   return FRAMESIZE_VGA;
  if (!strcmp(n, "SVGA"))  return FRAMESIZE_SVGA;
  if (!strcmp(n, "XGA"))   return FRAMESIZE_XGA;
  if (!strcmp(n, "HD"))    return FRAMESIZE_HD;
  if (!strcmp(n, "SXGA"))  return FRAMESIZE_SXGA;
  if (!strcmp(n, "UXGA"))  return FRAMESIZE_UXGA;
  if (!strcmp(n, "QXGA"))  return FRAMESIZE_QXGA;
  return FRAMESIZE_VGA;
}

static framesize_t clampFramesize(sensor_t *s, framesize_t fs) {
  if (!psramFound() && fs > FRAMESIZE_QVGA) fs = FRAMESIZE_QVGA;   // DRAM-only boards
  if (s && s->id.PID == OV7725_PID && fs > FRAMESIZE_VGA) fs = FRAMESIZE_VGA;   // OV7725 max
  if (s && (s->id.PID == OV2640_PID || s->id.PID == GC2145_PID) && fs > FRAMESIZE_UXGA)
    fs = FRAMESIZE_UXGA;
  return fs;
}

static void applyImageSettings() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  g_baseV = CAM_BASE_VFLIP;
  g_baseH = CAM_BASE_HMIRROR;
  int brightBias = 0;
  if (s->id.PID == OV3660_PID) {
    // Espressif/Seeed/DFRobot examples: OV3660 comes up upside-down, dark, oversaturated.
    g_baseV ^= 1;
    brightBias = 1;
    s->set_saturation(s, -2);
  }
  s->set_framesize(s, clampFramesize(s, fsFromName(cfgFramesize.c_str())));
  s->set_quality(s, constrain(cfgQuality, 6, 40));
  s->set_brightness(s, constrain(cfgBright + brightBias, -2, 2));
  s->set_vflip(s, g_baseV ^ (cfgVflip ? 1 : 0));
  s->set_hmirror(s, g_baseH ^ (cfgHmirror ? 1 : 0));
}

static void fillCameraConfig(camera_config_t *c) {
  const int8_t *p = CAM_PIN;
  memset(c, 0, sizeof(*c));
  c->pin_pwdn  = p[0];  c->pin_reset = p[1];  c->pin_xclk = p[2];
  c->pin_sccb_sda = p[3]; c->pin_sccb_scl = p[4];
  c->pin_d7 = p[5];  c->pin_d6 = p[6];  c->pin_d5 = p[7];  c->pin_d4 = p[8];
  c->pin_d3 = p[9];  c->pin_d2 = p[10]; c->pin_d1 = p[11]; c->pin_d0 = p[12];
  c->pin_vsync = p[13]; c->pin_href = p[14]; c->pin_pclk = p[15];
  c->ledc_channel = LEDC_CHANNEL_0;
  c->ledc_timer   = LEDC_TIMER_0;
  c->xclk_freq_hz = 20000000;
  c->pixel_format = PIXFORMAT_JPEG;
  c->grab_mode    = CAMERA_GRAB_LATEST;
  if (psramFound()) {
    // Allocate for the sensor's largest mode so the resolution can be raised
    // later without overflowing the frame buffers; driver clamps to sensor max.
    c->frame_size   = FRAMESIZE_UXGA;
    c->jpeg_quality = 10;
    c->fb_count     = 2;
    c->fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    c->frame_size   = FRAMESIZE_QVGA;
    c->jpeg_quality = 15;
    c->fb_count     = 1;
    c->fb_location  = CAMERA_FB_IN_DRAM;
  }
}

static bool startCamera() {
  Serial.printf("[CAM] init %s\n", CAM_NAME);
  camera_config_t config;
  fillCameraConfig(&config);
#if defined(CAM_ESP_EYE)
  pinMode(13, INPUT_PULLUP);   // per Espressif CameraWebServer example
  pinMode(14, INPUT_PULLUP);
#endif

  // A hang inside the driver becomes a reset (counted toward safe mode)
  // instead of a frozen board.
  r_camInit = 1;
  esp_task_wdt_init(20, true);
  esp_task_wdt_add(NULL);
  esp_err_t err = esp_camera_init(&config);
  esp_task_wdt_delete(NULL);
  esp_task_wdt_init(TWDT_DEFAULT_S, false);
  r_camInit = 0;

  if (err != ESP_OK) {
    Serial.printf("[CAM] init FAILED (0x%x). Check the camera ribbon is seated and that\n", err);
    Serial.println("[CAM] this firmware matches the board. Hotspot + web UI still running.");
    g_camState = "init_failed";
    return false;
  }
  g_ledPin = CAM_LED;
  g_ledInv = CAM_LED_INV;
  applyImageSettings();
  g_camState = "ok";
  return true;
}

#define PART_BOUNDARY "frameboundary"
static const char *STREAM_CTYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_PART  = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static const char *STREAM_BOUND = "\r\n--" PART_BOUNDARY "\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  if (!g_cameraOk) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera not running"); return ESP_FAIL; }
  camera_fb_t *fb = NULL;
  esp_err_t res = httpd_resp_set_type(req, STREAM_CTYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  g_streaming = true;
  uint32_t frames = 0, t0 = millis();
  size_t jlen = 0; uint8_t *jbuf = NULL; char part[80];
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; }
    else if (fb->format != PIXFORMAT_JPEG) {
      bool ok = frame2jpg(fb, 80, &jbuf, &jlen);
      esp_camera_fb_return(fb); fb = NULL;
      if (!ok) res = ESP_FAIL;
    } else { jlen = fb->len; jbuf = fb->buf; }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned) jlen);
      res = httpd_resp_send_chunk(req, part, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *) jbuf, jlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, STREAM_BOUND, strlen(STREAM_BOUND));
    if (fb) { esp_camera_fb_return(fb); fb = NULL; jbuf = NULL; }
    else if (jbuf) { free(jbuf); jbuf = NULL; }
    frames++;
    uint32_t dt = millis() - t0;
    if (dt >= 1000) { g_fps = frames * 1000.0f / dt; frames = 0; t0 = millis(); }
    if (res != ESP_OK) break;
  }
  g_streaming = false; g_fps = 0;
  return res;
}

static esp_err_t snapshot_handler(httpd_req_t *req) {
  if (!g_cameraOk) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "camera not running"); return ESP_FAIL; }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");
  esp_err_t r = httpd_resp_send(req, (const char *) fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return r;
}

// GET /controls -> current sensor state as JSON (UI reads this to sync sliders)
static esp_err_t controls_get_handler(httpd_req_t *req) {
  sensor_t *s = g_cameraOk ? esp_camera_sensor_get() : NULL;
  if (!s) { httpd_resp_send_500(req); return ESP_FAIL; }
  camera_status_t *st = &s->status;
  char b[640];
  snprintf(b, sizeof(b),
    "{\"framesize\":%d,\"quality\":%d,\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
    "\"sharpness\":%d,\"denoise\":%d,\"special_effect\":%d,\"wb_mode\":%d,\"awb\":%d,"
    "\"awb_gain\":%d,\"aec\":%d,\"aec2\":%d,\"ae_level\":%d,\"aec_value\":%d,\"agc\":%d,"
    "\"agc_gain\":%d,\"gainceiling\":%d,\"bpc\":%d,\"wpc\":%d,\"raw_gma\":%d,\"lenc\":%d,"
    "\"hmirror\":%d,\"vflip\":%d,\"dcw\":%d,\"colorbar\":%d}",
    st->framesize, st->quality, st->brightness, st->contrast, st->saturation,
    st->sharpness, st->denoise, st->special_effect, st->wb_mode, st->awb,
    st->awb_gain, st->aec, st->aec2, st->ae_level, st->aec_value, st->agc,
    st->agc_gain, st->gainceiling, st->bpc, st->wpc, st->raw_gma, st->lenc,
    st->hmirror ^ g_baseH, st->vflip ^ g_baseV, st->dcw, st->colorbar);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, b);
  return ESP_OK;
}

// GET /control?var=NAME&val=N -> set one sensor parameter.
// framesize/quality/brightness/vflip/hmirror are remembered across reboots.
static esp_err_t control_handler(httpd_req_t *req) {
  char q[128] = {0}, var[24] = {0}, val[24] = {0};
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK ||
      httpd_query_key_value(q, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(q, "val", val, sizeof(val)) != ESP_OK) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "need var & val");
    return ESP_FAIL;
  }
  int iv = atoi(val);
  sensor_t *s = g_cameraOk ? esp_camera_sensor_get() : NULL;
  int res = -1;
  if (s) {
    if      (!strcmp(var, "framesize")) {
      res = s->set_framesize(s, clampFramesize(s, fsFromName(val)));
      if (res == 0) prefs.putString("framesize", val);
    }
    else if (!strcmp(var, "quality"))        { res = s->set_quality(s, iv);    if (!res) prefs.putInt("quality", iv); }
    else if (!strcmp(var, "brightness"))     { res = s->set_brightness(s, iv); if (!res) prefs.putInt("brightness", iv); }
    else if (!strcmp(var, "hmirror"))        { res = s->set_hmirror(s, g_baseH ^ (iv ? 1 : 0)); if (!res) prefs.putBool("hmirror", iv != 0); }
    else if (!strcmp(var, "vflip"))          { res = s->set_vflip(s, g_baseV ^ (iv ? 1 : 0));   if (!res) prefs.putBool("vflip", iv != 0); }
    else if (!strcmp(var, "contrast"))       res = s->set_contrast(s, iv);
    else if (!strcmp(var, "saturation"))     res = s->set_saturation(s, iv);
    else if (!strcmp(var, "sharpness"))      res = s->set_sharpness(s, iv);
    else if (!strcmp(var, "denoise"))        res = s->set_denoise(s, iv);
    else if (!strcmp(var, "special_effect")) res = s->set_special_effect(s, iv);
    else if (!strcmp(var, "wb_mode"))        res = s->set_wb_mode(s, iv);
    else if (!strcmp(var, "awb"))            res = s->set_whitebal(s, iv);
    else if (!strcmp(var, "awb_gain"))       res = s->set_awb_gain(s, iv);
    else if (!strcmp(var, "aec"))            res = s->set_exposure_ctrl(s, iv);
    else if (!strcmp(var, "aec2"))           res = s->set_aec2(s, iv);
    else if (!strcmp(var, "ae_level"))       res = s->set_ae_level(s, iv);
    else if (!strcmp(var, "aec_value"))      res = s->set_aec_value(s, iv);
    else if (!strcmp(var, "agc"))            res = s->set_gain_ctrl(s, iv);
    else if (!strcmp(var, "agc_gain"))       res = s->set_agc_gain(s, iv);
    else if (!strcmp(var, "gainceiling"))    res = s->set_gainceiling(s, (gainceiling_t) iv);
    else if (!strcmp(var, "bpc"))            res = s->set_bpc(s, iv);
    else if (!strcmp(var, "wpc"))            res = s->set_wpc(s, iv);
    else if (!strcmp(var, "raw_gma"))        res = s->set_raw_gma(s, iv);
    else if (!strcmp(var, "lenc"))           res = s->set_lenc(s, iv);
    else if (!strcmp(var, "dcw"))            res = s->set_dcw(s, iv);
    else if (!strcmp(var, "colorbar"))       res = s->set_colorbar(s, iv);
  }
#if FEATURE_LED
  if (!strcmp(var, "led")) {
    if (g_ledPin >= 0) {
      int duty = constrain(iv, 0, 255);
      ledcWrite(LED_CHANNEL, g_ledInv ? 255 - duty : duty);
      res = 0;
    }
  }
#endif
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char out[48];
  snprintf(out, sizeof(out), "{\"ok\":%s}", res == 0 ? "true" : "false");
  httpd_resp_sendstr(req, out);
  return ESP_OK;
}
#endif // FEATURE_CAMERA

// ------------------------------------------------ status API ------
static esp_err_t status_handler(httpd_req_t *req) {
  char buf[1200];
  String staIp = WiFi.localIP().toString();
  String apIp  = WiFi.softAPIP().toString();
  const char *sensor = "none";
  float fps = 0; bool cam = false, motion = false;
#if FEATURE_CAMERA
  sensor = g_cameraOk ? sensorName() : "init_failed";
  cam = g_cameraOk; fps = g_fps;
#endif
#if FEATURE_MOTION
  motion = g_motion && (millis() - g_motionAt < 5000);
#endif
  snprintf(buf, sizeof(buf),
    "{\"fw\":\"%s\",\"name\":\"%s\",\"chip\":\"%s\",\"mac\":\"%s\","
    "\"uptime_s\":%lu,\"heap\":%u,\"psram\":%u,\"flash_mb\":%u,\"rssi\":%d,"
    "\"sta_connected\":%s,\"sta_ip\":\"%s\",\"sta_ssid\":\"%s\","
    "\"ap_ssid\":\"%s\",\"ap_ip\":\"%s\",\"ap_clients\":%d,"
    "\"nat\":%s,\"camera\":%s,\"cam_state\":\"%s\",\"sensor\":\"%s\",\"profile\":\"%s\","
    "\"board\":\"%s\",\"led_pin\":%d,\"fps\":%.1f,\"streaming\":%s,\"motion\":%s,"
    "\"safe_mode\":%s,\"reset_reason\":\"%s\",\"cfg_applied\":%s,\"stream_port\":8081}",
    FW_VERSION, cfgName.c_str(), ESP.getChipModel(), WiFi.macAddress().c_str(),
    (unsigned long)(millis() / 1000), (unsigned) ESP.getFreeHeap(),
    (unsigned) ESP.getPsramSize(), (unsigned) (ESP.getFlashChipSize() / (1024 * 1024)), WiFi.RSSI(),
    g_staOk ? "true" : "false", staIp.c_str(), cfgStSsid.c_str(),
    cfgApSsid.c_str(), apIp.c_str(), WiFi.softAPgetStationNum(),
    g_natOn ? "true" : "false", cam ? "true" : "false", g_camState, sensor, CAM_NAME,
    CAM_ID, g_ledPin, fps, g_streaming ? "true" : "false", motion ? "true" : "false",
    g_safeMode ? "true" : "false", g_resetReason, g_cfgApplied ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, buf);
  return ESP_OK;
}

// ------------------------------------------------ config / restart -
static esp_err_t config_post_handler(httpd_req_t *req) {
  if (!checkAuth(req)) return ESP_FAIL;
  char body[512] = {0};
  int len = httpd_req_recv(req, body, sizeof(body) - 1);
  if (len <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
  struct { const char *k; bool (*ok)(const String &); } keys[] = {
    {"st_ssid", validStSsid}, {"st_pass", validStPass}, {"ap_ssid", validSsid},
    {"ap_pass", validApPass}, {"name", validName}, {"admin", validAdmin},
  };
  char v[128];
  for (auto &k : keys) {
    if (httpd_query_key_value(body, k.k, v, sizeof(v)) == ESP_OK) {
      urlDecode(v);
      if (k.ok(String(v))) prefs.putString(k.k, v);
    }
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, "{\"ok\":true,\"note\":\"rebooting\"}");
  delay(300);
  esp_restart();
  return ESP_OK;
}

static esp_err_t restart_handler(httpd_req_t *req) {
  if (!checkAuth(req)) return ESP_FAIL;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, "{\"ok\":true}");
  delay(300);
  esp_restart();
  return ESP_OK;
}

// ------------------------------------------------ index viewer ----
static esp_err_t index_handler(httpd_req_t *req) {
  static const char INDEX_HTML[] = R"HTMLPAGE(<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MantisCam</title><style>
body{margin:0;background:#050a0a;color:#e8fff0;font-family:system-ui,Arial,sans-serif;text-align:center}
header{padding:12px;font-size:18px;font-weight:bold;background:#0a1212;color:#50c85a;border-bottom:3px solid #5D005D}
#v{max-width:100%;height:auto;background:#000;display:block;margin:0 auto}
.bar{padding:12px;display:flex;gap:8px;justify-content:center;flex-wrap:wrap}
button,a.btn{background:#004d4d;color:#fff;border:1px solid #007373;padding:11px 18px;font-size:15px;text-decoration:none;cursor:pointer}
#st,#diag{font-size:12px;color:#7a9a96;padding:6px}
</style></head><body>
<header id="hdr">MantisCam</header>
<img id="v" alt="loading stream...">
<div class="bar">
<button onclick="reload()">Reload</button>
<a class="btn" id="snap" target="_blank">Snapshot</a>
<button onclick="full()">Fullscreen</button>
</div>
<div id="st">connecting...</div>
<div id="diag"></div>
<script>
var host=location.hostname;
var stream="http://"+host+":8081/stream";
var v=document.getElementById('v');
document.getElementById('snap').href="http://"+host+"/snapshot";
function reload(){v.src=stream+"?t="+Date.now();}
function full(){if(v.requestFullscreen){v.requestFullscreen();}}
v.onload=function(){document.getElementById('st').textContent="live \u2022 "+stream;};
v.onerror=function(){document.getElementById('st').textContent="no stream \u2014 see camera state below";};
reload();
fetch("http://"+host+"/status").then(function(r){return r.json();}).then(function(d){
document.getElementById('hdr').textContent=d.name+"  ("+d.sensor+" / "+d.profile+")";
document.getElementById('diag').textContent="fw "+d.fw+" \u2022 "+d.chip+" \u2022 camera: "+d.cam_state+
 " \u2022 psram "+Math.round(d.psram/1048576)+"MB \u2022 last reset: "+d.reset_reason+(d.safe_mode?" \u2022 SAFE MODE":"");
}).catch(function(e){});
</script>
</body></html>)HTMLPAGE";
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_sendstr(req, INDEX_HTML);
}

// ------------------------------------------------ OTA -------------
#if FEATURE_OTA
static esp_err_t update_post_handler(httpd_req_t *req) {
  if (!checkAuth(req)) return ESP_FAIL;
  int remaining = req->content_len;
  if (remaining <= 0 || !Update.begin(UPDATE_SIZE_UNKNOWN)) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  char buf[1024];
  while (remaining > 0) {
    int r = httpd_req_recv(req, buf, remaining < (int) sizeof(buf) ? remaining : sizeof(buf));
    if (r <= 0) { Update.abort(); httpd_resp_send_500(req); return ESP_FAIL; }
    if (Update.write((uint8_t *) buf, r) != (size_t) r) {
      Update.abort(); httpd_resp_send_500(req); return ESP_FAIL;
    }
    remaining -= r;
  }
  if (!Update.end(true)) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, "{\"ok\":true,\"note\":\"flashed, rebooting\"}");
  delay(500);
  esp_restart();
  return ESP_OK;
}
#endif

// ------------------------------------------------ NAT + DNS fix ----
#if FEATURE_NAT
static void enableNAT() {
#if IP_NAPT
  esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_netif_t *ap  = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  esp_netif_dns_info_t dns;
  if (sta && ap && esp_netif_get_dns_info(sta, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK) {
    esp_netif_dhcps_stop(ap);
    esp_netif_set_dns_info(ap, ESP_NETIF_DNS_MAIN, &dns);
    uint8_t offer_dns = 2;  // OFFER_DNS
    esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER,
                           &offer_dns, sizeof(offer_dns));
    esp_netif_dhcps_start(ap);
  }
  ip_napt_enable((uint32_t) WiFi.softAPIP(), 1);
  g_natOn = true;
  Serial.println("[NAT] NAPT enabled with upstream DNS offered to clients.");
#else
  Serial.println("[NAT] Core built without IP_NAPT - NAT disabled.");
#endif
}
#endif

// ------------------------------------------------ servers ----------
static void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port   = 32080;
  cfg.max_uri_handlers = 14;
  cfg.stack_size  = 8192;           // /status builds a ~1 KB JSON with float formatting
  if (httpd_start(&web_httpd, &cfg) == ESP_OK) {
    httpd_uri_t u;
    u = { "/",         HTTP_GET,  index_handler,       NULL }; httpd_register_uri_handler(web_httpd, &u);
    u = { "/status",   HTTP_GET,  status_handler,      NULL }; httpd_register_uri_handler(web_httpd, &u);
    u = { "/config",   HTTP_POST, config_post_handler, NULL }; httpd_register_uri_handler(web_httpd, &u);
    u = { "/restart",  HTTP_POST, restart_handler,     NULL }; httpd_register_uri_handler(web_httpd, &u);
#if FEATURE_OTA
    u = { "/update",   HTTP_POST, update_post_handler, NULL }; httpd_register_uri_handler(web_httpd, &u);
#endif
#if FEATURE_CAMERA
    u = { "/snapshot", HTTP_GET,  snapshot_handler,    NULL }; httpd_register_uri_handler(web_httpd, &u);
    u = { "/control",  HTTP_GET,  control_handler,     NULL }; httpd_register_uri_handler(web_httpd, &u);
    u = { "/controls", HTTP_GET,  controls_get_handler,NULL }; httpd_register_uri_handler(web_httpd, &u);
#endif
  }
#if FEATURE_CAMERA
  httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
  scfg.server_port = 8081;
  scfg.ctrl_port   = 32081;
  scfg.stack_size  = 6144;
  if (httpd_start(&stream_httpd, &scfg) == ESP_OK) {
    httpd_uri_t su = { "/stream", HTTP_GET, stream_handler, NULL };
    httpd_register_uri_handler(stream_httpd, &su);
  }
#endif
}

// ------------------------------------------------ setup / loop -----
static void printBootBanner() {
  Serial.printf("\n[BOOT] MantisCam camera v%s for %s\n", FW_VERSION, CAM_NAME);
  Serial.printf("[BOOT] chip %s rev %d, %d cores, flash %u MB, PSRAM %s (%u KB)\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
                (unsigned) (ESP.getFlashChipSize() / (1024 * 1024)),
                psramFound() ? "yes" : "NO", (unsigned) (ESP.getPsramSize() / 1024));
  uint64_t m = ESP.getEfuseMac();   // read from eFuse; Wi-Fi isn't started yet
  Serial.printf("[BOOT] MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                (int) (m & 0xFF), (int) ((m >> 8) & 0xFF), (int) ((m >> 16) & 0xFF),
                (int) ((m >> 24) & 0xFF), (int) ((m >> 32) & 0xFF), (int) ((m >> 40) & 0xFF));
}

void setup() {
  Serial.begin(115200);
  delay(300);   // give USB-CDC hosts (XIAO) a moment to attach
  printBootBanner();
  bootBookkeeping();

  prefs.begin("camcfg", false);
  applyFlashConfig();
  loadConfig();
  Serial.printf("[CFG] name='%s' hotspot='%s' home-wifi='%s'\n",
                cfgName.c_str(), cfgApSsid.c_str(),
                cfgStSsid.length() ? cfgStSsid.c_str() : "(none)");

#if FEATURE_CAMERA
  if (g_safeMode) {
    g_camState = "safe_mode";
    Serial.println("[CAM] SAFE MODE: camera skipped after repeated crashes.");
    Serial.println("[CAM] it re-arms after 60 s of uptime; reboot or re-flash settings to retry.");
  } else {
    g_cameraOk = startCamera();
    if (g_cameraOk)
      Serial.printf("[CAM] OK - %s, sensor %s\n", CAM_NAME, sensorName());
  }
#else
  g_camState = "disabled";
#endif
#if FEATURE_LED
  if (g_ledPin >= 0) {
    ledcSetup(LED_CHANNEL, 5000, 8);
    ledcAttachPin(g_ledPin, LED_CHANNEL);
    ledcWrite(LED_CHANNEL, g_ledInv ? 255 : 0);
  }
#endif

  // Hotspot first - it's how the Core2 finds us. Home Wi-Fi is optional.
  WiFi.persistent(false);
  bool wantSta = cfgStSsid.length() > 0;
  WiFi.mode(wantSta ? WIFI_AP_STA : WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 5, 1), IPAddress(192, 168, 5, 1),
                    IPAddress(255, 255, 255, 0));
  if (!WiFi.softAP(cfgApSsid.c_str(), cfgApPass.c_str())) {
    Serial.println("[WIFI] hotspot settings rejected - falling back to defaults");
    WiFi.softAP(DEF_AP_SSID, DEF_AP_PASS);
  }
  Serial.printf("[WIFI] hotspot '%s' at %s\n", WiFi.softAPSSID().c_str(),
                WiFi.softAPIP().toString().c_str());
  if (wantSta) {
    WiFi.setAutoReconnect(true);
    WiFi.begin(cfgStSsid.c_str(), cfgStPass.c_str());
    Serial.printf("[WIFI] joining home Wi-Fi '%s' in the background\n", cfgStSsid.c_str());
  }

#if FEATURE_MDNS
  String host = cfgName; host.toLowerCase(); host.replace(" ", "-");
  if (MDNS.begin(host.c_str())) {
    MDNS.addService("esp32cam", "tcp", 80);
    MDNS.addServiceTxt("esp32cam", "tcp", "fw", FW_VERSION);
    Serial.printf("[MDNS] http://%s.local\n", host.c_str());
  }
#endif

  startServers();
  Serial.println("[HTTP] UI: http://192.168.5.1/   Stream: http://192.168.5.1:8081/stream");
}

void loop() {
  // Clear the crash counter once we've been up long enough to call it stable.
  static bool stable = false;
  if (!stable && millis() > STABLE_AFTER_MS) {
    stable = true;
    r_badBoots = 0;
  }

  // Home Wi-Fi state changes (connects in the background).
  bool sta = WiFi.status() == WL_CONNECTED;
  if (sta != g_staOk) {
    g_staOk = sta;
    if (sta) {
      Serial.printf("[WIFI] home Wi-Fi up: %s\n", WiFi.localIP().toString().c_str());
#if FEATURE_NAT
      if (!g_natOn) enableNAT();
#endif
    } else {
      Serial.println("[WIFI] home Wi-Fi down (hotspot unaffected)");
    }
  }

#if FEATURE_MOTION && FEATURE_CAMERA
  static uint32_t last = 0;
  if (g_cameraOk && !g_streaming && millis() - last > 1000) {
    last = millis();
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      if (g_lastLen > 0) {
        size_t diff = fb->len > g_lastLen ? fb->len - g_lastLen : g_lastLen - fb->len;
        if (diff * 100 > g_lastLen * 12) { g_motion = true; g_motionAt = millis(); }
      }
      g_lastLen = fb->len;
      esp_camera_fb_return(fb);
    }
  }
#endif
  delay(100);
}
