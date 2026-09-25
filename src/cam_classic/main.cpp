// ============================================================
//  ESP32-CAM Universal Firmware  v1.0.0  (generated)
//  Primary profile: AI-Thinker ESP32-CAM
//  Endpoints (port 80):  /  /status  /controls  /control  /snapshot
//                        /config  /restart  /update
//  MJPEG stream (port 8081):  /stream
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_netif.h"

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

#define FW_VERSION "1.0.0"
#define LED_CHANNEL 7

Preferences prefs;
String cfgStSsid, cfgStPass, cfgApSsid, cfgApPass, cfgName, cfgAdmin;

httpd_handle_t web_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

volatile bool  g_streaming = false;
volatile float g_fps       = 0.0f;
volatile bool  g_motion    = false;
uint32_t g_motionAt = 0;
size_t   g_lastLen  = 0;
bool g_cameraOk = false;
bool g_natOn    = false;
bool g_staOk    = false;
int  g_ledPin   = -1;
char g_activeProfile[48] = "none";

// ---- Camera pin-profile table (auto-probe order). Order per row:
//   PWDN RESET XCLK SIOD SIOC Y9 Y8 Y7 Y6 Y5 Y4 Y3 Y2 VSYNC HREF PCLK , LED
typedef struct { const char *name; int8_t p[16]; int8_t led; } cam_profile_t;
static const cam_profile_t CAM_PROFILES[] = {
    {"AI-Thinker ESP32-CAM", {32, -1, 0, 26, 27, 35, 34, 39, 36, 21, 19, 18, 5, 25, 23, 22}, 4},
    {"ESP-EYE (ESP32)", {-1, -1, 4, 18, 23, 36, 37, 38, 39, 35, 14, 13, 34, 5, 27, 25}, 22},
    {"ESP32 WROVER-KIT / Freenove WROVER", {-1, -1, 21, 26, 27, 35, 34, 39, 36, 19, 18, 5, 4, 25, 23, 22}, -1},
    {"M5Stack Camera (PSRAM / Model A)", {-1, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 32, 22, 26, 21}, -1},
    {"M5Stack Camera (V2 / Model B)", {-1, 15, 27, 22, 23, 19, 36, 18, 39, 5, 34, 35, 32, 25, 26, 21}, -1},
    {"M5Stack Wide", {-1, 15, 27, 22, 23, 19, 36, 18, 39, 5, 34, 35, 32, 25, 26, 21}, 2},
    {"M5Stack ESP32-CAM (no PSRAM)", {-1, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 17, 22, 26, 21}, -1},
    {"TTGO T-Journal", {0, 15, 27, 25, 23, 19, 36, 18, 39, 5, 34, 35, 17, 22, 26, 21}, -1},
};
static const int CAM_PROFILE_COUNT = sizeof(CAM_PROFILES) / sizeof(CAM_PROFILES[0]);

// ------------------------------------------------ config (NVS) ----
static void loadConfig() {
  prefs.begin("camcfg", false);
  cfgStSsid = prefs.getString("st_ssid", "");
  cfgStPass = prefs.getString("st_pass", "");
  cfgApSsid = prefs.getString("ap_ssid", "Mantis_1_Hotspot");
  cfgApPass = prefs.getString("ap_pass", "mantis33");
  cfgName   = prefs.getString("name",    "Mantis Cam");
  cfgAdmin  = prefs.getString("admin",   "admin1234");
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

static void applyCameraPins(camera_config_t *config, const cam_profile_t *pr) {
  config->pin_pwdn  = pr->p[0];  config->pin_reset = pr->p[1];
  config->pin_xclk  = pr->p[2];
  config->pin_sccb_sda = pr->p[3]; config->pin_sccb_scl = pr->p[4];
  config->pin_d7 = pr->p[5];  config->pin_d6 = pr->p[6];  config->pin_d5 = pr->p[7];
  config->pin_d4 = pr->p[8];  config->pin_d3 = pr->p[9];  config->pin_d2 = pr->p[10];
  config->pin_d1 = pr->p[11]; config->pin_d0 = pr->p[12];
  config->pin_vsync = pr->p[13]; config->pin_href = pr->p[14]; config->pin_pclk = pr->p[15];
}

// Tries the profile table in order; first one whose sensor probes is kept.
static bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  if (psramFound()) {
    config.frame_size  = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count    = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size  = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count    = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  for (int i = 0; i < CAM_PROFILE_COUNT; i++) {
    applyCameraPins(&config, &CAM_PROFILES[i]);
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
      strncpy(g_activeProfile, CAM_PROFILES[i].name, sizeof(g_activeProfile) - 1);
      g_ledPin = CAM_PROFILES[i].led;
      prefs.putString("camprofile", g_activeProfile);
      Serial.printf("[CAM] profile matched: %s\n", g_activeProfile);
      // OV3660/OV5640 boot a touch dark; nudge for typical indoor light.
      sensor_t *s = esp_camera_sensor_get();
      if (s && (s->id.PID == OV3660_PID || s->id.PID == OV5640_PID)) {
        s->set_brightness(s, 1);
        s->set_saturation(s, -1);
      }
      return true;
    }
    Serial.printf("[CAM] profile '%s' failed (0x%x), trying next...\n",
                  CAM_PROFILES[i].name, err);
    esp_camera_deinit();
  }
  return false;
}

#define PART_BOUNDARY "frameboundary"
static const char *STREAM_CTYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_PART  = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static const char *STREAM_BOUND = "\r\n--" PART_BOUNDARY "\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
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
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");
  esp_err_t r = httpd_resp_send(req, (const char *) fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return r;
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

// GET /controls -> current sensor state as JSON (UI reads this to sync sliders)
static esp_err_t controls_get_handler(httpd_req_t *req) {
  sensor_t *s = esp_camera_sensor_get();
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
    st->hmirror, st->vflip, st->dcw, st->colorbar);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, b);
  return ESP_OK;
}

// GET /control?var=NAME&val=N -> set one sensor parameter
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
  sensor_t *s = esp_camera_sensor_get();
  int res = -1;
  if (s) {
    if      (!strcmp(var, "framesize"))      res = s->set_framesize(s, fsFromName(val));
    else if (!strcmp(var, "quality"))        res = s->set_quality(s, iv);
    else if (!strcmp(var, "brightness"))     res = s->set_brightness(s, iv);
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
    else if (!strcmp(var, "hmirror"))        res = s->set_hmirror(s, iv);
    else if (!strcmp(var, "vflip"))          res = s->set_vflip(s, iv);
    else if (!strcmp(var, "dcw"))            res = s->set_dcw(s, iv);
    else if (!strcmp(var, "colorbar"))       res = s->set_colorbar(s, iv);
  }
#if FEATURE_LED
  if (!strcmp(var, "led")) {
    if (g_ledPin >= 0) { ledcWrite(LED_CHANNEL, constrain(iv, 0, 255)); res = 0; }
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
  char buf[900];
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
    "\"uptime_s\":%lu,\"heap\":%u,\"psram\":%u,\"rssi\":%d,"
    "\"sta_connected\":%s,\"sta_ip\":\"%s\",\"sta_ssid\":\"%s\","
    "\"ap_ssid\":\"%s\",\"ap_ip\":\"%s\",\"ap_clients\":%d,"
    "\"nat\":%s,\"camera\":%s,\"sensor\":\"%s\",\"profile\":\"%s\","
    "\"led_pin\":%d,\"fps\":%.1f,\"streaming\":%s,\"motion\":%s,\"stream_port\":8081}",
    FW_VERSION, cfgName.c_str(), ESP.getChipModel(), WiFi.macAddress().c_str(),
    (unsigned long)(millis() / 1000), (unsigned) ESP.getFreeHeap(),
    (unsigned) ESP.getPsramSize(), WiFi.RSSI(),
    g_staOk ? "true" : "false", staIp.c_str(), cfgStSsid.c_str(),
    cfgApSsid.c_str(), apIp.c_str(), WiFi.softAPgetStationNum(),
    g_natOn ? "true" : "false", cam ? "true" : "false", sensor, g_activeProfile,
    g_ledPin, fps, g_streaming ? "true" : "false", motion ? "true" : "false");
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
  const char *keys[] = {"st_ssid", "st_pass", "ap_ssid", "ap_pass", "name", "admin"};
  char v[128];
  for (auto k : keys) {
    if (httpd_query_key_value(body, k, v, sizeof(v)) == ESP_OK) {
      urlDecode(v);
      prefs.putString(k, v);
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
<title>ESP32-CAM</title><style>
body{margin:0;background:#111;color:#eee;font-family:system-ui,Arial,sans-serif;text-align:center}
header{padding:12px;font-size:18px;font-weight:bold;background:#1e2430;color:#7fd4ff}
#v{max-width:100%;height:auto;background:#000;display:block;margin:0 auto}
.bar{padding:12px;display:flex;gap:8px;justify-content:center;flex-wrap:wrap}
button,a.btn{background:#2c3a52;color:#fff;border:0;padding:11px 18px;border-radius:6px;font-size:15px;text-decoration:none;cursor:pointer}
#st{font-size:12px;color:#8aa;padding:8px}
</style></head><body>
<header id="hdr">ESP32-CAM</header>
<img id="v" alt="loading stream...">
<div class="bar">
<button onclick="reload()">Reload</button>
<a class="btn" id="snap" target="_blank">Snapshot</a>
<button onclick="full()">Fullscreen</button>
</div>
<div id="st">connecting...</div>
<script>
var host=location.hostname;
var stream="http://"+host+":8081/stream";
var v=document.getElementById('v');
document.getElementById('snap').href="http://"+host+"/snapshot";
function reload(){v.src=stream+"?t="+Date.now();}
function full(){if(v.requestFullscreen){v.requestFullscreen();}}
v.onload=function(){document.getElementById('st').textContent="live \u2022 "+stream;};
v.onerror=function(){document.getElementById('st').textContent="stream error \u2014 is the Camera feature enabled in this build?";};
reload();
fetch("http://"+host+"/status").then(function(r){return r.json();}).then(function(d){
document.getElementById('hdr').textContent=d.name+"  ("+d.sensor+" / "+d.profile+")";
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
  if (httpd_start(&stream_httpd, &scfg) == ESP_OK) {
    httpd_uri_t su = { "/stream", HTTP_GET, stream_handler, NULL };
    httpd_register_uri_handler(stream_httpd, &su);
  }
#endif
}

// ------------------------------------------------ setup / loop -----
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[BOOT] ESP32-CAM Universal Firmware v%s\n", FW_VERSION);
  loadConfig();

#if FEATURE_CAMERA
  g_cameraOk = initCamera();
  if (g_cameraOk) {
    Serial.printf("[CAM] OK - profile '%s', sensor %s\n", g_activeProfile, sensorName());
  } else {
    Serial.println("[CAM] init FAILED for every known profile. Wrong board or bad cable.");
  }
#endif
#if FEATURE_LED
  if (g_ledPin >= 0) {
    ledcSetup(LED_CHANNEL, 5000, 8);
    ledcAttachPin(g_ledPin, LED_CHANNEL);
    ledcWrite(LED_CHANNEL, 0);
  }
#endif

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(IPAddress(192, 168, 5, 1), IPAddress(192, 168, 5, 1),
                    IPAddress(255, 255, 255, 0));
  WiFi.softAP(cfgApSsid.c_str(), cfgApPass.c_str());
  WiFi.begin(cfgStSsid.c_str(), cfgStPass.c_str());

  Serial.print("[WIFI] Joining upstream");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  g_staOk = WiFi.status() == WL_CONNECTED;
  if (g_staOk) {
    Serial.print("[WIFI] STA IP: ");
    Serial.println(WiFi.localIP());
#if FEATURE_NAT
    enableNAT();
#endif
  } else {
    Serial.println("[WIFI] Upstream unreachable - AP-only (portable) mode.");
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
  Serial.printf("[HTTP] UI: http://%s/   Stream: http://%s:8081/stream\n",
                WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
}

void loop() {
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
