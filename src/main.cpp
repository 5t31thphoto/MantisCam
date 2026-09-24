/*
 * MANTIS Core2 v3 — multi-camera profiles, zone-aligned buttons, SD media
 * BtnA left | BtnB middle | BtnC right  — labels drawn under each zone
 */

#include <M5Unified.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <JPEGDEC.h>
#include <SD.h>
#include <SPI.h>
#include "mantis_bitmap.h"

static constexpr int CONTENT_TOP = 28;
static constexpr int CONTENT_BOTTOM = 210;  // footer starts here
static constexpr int CONTENT_H = CONTENT_BOTTOM - CONTENT_TOP; // 182
static constexpr int CONTENT_W = 320;

static const uint16_t COL_BG=0x0A20, COL_PANEL=0x1A40, COL_ACCENT=0x5FE0,
                      COL_TEXT=0xE7FF, COL_DIM=0x7BEF, COL_ALERT=0xF800,
                      COL_OK=0x07E0, COL_BTN=0x2A60, COL_REC=0xF800, COL_HI=0x3C80;

enum Screen : uint8_t {
  SCR_HOME=0, SCR_CAMS, SCR_STREAM, SCR_SNAPSHOT, SCR_CAMCTRL,
  SCR_WIFI, SCR_AUDIO_REC, SCR_AUDIO_PLAY, SCR_STATUS, SCR_COUNT
};
static const char *SCR_NAMES[] = {
  "Home","Cameras","Stream","Snapshot","Cam Ctrl",
  "WiFi","Audio Rec","Audio Play","Status"
};

static constexpr int SD_CS=4, SD_SCK=18, SD_MISO=38, SD_MOSI=23;

// ---- camera profile (one SoftAP target) ----
struct CamProfile {
  char label[24];
  char ssid[33];
  char pass[65];
  char ip[16];
  bool used;
};
static constexpr int MAX_PROFILES = 8;
CamProfile g_profiles[MAX_PROFILES];
int  g_profileCount = 0;
int  g_activeProfile = -1;   // index into g_profiles
int  g_camListSel = 0;

Preferences prefs;
String cfgSsid="Mantis_1_Hotspot", cfgPass="mantis33", cfgCamIp="192.168.5.1", cfgAdmin="admin1234";

Screen g_screen=SCR_HOME;
bool g_wifiOk=false, g_sdOk=false, g_needRedraw=true;
uint32_t g_lastBtnMs=0;
JPEGDEC jpeg;
uint8_t *g_jpgBuf=nullptr;
size_t g_jpgCap=96*1024, g_jpgLen=0;
bool g_hasSnapPreview=false;   // true after capture, until saved or left screen

bool g_recActive=false;
uint32_t g_recStartMs=0;
static constexpr uint32_t REC_MAX_MS=30000;
File g_recFile;
uint32_t g_recFrames=0;

bool g_oskActive=false;
String *g_oskTarget=nullptr;
String g_oskTitle;
char g_oskBuf[64]={0};
int g_oskCursor=0;

int g_quality=12, g_bright=0, g_led=0;
bool g_hmirror=false, g_vflip=false;

bool g_audioRecActive=false;
uint32_t g_audioRecStart=0;
static constexpr uint32_t AUDIO_MAX_MS=20000;
static constexpr int AUDIO_SR=16000, AUDIO_CHUNK=512;
int16_t *g_audioBuf=nullptr;
size_t g_audioBufSamples=0;
String g_audioList[32];
int g_audioCount=0, g_audioSel=0;
bool g_audioPlaying=false;

// scan results (SSID list from WiFi.scan)
String g_scanSsid[16];
int g_scanCount=0;
bool g_scanning=false;

void loadConfig(); void saveConfig();
void loadProfiles(); void saveProfiles();
void connectToProfile(int idx);
void connectWifi(const char *ssid, const char *pass);
bool initSD(); void ensureDirs();
void drawHeader(const char *t); void drawFooter(const char *a, const char *b, const char *c);
void drawHome(); void drawCams(); void drawStream(); void drawSnapshot();
void drawCamCtrl(); void drawWifi(); void drawAudioRec(); void drawAudioPlay(); void drawStatus(); void drawOsk();
void drawMicScope();
void drawSplash();
void decodeJpegFit(uint8_t *buf, size_t len);
void handleButtons(); void handleTouch();
bool httpGetJson(const String &path, JsonDocument &doc);
bool httpControl(const char *var, int val);
bool fetchSnapshot();  // capture + preview only
bool saveLastSnapshot(); // write preview buffer to /images
void streamFrameTick();
void startVideoRec(); void stopVideoRec();
void startAudioRec(); void stopAudioRec();
void writeWavHeader(File &f, uint32_t dataBytes);
void scanAudioFiles(); void playSelectedAudio();
void startWifiScan(); void applyScanToProfiles();
int jpegDrawCallback(JPEGDRAW *p);
bool addOrUpdateProfile(const char *label, const char *ssid, const char *pass, const char *ip);

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  M5.Display.setRotation(1);
  M5.Display.setBrightness(180);
  M5.Display.fillScreen(COL_BG);

  if (psramFound()) {
    g_jpgBuf = (uint8_t*)ps_malloc(g_jpgCap);
    g_audioBuf = (int16_t*)ps_malloc(AUDIO_SR * 22 * sizeof(int16_t));
  }
  if (!g_jpgBuf) { g_jpgCap=48*1024; g_jpgBuf=(uint8_t*)malloc(g_jpgCap); }
  if (!g_audioBuf) g_audioBuf=(int16_t*)malloc(AUDIO_SR*12*sizeof(int16_t));

  for (int i=0;i<MAX_PROFILES;i++) g_profiles[i].used=false;
  loadConfig();
  loadProfiles();
  // seed default profile if empty
  if (g_profileCount==0) {
    addOrUpdateProfile("Mantis 1", "Mantis_1_Hotspot", "mantis33", "192.168.5.1");
    saveProfiles();
  }
  g_sdOk = initSD();
  if (g_sdOk) ensureDirs();

  // connect to active or first profile
  int idx = (g_activeProfile>=0) ? g_activeProfile : 0;
  if (g_profiles[idx].used) connectToProfile(idx);
  g_needRedraw=true;
}

void loop() {
  M5.update();
  handleButtons();
  handleTouch();

  if (g_recActive && millis()-g_recStartMs>=REC_MAX_MS) { stopVideoRec(); g_needRedraw=true; }
  if (g_audioRecActive && millis()-g_audioRecStart>=AUDIO_MAX_MS) { stopAudioRec(); g_needRedraw=true; }

  if (g_oskActive) {
    if (g_needRedraw) { drawOsk(); g_needRedraw=false; }
    delay(20); return;
  }

  if (g_needRedraw) {
    M5.Display.fillScreen(COL_BG);
    switch (g_screen) {
      case SCR_HOME: drawHome(); break;
      case SCR_CAMS: drawCams(); break;
      case SCR_STREAM: drawStream(); break;
      case SCR_SNAPSHOT: drawSnapshot(); break;
      case SCR_CAMCTRL: drawCamCtrl(); break;
      case SCR_WIFI: drawWifi(); break;
      case SCR_AUDIO_REC: drawAudioRec(); break;
      case SCR_AUDIO_PLAY: drawAudioPlay(); break;
      case SCR_STATUS: drawStatus(); break;
      default: break;
    }
    g_needRedraw=false;
  }

  if (g_screen==SCR_STREAM && g_wifiOk) streamFrameTick();
  // live scope refresh on audio screen
  static uint32_t lastScope;
  if (g_screen == SCR_AUDIO_REC && !g_oskActive && millis() - lastScope > 80) {
    lastScope = millis();
    drawMicScope();
  }

  if (g_audioRecActive && g_audioBuf) {
    static int16_t chunk[AUDIO_CHUNK];
    if (M5.Mic.isEnabled() && M5.Mic.record(chunk, AUDIO_CHUNK, AUDIO_SR)) {
      size_t maxS = AUDIO_SR*22;
      if (g_audioBufSamples+AUDIO_CHUNK < maxS) {
        memcpy(g_audioBuf+g_audioBufSamples, chunk, AUDIO_CHUNK*sizeof(int16_t));
        g_audioBufSamples += AUDIO_CHUNK;
      }
    }
  }
  delay(10);
}

// ---------------- config / profiles ----------------
void loadConfig() {
  prefs.begin("mantis", true);
  cfgSsid  = prefs.getString("ssid", "Mantis_1_Hotspot");
  cfgPass  = prefs.getString("pass", "mantis33");
  cfgCamIp = prefs.getString("camip", "192.168.5.1");
  cfgAdmin = prefs.getString("admin", "admin1234");
  g_activeProfile = prefs.getInt("active", -1);
  prefs.end();
}
void saveConfig() {
  prefs.begin("mantis", false);
  prefs.putString("ssid", cfgSsid);
  prefs.putString("pass", cfgPass);
  prefs.putString("camip", cfgCamIp);
  prefs.putString("admin", cfgAdmin);
  prefs.putInt("active", g_activeProfile);
  prefs.end();
}

void loadProfiles() {
  prefs.begin("camprofs", true);
  g_profileCount = prefs.getInt("n", 0);
  if (g_profileCount > MAX_PROFILES) g_profileCount = MAX_PROFILES;
  for (int i=0;i<g_profileCount;i++) {
    char k[16];
    snprintf(k,sizeof(k),"l%d",i); String L=prefs.getString(k,"");
    snprintf(k,sizeof(k),"s%d",i); String S=prefs.getString(k,"");
    snprintf(k,sizeof(k),"p%d",i); String P=prefs.getString(k,"");
    snprintf(k,sizeof(k),"i%d",i); String I=prefs.getString(k,"192.168.5.1");
    strncpy(g_profiles[i].label, L.c_str(), 23);
    strncpy(g_profiles[i].ssid,  S.c_str(), 32);
    strncpy(g_profiles[i].pass,  P.c_str(), 64);
    strncpy(g_profiles[i].ip,    I.c_str(), 15);
    g_profiles[i].used = (S.length()>0);
  }
  prefs.end();
}

void saveProfiles() {
  prefs.begin("camprofs", false);
  int n=0;
  for (int i=0;i<MAX_PROFILES;i++) if (g_profiles[i].used) n++;
  prefs.putInt("n", n);
  int slot=0;
  for (int i=0;i<MAX_PROFILES;i++) {
    if (!g_profiles[i].used) continue;
    char k[16];
    snprintf(k,sizeof(k),"l%d",slot); prefs.putString(k, g_profiles[i].label);
    snprintf(k,sizeof(k),"s%d",slot); prefs.putString(k, g_profiles[i].ssid);
    snprintf(k,sizeof(k),"p%d",slot); prefs.putString(k, g_profiles[i].pass);
    snprintf(k,sizeof(k),"i%d",slot); prefs.putString(k, g_profiles[i].ip);
    slot++;
  }
  g_profileCount = n;
  prefs.end();
}

bool addOrUpdateProfile(const char *label, const char *ssid, const char *pass, const char *ip) {
  // update if ssid exists
  for (int i=0;i<MAX_PROFILES;i++) {
    if (g_profiles[i].used && strcmp(g_profiles[i].ssid, ssid)==0) {
      strncpy(g_profiles[i].label, label, 23);
      strncpy(g_profiles[i].pass, pass, 64);
      strncpy(g_profiles[i].ip, ip, 15);
      return true;
    }
  }
  for (int i=0;i<MAX_PROFILES;i++) {
    if (!g_profiles[i].used) {
      strncpy(g_profiles[i].label, label, 23);
      strncpy(g_profiles[i].ssid, ssid, 32);
      strncpy(g_profiles[i].pass, pass, 64);
      strncpy(g_profiles[i].ip, ip, 15);
      g_profiles[i].used = true;
      g_profileCount++;
      return true;
    }
  }
  return false;
}

void connectToProfile(int idx) {
  if (idx<0 || idx>=MAX_PROFILES || !g_profiles[idx].used) return;
  // disconnect current
  WiFi.disconnect(true);
  delay(200);
  g_wifiOk=false;
  g_activeProfile = idx;
  cfgSsid = g_profiles[idx].ssid;
  cfgPass = g_profiles[idx].pass;
  cfgCamIp = g_profiles[idx].ip;
  saveConfig();
  connectWifi(g_profiles[idx].ssid, g_profiles[idx].pass);
}


void drawSplash() {
  M5.Display.fillScreen(COL_BG);
  // pixel mantis centered
  int x0 = (320 - MANTIS_W) / 2;
  int y0 = 36;
  for (int y = 0; y < MANTIS_H; y++) {
    for (int x = 0; x < MANTIS_W; x++) {
      uint16_t c = MANTIS_PIX[y * MANTIS_W + x];
      if (c) M5.Display.drawPixel(x0 + x, y0 + y, c);
    }
  }
  M5.Display.setTextColor(COL_ACCENT);
  M5.Display.setTextSize(2);
  int tw = 9 * 12; // approx
  M5.Display.setCursor((320 - 9 * 12) / 2, y0 + MANTIS_H + 12);
  M5.Display.print("MantisCam");
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(COL_DIM);
  M5.Display.setCursor(100, y0 + MANTIS_H + 40);
  M5.Display.print("connecting...");
}

void connectWifi(const char *ssid, const char *pass) {
  drawSplash();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
  }
  g_wifiOk = (WiFi.status() == WL_CONNECTED);
  if (g_wifiOk && g_activeProfile >= 0) {
    JsonDocument doc;
    if (httpGetJson("/status", doc)) {
      const char *nm = doc["name"] | "";
      if (nm[0]) {
        strncpy(g_profiles[g_activeProfile].label, nm, 23);
        saveProfiles();
      }
    }
  }
  if (!g_wifiOk) {
    // timeout → Cameras screen
    g_screen = SCR_CAMS;
  }
  g_needRedraw = true;
}

bool initSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI, 25000000)) return false;
  return true;
}
void ensureDirs() {
  if (!SD.exists("/images")) SD.mkdir("/images");
  if (!SD.exists("/videos")) SD.mkdir("/videos");
  if (!SD.exists("/audio"))  SD.mkdir("/audio");
}

void startWifiScan() {
  g_scanning=true;
  g_needRedraw=true;
  // draw will show scanning; actual scan in handler after redraw request
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks(false, false, false, 300);
  g_scanCount=0;
  for (int i=0;i<n && g_scanCount<16;i++) {
    String s = WiFi.SSID(i);
    // prefer names that look like camera hotspots
    if (s.length()==0) continue;
    g_scanSsid[g_scanCount++] = s;
  }
  g_scanning=false;
  applyScanToProfiles();
  g_needRedraw=true;
}

void applyScanToProfiles() {
  // auto-add SSIDs containing Mantis / ESP32 / CAM if not already profiles
  for (int i=0;i<g_scanCount;i++) {
    String s = g_scanSsid[i];
    bool interesting = s.startsWith("Mantis") || s.indexOf("CAM")>=0 || s.indexOf("ESP32")>=0
                       || s.indexOf("Hotspot")>=0;
    if (!interesting) continue;
    bool exists=false;
    for (int p=0;p<MAX_PROFILES;p++)
      if (g_profiles[p].used && s.equals(g_profiles[p].ssid)) { exists=true; break; }
    if (exists) continue;
    // default password guess for Mantis fleet; user can edit on WiFi screen
    const char *pw = "mantis33";
    char label[24];
    snprintf(label, sizeof(label), "%s", s.c_str());
    addOrUpdateProfile(label, s.c_str(), pw, "192.168.5.1");
  }
  saveProfiles();
}

// ---------------- UI ----------------
void drawHeader(const char *t) {
  M5.Display.fillRect(0,0,320,26,COL_PANEL);
  M5.Display.setTextColor(COL_ACCENT);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(6,7);
  M5.Display.printf("MANTIS  %s", t);
  M5.Display.setCursor(200,7);
  M5.Display.setTextColor(g_wifiOk?COL_OK:COL_ALERT);
  M5.Display.print(g_wifiOk?"WiFi":"--");
  M5.Display.setCursor(245,7);
  M5.Display.setTextColor(g_sdOk?COL_OK:COL_ALERT);
  M5.Display.print(g_sdOk?"SD":"--");
  if (g_activeProfile>=0 && g_profiles[g_activeProfile].used) {
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(6, 27);
    // thin strip under header for active cam name
  }
}

// Zone-aligned footer: three equal columns under A / B / C capacitive pads
void drawFooter(const char *aLabel, const char *bLabel, const char *cLabel) {
  M5.Display.fillRect(0, 210, 320, 30, COL_PANEL);
  // vertical guides matching button zones (~0-106, 107-213, 214-319)
  M5.Display.drawFastVLine(106, 210, 30, 0x1A30);
  M5.Display.drawFastVLine(213, 210, 30, 0x1A30);
  M5.Display.setTextSize(1);
  auto center = [](const char *s, int zone) {
    // zone 0=A, 1=B, 2=C ; each ~106 px wide
    int x0 = zone * 107;
    int w = 106;
    int tw = strlen(s) * 6;
    int x = x0 + (w - tw) / 2;
    if (x < x0+2) x = x0+2;
    M5.Display.setCursor(x, 220);
    M5.Display.print(s);
  };
  M5.Display.setTextColor(COL_TEXT);
  center(aLabel, 0);
  center(bLabel, 1);
  center(cLabel, 2);
}

void drawHome() {
  drawHeader("Home");
  M5.Display.setTextColor(COL_ACCENT);
  M5.Display.setTextSize(3);
  M5.Display.setCursor(36, 50);
  M5.Display.print("MANTIS");
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.setCursor(36, 95);
  M5.Display.print("Multi-camera companion");
  if (g_activeProfile>=0 && g_profiles[g_activeProfile].used) {
    M5.Display.setCursor(36, 120);
    M5.Display.printf("Active: %s", g_profiles[g_activeProfile].label);
    M5.Display.setCursor(36, 138);
    M5.Display.printf("SSID: %s", g_profiles[g_activeProfile].ssid);
  }
  M5.Display.setTextColor(COL_DIM);
  M5.Display.setCursor(36, 165);
  M5.Display.print("B = Cameras list");
  drawFooter("< Prev", "Cameras", "Next >");
}

void drawCams() {
  drawHeader("Cameras");
  M5.Display.setTextColor(COL_DIM);
  M5.Display.setCursor(8, 32);
  M5.Display.print("Tap a camera  |  B=connect  |  Scan adds profiles");

  int y = 50;
  int shown=0;
  for (int i=0;i<MAX_PROFILES && shown<6;i++) {
    if (!g_profiles[i].used) continue;
    bool sel = (shown == g_camListSel);
    bool act = (i == g_activeProfile);
    uint16_t bg = sel ? COL_HI : COL_PANEL;
    M5.Display.fillRoundRect(8, y, 304, 24, 4, bg);
    M5.Display.setTextColor(act ? COL_OK : COL_TEXT);
    M5.Display.setCursor(14, y+6);
    M5.Display.printf("%s%s", g_profiles[i].label, act?"  [ON]":"");
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(180, y+6);
    M5.Display.print(g_profiles[i].ssid);
    y += 28;
    shown++;
  }
  if (shown==0) {
    M5.Display.setTextColor(COL_ALERT);
    M5.Display.setCursor(20, 80);
    M5.Display.print("No profiles — press C to Scan");
  }
  // Scan soft button
  M5.Display.fillRoundRect(8, 180, 148, 26, 4, COL_BTN);
  M5.Display.setTextColor(COL_ACCENT);
  M5.Display.setCursor(40, 187);
  M5.Display.print(g_scanning ? "Scanning..." : "Scan WiFi");
  M5.Display.fillRoundRect(164, 180, 148, 26, 4, COL_BTN);
  M5.Display.setCursor(190, 187);
  M5.Display.print("Save active");

  drawFooter("< Prev", "Connect", "Next >");
}

void drawStream() {
  drawHeader("Stream");
  M5.Display.fillRect(0, CONTENT_TOP, CONTENT_W, CONTENT_H, COL_BG);
  if (g_activeProfile>=0 && g_profiles[g_activeProfile].used) {
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(8, CONTENT_TOP + 2);
    M5.Display.print(g_profiles[g_activeProfile].label);
  }
  if (!g_wifiOk) {
    M5.Display.setTextColor(COL_ALERT);
    M5.Display.setCursor(30, 100);
    M5.Display.print("Not connected — use Cameras");
  } else if (g_recActive) {
    uint32_t sec=(millis()-g_recStartMs)/1000;
    M5.Display.fillRoundRect(10, CONTENT_TOP + 4, 130, 18, 3, COL_REC);
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(16, CONTENT_TOP + 8);
    M5.Display.printf("REC %02lu:%02lu f:%lu", sec/60, sec%60, (unsigned long)g_recFrames);
  } else {
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(80, 100);
    M5.Display.print("Waiting for frames...");
  }
  drawFooter("< Prev", g_recActive?"STOP":"REC", "Next >");
}

void drawSnapshot() {
  drawHeader("Snapshot");
  // If we already have a preview in the buffer, re-decode it onto the content area
  if (g_hasSnapPreview && g_jpgBuf && g_jpgLen > 100) {
    decodeJpegFit(g_jpgBuf, g_jpgLen);
// Save button overlaid near bottom content area
    M5.Display.fillRoundRect(20, 178, 130, 28, 5, COL_BTN);
    M5.Display.setTextColor(COL_ACCENT);
    M5.Display.setCursor(40, 186);
    M5.Display.print("Save to SD");
    M5.Display.fillRoundRect(170, 178, 130, 28, 5, COL_PANEL);
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(200, 186);
    M5.Display.print("Retake=B");
  } else {
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(20, 70);
    M5.Display.print("B = capture preview from CAM");
    M5.Display.setCursor(20, 95);
    M5.Display.setTextColor(COL_DIM);
    M5.Display.print("Then tap Save to SD to write");
    M5.Display.setCursor(20, 112);
    M5.Display.print("/images/snap_....jpg");
    if (!g_sdOk) {
      M5.Display.setTextColor(COL_ALERT);
      M5.Display.setCursor(20, 140);
      M5.Display.print("SD not mounted — preview still works");
    }
  }
  drawFooter("< Prev", "Capture", "Next >");
}

void drawCamCtrl() {
  drawHeader("Cam Ctrl");
  M5.Display.setTextColor(COL_TEXT);
  auto row=[&](int y,const char*l,int v){
    M5.Display.fillRoundRect(10,y,300,30,4,COL_PANEL);
    M5.Display.setCursor(18,y+9); M5.Display.printf("%s %d",l,v);
    M5.Display.setTextColor(COL_ACCENT); M5.Display.setCursor(200,y+9); M5.Display.print("- / +");
    M5.Display.setTextColor(COL_TEXT);
  };
  row(50,"Quality",g_quality);
  row(88,"Bright",g_bright);
  row(126,"LED",g_led);
  M5.Display.fillRoundRect(10,164,145,28,4,COL_PANEL);
  M5.Display.setCursor(22,172); M5.Display.printf("Mirror %s", g_hmirror?"ON":"off");
  M5.Display.fillRoundRect(165,164,145,28,4,COL_PANEL);
  M5.Display.setCursor(178,172); M5.Display.printf("Flip %s", g_vflip?"ON":"off");
  drawFooter("< Prev", "LED", "Next >");
}

void drawWifi() {
  drawHeader("WiFi / Edit");
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.fillRoundRect(10,40,300,36,4,COL_PANEL);
  M5.Display.setCursor(18,45); M5.Display.setTextColor(COL_DIM); M5.Display.print("SSID (tap)");
  M5.Display.setCursor(18,60); M5.Display.setTextColor(COL_ACCENT); M5.Display.print(cfgSsid);
  M5.Display.fillRoundRect(10,86,300,36,4,COL_PANEL);
  M5.Display.setCursor(18,91); M5.Display.setTextColor(COL_DIM); M5.Display.print("Password (tap)");
  M5.Display.setCursor(18,106); M5.Display.setTextColor(COL_ACCENT); M5.Display.print(cfgPass);
  M5.Display.fillRoundRect(10,132,300,36,4,COL_PANEL);
  M5.Display.setCursor(18,137); M5.Display.setTextColor(COL_DIM); M5.Display.print("CAM IP (tap)");
  M5.Display.setCursor(18,152); M5.Display.setTextColor(COL_ACCENT); M5.Display.print(cfgCamIp);
  M5.Display.fillRoundRect(10,176,300,28,4,COL_BTN);
  M5.Display.setCursor(70,184); M5.Display.setTextColor(COL_TEXT); M5.Display.print("Apply to active profile");
  drawFooter("< Prev", "Edit SSID", "Next >");
}

void drawMicScope() {
  // live waveform in content area
  static int16_t scope[160];
  static int scopeIdx = 0;
  int16_t chunk[AUDIO_CHUNK];
  bool clipped = false;
  if (M5.Mic.isEnabled()) {
    // peek one block if possible — while recording data already flows in loop
  }
  // draw from recent g_audioBuf tail or zero
  int samples = 160;
  int16_t peak = 0;
  if (g_audioBuf && g_audioBufSamples > samples) {
    size_t start = g_audioBufSamples - samples;
    for (int i = 0; i < samples; i++) {
      int16_t v = g_audioBuf[start + i];
      scope[i] = v;
      int16_t a = v < 0 ? -v : v;
      if (a > peak) peak = a;
      if (a > 28000) clipped = true;
    }
  } else if (g_audioRecActive && M5.Mic.isEnabled()) {
    if (M5.Mic.record(chunk, AUDIO_CHUNK, AUDIO_SR)) {
      for (int i = 0; i < 160 && i < AUDIO_CHUNK; i++) {
        scope[i] = chunk[i];
        int16_t a = chunk[i] < 0 ? -chunk[i] : chunk[i];
        if (a > peak) peak = a;
        if (a > 28000) clipped = true;
      }
    }
  } else {
    for (int i = 0; i < 160; i++) scope[i] = 0;
  }

  int boxY = CONTENT_TOP + 8;
  int boxH = 90;
  M5.Display.fillRect(10, boxY, 300, boxH, COL_PANEL);
  int mid = boxY + boxH / 2;
  M5.Display.drawFastHLine(10, mid, 300, COL_DIM);
  for (int i = 1; i < 160; i++) {
    int x0 = 10 + (i - 1) * 300 / 160;
    int x1 = 10 + i * 300 / 160;
    int y0 = mid - (int)scope[i - 1] * (boxH / 2 - 4) / 32768;
    int y1 = mid - (int)scope[i] * (boxH / 2 - 4) / 32768;
    M5.Display.drawLine(x0, y0, x1, y1, clipped ? COL_ALERT : COL_ACCENT);
  }
  // clipping indicator
  if (clipped) {
    M5.Display.fillRoundRect(240, boxY + 4, 64, 16, 3, COL_ALERT);
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(248, boxY + 7);
    M5.Display.print("CLIP!");
  } else {
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(240, boxY + 7);
    M5.Display.printf("pk:%d", (int)peak);
  }
}

void drawAudioRec() {
  drawHeader("Audio Rec");
  // enable mic for scope even when not recording
  if (!g_audioRecActive) {
    M5.Speaker.end();
    if (!M5.Mic.isEnabled()) M5.Mic.begin();
  }
  drawMicScope();
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.setCursor(16, CONTENT_TOP + 105);
  M5.Display.print("Mic -> /audio/*.wav  (max 20s)");
  if (g_audioRecActive) {
    uint32_t sec = (millis() - g_audioRecStart) / 1000;
    M5.Display.fillRoundRect(60, CONTENT_TOP + 125, 200, 36, 8, COL_REC);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(90, CONTENT_TOP + 133);
    M5.Display.printf("REC %02lu:%02lu", sec / 60, sec % 60);
    M5.Display.setTextSize(1);
  } else {
    M5.Display.fillRoundRect(60, CONTENT_TOP + 125, 200, 36, 8, COL_BTN);
    M5.Display.setTextColor(COL_ACCENT);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(95, CONTENT_TOP + 133);
    M5.Display.print("B = REC");
    M5.Display.setTextSize(1);
  }
  drawFooter("< Prev", g_audioRecActive ? "Stop" : "REC", "Next >");
}

void drawAudioPlay() {
  drawHeader("Audio Play");
  if (!g_sdOk) {
    M5.Display.setTextColor(COL_ALERT); M5.Display.setCursor(30,100); M5.Display.print("No SD");
    drawFooter("< Prev", "Play", "Next >"); return;
  }
  scanAudioFiles();
  if (g_audioCount==0) {
    M5.Display.setTextColor(COL_DIM); M5.Display.setCursor(30,100); M5.Display.print("No files in /audio");
  } else {
    for (int i=0;i<g_audioCount && i<6;i++) {
      int y=48+i*24;
      if (i==g_audioSel) M5.Display.fillRoundRect(8,y-2,304,22,3,COL_HI);
      M5.Display.setTextColor(COL_TEXT); M5.Display.setCursor(14,y+4); M5.Display.print(g_audioList[i]);
    }
  }
  drawFooter("< Prev", "Play", "Next >");
}

void drawStatus() {
  drawHeader("Status");
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.setCursor(16,40);
  M5.Display.printf("WiFi %s  IP %s", g_wifiOk?"OK":"down", WiFi.localIP().toString().c_str());
  M5.Display.setCursor(16,58);
  M5.Display.printf("RSSI %d  SD %s", WiFi.RSSI(), g_sdOk?"yes":"no");
  M5.Display.setCursor(16,76);
  M5.Display.printf("Heap %uKB  PSRAM %uKB", ESP.getFreeHeap()/1024, ESP.getPsramSize()/1024);
  if (g_activeProfile>=0 && g_profiles[g_activeProfile].used) {
    M5.Display.setCursor(16,94);
    M5.Display.printf("Profile: %s", g_profiles[g_activeProfile].label);
  }
  if (g_wifiOk) {
    JsonDocument doc;
    if (httpGetJson("/status", doc)) {
      M5.Display.setCursor(16,120);
      M5.Display.printf("CAM %s  %s", doc["fw"]|"?", doc["sensor"]|"?");
      M5.Display.setCursor(16,138);
      M5.Display.printf("heap %u  fps %.1f", (unsigned)(doc["heap"]|0)/1024, (double)(doc["fps"]|0.0));
    }
  }
  drawFooter("< Prev", "Refresh", "Next >");
}

void drawOsk() {
  M5.Display.fillScreen(COL_BG);
  M5.Display.fillRect(0,0,320,48,COL_PANEL);
  M5.Display.setTextColor(COL_ACCENT); M5.Display.setCursor(8,6); M5.Display.print(g_oskTitle);
  M5.Display.setCursor(8,26); M5.Display.setTextColor(COL_TEXT);
  String shown=String(g_oskBuf);
  M5.Display.print(shown.substring(0,g_oskCursor));
  M5.Display.setTextColor(COL_ACCENT); M5.Display.print("|");
  M5.Display.setTextColor(COL_TEXT); M5.Display.print(shown.substring(g_oskCursor));
  static const char *rows[4]={"1234567890","qwertyuiop","asdfghjkl-","zxcvbnm._@"};
  for (int r=0;r<4;r++) for (int c=0;c<10;c++) {
    int x=c*32, y=54+r*36;
    M5.Display.fillRoundRect(x+1,y+1,30,32,3,COL_BTN);
    M5.Display.setTextColor(COL_TEXT); M5.Display.setCursor(x+10,y+10); M5.Display.print(rows[r][c]);
  }
  drawFooter("Left", "OK", "Right");
}

// ---------------- input ----------------
void handleButtons() {
  if (millis()-g_lastBtnMs<220) return;

  if (M5.BtnA.wasPressed()) {
    g_lastBtnMs=millis();
    if (g_oskActive) { if (g_oskCursor>0) g_oskCursor--; g_needRedraw=true; return; }
    if (g_recActive) stopVideoRec();
    if (g_audioRecActive) stopAudioRec();
    if (g_audioPlaying) { M5.Speaker.stop(); g_audioPlaying=false; }
    // A always changes screen (never stolen by Cameras list)
    g_screen = (Screen)((g_screen+SCR_COUNT-1)%SCR_COUNT);
    g_needRedraw=true;
  }

  if (M5.BtnC.wasPressed()) {
    g_lastBtnMs=millis();
    if (g_oskActive) { if (g_oskCursor<(int)strlen(g_oskBuf)) g_oskCursor++; g_needRedraw=true; return; }
    if (g_recActive) stopVideoRec();
    if (g_audioRecActive) stopAudioRec();
    if (g_audioPlaying) { M5.Speaker.stop(); g_audioPlaying=false; }
    // C always changes screen
    g_screen = (Screen)((g_screen+1)%SCR_COUNT);
    g_needRedraw=true;
  }

  if (M5.BtnB.wasPressed()) {
    g_lastBtnMs=millis();
    if (g_oskActive) {
      if (g_oskTarget) *g_oskTarget = String(g_oskBuf);
      g_oskActive=false; g_oskTarget=nullptr;
      saveConfig();
      // also push into active profile if editing wifi fields
      if (g_activeProfile>=0 && g_profiles[g_activeProfile].used) {
        strncpy(g_profiles[g_activeProfile].ssid, cfgSsid.c_str(), 32);
        strncpy(g_profiles[g_activeProfile].pass, cfgPass.c_str(), 64);
        strncpy(g_profiles[g_activeProfile].ip, cfgCamIp.c_str(), 15);
        saveProfiles();
      }
      g_needRedraw=true; return;
    }
    switch (g_screen) {
      case SCR_HOME: g_screen=SCR_CAMS; g_needRedraw=true; break;
      case SCR_CAMS: {
        int used[MAX_PROFILES], n=0;
        for (int i=0;i<MAX_PROFILES;i++) if (g_profiles[i].used) used[n++]=i;
        if (n>0 && g_camListSel<n) connectToProfile(used[g_camListSel]);
        g_needRedraw=true;
      } break;
      case SCR_STREAM:
        if (g_recActive) stopVideoRec(); else startVideoRec();
        g_needRedraw=true; break;
      case SCR_SNAPSHOT: fetchSnapshot(); g_needRedraw=true; break;
      case SCR_CAMCTRL: g_led=g_led?0:180; httpControl("led",g_led); g_needRedraw=true; break;
      case SCR_WIFI:
        g_oskTarget=&cfgSsid; g_oskTitle="Edit SSID";
        strncpy(g_oskBuf,cfgSsid.c_str(),sizeof(g_oskBuf)-1);
        g_oskCursor=strlen(g_oskBuf); g_oskActive=true; g_needRedraw=true; break;
      case SCR_AUDIO_REC:
        if (g_audioRecActive) stopAudioRec(); else startAudioRec();
        g_needRedraw=true; break;
      case SCR_AUDIO_PLAY: playSelectedAudio(); g_needRedraw=true; break;
      case SCR_STATUS: g_needRedraw=true; break;
      default: break;
    }
  }
}

void handleTouch() {
  auto t=M5.Touch.getDetail();
  if (!t.wasPressed()) return;

  if (g_oskActive) {
    int col=t.x/32, row=(t.y-54)/36;
    if (row<0||row>3||col<0||col>9) return;
    static const char *rows[4]={"1234567890","qwertyuiop","asdfghjkl-","zxcvbnm._@"};
    char ch=rows[row][col];
    if (g_oskCursor<(int)sizeof(g_oskBuf)-1) {
      memmove(g_oskBuf+g_oskCursor+1, g_oskBuf+g_oskCursor, strlen(g_oskBuf)-g_oskCursor+1);
      g_oskBuf[g_oskCursor++]=ch;
    }
    g_needRedraw=true; return;
  }

  if (g_screen==SCR_CAMS) {
    // list hits
    int y=50, shown=0;
    for (int i=0;i<MAX_PROFILES;i++) {
      if (!g_profiles[i].used) continue;
      if (t.y>=y && t.y<y+24) {
        g_camListSel=shown;
        connectToProfile(i);
        g_needRedraw=true; return;
      }
      y+=28; shown++;
    }
    if (t.y>=180 && t.y<206) {
      if (t.x<160) { startWifiScan(); }
      else if (g_activeProfile>=0) {
        // save active into list (already there) — refresh label from status
        if (g_wifiOk) {
          JsonDocument doc;
          if (httpGetJson("/status", doc)) {
            const char *nm=doc["name"]|"";
            if (nm[0]) {
              strncpy(g_profiles[g_activeProfile].label, nm, 23);
              saveProfiles();
            }
          }
        }
        g_needRedraw=true;
      }
    }
  }

  if (g_screen==SCR_WIFI) {
    if (t.y>40 && t.y<76) {
      g_oskTarget=&cfgSsid; g_oskTitle="Edit SSID";
      strncpy(g_oskBuf,cfgSsid.c_str(),sizeof(g_oskBuf)-1);
      g_oskCursor=strlen(g_oskBuf); g_oskActive=true; g_needRedraw=true;
    } else if (t.y>86 && t.y<122) {
      g_oskTarget=&cfgPass; g_oskTitle="Edit Password";
      strncpy(g_oskBuf,cfgPass.c_str(),sizeof(g_oskBuf)-1);
      g_oskCursor=strlen(g_oskBuf); g_oskActive=true; g_needRedraw=true;
    } else if (t.y>132 && t.y<168) {
      g_oskTarget=&cfgCamIp; g_oskTitle="Edit CAM IP";
      strncpy(g_oskBuf,cfgCamIp.c_str(),sizeof(g_oskBuf)-1);
      g_oskCursor=strlen(g_oskBuf); g_oskActive=true; g_needRedraw=true;
    } else if (t.y>176 && t.y<204) {
      if (g_activeProfile>=0 && g_profiles[g_activeProfile].used) {
        strncpy(g_profiles[g_activeProfile].ssid, cfgSsid.c_str(), 32);
        strncpy(g_profiles[g_activeProfile].pass, cfgPass.c_str(), 64);
        strncpy(g_profiles[g_activeProfile].ip, cfgCamIp.c_str(), 15);
        saveProfiles();
        connectToProfile(g_activeProfile);
      }
    }
  }

  if (g_screen==SCR_CAMCTRL) {
    if (t.y>50&&t.y<80) {
      if (t.x<160){g_quality=constrain(g_quality-2,4,63);}else{g_quality=constrain(g_quality+2,4,63);}
      httpControl("quality",g_quality); g_needRedraw=true;
    }
    if (t.y>88&&t.y<118) {
      if (t.x<160){g_bright=constrain(g_bright-1,-2,2);}else{g_bright=constrain(g_bright+1,-2,2);}
      httpControl("brightness",g_bright); g_needRedraw=true;
    }
    if (t.y>126&&t.y<156) {
      if (t.x<160){g_led=constrain(g_led-40,0,255);}else{g_led=constrain(g_led+40,0,255);}
      httpControl("led",g_led); g_needRedraw=true;
    }
    if (t.y>164&&t.y<192) {
      if (t.x<160){g_hmirror=!g_hmirror;httpControl("hmirror",g_hmirror);}
      else{g_vflip=!g_vflip;httpControl("vflip",g_vflip);}
      g_needRedraw=true;
    }
  }

  if (g_screen==SCR_SNAPSHOT) {
    // Save button region (only when preview exists)
    if (g_hasSnapPreview && t.y>=178 && t.y<208 && t.x>=20 && t.x<150) {
      if (saveLastSnapshot()) {
        // brief confirmation
        M5.Display.fillRoundRect(60, 90, 200, 40, 6, COL_OK);
        M5.Display.setTextColor(COL_BG);
        M5.Display.setCursor(95, 104);
        M5.Display.print("Saved!");
        delay(800);
      } else {
        M5.Display.fillRoundRect(40, 90, 240, 40, 6, COL_ALERT);
        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setCursor(70, 104);
        M5.Display.print("Save failed / no SD");
        delay(1000);
      }
      g_needRedraw=true;
    }
  }
  if (g_screen==SCR_AUDIO_PLAY) {
    int idx=(t.y-48)/24;
    if (idx>=0 && idx<g_audioCount) { g_audioSel=idx; g_needRedraw=true; }
  }
}

// ---------------- network / media (same as v2) ----------------
bool httpGetJson(const String &path, JsonDocument &doc) {
  HTTPClient http;
  http.begin("http://"+cfgCamIp+path);
  http.setTimeout(2500);
  int code=http.GET();
  if (code!=200){http.end();return false;}
  String body=http.getString(); http.end();
  return !deserializeJson(doc, body);
}
bool httpControl(const char *var, int val) {
  HTTPClient http;
  http.begin("http://"+cfgCamIp+"/control?var="+String(var)+"&val="+String(val));
  http.setTimeout(2000);
  int code=http.GET(); http.end();
  return code==200;
}

// Scale factor so decoded image fits CONTENT_W x CONTENT_H
int fitScale(int imgW, int imgH) {
  // JPEGDEC scale: 0=1:1, 1=1:2, 2=1:4, 3=1:8
  for (int s = 0; s <= 3; s++) {
    int w = imgW >> s;
    int h = imgH >> s;
    if (w <= CONTENT_W && h <= CONTENT_H) return s;
  }
  return 3;
}

void decodeJpegFit(uint8_t *buf, size_t len) {
  if (!buf || len < 100) return;
  if (!jpeg.openRAM(buf, len, jpegDrawCallback)) return;
  jpeg.setPixelType(RGB565_BIG_ENDIAN);
  int s = fitScale(jpeg.getWidth(), jpeg.getHeight());
  int w = jpeg.getWidth() >> s;
  int h = jpeg.getHeight() >> s;
  int x = (CONTENT_W - w) / 2;
  int y = CONTENT_TOP + (CONTENT_H - h) / 2;
  if (x < 0) x = 0;
  if (y < CONTENT_TOP) y = CONTENT_TOP;
  // clear content only
  M5.Display.fillRect(0, CONTENT_TOP, CONTENT_W, CONTENT_H, COL_BG);
  jpeg.decode(x, y, s);
  jpeg.close();
}

int jpegDrawCallback(JPEGDRAW *p) {
  M5.Display.pushImage(p->x,p->y,p->iWidth,p->iHeight,(uint16_t*)p->pPixels);
  return 1;
}

bool fetchSnapshot() {
  if (!g_wifiOk || !g_jpgBuf) return false;
  HTTPClient http;
  http.begin("http://" + cfgCamIp + "/snapshot");
  http.setTimeout(6000);
  if (http.GET() != 200) { http.end(); return false; }
  g_jpgLen = 0;
  g_hasSnapPreview = false;
  WiFiClient *stream = http.getStreamPtr();
  uint32_t t0 = millis();
  while (millis() - t0 < 5000 && g_jpgLen < g_jpgCap) {
    size_t a = stream->available();
    if (a) g_jpgLen += stream->readBytes(g_jpgBuf + g_jpgLen, (size_t)min((size_t)a, g_jpgCap - g_jpgLen));
    else delay(1);
  }
  http.end();
  if (g_jpgLen < 100) return false;
  g_hasSnapPreview = true;
  // paint preview now; drawSnapshot will also re-paint on full redraw
  decodeJpegFit(g_jpgBuf, g_jpgLen);
  return true;
}

bool saveLastSnapshot() {
  if (!g_hasSnapPreview || !g_sdOk || !g_jpgBuf || g_jpgLen < 100) return false;
  ensureDirs();
  char name[64];
  snprintf(name, sizeof(name), "/images/snap_%lu.jpg", (unsigned long)millis());
  File f = SD.open(name, FILE_WRITE);
  if (!f) return false;
  size_t w = f.write(g_jpgBuf, g_jpgLen);
  f.close();
  return w == g_jpgLen;
}

void streamFrameTick() {
  static uint32_t last=0;
  if (millis()-last<100) return;
  last=millis();
  if (!g_jpgBuf) return;
  HTTPClient http;
  http.begin("http://"+cfgCamIp+":8081/stream");
  http.setTimeout(2500);
  if (http.GET()!=200){http.end();return;}
  WiFiClient *stream=http.getStreamPtr();
  g_jpgLen=0; bool inFrame=false;
  uint32_t t0=millis();
  while (millis()-t0<2000 && g_jpgLen<g_jpgCap) {
    if (!stream->available()){delay(1);continue;}
    uint8_t b=stream->read();
    if (!inFrame) {
      if (g_jpgLen==0&&b==0xFF) g_jpgBuf[g_jpgLen++]=b;
      else if (g_jpgLen==1&&b==0xD8){g_jpgBuf[g_jpgLen++]=b;inFrame=true;}
      else g_jpgLen=0;
    } else {
      g_jpgBuf[g_jpgLen++]=b;
      if (g_jpgLen>2&&g_jpgBuf[g_jpgLen-2]==0xFF&&g_jpgBuf[g_jpgLen-1]==0xD9) break;
    }
  }
  http.end();
  if (g_jpgLen<200) return;
  decodeJpegFit(g_jpgBuf, g_jpgLen);
  // re-anchor chrome so video never eats footer/header
  drawHeader("Stream");
  if (g_recActive) {
    uint32_t sec=(millis()-g_recStartMs)/1000;
    M5.Display.fillRoundRect(10, CONTENT_TOP + 4, 130, 18, 3, COL_REC);
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(16, CONTENT_TOP + 8);
    M5.Display.printf("REC %02lu:%02lu f:%lu", sec/60, sec%60, (unsigned long)g_recFrames);
  }
  drawFooter("< Prev", g_recActive?"STOP":"REC", "Next >");
  if (g_recActive && g_recFile) { g_recFile.write(g_jpgBuf,g_jpgLen); g_recFrames++; }
}

void startVideoRec() {
  if (!g_sdOk||g_recActive) return;
  ensureDirs();
  char name[64];
  snprintf(name,sizeof(name),"/videos/rec_%lu.mjpeg",(unsigned long)millis());
  g_recFile=SD.open(name,FILE_WRITE);
  if (!g_recFile) return;
  g_recActive=true; g_recStartMs=millis(); g_recFrames=0;
}
void stopVideoRec() {
  if (!g_recActive) return;
  g_recActive=false;
  if (g_recFile) g_recFile.close();
}

void writeWavHeader(File &f, uint32_t dataBytes) {
  uint32_t sr=AUDIO_SR; uint16_t ch=1, bits=16;
  uint32_t br=sr*ch*bits/8; uint16_t ba=ch*bits/8;
  uint32_t cs=36+dataBytes;
  f.write((const uint8_t*)"RIFF",4); f.write((uint8_t*)&cs,4);
  f.write((const uint8_t*)"WAVE",4); f.write((const uint8_t*)"fmt ",4);
  uint32_t s1=16; f.write((uint8_t*)&s1,4);
  uint16_t fmt=1; f.write((uint8_t*)&fmt,2);
  f.write((uint8_t*)&ch,2); f.write((uint8_t*)&sr,4);
  f.write((uint8_t*)&br,4); f.write((uint8_t*)&ba,2); f.write((uint8_t*)&bits,2);
  f.write((const uint8_t*)"data",4); f.write((uint8_t*)&dataBytes,4);
}
void startAudioRec() {
  if (!g_sdOk||!g_audioBuf||g_audioRecActive) return;
  M5.Speaker.end(); M5.Mic.begin();
  g_audioBufSamples=0; g_audioRecActive=true; g_audioRecStart=millis();
}
void stopAudioRec() {
  if (!g_audioRecActive) return;
  g_audioRecActive=false; M5.Mic.end(); M5.Speaker.begin();
  if (!g_sdOk||g_audioBufSamples==0) return;
  ensureDirs();
  char name[64];
  snprintf(name,sizeof(name),"/audio/rec_%lu.wav",(unsigned long)millis());
  File f=SD.open(name,FILE_WRITE);
  if (!f) return;
  uint32_t db=g_audioBufSamples*sizeof(int16_t);
  writeWavHeader(f,db); f.write((uint8_t*)g_audioBuf,db); f.close();
}
void scanAudioFiles() {
  g_audioCount=0; if (!g_sdOk) return;
  File root=SD.open("/audio"); if (!root) return;
  File e=root.openNextFile();
  while (e && g_audioCount<32) {
    if (!e.isDirectory()) {
      String n=e.name();
      if (n.endsWith(".wav")||n.endsWith(".WAV")) {
        int s=n.lastIndexOf('/');
        g_audioList[g_audioCount++]=(s>=0)?n.substring(s+1):n;
      }
    }
    e=root.openNextFile();
  }
  root.close();
}
void playSelectedAudio() {
  if (!g_sdOk||g_audioCount==0||g_audioSel>=g_audioCount) return;
  if (g_audioPlaying){M5.Speaker.stop();g_audioPlaying=false;return;}
  String path="/audio/"+g_audioList[g_audioSel];
  File f=SD.open(path); if (!f) return;
  f.seek(44);
  size_t got=f.read((uint8_t*)g_audioBuf, (size_t)min((size_t)(f.size()-44),(size_t)(AUDIO_SR*22*sizeof(int16_t))));
  f.close();
  M5.Mic.end(); M5.Speaker.begin(); M5.Speaker.setVolume(180);
  M5.Speaker.playRaw(g_audioBuf, got/sizeof(int16_t), AUDIO_SR, false);
  g_audioPlaying=true;
}
