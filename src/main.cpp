/*
 * MANTIS Core2 — multi-cam, anchored UI, splash, REC, SD, audio scope
 */
#include <M5Unified.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <JPEGDEC.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>
#include "mantis_bitmap.h"

static constexpr int CONTENT_TOP = 28;
static constexpr int CONTENT_BOTTOM = 210;
static constexpr int CONTENT_H = CONTENT_BOTTOM - CONTENT_TOP;
static constexpr int CONTENT_W = 320;

// v4 palette: teal #007373, purple #5D005D, mantis green
static const uint16_t COL_BG      = 0x0108;
static const uint16_t COL_PANEL   = 0x020A;
static const uint16_t COL_TEAL    = 0x038E;
static const uint16_t COL_TEAL_DK = 0x0248;
static const uint16_t COL_TEAL_LT = 0x14F5;
static const uint16_t COL_PURP    = 0x580B;
static const uint16_t COL_PURP_LT = 0x88B1;
static const uint16_t COL_ACCENT  = 0x564B;
static const uint16_t COL_TEXT    = 0xDFFF;
static const uint16_t COL_DIM     = 0x6C71;
static const uint16_t COL_ALERT   = 0xF80B;
static const uint16_t COL_OK      = 0x564B;
static const uint16_t COL_BTN     = 0x030C;
static const uint16_t COL_BTN_HI  = 0x048E;
static const uint16_t COL_REC     = 0xF80B;
static const uint16_t COL_HI      = 0x580B;

enum Screen : uint8_t {
  SCR_HOME=0, SCR_CAMS, SCR_STREAM, SCR_SNAPSHOT, SCR_CAMCTRL,
  SCR_WIFI, SCR_AUDIO_REC, SCR_AUDIO_PLAY, SCR_STATUS, SCR_COUNT
};

static constexpr int SD_CS=4, SD_SCK=18, SD_MISO=38, SD_MOSI=23;
static constexpr int MAX_PROFILES = 8;

struct CamProfile {
  char label[24], ssid[33], pass[65], ip[16];
  bool used;
};
CamProfile g_profiles[MAX_PROFILES];
int g_profileCount=0, g_activeProfile=-1, g_camListSel=0;

Preferences prefs;
String cfgSsid="Mantis_1_Hotspot", cfgPass="mantis33", cfgCamIp="192.168.5.1", cfgAdmin="admin1234";

Screen g_screen=SCR_HOME;
bool g_wifiOk=false, g_sdOk=false, g_needRedraw=true;
uint32_t g_lastBtnMs=0;
JPEGDEC jpeg;
uint8_t *g_jpgBuf=nullptr;
size_t g_jpgCap=96*1024, g_jpgLen=0;
bool g_hasSnapPreview=false;

bool g_recActive=false;
uint32_t g_recStartMs=0, g_recFrames=0;
static constexpr uint32_t REC_MAX_MS=30000;
File g_recFile;

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

static constexpr int SCOPE_N=160;
int16_t g_scope[SCOPE_N];
int g_scopeWrite=0;
bool g_scopeClip=false;
int16_t g_scopePeak=0;

bool g_scanning=false;
String g_scanSsid[16];
int g_scanCount=0;

void loadConfig(); void saveConfig();
void loadProfiles(); void saveProfiles();
void connectToProfile(int idx);
void connectWifi(const char *ssid, const char *pass);
void drawSplash();
bool initSD(); void ensureDirs();
void drawPanel(int x,int y,int w,int h,bool raised);
void drawBtn(int x,int y,int w,int h,const char *label,bool hot);
void drawHeader(const char *t);
void drawFooter(const char *a, const char *b, const char *c);
void drawHome(); void drawCams(); void drawStream(); void drawSnapshot();
void drawCamCtrl(); void drawWifi(); void drawAudioRec(); void drawAudioPlay();
void drawStatus(); void drawOsk(); void drawMicScope();
void handleButtons(); void handleTouch();
bool httpGetJson(const String &path, JsonDocument &doc);
bool httpControl(const char *var, int val);
bool fetchSnapshot(); bool saveLastSnapshot();
void streamFrameTick();
void startVideoRec(); void stopVideoRec();
void startAudioRec(); void stopAudioRec();
void writeWavHeader(File &f, uint32_t dataBytes);
void scanAudioFiles(); void playSelectedAudio();
void startWifiScan(); void applyScanToProfiles();
bool addOrUpdateProfile(const char *label, const char *ssid, const char *pass, const char *ip);
int jpegDrawCallback(JPEGDRAW *p);
void decodeJpegFit(uint8_t *buf, size_t len);

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
  memset(g_scope, 0, sizeof(g_scope));

  for (int i=0;i<MAX_PROFILES;i++) g_profiles[i].used=false;
  loadConfig();
  loadProfiles();
  if (g_profileCount==0) {
    addOrUpdateProfile("Mantis 1", "Mantis_1_Hotspot", "mantis33", "192.168.5.1");
    saveProfiles();
  }
  g_sdOk = initSD();
  if (g_sdOk) ensureDirs();

  int idx = (g_activeProfile>=0) ? g_activeProfile : 0;
  if (g_profiles[idx].used) connectToProfile(idx);
  g_needRedraw=true;
}

void loop() {
  M5.update();
  handleButtons();
  handleTouch();

  if (g_recActive && millis()-g_recStartMs >= REC_MAX_MS) {
    stopVideoRec();
    g_needRedraw=true;
  }
  if (g_audioRecActive && millis()-g_audioRecStart >= AUDIO_MAX_MS) {
    stopAudioRec();
    g_needRedraw=true;
  }

  if (g_oskActive) {
    if (g_needRedraw) { drawOsk(); g_needRedraw=false; }
    delay(20);
    return;
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

  // One mic consumer for scope + recording
  bool needMic = (g_screen==SCR_AUDIO_REC) || g_audioRecActive;
  if (needMic) {
    if (!M5.Mic.isEnabled()) {
      M5.Speaker.end();
      M5.Mic.begin();
    }
    static int16_t chunk[AUDIO_CHUNK];
    if (M5.Mic.record(chunk, AUDIO_CHUNK, AUDIO_SR)) {
      int16_t peak=0;
      bool clip=false;
      for (int i=0;i<AUDIO_CHUNK;i++) {
        int16_t v=chunk[i];
        g_scope[g_scopeWrite]=v;
        g_scopeWrite=(g_scopeWrite+1)%SCOPE_N;
        int16_t a = v<0 ? (int16_t)(-v) : v;
        if (a>peak) peak=a;
        if (a>30000) clip=true;
      }
      g_scopePeak=peak;
      g_scopeClip=clip;
      if (g_audioRecActive && g_audioBuf) {
        size_t maxS=AUDIO_SR*22;
        if (g_audioBufSamples+AUDIO_CHUNK < maxS) {
          memcpy(g_audioBuf+g_audioBufSamples, chunk, AUDIO_CHUNK*sizeof(int16_t));
          g_audioBufSamples+=AUDIO_CHUNK;
        }
      }
    }
    static uint32_t lastScope=0;
    if (g_screen==SCR_AUDIO_REC && millis()-lastScope>80) {
      lastScope=millis();
      drawMicScope();
    }
  }

  delay(5);
}

// ---------- config / profiles ----------
void loadConfig() {
  prefs.begin("mantis", true);
  cfgSsid=prefs.getString("ssid","Mantis_1_Hotspot");
  cfgPass=prefs.getString("pass","mantis33");
  cfgCamIp=prefs.getString("camip","192.168.5.1");
  cfgAdmin=prefs.getString("admin","admin1234");
  g_activeProfile=prefs.getInt("active",-1);
  prefs.end();
}
void saveConfig() {
  prefs.begin("mantis", false);
  prefs.putString("ssid",cfgSsid);
  prefs.putString("pass",cfgPass);
  prefs.putString("camip",cfgCamIp);
  prefs.putString("admin",cfgAdmin);
  prefs.putInt("active",g_activeProfile);
  prefs.end();
}
void loadProfiles() {
  prefs.begin("camprofs", true);
  g_profileCount=prefs.getInt("n",0);
  if (g_profileCount>MAX_PROFILES) g_profileCount=MAX_PROFILES;
  for (int i=0;i<g_profileCount;i++) {
    char k[16];
    snprintf(k,sizeof(k),"l%d",i); String L=prefs.getString(k,"");
    snprintf(k,sizeof(k),"s%d",i); String S=prefs.getString(k,"");
    snprintf(k,sizeof(k),"p%d",i); String P=prefs.getString(k,"");
    snprintf(k,sizeof(k),"i%d",i); String I=prefs.getString(k,"192.168.5.1");
    strncpy(g_profiles[i].label,L.c_str(),23);
    strncpy(g_profiles[i].ssid,S.c_str(),32);
    strncpy(g_profiles[i].pass,P.c_str(),64);
    strncpy(g_profiles[i].ip,I.c_str(),15);
    g_profiles[i].used=(S.length()>0);
  }
  prefs.end();
}
void saveProfiles() {
  prefs.begin("camprofs", false);
  int n=0;
  for (int i=0;i<MAX_PROFILES;i++) if (g_profiles[i].used) n++;
  prefs.putInt("n",n);
  int slot=0;
  for (int i=0;i<MAX_PROFILES;i++) {
    if (!g_profiles[i].used) continue;
    char k[16];
    snprintf(k,sizeof(k),"l%d",slot); prefs.putString(k,g_profiles[i].label);
    snprintf(k,sizeof(k),"s%d",slot); prefs.putString(k,g_profiles[i].ssid);
    snprintf(k,sizeof(k),"p%d",slot); prefs.putString(k,g_profiles[i].pass);
    snprintf(k,sizeof(k),"i%d",slot); prefs.putString(k,g_profiles[i].ip);
    slot++;
  }
  g_profileCount=n;
  prefs.end();
}
bool addOrUpdateProfile(const char *label, const char *ssid, const char *pass, const char *ip) {
  for (int i=0;i<MAX_PROFILES;i++) {
    if (g_profiles[i].used && strcmp(g_profiles[i].ssid,ssid)==0) {
      strncpy(g_profiles[i].label,label,23);
      strncpy(g_profiles[i].pass,pass,64);
      strncpy(g_profiles[i].ip,ip,15);
      return true;
    }
  }
  for (int i=0;i<MAX_PROFILES;i++) {
    if (!g_profiles[i].used) {
      strncpy(g_profiles[i].label,label,23);
      strncpy(g_profiles[i].ssid,ssid,32);
      strncpy(g_profiles[i].pass,pass,64);
      strncpy(g_profiles[i].ip,ip,15);
      g_profiles[i].used=true;
      g_profileCount++;
      return true;
    }
  }
  return false;
}

void drawSplash() {
  M5.Display.fillScreen(COL_BG);
  M5.Display.fillRect(0, 0, 320, 4, COL_PURP);
  M5.Display.fillRect(0, 4, 320, 2, COL_TEAL);
  int x0 = (320 - MANTIS_W) / 2, y0 = 28;
  drawPanel(x0 - 8, y0 - 8, MANTIS_W + 16, MANTIS_H + 16, true);
  for (int y = 0; y < MANTIS_H; y++)
    for (int x = 0; x < MANTIS_W; x++) {
      uint16_t c = MANTIS_PIX[y * MANTIS_W + x];
      if (c) M5.Display.drawPixel(x0 + x, y0 + y, c);
    }
  M5.Display.setTextColor(COL_ACCENT);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(88, y0 + MANTIS_H + 20);
  M5.Display.print("MantisCam");
  M5.Display.setTextSize(1);
  M5.Display.fillRect(0, 236, 320, 4, COL_PURP);
}

void connectWifi(const char *ssid, const char *pass) {
  drawSplash();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  uint32_t t0=millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<15000) delay(200);
  g_wifiOk=(WiFi.status()==WL_CONNECTED);
  if (g_wifiOk && g_activeProfile>=0) {
    JsonDocument doc;
    if (httpGetJson("/status", doc)) {
      const char *nm=doc["name"]|"";
      if (nm[0]) {
        strncpy(g_profiles[g_activeProfile].label,nm,23);
        saveProfiles();
      }
    }
  }
  if (!g_wifiOk) g_screen=SCR_CAMS;
  g_needRedraw=true;
}

void connectToProfile(int idx) {
  if (idx<0||idx>=MAX_PROFILES||!g_profiles[idx].used) return;
  WiFi.disconnect(true);
  delay(150);
  g_wifiOk=false;
  g_activeProfile=idx;
  cfgSsid=g_profiles[idx].ssid;
  cfgPass=g_profiles[idx].pass;
  cfgCamIp=g_profiles[idx].ip;
  saveConfig();
  connectWifi(g_profiles[idx].ssid, g_profiles[idx].pass);
}

bool initSD() {
  SPI.begin(SD_SCK,SD_MISO,SD_MOSI,SD_CS);
  return SD.begin(SD_CS, SPI, 25000000);
}
void ensureDirs() {
  if (!SD.exists("/images")) SD.mkdir("/images");
  if (!SD.exists("/videos")) SD.mkdir("/videos");
  if (!SD.exists("/audio")) SD.mkdir("/audio");
}

void startWifiScan() {
  g_scanning=true;
  g_needRedraw=true;
  WiFi.mode(WIFI_STA);
  int n=WiFi.scanNetworks(false,false,false,300);
  g_scanCount=0;
  for (int i=0;i<n && g_scanCount<16;i++) {
    String s=WiFi.SSID(i);
    if (s.length()) g_scanSsid[g_scanCount++]=s;
  }
  g_scanning=false;
  applyScanToProfiles();
  g_needRedraw=true;
}
void applyScanToProfiles() {
  for (int i=0;i<g_scanCount;i++) {
    String s=g_scanSsid[i];
    bool interesting=s.startsWith("Mantis")||s.indexOf("CAM")>=0||s.indexOf("ESP32")>=0||s.indexOf("Hotspot")>=0;
    if (!interesting) continue;
    bool exists=false;
    for (int p=0;p<MAX_PROFILES;p++)
      if (g_profiles[p].used && s.equals(g_profiles[p].ssid)) { exists=true; break; }
    if (!exists) addOrUpdateProfile(s.c_str(), s.c_str(), "mantis33", "192.168.5.1");
  }
  saveProfiles();
}

// ---------- UI ----------

void drawPanel(int x, int y, int w, int h, bool raised) {
  M5.Display.fillRect(x, y, w, h, COL_PANEL);
  uint16_t hi = raised ? COL_TEAL_LT : COL_TEAL_DK;
  uint16_t lo = raised ? COL_TEAL_DK : COL_TEAL_LT;
  M5.Display.drawFastHLine(x, y, w, hi);
  M5.Display.drawFastVLine(x, y, h, hi);
  M5.Display.drawFastHLine(x, y + h - 1, w, lo);
  M5.Display.drawFastVLine(x + w - 1, y, h, lo);
  M5.Display.drawRect(x + 1, y + 1, w - 2, h - 2, COL_PURP);
}
void drawBtn(int x, int y, int w, int h, const char *label, bool hot) {
  M5.Display.fillRect(x, y, w, h, hot ? COL_PURP : COL_BTN);
  M5.Display.drawFastHLine(x, y, w, COL_TEAL_LT);
  M5.Display.drawFastVLine(x, y, h, COL_TEAL_LT);
  M5.Display.drawFastHLine(x, y + h - 1, w, COL_TEAL_DK);
  M5.Display.drawFastVLine(x + w - 1, y, h, COL_TEAL_DK);
  M5.Display.drawRect(x + 1, y + 1, w - 2, h - 2, COL_TEAL);
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.setTextSize(1);
  int tw = strlen(label) * 6;
  M5.Display.setCursor(x + (w - tw) / 2, y + (h - 8) / 2);
  M5.Display.print(label);
}

void drawHeader(const char *t) {
  M5.Display.fillRect(0, 0, 320, CONTENT_TOP - 2, COL_PANEL);
  M5.Display.fillRect(0, 0, 320, 3, COL_TEAL);
  M5.Display.drawFastHLine(0, CONTENT_TOP - 3, 320, COL_PURP);
  M5.Display.setTextColor(COL_ACCENT);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(6, 9);
  M5.Display.print("MantisCam");
  M5.Display.setTextColor(COL_DIM);
  M5.Display.print(" // ");
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.print(t);
  M5.Display.fillRect(198, 6, 54, 14, g_wifiOk ? COL_TEAL_DK : COL_PURP);
  M5.Display.drawRect(198, 6, 54, 14, COL_TEAL);
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.setCursor(204, 9);
  M5.Display.print(g_wifiOk ? "LINK" : "DOWN");
  M5.Display.fillRect(256, 6, 58, 14, g_sdOk ? COL_TEAL_DK : COL_PURP);
  M5.Display.drawRect(256, 6, 58, 14, COL_TEAL);
  M5.Display.setCursor(264, 9);
  M5.Display.print(g_sdOk ? "DISK" : "----");
}

void drawFooter(const char *aLabel, const char *bLabel, const char *cLabel) {
  M5.Display.fillRect(0, CONTENT_BOTTOM, 320, 240 - CONTENT_BOTTOM, COL_PANEL);
  M5.Display.drawFastHLine(0, CONTENT_BOTTOM, 320, COL_PURP);
  M5.Display.drawFastHLine(0, CONTENT_BOTTOM + 1, 320, COL_TEAL);
  M5.Display.drawFastVLine(106, CONTENT_BOTTOM + 2, 28, COL_TEAL_DK);
  M5.Display.drawFastVLine(213, CONTENT_BOTTOM + 2, 28, COL_TEAL_DK);
  M5.Display.setTextSize(1);
  auto zone = [&](const char *s, int z, bool primary) {
    int x0 = z * 107;
    if (primary) {
      M5.Display.fillRect(x0 + 4, CONTENT_BOTTOM + 6, 98, 18, COL_TEAL_DK);
      M5.Display.drawRect(x0 + 4, CONTENT_BOTTOM + 6, 98, 18, COL_TEAL);
    }
    M5.Display.setTextColor(primary ? COL_ACCENT : COL_DIM);
    int tw = strlen(s) * 6;
    M5.Display.setCursor(x0 + (106 - tw) / 2, CONTENT_BOTTOM + 11);
    M5.Display.print(s);
  };
  zone(aLabel, 0, false);
  zone(bLabel, 1, true);
  zone(cLabel, 2, false);
}

void drawHome() {
  drawHeader("Home");
  drawPanel(16, CONTENT_TOP + 12, 288, 130, true);
  int x0 = 28, y0 = CONTENT_TOP + 28;
  for (int y = 0; y < MANTIS_H; y += 2)
    for (int x = 0; x < MANTIS_W; x += 2) {
      uint16_t c = MANTIS_PIX[y * MANTIS_W + x];
      if (c) M5.Display.drawPixel(x0 + x / 2, y0 + y / 2, c);
    }
  M5.Display.setTextColor(COL_ACCENT);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(110, CONTENT_TOP + 36);
  M5.Display.print("MantisCam");
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(COL_TEAL_LT);
  M5.Display.setCursor(110, CONTENT_TOP + 58);
  M5.Display.print("camera companion");
  M5.Display.setTextColor(COL_DIM);
  M5.Display.setCursor(110, CONTENT_TOP + 78);
  M5.Display.print("A/C navigate  ·  B select");
  if (g_activeProfile >= 0 && g_profiles[g_activeProfile].used) {
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(110, CONTENT_TOP + 100);
    M5.Display.printf("link: %s", g_profiles[g_activeProfile].label);
  }
  drawFooter("< Prev", "Cameras", "Next >");
}

void drawCams() {
  drawHeader("Cameras");
  M5.Display.setTextColor(COL_DIM);
  M5.Display.setCursor(8,CONTENT_TOP+4);
  M5.Display.print("Tap camera to connect  |  A/C change screen");
  int y=CONTENT_TOP+22, shown=0;
  for (int i=0;i<MAX_PROFILES && shown<5;i++) {
    if (!g_profiles[i].used) continue;
    bool sel=(shown==g_camListSel), act=(i==g_activeProfile);
    M5.Display.fillRect(8,y,304,24, sel?COL_PURP:COL_PANEL);
    M5.Display.drawRect(8,y,304,24, sel?COL_TEAL_LT:COL_TEAL_DK);
    M5.Display.setTextColor(act?COL_ACCENT:COL_TEXT);
    M5.Display.setCursor(14,y+6);
    M5.Display.printf("%s%s", g_profiles[i].label, act?" [ON]":"");
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(180,y+6);
    M5.Display.print(g_profiles[i].ssid);
    y+=28; shown++;
  }
  if (!shown) {
    M5.Display.setTextColor(COL_ALERT);
    M5.Display.setCursor(20,90);
    M5.Display.print("No profiles — tap Scan WiFi");
  }
  drawBtn(8, CONTENT_BOTTOM-32, 148, 26, g_scanning?"Scanning...":"Scan WiFi", false);
  drawBtn(164, CONTENT_BOTTOM-32, 148, 26, "Save active", false);
  drawFooter("< Prev","Connect","Next >");
}

void drawStream() {
  drawHeader("Stream");
  M5.Display.fillRect(0,CONTENT_TOP,CONTENT_W,CONTENT_H,COL_BG);
  if (!g_wifiOk) {
    M5.Display.setTextColor(COL_ALERT);
    M5.Display.setCursor(40,100);
    M5.Display.print("Not connected — Cameras");
  } else {
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(80,100);
    M5.Display.print("Waiting for frames...");
  }
  if (g_recActive) {
    uint32_t sec=(millis()-g_recStartMs)/1000;
    M5.Display.fillRoundRect(8,CONTENT_TOP+2,120,16,2,COL_REC);
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(14,CONTENT_TOP+5);
    M5.Display.printf("REC %02lu:%02lu", sec/60, sec%60);
  }
  drawFooter("< Prev", g_recActive?"STOP":"REC", "Next >");
}

void drawSnapshot() {
  drawHeader("Snapshot");
  if (g_hasSnapPreview && g_jpgBuf && g_jpgLen>100) {
    decodeJpegFit(g_jpgBuf, g_jpgLen);
    drawBtn(20, CONTENT_BOTTOM-32, 130, 26, "Save to SD", true);
    drawBtn(170, CONTENT_BOTTOM-32, 130, 26, "Retake=B", false);
  } else {
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(20,80);
    M5.Display.print("B = capture preview");
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(20,100);
    M5.Display.print("Then tap Save to SD");
  }
  drawFooter("< Prev","Capture","Next >");
}

void drawCamCtrl() {
  drawHeader("Cam Ctrl");
  auto row=[&](int y,const char*l,int v){
    M5.Display.fillRoundRect(10,y,300,28,4,COL_PANEL);
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(18,y+8); M5.Display.printf("%s %d",l,v);
    M5.Display.setTextColor(COL_ACCENT);
    M5.Display.setCursor(200,y+8); M5.Display.print("- / +");
  };
  row(CONTENT_TOP+8,"Quality",g_quality);
  row(CONTENT_TOP+44,"Bright",g_bright);
  row(CONTENT_TOP+80,"LED",g_led);
  M5.Display.fillRoundRect(10,CONTENT_TOP+116,145,28,4,COL_PANEL);
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.setCursor(22,CONTENT_TOP+124);
  M5.Display.printf("Mirror %s", g_hmirror?"ON":"off");
  M5.Display.fillRoundRect(165,CONTENT_TOP+116,145,28,4,COL_PANEL);
  M5.Display.setCursor(178,CONTENT_TOP+124);
  M5.Display.printf("Flip %s", g_vflip?"ON":"off");
  drawFooter("< Prev","LED","Next >");
}

void drawWifi() {
  drawHeader("WiFi / Edit");
  M5.Display.fillRoundRect(10,CONTENT_TOP+8,300,34,4,COL_PANEL);
  M5.Display.setTextColor(COL_DIM); M5.Display.setCursor(18,CONTENT_TOP+12); M5.Display.print("SSID");
  M5.Display.setTextColor(COL_ACCENT); M5.Display.setCursor(18,CONTENT_TOP+26); M5.Display.print(cfgSsid);
  M5.Display.fillRoundRect(10,CONTENT_TOP+50,300,34,4,COL_PANEL);
  M5.Display.setTextColor(COL_DIM); M5.Display.setCursor(18,CONTENT_TOP+54); M5.Display.print("Password");
  M5.Display.setTextColor(COL_ACCENT); M5.Display.setCursor(18,CONTENT_TOP+68); M5.Display.print(cfgPass);
  M5.Display.fillRoundRect(10,CONTENT_TOP+92,300,34,4,COL_PANEL);
  M5.Display.setTextColor(COL_DIM); M5.Display.setCursor(18,CONTENT_TOP+96); M5.Display.print("CAM IP");
  M5.Display.setTextColor(COL_ACCENT); M5.Display.setCursor(18,CONTENT_TOP+110); M5.Display.print(cfgCamIp);
  M5.Display.fillRoundRect(10,CONTENT_TOP+134,300,28,4,COL_BTN);
  M5.Display.setTextColor(COL_TEXT); M5.Display.setCursor(70,CONTENT_TOP+142); M5.Display.print("Apply to active profile");
  drawFooter("< Prev","Edit SSID","Next >");
}

void drawMicScope() {
  int boxY=CONTENT_TOP+6, boxH=88;
  M5.Display.fillRect(10,boxY,300,boxH,COL_PANEL);
  int mid=boxY+boxH/2;
  M5.Display.drawFastHLine(10,mid,300,COL_DIM);
  int start=g_scopeWrite;
  for (int i=1;i<SCOPE_N;i++) {
    int i0=(start+i-1)%SCOPE_N, i1=(start+i)%SCOPE_N;
    int x0=10+(i-1)*300/SCOPE_N, x1=10+i*300/SCOPE_N;
    int y0=mid-(int)g_scope[i0]*(boxH/2-4)/32768;
    int y1=mid-(int)g_scope[i1]*(boxH/2-4)/32768;
    M5.Display.drawLine(x0,y0,x1,y1, g_scopeClip?COL_ALERT:COL_ACCENT);
  }
  if (g_scopeClip) {
    M5.Display.fillRoundRect(240,boxY+4,64,16,3,COL_ALERT);
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(248,boxY+7);
    M5.Display.print("CLIP!");
  } else {
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(240,boxY+7);
    M5.Display.printf("pk:%d",(int)g_scopePeak);
  }
}

void drawAudioRec() {
  drawHeader("Audio Rec");
  drawMicScope();
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.setCursor(16,CONTENT_TOP+100);
  M5.Display.print("Live mic scope  |  max 20s -> /audio");
  if (g_audioRecActive) {
    uint32_t sec=(millis()-g_audioRecStart)/1000;
    M5.Display.fillRoundRect(60,CONTENT_TOP+120,200,34,8,COL_REC);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(90,CONTENT_TOP+128);
    M5.Display.printf("REC %02lu:%02lu",sec/60,sec%60);
    M5.Display.setTextSize(1);
  } else {
    M5.Display.fillRoundRect(60,CONTENT_TOP+120,200,34,8,COL_BTN);
    M5.Display.setTextColor(COL_ACCENT);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(95,CONTENT_TOP+128);
    M5.Display.print("B = REC");
    M5.Display.setTextSize(1);
  }
  drawFooter("< Prev", g_audioRecActive?"Stop":"REC", "Next >");
}

void drawAudioPlay() {
  drawHeader("Audio Play");
  if (!g_sdOk) {
    M5.Display.setTextColor(COL_ALERT);
    M5.Display.setCursor(30,100);
    M5.Display.print("No SD");
    drawFooter("< Prev","Play","Next >");
    return;
  }
  scanAudioFiles();
  if (!g_audioCount) {
    M5.Display.setTextColor(COL_DIM);
    M5.Display.setCursor(30,100);
    M5.Display.print("No files in /audio");
  } else {
    for (int i=0;i<g_audioCount && i<6;i++) {
      int y=CONTENT_TOP+8+i*24;
      if (i==g_audioSel) M5.Display.fillRoundRect(8,y-2,304,22,3,COL_HI);
      M5.Display.setTextColor(COL_TEXT);
      M5.Display.setCursor(14,y+4);
      M5.Display.print(g_audioList[i]);
    }
  }
  drawFooter("< Prev","Play","Next >");
}

void drawStatus() {
  drawHeader("Status");
  M5.Display.setTextColor(COL_TEXT);
  M5.Display.setCursor(16,CONTENT_TOP+12);
  M5.Display.printf("WiFi %s  IP %s", g_wifiOk?"OK":"down", WiFi.localIP().toString().c_str());
  M5.Display.setCursor(16,CONTENT_TOP+30);
  M5.Display.printf("RSSI %d  SD %s", WiFi.RSSI(), g_sdOk?"yes":"no");
  M5.Display.setCursor(16,CONTENT_TOP+48);
  M5.Display.printf("Heap %uKB  PSRAM %uKB", ESP.getFreeHeap()/1024, ESP.getPsramSize()/1024);
  if (g_activeProfile>=0 && g_profiles[g_activeProfile].used) {
    M5.Display.setCursor(16,CONTENT_TOP+66);
    M5.Display.printf("Profile: %s", g_profiles[g_activeProfile].label);
  }
  if (g_wifiOk) {
    JsonDocument doc;
    if (httpGetJson("/status", doc)) {
      M5.Display.setCursor(16,CONTENT_TOP+90);
      M5.Display.printf("CAM %s %s", doc["fw"]|"?", doc["sensor"]|"?");
    }
  }
  drawFooter("< Prev","Refresh","Next >");
}

void drawOsk() {
  M5.Display.fillScreen(COL_BG);
  M5.Display.fillRect(0,0,320,48,COL_PANEL);
  M5.Display.setTextColor(COL_ACCENT);
  M5.Display.setCursor(8,6); M5.Display.print(g_oskTitle);
  M5.Display.setCursor(8,26); M5.Display.setTextColor(COL_TEXT);
  String shown=String(g_oskBuf);
  M5.Display.print(shown.substring(0,g_oskCursor));
  M5.Display.setTextColor(COL_ACCENT); M5.Display.print("|");
  M5.Display.setTextColor(COL_TEXT); M5.Display.print(shown.substring(g_oskCursor));
  static const char *rows[4]={"1234567890","qwertyuiop","asdfghjkl-","zxcvbnm._@"};
  for (int r=0;r<4;r++) for (int c=0;c<10;c++) {
    int x=c*32, y=54+r*36;
    M5.Display.fillRoundRect(x+1,y+1,30,32,3,COL_BTN);
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setCursor(x+10,y+10);
    M5.Display.print(rows[r][c]);
  }
  drawFooter("Left","OK","Right");
}

// ---------- input: A/C always navigate ----------
void handleButtons() {
  if (millis()-g_lastBtnMs<220) return;

  if (M5.BtnA.wasPressed()) {
    g_lastBtnMs=millis();
    if (g_oskActive) { if (g_oskCursor>0) g_oskCursor--; g_needRedraw=true; return; }
    if (g_recActive) stopVideoRec();
    if (g_audioRecActive) stopAudioRec();
    if (g_audioPlaying) { M5.Speaker.stop(); g_audioPlaying=false; }
    g_screen=(Screen)((g_screen+SCR_COUNT-1)%SCR_COUNT);
    g_needRedraw=true;
  }
  if (M5.BtnC.wasPressed()) {
    g_lastBtnMs=millis();
    if (g_oskActive) { if (g_oskCursor<(int)strlen(g_oskBuf)) g_oskCursor++; g_needRedraw=true; return; }
    if (g_recActive) stopVideoRec();
    if (g_audioRecActive) stopAudioRec();
    if (g_audioPlaying) { M5.Speaker.stop(); g_audioPlaying=false; }
    g_screen=(Screen)((g_screen+1)%SCR_COUNT);
    g_needRedraw=true;
  }
  if (M5.BtnB.wasPressed()) {
    g_lastBtnMs=millis();
    if (g_oskActive) {
      if (g_oskTarget) *g_oskTarget=String(g_oskBuf);
      g_oskActive=false; g_oskTarget=nullptr;
      saveConfig();
      if (g_activeProfile>=0 && g_profiles[g_activeProfile].used) {
        strncpy(g_profiles[g_activeProfile].ssid,cfgSsid.c_str(),32);
        strncpy(g_profiles[g_activeProfile].pass,cfgPass.c_str(),64);
        strncpy(g_profiles[g_activeProfile].ip,cfgCamIp.c_str(),15);
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
        g_needRedraw=true;
        break;
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
      memmove(g_oskBuf+g_oskCursor+1,g_oskBuf+g_oskCursor,strlen(g_oskBuf)-g_oskCursor+1);
      g_oskBuf[g_oskCursor++]=ch;
    }
    g_needRedraw=true; return;
  }
  if (g_screen==SCR_CAMS) {
    int y=CONTENT_TOP+22, shown=0;
    for (int i=0;i<MAX_PROFILES;i++) {
      if (!g_profiles[i].used) continue;
      if (t.y>=y && t.y<y+24) {
        g_camListSel=shown;
        connectToProfile(i);
        g_needRedraw=true; return;
      }
      y+=28; shown++;
    }
    if (t.y>=CONTENT_BOTTOM-32 && t.y<CONTENT_BOTTOM-4) {
      if (t.x<160) startWifiScan();
      else if (g_activeProfile>=0 && g_wifiOk) {
        JsonDocument doc;
        if (httpGetJson("/status",doc)) {
          const char *nm=doc["name"]|"";
          if (nm[0]) { strncpy(g_profiles[g_activeProfile].label,nm,23); saveProfiles(); }
        }
        g_needRedraw=true;
      }
    }
  }
  if (g_screen==SCR_WIFI) {
    if (t.y>CONTENT_TOP+8 && t.y<CONTENT_TOP+42) {
      g_oskTarget=&cfgSsid; g_oskTitle="Edit SSID";
      strncpy(g_oskBuf,cfgSsid.c_str(),sizeof(g_oskBuf)-1);
      g_oskCursor=strlen(g_oskBuf); g_oskActive=true; g_needRedraw=true;
    } else if (t.y>CONTENT_TOP+50 && t.y<CONTENT_TOP+84) {
      g_oskTarget=&cfgPass; g_oskTitle="Edit Password";
      strncpy(g_oskBuf,cfgPass.c_str(),sizeof(g_oskBuf)-1);
      g_oskCursor=strlen(g_oskBuf); g_oskActive=true; g_needRedraw=true;
    } else if (t.y>CONTENT_TOP+92 && t.y<CONTENT_TOP+126) {
      g_oskTarget=&cfgCamIp; g_oskTitle="Edit CAM IP";
      strncpy(g_oskBuf,cfgCamIp.c_str(),sizeof(g_oskBuf)-1);
      g_oskCursor=strlen(g_oskBuf); g_oskActive=true; g_needRedraw=true;
    } else if (t.y>CONTENT_TOP+134 && t.y<CONTENT_TOP+162 && g_activeProfile>=0) {
      strncpy(g_profiles[g_activeProfile].ssid,cfgSsid.c_str(),32);
      strncpy(g_profiles[g_activeProfile].pass,cfgPass.c_str(),64);
      strncpy(g_profiles[g_activeProfile].ip,cfgCamIp.c_str(),15);
      saveProfiles();
      connectToProfile(g_activeProfile);
    }
  }
  if (g_screen==SCR_CAMCTRL) {
    if (t.y>CONTENT_TOP+8 && t.y<CONTENT_TOP+36) {
      g_quality=constrain(g_quality+(t.x<160?-2:2),4,63); httpControl("quality",g_quality); g_needRedraw=true;
    } else if (t.y>CONTENT_TOP+44 && t.y<CONTENT_TOP+72) {
      g_bright=constrain(g_bright+(t.x<160?-1:1),-2,2); httpControl("brightness",g_bright); g_needRedraw=true;
    } else if (t.y>CONTENT_TOP+80 && t.y<CONTENT_TOP+108) {
      g_led=constrain(g_led+(t.x<160?-40:40),0,255); httpControl("led",g_led); g_needRedraw=true;
    } else if (t.y>CONTENT_TOP+116 && t.y<CONTENT_TOP+144) {
      if (t.x<160){g_hmirror=!g_hmirror;httpControl("hmirror",g_hmirror);}
      else{g_vflip=!g_vflip;httpControl("vflip",g_vflip);}
      g_needRedraw=true;
    }
  }
  if (g_screen==SCR_SNAPSHOT && g_hasSnapPreview &&
      t.y>=CONTENT_BOTTOM-32 && t.y<CONTENT_BOTTOM-4 && t.x>=20 && t.x<150) {
    if (saveLastSnapshot()) {
      M5.Display.fillRoundRect(80,90,160,36,6,COL_OK);
      M5.Display.setTextColor(COL_BG);
      M5.Display.setCursor(120,102); M5.Display.print("Saved!");
      delay(600);
    } else {
      M5.Display.fillRoundRect(40,90,240,36,6,COL_ALERT);
      M5.Display.setTextColor(COL_TEXT);
      M5.Display.setCursor(70,102); M5.Display.print("Save failed / no SD");
      delay(800);
    }
    g_needRedraw=true;
  }
  if (g_screen==SCR_AUDIO_PLAY) {
    int idx=(t.y-(CONTENT_TOP+8))/24;
    if (idx>=0 && idx<g_audioCount) { g_audioSel=idx; g_needRedraw=true; }
  }
}

// ---------- network / jpeg ----------
bool httpGetJson(const String &path, JsonDocument &doc) {
  HTTPClient http;
  http.begin("http://"+cfgCamIp+path);
  http.setTimeout(2500);
  if (http.GET()!=200){http.end();return false;}
  String body=http.getString(); http.end();
  return !deserializeJson(doc,body);
}
bool httpControl(const char *var, int val) {
  HTTPClient http;
  http.begin("http://"+cfgCamIp+"/control?var="+String(var)+"&val="+String(val));
  http.setTimeout(2000);
  int code=http.GET(); http.end();
  return code==200;
}

int jpegDrawCallback(JPEGDRAW *p) {
  M5.Display.pushImage(p->x, p->y, p->iWidth, p->iHeight, (uint16_t*)p->pPixels);
  return 1;
}

// Clip to content band; draw from top-left of content (no centering shift)
void decodeJpegFit(uint8_t *buf, size_t len) {
  if (!buf || len<100) return;
  if (!jpeg.openRAM(buf,len,jpegDrawCallback)) return;
  jpeg.setPixelType(RGB565_BIG_ENDIAN);
  int iw=jpeg.getWidth();
  int scale=0;
  if (iw>640) scale=2;
  else if (iw>320) scale=1;
  M5.Display.setClipRect(0, CONTENT_TOP, CONTENT_W, CONTENT_H);
  jpeg.decode(0, CONTENT_TOP, scale);
  jpeg.close();
  M5.Display.clearClipRect();
}

bool fetchSnapshot() {
  if (!g_wifiOk||!g_jpgBuf) return false;
  HTTPClient http;
  http.begin("http://"+cfgCamIp+"/snapshot");
  http.setTimeout(6000);
  if (http.GET()!=200){http.end();return false;}
  g_jpgLen=0; g_hasSnapPreview=false;
  WiFiClient *stream=http.getStreamPtr();
  uint32_t t0=millis();
  while (millis()-t0<5000 && g_jpgLen<g_jpgCap) {
    size_t a=stream->available();
    if (a) g_jpgLen+=stream->readBytes(g_jpgBuf+g_jpgLen,(size_t)min((size_t)a,g_jpgCap-g_jpgLen));
    else delay(1);
  }
  http.end();
  if (g_jpgLen<100) return false;
  g_hasSnapPreview=true;
  decodeJpegFit(g_jpgBuf,g_jpgLen);
  return true;
}

bool saveLastSnapshot() {
  if (!g_hasSnapPreview||!g_sdOk||!g_jpgBuf||g_jpgLen<100) return false;
  ensureDirs();
  char name[64];
  snprintf(name,sizeof(name),"/images/snap_%lu.jpg",(unsigned long)millis());
  File f=SD.open(name,FILE_WRITE);
  if (!f) return false;
  size_t w=f.write(g_jpgBuf,g_jpgLen);
  f.close();
  return w==g_jpgLen;
}

void streamFrameTick() {
  static uint32_t last=0;
  if (millis()-last<50) return; // ~20 fps cap
  last=millis();
  if (!g_jpgBuf) return;

  HTTPClient http;
  http.begin("http://"+cfgCamIp+":8081/stream");
  http.setTimeout(2000);
  if (http.GET()!=200){http.end();return;}
  WiFiClient *stream=http.getStreamPtr();
  g_jpgLen=0; bool inFrame=false;
  uint32_t t0=millis();
  while (millis()-t0<1500 && g_jpgLen<g_jpgCap) {
    if (!stream->available()){delay(0);continue;}
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

  decodeJpegFit(g_jpgBuf,g_jpgLen);

  // REC badge only — no full footer/header redraw
  if (g_recActive) {
    uint32_t sec=(millis()-g_recStartMs)/1000;
    M5.Display.fillRoundRect(8,CONTENT_TOP+2,120,16,2,COL_REC);
    M5.Display.setTextColor(COL_TEXT);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(14,CONTENT_TOP+5);
    M5.Display.printf("REC %02lu:%02lu f:%lu",sec/60,sec%60,(unsigned long)g_recFrames);
    if (g_recFile) { g_recFile.write(g_jpgBuf,g_jpgLen); g_recFrames++; }
  }
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
  M5.Speaker.end();
  if (!M5.Mic.isEnabled()) M5.Mic.begin();
  g_audioBufSamples=0; g_audioRecActive=true; g_audioRecStart=millis();
}
void stopAudioRec() {
  if (!g_audioRecActive) return;
  g_audioRecActive=false;
  if (g_screen!=SCR_AUDIO_REC) { M5.Mic.end(); M5.Speaker.begin(); }
  if (!g_sdOk||g_audioBufSamples==0) return;
  ensureDirs();
  char name[64];
  snprintf(name,sizeof(name),"/audio/rec_%lu.wav",(unsigned long)millis());
  File f=SD.open(name,FILE_WRITE);
  if (!f) return;
  uint32_t db=g_audioBufSamples*sizeof(int16_t);
  writeWavHeader(f,db);
  f.write((uint8_t*)g_audioBuf,db);
  f.close();
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
  size_t got=f.read((uint8_t*)g_audioBuf,(size_t)min((size_t)(f.size()-44),(size_t)(AUDIO_SR*22*sizeof(int16_t))));
  f.close();
  M5.Mic.end(); M5.Speaker.begin(); M5.Speaker.setVolume(180);
  M5.Speaker.playRaw(g_audioBuf,got/sizeof(int16_t),AUDIO_SR,false);
  g_audioPlaying=true;
}
