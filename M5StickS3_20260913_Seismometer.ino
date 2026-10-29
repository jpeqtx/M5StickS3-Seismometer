#include <M5Unified.h>
#include <M5StickS3_ENV_Hat_III.h>
#include <Network.h>
#include <esp32-hal-psram.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <time.h>
#include <math.h>
#include "config.h"
#include "filters.h"
#include "detector.h"

M5StickS3_ENV_Hat_III env;
WebServer   server(WEB_PORT);

SeismicIntensity g_intensity;
SpectrumAnalyzer g_fft;
StaLtaDetector   g_stalta;
FusionJudge      g_fusion;

float offsetX = 0, offsetY = 0, offsetZ = 0;
float curTemp = 0, curHum = 0, curPress = 0;
bool  envOK = false;

unsigned long lastDisplay   = 0;
unsigned long lastEnvUpdate = 0;
unsigned long lastStaRetry  = 0;
unsigned long sleepTimer    = 0;
bool          sleeping      = false;

M5Canvas canvas(&M5.Lcd);

struct Sample { float x, y, z; uint32_t t_us; };
Sample         sampleBuf[SAMPLE_BUF_SIZE];
volatile int   sampleHead = 0;
volatile int   sampleTail = 0;
volatile int   sampleCount = 0;
portMUX_TYPE   sampleMux = portMUX_INITIALIZER_UNLOCKED;

bool timeSynced    = false;
bool staConnected  = false;
String currentSSID = "";
String staIP       = "";
struct tm timeinfo;

float intOffset   = 0.0f;
float calMeasured = 0.0f;
float calActual   = 0.0f;
int   calCount    = 0;

int wakeMode = 2;

uint8_t       cfgBrightnessNormal  = BRIGHTNESS_NORMAL;
unsigned long cfgSleepDelayMs      = SLEEP_DELAY_MS;
unsigned long cfgDisplayIntervalMs = DISPLAY_INTERVAL_MS;
float         cfgStaLtaTrigger     = STA_LTA_TRIGGER;
float         cfgStaLtaDeactivate  = STA_LTA_DEACTIVATE;
float         cfgPgaTriggerGal     = PGA_TRIGGER_GAL;
float         cfgSeisIntTrigger    = SEIS_INT_TRIGGER;
int           cfgRequiredVotes     = REQUIRED_VOTES;
bool          cfgShowWarming       = true;
bool          cfgBeepEnabled       = false;
uint8_t       cfgBeepVolume        = 64;
uint8_t       cfgWavVolume         = 128;

bool          pauseMode       = false;
unsigned long pauseStartTime  = 0;
constexpr unsigned long PAUSE_AUTO_RESUME_MS = 300000;
unsigned long bootCompletedTime = 0;
constexpr unsigned long BOOT_GRACE_MS = 10000;

bool          beepActive         = false;
int           beepLevel          = 0;
unsigned long beepStartTime      = 0;
unsigned long beepStepStart      = 0;
int           beepStepIndex      = 0;
bool          beepToneOn         = false;
int           lastTriggeredLevel = 0;
unsigned long levelZeroTime      = 0;
unsigned long lastTriggeredTime  = 0;
int           sustainedLevel     = 0;
unsigned long sustainedStart     = 0;
constexpr unsigned long SUSTAIN_MS = 200;
constexpr unsigned long TRIGGER_RESET_MS = 30000;

uint8_t*      g_wavBuf        = nullptr;
size_t        g_wavBufSize    = 0;
size_t        g_wavBufMax     = 0;
unsigned long g_wavStartTime  = 0;
unsigned long g_wavDelayUntil = 0;
int           pendingWavLevel = 0;
int           wavPlayCount    = 0;
bool          wavPlaying      = false;
constexpr unsigned long WAV_MAX_DURATION_MS = 12000;
constexpr unsigned long BEEP_LONG_DURATION    = 2000;
constexpr uint16_t       BEEP_TONE_FREQ       = 2000;
constexpr uint16_t       BEEP_SHORT_DURATION  = 80;
constexpr uint16_t       BEEP_GAP_DURATION    = 100;
constexpr uint16_t       BEEP_TAIL_SILENCE    = 300;

float cachedTemp = -999, cachedHum = -999, cachedPress = -999;
float cachedIntensity = -999, cachedPga = -999;
int   cachedVotes = -1;
bool  cachedWarmup = false;
bool  cachedSta = false;
bool  cachedPause = false;
float cachedFftEnergy = -999;
char  cachedDateStr[32] = "";
char  cachedTimeStr[16] = "";

void sampleTask(void* pvParam) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ);
    while (true) {
        vTaskDelayUntil(&lastWake, period);
        if (M5.Imu.update()) {
            auto d = M5.Imu.getImuData();
            portENTER_CRITICAL(&sampleMux);
            int next = (sampleHead + 1) % SAMPLE_BUF_SIZE;
            if (next != sampleTail) {
                sampleBuf[sampleHead].x = d.accel.x * 980.665f;
                sampleBuf[sampleHead].y = d.accel.y * 980.665f;
                sampleBuf[sampleHead].z = d.accel.z * 980.665f;
                sampleBuf[sampleHead].t_us = (uint32_t)esp_timer_get_time();
                sampleHead = next;
                if (sampleCount < SAMPLE_BUF_SIZE) sampleCount++;
            }
            portEXIT_CRITICAL(&sampleMux);
        }
    }
}
void saveWiFiCreds(const String& ssid, const String& pass) {
    Preferences prefs;
    prefs.begin("wifi_cfg", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
}
bool loadWiFiCreds(String& ssid, String& pass) {
    Preferences prefs;
    prefs.begin("wifi_cfg", true);
    ssid = prefs.getString("ssid", "");
    pass = prefs.getString("pass", "");
    prefs.end();
    return ssid.length() > 0;
}
void saveCalibration() {
    Preferences prefs;
    prefs.begin("seismo_cal", false);
    prefs.putFloat("int_offset", intOffset);
    prefs.putFloat("cal_meas",   calMeasured);
    prefs.putFloat("cal_actual", calActual);
    prefs.putInt  ("cal_count",  calCount);
    prefs.putInt  ("wake_mode",  wakeMode);
    prefs.end();
}
void loadCalibration() {
    Preferences prefs;
    prefs.begin("seismo_cal", true);
    intOffset   = prefs.getFloat("int_offset", 0.0f);
    calMeasured = prefs.getFloat("cal_meas",   0.0f);
    calActual   = prefs.getFloat("cal_actual", 0.0f);
    calCount    = prefs.getInt  ("cal_count",  0);
    wakeMode    = prefs.getInt  ("wake_mode",  2);
    prefs.end();
}
void saveConfig() {
    Preferences prefs;
    prefs.begin("seismo_cfg", false);
    prefs.putUChar("bright",     cfgBrightnessNormal);
    prefs.putULong("sleep_ms",   cfgSleepDelayMs);
    prefs.putULong("disp_ms",    cfgDisplayIntervalMs);
    prefs.putFloat("sta_trig",   cfgStaLtaTrigger);
    prefs.putFloat("sta_deact",  cfgStaLtaDeactivate);
    prefs.putFloat("pga_trig",   cfgPgaTriggerGal);
    prefs.putFloat("int_trig",   cfgSeisIntTrigger);
    prefs.putInt  ("req_votes",  cfgRequiredVotes);
    prefs.putBool ("show_warm",  cfgShowWarming);
    prefs.putBool ("beep_en",    cfgBeepEnabled);
    prefs.putUChar("beep_vol",   cfgBeepVolume);
    prefs.putUChar("wav_vol",    cfgWavVolume);
    prefs.end();
}
void loadConfig() {
    Preferences prefs;
    prefs.begin("seismo_cfg", true);
    cfgBrightnessNormal  = prefs.getUChar("bright",    BRIGHTNESS_NORMAL);
    cfgSleepDelayMs      = prefs.getULong("sleep_ms",  SLEEP_DELAY_MS);
    cfgDisplayIntervalMs = prefs.getULong("disp_ms",   DISPLAY_INTERVAL_MS);
    cfgStaLtaTrigger     = prefs.getFloat("sta_trig",  STA_LTA_TRIGGER);
    cfgStaLtaDeactivate  = prefs.getFloat("sta_deact", STA_LTA_DEACTIVATE);
    cfgPgaTriggerGal     = prefs.getFloat("pga_trig",  PGA_TRIGGER_GAL);
    cfgSeisIntTrigger    = prefs.getFloat("int_trig",  SEIS_INT_TRIGGER);
    cfgRequiredVotes     = prefs.getInt  ("req_votes", REQUIRED_VOTES);
    cfgShowWarming       = prefs.getBool ("show_warm", true);
    cfgBeepEnabled       = prefs.getBool ("beep_en",   false);
    cfgBeepVolume        = prefs.getUChar("beep_vol",  64);
    cfgWavVolume         = prefs.getUChar("wav_vol",   128);
    prefs.end();
    g_stalta.setThresholds(cfgStaLtaTrigger, cfgStaLtaDeactivate);
    g_fusion.setThresholds(cfgPgaTriggerGal, cfgSeisIntTrigger, cfgRequiredVotes);
}
void playWavForLevel(int level) {
    const char* path = nullptr;
    switch (level) {
        case 2: path = "/i2.wav"; break;
        case 3: path = "/i3.wav"; break;
        case 4: path = "/i4.wav"; break;
        case 5: path = "/i5.wav"; break;
        default: return;
    }
    if (!g_wavBuf) { wavPlaying = false; return; }
    File f = LittleFS.open(path, "r");
    if (!f) { wavPlaying = false; return; }
    size_t size = f.size();
    if (size > g_wavBufMax) { f.close(); wavPlaying = false; return; }
    size_t read = f.read(g_wavBuf, size);
    f.close();
    g_wavBufSize = read;
    M5.Speaker.setVolume(cfgWavVolume);
    bool ok = M5.Speaker.playWav(g_wavBuf, g_wavBufSize, 1, -1, true);
    wavPlaying = ok;
    g_wavStartTime = millis();
}
void stopWav() {
    wavPlaying = false; g_wavBufSize = 0; pendingWavLevel = 0; wavPlayCount = 0; g_wavDelayUntil = 0;
}
void startBeep(int level) {
    if (level < 2) return;
    if (pauseMode) return;
    M5.Speaker.stop(); delay(5);
    beepActive = true; beepLevel = level; beepStartTime = millis();
    beepStepStart = millis(); beepStepIndex = 0; beepToneOn = false;
    M5.Speaker.setVolume(cfgBeepVolume);
    if (level >= 5) { M5.Speaker.tone(BEEP_TONE_FREQ, 100); beepToneOn = true; }
    else { M5.Speaker.tone(BEEP_TONE_FREQ, BEEP_SHORT_DURATION); beepToneOn = true; }
}
void stopBeep() { M5.Speaker.stop(); beepActive = false; beepToneOn = false; }
void updateBeep() {
    if (!beepActive) return;
    unsigned long now = millis(); unsigned long elapsed = now - beepStartTime;
    if (beepLevel >= 5) {
        if (elapsed >= BEEP_LONG_DURATION) {
            M5.Speaker.stop(); beepToneOn = false; beepActive = false;
            pendingWavLevel = 5; wavPlayCount = 0; return;
        }
        bool wantOn = ((elapsed % 150) < 100);
        if (wantOn && !beepToneOn) { M5.Speaker.tone(BEEP_TONE_FREQ, 100); beepToneOn = true; }
        else if (!wantOn && beepToneOn) { beepToneOn = false; }
        return;
    }
    int totalSteps = beepLevel * 2;
    if (beepStepIndex >= totalSteps) { beepActive = false; pendingWavLevel = beepLevel; wavPlayCount = 0; return; }
    unsigned long stepDur;
    if (beepStepIndex == totalSteps - 1) stepDur = BEEP_TAIL_SILENCE;
    else stepDur = (beepStepIndex % 2 == 0) ? BEEP_SHORT_DURATION : BEEP_GAP_DURATION;
    if (now - beepStepStart >= stepDur) {
        beepStepIndex++; beepStepStart = now;
        if (beepStepIndex >= totalSteps) { beepActive = false; pendingWavLevel = beepLevel; wavPlayCount = 0; return; }
        if (beepStepIndex % 2 == 0) { M5.Speaker.tone(BEEP_TONE_FREQ, BEEP_SHORT_DURATION); beepToneOn = true; }
        else { beepToneOn = false; }
    }
}
void updateWav() {
    if (pauseMode) { if (wavPlaying) { M5.Speaker.stop(); stopWav(); } return; }
    if (g_wavDelayUntil > 0) {
        if (millis() >= g_wavDelayUntil) { g_wavDelayUntil = 0; playWavForLevel(pendingWavLevel); }
        return;
    }
    if (wavPlaying) {
        unsigned long elapsed = millis() - g_wavStartTime;
        bool done = !M5.Speaker.isPlaying(); bool timeout = (elapsed > WAV_MAX_DURATION_MS);
        if (done || timeout) {
            if (timeout) M5.Speaker.stop();
            wavPlaying = false; g_wavBufSize = 0;
            if (pendingWavLevel >= 5 && wavPlayCount < 1) { wavPlayCount++; g_wavDelayUntil = millis() + 500; return; }
            pendingWavLevel = 0; wavPlayCount = 0;
        }
        return;
    }
    if (pendingWavLevel > 0) playWavForLevel(pendingWavLevel);
}
void triggerBeepIfNeeded(float instantIntensity) {
    if (bootCompletedTime == 0 || (millis() - bootCompletedTime) < BOOT_GRACE_MS) return;
    if (pauseMode) {
        if (beepActive) stopBeep(); if (wavPlaying) { M5.Speaker.stop(); stopWav(); } sustainedLevel = 0; return;
    }
    if (!cfgBeepEnabled) {
        if (beepActive) stopBeep(); if (wavPlaying) { M5.Speaker.stop(); stopWav(); } sustainedLevel = 0; return;
    }
    int level = 0;
    if (instantIntensity >= 4.5f) level = 5;
    else if (instantIntensity >= 3.5f) level = 4;
    else if (instantIntensity >= 2.5f) level = 3;
    else if (instantIntensity >= 1.5f) level = 2;
    if (level > 0) {
        if (level != sustainedLevel) { sustainedLevel = level; sustainedStart = millis(); return; }
        if (millis() - sustainedStart < SUSTAIN_MS) return;
        bool shouldTrigger = (level > lastTriggeredLevel);
        if (!shouldTrigger && lastTriggeredTime > 0 && (millis() - lastTriggeredTime > TRIGGER_RESET_MS)) shouldTrigger = true;
        if (shouldTrigger) {
            lastTriggeredLevel = level; lastTriggeredTime = millis();
            if (wavPlaying) { M5.Speaker.stop(); stopWav(); }
            startBeep(level);
        }
    } else {
        sustainedLevel = 0; sustainedStart = 0;
        if (lastTriggeredLevel > 0 && lastTriggeredTime > 0 && (millis() - lastTriggeredTime > TRIGGER_RESET_MS)) {
            lastTriggeredLevel = 0; lastTriggeredTime = 0;
        }
    }
}
void calibrateIMU() {
    M5.Lcd.fillScreen(BLACK); M5.Lcd.setTextColor(WHITE); M5.Lcd.setFont(&fonts::Font2); M5.Lcd.setCursor(4, 4);
    M5.Lcd.printf("Calibrating IMU...");
    float sx = 0, sy = 0, sz = 0; int n = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (M5.Imu.update()) {
            auto d = M5.Imu.getImuData();
            sx += d.accel.x * 980.665f; sy += d.accel.y * 980.665f; sz += d.accel.z * 980.665f; n++;
        }
        delay(10); yield();
    }
    if (n > 0) { offsetX = sx / n; offsetY = sy / n; offsetZ = sz / n; }
    M5.Lcd.setCursor(4, 40); M5.Lcd.printf("Done."); delay(300);
}
bool tryConnectSTA(unsigned long timeoutMs) {
    String ssid, pass; if (!loadWiFiCreds(ssid, pass)) return false;
    WiFi.disconnect(); delay(200); WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) { delay(100); yield(); }
    if (WiFi.status() == WL_CONNECTED) { staIP = WiFi.localIP().toString(); return true; }
    return false;
}
void syncNTP() {
    if (WiFi.status() != WL_CONNECTED) return;
    configTime(NTP_TZ_OFFSET, 0, NTP_SERVER_1, NTP_SERVER_2);
    unsigned long t0 = millis();
    while (!getLocalTime(&timeinfo) && millis() - t0 < 5000) { delay(100); yield(); }
    if (getLocalTime(&timeinfo)) timeSynced = true;
}
void initWiFiHybrid() {
    M5.Lcd.setCursor(4, 60); M5.Lcd.printf("WiFi hybrid...");
    setenv("TZ", "JST-9", 1); tzset();
    WiFi.mode(WIFI_AP_STA); WiFi.softAP(AP_SSID, AP_PASS); IPAddress apIP = WiFi.softAPIP();
    staConnected = tryConnectSTA(STA_CONNECT_TIMEOUT_MS);
    if (staConnected) { currentSSID = WiFi.SSID(); syncNTP(); }
    M5.Lcd.fillScreen(BLACK); M5.Lcd.setCursor(4, 4); M5.Lcd.setTextColor(TFT_GREEN); M5.Lcd.printf("AP: %s", AP_SSID);
    M5.Lcd.setCursor(4, 26); M5.Lcd.setTextColor(TFT_WHITE); M5.Lcd.printf("URL: %s", apIP.toString().c_str());
    M5.Lcd.setCursor(4, 48); M5.Lcd.setTextColor(staConnected ? TFT_CYAN : TFT_ORANGE);
    M5.Lcd.printf("STA: %s", staConnected ? staIP.c_str() : "not connected");
    delay(2000);
}
void setManualTime(int year, int month, int day, int hour, int minute, int second) {
    setenv("TZ", "JST-9", 1); tzset();
    struct tm t = {}; t.tm_year = year - 1900; t.tm_mon = month - 1; t.tm_mday = day;
    t.tm_hour = hour; t.tm_min = minute; t.tm_sec = second; t.tm_isdst = 0;
    time_t epoch = mktime(&t); struct timeval now = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&now, nullptr); timeSynced = true;
}
String htmlPage() {
    String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    h += "<style>body{font-family:sans-serif;margin:20px;background:#222;color:#eee}";
    h += "input,select{width:100%;padding:8px;margin:4px 0;box-sizing:border-box}";
    h += "button{padding:10px 20px;margin:8px 4px;font-size:16px}";
    h += "hr{border-color:#444}";
    h += ".ok{color:#4caf50}.ng{color:#f44336}";
    h += ".info{background:#333;padding:8px;border-radius:4px;margin:8px 0}";
    h += ".instant{color:#4fc3f7;font-weight:bold}";
    h += ".reboot{color:#ffb74d;font-weight:bold}";
    h += ".warn{color:#ff9800;font-weight:bold}</style></head><body>";
    h += "<h2>M5StickS3 Seismometer</h2>";
    h += "<div class='info' style='background:#0d47a1'>Filter: Kunugi 2012 (2nd-order x6) + FFT</div>";
    if (pauseMode) h += "<div class='info' style='background:#a33;font-size:20px;text-align:center'><b>PAUSED</b> - Press blue button to resume</div>";
    float cpuT = temperatureRead(); float envT = envOK ? env.getRawTemperatureSHT() : 0.0f;
    h += "<h3>Status</h3><div class='info'>";
    h += "ENV Temp (raw): " + String(envT, 1) + " C<br>"; h += "CPU Temp: " + String(cpuT, 1) + " C<br>";
    h += "CPU-ENV Diff: " + String(cpuT - envT, 1) + " C<br>";
    h += "Thermal Equilibrium: " + String(env.isThermalEquilibrium() ? "YES" : "NO") + "<br>";
    h += "Base Offset: " + String(env.getBaseTempOffset(), 2) + " C<br>";
    h += "Effective Offset: " + String(env.getEffectiveTempOffset(), 2) + " C<br>";
    h += "Noise Floor: " + String(g_intensity.noiseFloor, 2) + " gal<br>";
    h += "Current PGA: " + String(g_intensity.peakAccel, 2) + " gal<br>";
    h += "Instant Intensity: " + String(g_intensity.getInstantIntensity(), 2) + "<br>";
    h += "Sustained Level: " + String(sustainedLevel) + "<br>";
    h += "Pause Mode: <b>" + String(pauseMode ? "ON" : "OFF") + "</b></div>";
    h += "<h3>FFT Analysis</h3><div class='info'>";
    h += "Band Energy (0.5-10Hz): " + String(g_fft.bandEnergy, 2) + "<br>";
    h += "Dominant Freq: " + String(g_fft.dominantFreq, 2) + " Hz<br>";
    h += "FFT Vote: <b>" + String(g_fft.fftVote ? "YES" : "NO") + "</b> (threshold " + String(FFT_NOISE_THRESHOLD, 2) + ")</div>";
    h += "<h3>Seismic Status</h3><div class='info'>";
    h += "Raw Intensity: " + String(g_intensity.currentIntensity, 2) + "<br>";
    h += "<b class='warn'>Calibration Offset: " + String(intOffset, 2) + "</b><br>";
    float correctedDisp = g_intensity.currentIntensity + intOffset; if (correctedDisp < 0) correctedDisp = 0;
    h += "Displayed Intensity: " + String(correctedDisp, 2) + "<br>"; h += "Correction Count: " + String(calCount) + "<br>";
    if (fabs(intOffset) > 0.01f) h += "<span class='ng'>WARNING: Offset is not 0.</span>";
    h += "</div>";
    h += "<h3>WiFi Status</h3><div class='info'>";
    h += "AP SSID: <b>" + String(AP_SSID) + "</b><br>"; h += "AP IP: " + WiFi.softAPIP().toString() + "<br>";
    if (staConnected) h += "STA: <span class='ok'>Connected</span> (" + currentSSID + " / " + staIP + ")";
    else h += "STA: <span class='ng'>Disconnected</span>";
    h += "</div>";
    h += "<h3>Home WiFi Setting <span class='instant'>[instant]</span></h3>";
    h += "<form action='/wifi' method='POST'>SSID: <input name='ssid' value='" + currentSSID + "'><br>";
    h += "Password: <input name='pass' type='password'><br><button type='submit'>Connect & Save</button></form><hr>";
    h += "<h3>Manual Time Setting (JST) <span class='instant'>[instant]</span></h3>";
    h += "<form action='/settime' method='POST'><input name='datetime' type='text' placeholder='2026-09-13 14:30:00'><br>";
    h += "<button type='submit'>Set Time</button></form><hr>";
    h += "<h3>Wake Mode <span class='instant'>[instant]</span></h3>";
    h += "<form action='/wakemode' method='POST'><select name='mode'>";
    h += "<option value='0'" + String(wakeMode == 0 ? " selected" : "") + ">Always On</option>";
    h += "<option value='1'" + String(wakeMode == 1 ? " selected" : "") + ">QUAKE only</option>";
    h += "<option value='2'" + String(wakeMode == 2 ? " selected" : "") + ">QUAKE + Intensity 1.5+</option>";
    h += "</select><br><button type='submit'>Apply</button></form><hr>";
    h += "<h3>Brightness <span class='instant'>[instant]</span></h3>";
    h += "<form action='/brightness' method='POST'>Brightness (0-255): <input name='bright' type='number' min='0' max='255' value='" + String(cfgBrightnessNormal) + "'><br>";
    h += "<button type='submit'>Apply</button></form><hr>";
    h += "<h3>Sound Alert <span class='instant'>[instant]</span></h3>";
    h += "<form action='/beep' method='POST'><label><input type='checkbox' name='en' value='1' " + String(cfgBeepEnabled ? "checked" : "") + "> Enable Sound Alerts</label><br><br>";
    h += "<b>BEEP Volume (0-255):</b><br><input name='vol' type='number' min='0' max='255' value='" + String(cfgBeepVolume) + "'><br>";
    h += "<b>WAV Volume (0-255):</b><br><input name='wav_vol' type='number' min='0' max='255' value='" + String(cfgWavVolume) + "'><br>";
    h += "<small>BEEP and WAV volumes are independent.</small><br><button type='submit'>Apply</button></form>";
    h += "<div class='info'><b>Patterns:</b><br>Shindo 2: BEEP x2 + 'Intensity 2'<br>Shindo 3: BEEP x3 + 'Intensity 3'<br>";
    h += "Shindo 4: BEEP x4 + 'Intensity 4'<br>Shindo 5+: BEEP 2s + 'Major Earthquake' x2</div>";
    h += "<small>200ms sustained + 10s boot grace. Press blue button to pause.</small><hr>";
    h += "<h3>Warming Display <span class='instant'>[instant]</span></h3>";
    h += "<form action='/warming' method='POST'><label><input type='checkbox' name='show' value='1' " + String(cfgShowWarming ? "checked" : "") + "> Show 'Warming' indicator</label><br>";
    h += "<button type='submit'>Apply</button></form><hr>";
    h += "<h3>Detection Settings <span class='instant'>[instant]</span></h3>";
    h += "<form action='/config' method='POST'>Sleep Delay (ms): <input name='sleep_ms' type='number' min='1000' max='600000' value='" + String(cfgSleepDelayMs) + "'><br>";
    h += "Display Interval (ms): <input name='disp_ms' type='number' min='200' max='10000' value='" + String(cfgDisplayIntervalMs) + "'><br>";
    h += "STA/LTA Trigger: <input name='sta_trig' type='number' step='0.1' value='" + String(cfgStaLtaTrigger, 1) + "'><br>";
    h += "STA/LTA Deactivate: <input name='sta_deact' type='number' step='0.1' value='" + String(cfgStaLtaDeactivate, 1) + "'><br>";
    h += "PGA Trigger (gal): <input name='pga_trig' type='number' step='0.1' value='" + String(cfgPgaTriggerGal, 1) + "'><br>";
    h += "Intensity Trigger: <input name='int_trig' type='number' step='0.1' value='" + String(cfgSeisIntTrigger, 1) + "'><br>";
    h += "Required Votes (1-4): <input name='req_votes' type='number' min='1' max='4' value='" + String(cfgRequiredVotes) + "'><br>";
    h += "<button type='submit'>Apply Settings</button></form><hr>";
    h += "<h3>Intensity Calibration <span class='instant'>[instant]</span></h3>";
    h += "<form action='/calibrate' method='POST'>Measured Intensity: <input name='meas' type='number' step='0.01' value='" + String(g_intensity.currentIntensity, 2) + "'><br>";
    h += "Actual Intensity (JMA): <input name='actual' type='number' step='0.01' value='" + String(calActual, 2) + "'><br>";
    h += "<button type='submit'>Apply Calibration</button></form>";
    h += "<form action='/calreset' method='POST'><button type='submit' style='background:#a33'>Reset Calibration</button></form><hr>";
    h += "<h3>Environmental Calibration <span class='reboot'>[reboot]</span></h3>";
    h += "<form action='/save' method='POST'>Actual Temp: <input name='actual_temp' type='number' step='0.1' value='" + String(env.getActualTempRef(), 1) + "'><br>";
    h += "Temp Offset (C): <input name='t_off' type='number' step='0.1' value='" + String(env.getBaseTempOffset(), 1) + "'><br>";
    h += "Hum Offset (%RH): <input name='h_off' type='number' step='0.1' value='" + String(env.getHumOffset(), 1) + "'><br>";
    h += "Press Offset (hPa): <input name='p_off' type='number' step='0.1' value='" + String(env.getPressOffset(), 1) + "'><br>";
    h += "<label><input type='checkbox' name='ac_en' value='1' " + String(env.isAutoCompEnabled() ? "checked" : "") + "> Auto Correction</label><br>";
    h += "<label><input type='checkbox' name='use_actual' value='1' checked> Use actual temp</label><br>";
    h += "<button type='submit'>Save & Reboot</button></form>";
    h += "<small>Environmental calibration requires a reboot (library reinitialization).</small>";
    h += "</body></html>";
    return h;
}
void handleRoot() { server.send(200, "text/html", htmlPage()); }
void handleSave() {
    stopBeep(); stopWav();
    float tManual = server.arg("t_off").toFloat(); float hOff = server.arg("h_off").toFloat();
    float pOff = server.arg("p_off").toFloat(); float actualT = server.arg("actual_temp").toFloat();
    bool ac = server.hasArg("ac_en"); bool useActual = server.hasArg("use_actual");
    if (useActual && envOK) env.setActualTemp(actualT); else env.setBaseOffsets(tManual, hOff, pOff);
    env.setHumOffset(hOff); env.setPressOffset(pOff); env.enableAutoComp(ac);
    server.send(200, "text/html", "<html><body><h2>Saved. Rebooting...</h2></body></html>");
    delay(1000); ESP.restart();
}
void handleConfig() {
    unsigned long sleepMs = server.arg("sleep_ms").toInt(); unsigned long dispMs = server.arg("disp_ms").toInt();
    float staTrig = server.arg("sta_trig").toFloat(); float staDeact = server.arg("sta_deact").toFloat();
    float pgaTrig = server.arg("pga_trig").toFloat(); float intTrig = server.arg("int_trig").toFloat();
    int reqVotes = server.arg("req_votes").toInt();
    if (sleepMs < 1000) sleepMs = 1000; if (dispMs < 200) dispMs = 200;
    if (reqVotes < 1) reqVotes = 1; if (reqVotes > 4) reqVotes = 4;
    cfgSleepDelayMs = sleepMs; cfgDisplayIntervalMs = dispMs; cfgStaLtaTrigger = staTrig;
    cfgStaLtaDeactivate = staDeact; cfgPgaTriggerGal = pgaTrig; cfgSeisIntTrigger = intTrig; cfgRequiredVotes = reqVotes;
    g_stalta.setThresholds(cfgStaLtaTrigger, cfgStaLtaDeactivate); g_fusion.setThresholds(cfgPgaTriggerGal, cfgSeisIntTrigger, cfgRequiredVotes);
    saveConfig();
    server.send(200, "text/html", "<html><body><h2>Settings Applied</h2><p>No reboot required.</p><p><a href='/'>Back</a></p></body></html>");
}
void handleBrightness() {
    int bright = server.arg("bright").toInt(); if (bright < 0) bright = 0; if (bright > 255) bright = 255;
    cfgBrightnessNormal = (uint8_t)bright; if (!sleeping) M5.Lcd.setBrightness(cfgBrightnessNormal); saveConfig();
    server.send(200, "text/html", "<html><body><h2>Brightness: " + String(bright) + "</h2><p><a href='/'>Back</a></p></body></html>");
}
void handleBeep() {
    bool en = server.hasArg("en"); cfgBeepEnabled = en;
    if (server.hasArg("vol")) { int v = server.arg("vol").toInt(); if (v < 0) v = 0; if (v > 255) v = 255; cfgBeepVolume = (uint8_t)v; }
    if (server.hasArg("wav_vol")) { int v = server.arg("wav_vol").toInt(); if (v < 0) v = 0; if (v > 255) v = 255; cfgWavVolume = (uint8_t)v; }
    if (!cfgBeepEnabled) { if (beepActive) stopBeep(); if (wavPlaying) { M5.Speaker.stop(); stopWav(); } }
    saveConfig();
    server.send(200, "text/html", "<html><body><h2>Sound Alert Updated</h2><p><a href='/'>Back</a></p></body></html>");
}
void handleWarming() {
    cfgShowWarming = server.hasArg("show"); saveConfig();
    server.send(200, "text/html", "<html><body><h2>Warming Display: " + String(cfgShowWarming ? "ON" : "OFF") + "</h2><p><a href='/'>Back</a></p></body></html>");
}
void handleCalibrate() {
    float meas = server.arg("meas").toFloat(); float actual = server.arg("actual").toFloat();
    intOffset = actual - meas; calMeasured = meas; calActual = actual; calCount++; saveCalibration();
    server.send(200, "text/html", "<html><body><h2>Calibration Applied</h2><p>Offset: " + String(intOffset, 2) + "<br>Count: " + String(calCount) + "</p><p><a href='/'>Back</a></p></body></html>");
}
void handleCalReset() {
    intOffset = 0.0f; calMeasured = 0.0f; calActual = 0.0f; calCount = 0; saveCalibration();
    server.send(200, "text/html", "<html><body><h2>Calibration Reset</h2><p><a href='/'>Back</a></p></body></html>");
}
void handleWakeMode() {
    int mode = server.arg("mode").toInt(); if (mode < 0 || mode > 2) mode = 2;
    wakeMode = mode; saveCalibration();
    server.send(200, "text/html", "<html><body><h2>Wake Mode Updated</h2><p><a href='/'>Back</a></p></body></html>");
}
void handleSetTime() {
    String dt = server.arg("datetime"); int y, mo, d, hh, mi, ss;
    if (sscanf(dt.c_str(), "%d-%d-%d %d:%d:%d", &y, &mo, &d, &hh, &mi, &ss) == 6) {
        setManualTime(y, mo, d, hh, mi, ss);
        server.send(200, "text/html", "<html><body><h2>Time set. <a href='/'>Back</a></h2></body></html>");
    } else {
        server.send(400, "text/html", "<html><body><h2>Invalid format.</h2><p><a href='/'>Back</a></p></body></html>");
    }
}
void handleWiFi() {
    stopBeep(); stopWav(); String ssid = server.arg("ssid"); String pass = server.arg("pass");
    if (ssid.length() == 0) { server.send(400, "text/html", "<html><body><h2>SSID required.</h2><p><a href='/'>Back</a></p></body></html>"); return; }
    saveWiFiCreds(ssid, pass); currentSSID = ssid;
    server.send(200, "text/html", "<html><body><h2>Connecting to " + ssid + "...</h2><p>Wait ~10 sec, then <a href='/'>reload</a>.</p></body></html>");
    delay(500); WiFi.disconnect(); delay(200); WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < STA_CONNECT_TIMEOUT_MS) { delay(100); yield(); }
    if (WiFi.status() == WL_CONNECTED) { staConnected = true; staIP = WiFi.localIP().toString(); syncNTP(); }
    else { staConnected = false; staIP = ""; }
}
void initWebServer() {
    server.on("/", handleRoot); server.on("/save", HTTP_POST, handleSave);
    server.on("/config", HTTP_POST, handleConfig); server.on("/brightness", HTTP_POST, handleBrightness);
    server.on("/beep", HTTP_POST, handleBeep); server.on("/warming", HTTP_POST, handleWarming);
    server.on("/settime", HTTP_POST, handleSetTime); server.on("/wifi", HTTP_POST, handleWiFi);
    server.on("/calibrate", HTTP_POST, handleCalibrate); server.on("/calreset", HTTP_POST, handleCalReset);
    server.on("/wakemode", HTTP_POST, handleWakeMode); server.begin();
}
void enterSleep() { if (sleeping) return; sleeping = true; M5.Lcd.setBrightness(BRIGHTNESS_SLEEP); }
void wakeUp() { if (!sleeping) return; sleeping = false; M5.Lcd.setBrightness(cfgBrightnessNormal); cachedIntensity = -999; lastDisplay = 0; }
void drawScreen() {
    canvas.fillScreen(BLACK);
    const int w = canvas.width(); const int cx = w / 2;
    uint16_t colPanel = canvas.color565(16, 20, 28); uint16_t colAccent = canvas.color565(0, 210, 255);
    uint16_t colLabel = canvas.color565(140, 150, 165); uint16_t colPause = canvas.color565(255, 60, 60);
    canvas.setTextDatum(top_center);
    if (timeSynced && getLocalTime(&timeinfo)) {
        char dateBuf[32], timeBuf[16];
        strftime(dateBuf, sizeof(dateBuf), "%Y/%m/%d (%a)", &timeinfo);
        strftime(timeBuf, sizeof(timeBuf), "%H:%M", &timeinfo);
        canvas.setFont(&fonts::Font2); canvas.setTextColor(colLabel); canvas.drawString(dateBuf, cx, 2);
        canvas.setFont(&fonts::FreeSansBold18pt7b); canvas.setTextColor(TFT_WHITE); canvas.drawString(timeBuf, cx, 16);
    } else {
        canvas.setFont(&fonts::Font2); canvas.setTextColor(TFT_ORANGE); canvas.drawString("Set time via web", cx, 14);
    }
    canvas.fillRect(0, 44, w, 2, colAccent);
    int envY = 48, envH = 40;
    canvas.fillRoundRect(4, envY, w - 8, envH, 4, colPanel);
    int colW = (w - 8) / 3;
    canvas.setFont(&fonts::Font2); canvas.setTextColor(TFT_CYAN);
    canvas.drawString(String(curTemp, 1) + "C", 4 + colW * 0 + colW / 2, envY + 4);
    canvas.setTextColor(TFT_YELLOW); canvas.drawString(String((int)(curHum + 0.5f)) + "%", 4 + colW * 1 + colW / 2, envY + 4);
    canvas.setTextColor(TFT_GREEN); canvas.drawString(String((int)(curPress + 0.5f)), 4 + colW * 2 + colW / 2, envY + 4);
    canvas.setFont(&fonts::Font2); canvas.setTextColor(colLabel);
    canvas.drawString("TEMP", 4 + colW * 0 + colW / 2, envY + 22);
    canvas.drawString("HUM", 4 + colW * 1 + colW / 2, envY + 22);
    canvas.drawString("hPa", 4 + colW * 2 + colW / 2, envY + 22);
    int pauseY = 92, pauseH = 18;
    if (pauseMode) {
        canvas.fillRoundRect(4, pauseY, w - 8, pauseH, 4, colPause);
        canvas.setFont(&fonts::Font2); canvas.setTextColor(TFT_WHITE); canvas.drawString("PAUSED", cx, pauseY + 2);
    }
    float displayIntensity = g_intensity.currentIntensity + intOffset; if (displayIntensity < 0) displayIntensity = 0;
    const char* shindo = intensityToShindo(displayIntensity); uint16_t sColor = shindoColor(displayIntensity);
    int intY = 114, intH = 74;
    canvas.fillRoundRect(4, intY, w - 8, intH, 4, colPanel); canvas.drawRoundRect(4, intY, w - 8, intH, 4, sColor);
    canvas.setFont(&fonts::Font2); canvas.setTextColor(colLabel); canvas.drawString("INTENSITY", cx, intY + 2);
    canvas.setFont(&fonts::FreeSans24pt7b); canvas.setTextColor(sColor); canvas.drawString(shindo, cx, intY + 14);
    canvas.setFont(&fonts::Font2); canvas.setTextColor(TFT_WHITE); canvas.drawString("Meas " + String(displayIntensity, 2), cx, intY + 54);
    int badgeY = 192, badgeH = 18;
    uint16_t badgeColor = g_fusion.quakeConfirmed ? TFT_RED : colPanel;
    canvas.fillRoundRect(4, badgeY, w - 8, badgeH, badgeH / 2, badgeColor);
    canvas.setFont(&fonts::Font2); canvas.setTextColor(g_fusion.quakeConfirmed ? TFT_WHITE : colLabel);
    if (g_fusion.quakeConfirmed) canvas.drawString("QUAKE " + String(g_fusion.votes) + "/4", cx, badgeY + 2);
    else canvas.drawString("MONITOR " + String(g_fusion.votes) + "/4", cx, badgeY + 2);
    canvas.setFont(&fonts::Font2); canvas.setTextColor(TFT_WHITE);
    canvas.drawString("PGA " + String(g_intensity.peakAccel, 1) + " gal", cx, 214);
    canvas.setTextDatum(top_center); canvas.setFont(&fonts::Font0);
    if (cfgShowWarming && env.isAutoCompEnabled() && !env.isThermalEquilibrium()) {
        canvas.setTextColor(TFT_ORANGE); canvas.drawString("Warming", cx, 231);
    } else {
        canvas.setTextColor(staConnected ? TFT_CYAN : colLabel);
        canvas.drawString(staConnected ? "WiFi OK" : "AP only", cx, 231);
    }
    canvas.pushSprite(0, 0);
}
void setup() {
    auto cfg = M5.config(); M5.begin(cfg); setCpuFrequencyMhz(80); M5.Lcd.setRotation(0);
    Serial.begin(115200); delay(300);
    Serial.println("\n=== M5StickS3 Seismometer (Kunugi 2012 + FFT) ===");
    if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
    else {
        Serial.println("LittleFS mounted");
        File root = LittleFS.open("/"); File f = root.openNextFile();
        while (f) { Serial.printf("  %s (%u bytes)\n", f.name(), f.size()); f = root.openNextFile(); }
    }
    {
        const char* paths[] = {"/i2.wav", "/i3.wav", "/i4.wav", "/i5.wav"}; size_t maxWavSize = 0;
        for (int i = 0; i < 4; i++) {
            File wf = LittleFS.open(paths[i], "r");
            if (wf) { if (wf.size() > maxWavSize) maxWavSize = wf.size(); wf.close(); }
        }
        if (maxWavSize > 0) {
            g_wavBufMax = maxWavSize; g_wavBuf = (uint8_t*)ps_malloc(g_wavBufMax);
            if (g_wavBuf) Serial.printf("WAV buffer allocated: %u bytes (free heap: %u)\n", g_wavBufMax, ESP.getFreeHeap());
            else Serial.printf("FATAL: WAV buffer alloc failed (%u bytes)\n", g_wavBufMax);
        }
    }
    loadCalibration(); loadConfig();
    M5.Lcd.setBrightness(cfgBrightnessNormal); M5.Speaker.setVolume(cfgBeepVolume);
    canvas.setPsram(true); canvas.setColorDepth(16); canvas.createSprite(M5.Lcd.width(), M5.Lcd.height());
    canvas.setTextColor(WHITE);
    M5.Lcd.fillScreen(BLACK); M5.Lcd.setTextColor(WHITE); M5.Lcd.setFont(&fonts::Font2);
    envOK = env.begin();
    if (envOK) { env.update(); curTemp = env.getTemperatureSHT(); curHum = env.getHumidity(); curPress = env.getPressure(); }
    if (!M5.Imu.isEnabled()) { M5.Lcd.fillScreen(BLACK); M5.Lcd.setTextColor(TFT_RED); M5.Lcd.setCursor(4, 4); M5.Lcd.printf("IMU NOT FOUND"); while (true) delay(1000); }
    calibrateIMU(); g_intensity.init(SAMPLE_RATE_HZ); g_fft.init();
    M5.Lcd.fillScreen(BLACK); M5.Lcd.setCursor(4, 4); M5.Lcd.setTextColor(TFT_CYAN); M5.Lcd.printf("Measuring noise floor...");
    M5.Lcd.setCursor(4, 26); M5.Lcd.setTextColor(TFT_WHITE); M5.Lcd.printf("Keep device still");
    g_intensity.measureNoiseFloor(offsetX, offsetY, offsetZ);
    M5.Lcd.fillScreen(BLACK); M5.Lcd.setCursor(4, 4); M5.Lcd.setTextColor(TFT_GREEN); M5.Lcd.printf("Noise floor measured");
    M5.Lcd.setCursor(4, 26); M5.Lcd.setTextColor(TFT_WHITE); M5.Lcd.printf("Floor: %.2f gal", g_intensity.noiseFloor); delay(1500);
    M5.Lcd.fillScreen(BLACK); initWiFiHybrid(); initWebServer(); g_stalta.init();
    M5.Lcd.fillScreen(BLACK); M5.Lcd.setCursor(4, 4); M5.Lcd.setTextColor(TFT_WHITE); M5.Lcd.printf("Settling...");
    unsigned long t0 = millis();
    while (millis() - t0 < 3000) {
        M5.update();
        if (M5.Imu.update()) {
            auto d = M5.Imu.getImuData();
            float ax = d.accel.x * 980.665f - offsetX; float ay = d.accel.y * 980.665f - offsetY; float az = d.accel.z * 980.665f - offsetZ;
            float comp = sqrtf(ax * ax + ay * ay + az * az); g_stalta.push(comp); g_intensity.processSample(ax, ay, az);
        }
        delay(10); yield();
    }
    g_intensity.resetPeak();
    xTaskCreatePinnedToCore(sampleTask, "Sampler", 4096, nullptr, 10, nullptr, 0);
    sleepTimer = millis(); lastDisplay = millis(); lastStaRetry = millis(); bootCompletedTime = millis();
    Serial.println("Boot completed. Sampler running on Core 0.");
}
void loop() {
    M5.update(); updateBeep(); updateWav();
    if (M5.BtnA.wasPressed()) {
        pauseMode = !pauseMode;
        if (pauseMode) { stopBeep(); stopWav(); pauseStartTime = millis(); Serial.println("PAUSE ON (BtnA)"); }
        else Serial.println("PAUSE OFF (BtnA)");
        wakeUp(); sleepTimer = millis(); lastDisplay = 0;
    }
    if (pauseMode && (millis() - pauseStartTime > PAUSE_AUTO_RESUME_MS)) { pauseMode = false; lastDisplay = 0; Serial.println("PAUSE auto-resumed"); }
    server.handleClient();
    if (!staConnected && millis() - lastStaRetry > STA_RETRY_INTERVAL_MS) {
        lastStaRetry = millis(); String ssid, pass;
        if (loadWiFiCreds(ssid, pass)) {
            Serial.println("STA auto retry..."); WiFi.disconnect(); delay(200); WiFi.begin(ssid.c_str(), pass.c_str());
            unsigned long t0 = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t0 < 5000) { delay(100); yield(); }
            if (WiFi.status() == WL_CONNECTED) { staConnected = true; staIP = WiFi.localIP().toString(); syncNTP(); }
        }
    }
    if (staConnected && WiFi.status() != WL_CONNECTED) { staConnected = false; staIP = ""; Serial.println("STA disconnected"); }
    if (envOK && millis() - lastEnvUpdate >= 1000) {
        lastEnvUpdate = millis(); env.update(); curTemp = env.getTemperatureSHT(); curHum = env.getHumidity(); curPress = env.getPressure();
    }
    int processed = 0; constexpr int MAX_BATCH = 32;
    while (sampleTail != sampleHead && processed < MAX_BATCH) {
        Sample s;
        portENTER_CRITICAL(&sampleMux);
        s = sampleBuf[sampleTail]; sampleTail = (sampleTail + 1) % SAMPLE_BUF_SIZE;
        if (sampleCount > 0) sampleCount--;
        portEXIT_CRITICAL(&sampleMux);
        float ax = s.x - offsetX; float ay = s.y - offsetY; float az = s.z - offsetZ; float comp = sqrtf(ax * ax + ay * ay + az * az);
        g_stalta.push(comp); bool staActive = g_stalta.update(); float intensity = g_intensity.processSample(ax, ay, az);
        g_fft.pushSample(g_intensity.lastFilteredComp); g_fusion.update(staActive, g_fft.fftVote, g_intensity.peakAccel, intensity);
        float wakeIntensity = g_intensity.currentIntensity + intOffset; bool shouldWake = false;
        if (wakeMode == 0) shouldWake = true; else if (wakeMode == 1) shouldWake = g_fusion.quakeConfirmed; else shouldWake = (g_fusion.quakeConfirmed || wakeIntensity > 1.5f);
        if (shouldWake) { sleepTimer = millis(); wakeUp(); }
        float instantI = g_intensity.getInstantIntensity() + intOffset; if (instantI < 0) instantI = 0;
        triggerBeepIfNeeded(instantI); processed++;
    }
    if (wakeMode != 0 && !sleeping && !pauseMode && (millis() - sleepTimer > cfgSleepDelayMs)) enterSleep();
    if (!sleeping && (millis() - lastDisplay >= cfgDisplayIntervalMs)) {
        lastDisplay = millis(); bool changed = false;
        if (fabs(curTemp - cachedTemp) > 0.1f) changed = true;
        if (fabs(curHum - cachedHum) > 0.5f) changed = true;
        if (fabs(curPress - cachedPress) > 0.1f) changed = true;
        if (fabs(g_intensity.currentIntensity - cachedIntensity) > 0.01f) changed = true;
        if (g_fusion.votes != cachedVotes) changed = true;
        if (fabs(g_intensity.peakAccel - cachedPga) > 0.5f) changed = true;
        if (env.isThermalEquilibrium() != cachedWarmup) changed = true;
        if (staConnected != cachedSta) changed = true;
        if (pauseMode != cachedPause) changed = true;
        if (fabs(g_fft.bandEnergy - cachedFftEnergy) > 0.1f) changed = true;
        char dateBuf[32] = ""; char timeBuf[16] = "";
        if (timeSynced && getLocalTime(&timeinfo)) {
            strftime(dateBuf, sizeof(dateBuf), "%Y/%m/%d (%a)", &timeinfo); strftime(timeBuf, sizeof(timeBuf), "%H:%M", &timeinfo);
        }
        if (strcmp(dateBuf, cachedDateStr) != 0) changed = true;
        if (strcmp(timeBuf, cachedTimeStr) != 0) changed = true;
        if (changed) {
            cachedTemp = curTemp; cachedHum = curHum; cachedPress = curPress;
            cachedIntensity = g_intensity.currentIntensity; cachedVotes = g_fusion.votes; cachedPga = g_intensity.peakAccel;
            cachedWarmup = env.isThermalEquilibrium(); cachedSta = staConnected; cachedPause = pauseMode; cachedFftEnergy = g_fft.bandEnergy;
            strncpy(cachedDateStr, dateBuf, sizeof(cachedDateStr)); strncpy(cachedTimeStr, timeBuf, sizeof(cachedTimeStr));
            drawScreen();
        }
    }
    static unsigned long lastHeapLog = 0;
    if (millis() - lastHeapLog > 60000) {
        lastHeapLog = millis();
        Serial.printf("[Heap] free=%u max=%u bufQ=%d fftE=%.2f fftV=%d\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), sampleCount, g_fft.bandEnergy, g_fft.fftVote ? 1 : 0);
    }
}
