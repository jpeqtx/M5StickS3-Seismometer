#pragma once

// ===== サンプリング =====
constexpr int   SAMPLE_RATE_HZ      = 100;

// ===== STA/LTA =====
constexpr float STA_WINDOW_SEC      = 0.5f;
constexpr float LTA_WINDOW_SEC      = 10.0f;
constexpr float STA_LTA_TRIGGER     = 3.0f;
constexpr float STA_LTA_DEACTIVATE  = 1.5f;

// ===== 功刀（Kunugi）2012フィルタのパラメータ =====
constexpr float K2012_F0   = 0.45f;
constexpr float K2012_F1   = 7.0f;
constexpr float K2012_F2   = 0.5f;
constexpr float K2012_F3   = 12.0f;
constexpr float K2012_F4   = 20.0f;
constexpr float K2012_F5   = 30.0f;
constexpr float K2012_H2A  = 1.0f;
constexpr float K2012_H2B  = 0.75f;
constexpr float K2012_H3   = 0.9f;
constexpr float K2012_H4   = 0.6f;
constexpr float K2012_H5   = 0.6f;
constexpr float K2012_GAIN = 1.262f;

// ===== 0.3秒ルール =====
constexpr int   RING_BUFFER_SIZE    = 100;
constexpr int   RING_SORT_INDEX     = 30;

// ===== FFT =====
constexpr int   FFT_SAMPLES         = 256;
constexpr float FFT_NOISE_THRESHOLD = 0.5f;
constexpr float QUAKE_FREQ_LOW      = 0.5f;
constexpr float QUAKE_FREQ_HIGH     = 10.0f;

// ===== 統合判定 =====
constexpr int   REQUIRED_VOTES      = 2;
constexpr float PGA_TRIGGER_GAL     = 5.0f;
constexpr float SEIS_INT_TRIGGER    = 1.0f;

// ===== キャリブレーション =====
constexpr int   CALIBRATION_SAMPLES = 300;

// ===== スリープ・輝度 =====
constexpr uint8_t BRIGHTNESS_NORMAL = 20;
constexpr uint8_t BRIGHTNESS_SLEEP  = 0;
constexpr unsigned long SLEEP_DELAY_MS = 10000;

// ===== 表示更新 =====
constexpr unsigned long DISPLAY_INTERVAL_MS = 2000;

// ===== Web設定 =====
constexpr int WEB_PORT = 80;

// ===== AP設定 =====
constexpr const char* AP_SSID = "M5Seismo";
constexpr const char* AP_PASS = "seismo1234";

// ===== STA再接続 =====
constexpr unsigned long STA_CONNECT_TIMEOUT_MS = 10000;
constexpr unsigned long STA_RETRY_INTERVAL_MS  = 30000;

// ===== NTP =====
constexpr const char* NTP_SERVER_1 = "ntp.jst.mfeed.ad.jp";
constexpr const char* NTP_SERVER_2 = "ntp.nict.jp";
constexpr long        NTP_TZ_OFFSET = 9 * 3600;

// ===== サンプルバッファ =====
constexpr int   SAMPLE_BUF_SIZE = 512;
