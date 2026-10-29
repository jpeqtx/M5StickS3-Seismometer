#ifndef M5StickS3_ENV_Hat_III_h
#define M5StickS3_ENV_Hat_III_h

#include <M5UnitENV.h>
#include <M5Unified.h>
#include <Preferences.h>

class M5StickS3_ENV_Hat_III {
public:
  // ===== 初期化・更新 =====
  bool begin();
  bool update();

  // ===== 生値（補正なし） =====
  float getRawTemperatureSHT() const { return _sht.cTemp; }
  float getRawHumidity() const       { return _sht.humidity; }
  float getRawPressure() const       { return _qmp_ok ? _qmp.pressure / 100.0f : 0.0f; }

  // ===== 補正済み値 =====
  float getTemperatureSHT() const;   // 自動補正を含む実効値
  float getHumidity() const          { return _sht.humidity + _humOffset; }
  float getPressure() const          { return _qmp_ok ? _qmp.pressure / 100.0f + _pressOffset : 0.0f; }

  // ===== オフセット設定（NVSに自動保存） =====
  void setBaseOffsets(float t, float h, float p);
  void setBaseTempOffset(float t);
  void setHumOffset(float h);
  void setPressOffset(float p);

  float getBaseTempOffset() const { return _baseTempOffset; }
  float getHumOffset()      const { return _humOffset; }
  float getPressOffset()    const { return _pressOffset; }
  float getEffectiveTempOffset() const { return _computeEffectiveTempOffset(); }

  // ===== 自動補正 =====
  void  enableAutoComp(bool en);
  bool  isAutoCompEnabled()    const { return _autoCompEnabled; }
  bool  isThermalEquilibrium() const { return _thermalEquilibrium; }
  void  setActualTemp(float actualTemp);   // 基準気温を設定（オフセット自動導出）
  float getActualTempRef() const { return _refTempAtCal; }
  float getCpuEnvDiff()    const { return _diffEMA; }

  // ===== センサー健全性 =====
  bool isQMPOK() const { return _qmp_ok; }
  bool isSHTOK() const { return _sht_ok; }

  // ===== NVS =====
  void loadSettings();
  void saveSettings();
  void resetSettings();

private:
  SHT3X   _sht;
  QMP6988 _qmp;
  bool    _qmp_ok = false;
  bool    _sht_ok = false;

  // 補正値
  float _baseTempOffset = -9.0f;
  float _humOffset      = 16.0f;
  float _pressOffset    = 3.0f;

  // 自動補正
  bool  _autoCompEnabled = true;
  float _refTempAtCal    = 25.0f;
  float _refCpuEnvDiff   = 0.0f;

  // 内部状態
  float _diffEMA = 0.0f;
  float _lastDiff = 0.0f;
  unsigned long _diffStableSince = 0;
  bool  _thermalEquilibrium = false;

  Preferences _prefs;

  void  _updateAutoComp(float rawTemp);
  float _computeEffectiveTempOffset() const;
};

#endif