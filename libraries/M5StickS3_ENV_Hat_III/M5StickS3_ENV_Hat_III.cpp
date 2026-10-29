#include "M5StickS3_ENV_Hat_III.h"
#include <math.h>

#define NVS_NAMESPACE         "seismo"
#define AUTOCOMP_GAIN         0.5f
#define AUTOCOMP_DIFF_STABLE  0.5f

// ============================================================
bool M5StickS3_ENV_Hat_III::begin() {
  M5.Power.setExtOutput(true);
  delay(300);
  Wire.begin(8, 0, 400000U);   // SDA=8, SCL=0 for StickS3 HAT

  _qmp_ok = _qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, 8, 0, 400000U);
  _sht_ok = _sht.begin(&Wire, SHT3X_I2C_ADDR, 8, 0, 400000U);

  loadSettings();
  return (_qmp_ok && _sht_ok);
}

// ============================================================
bool M5StickS3_ENV_Hat_III::update() {
  bool ok = _sht.update();
  if (_qmp_ok) {
    bool qok = _qmp.update();
    ok = ok && qok;
  }
  if (ok) {
    _updateAutoComp(_sht.cTemp);
  }
  return ok;
}

// ============================================================
float M5StickS3_ENV_Hat_III::getTemperatureSHT() const {
  return _sht.cTemp + _computeEffectiveTempOffset();
}

// ============================================================
void M5StickS3_ENV_Hat_III::setBaseOffsets(float t, float h, float p) {
  _baseTempOffset = t;
  _humOffset      = h;
  _pressOffset    = p;
  saveSettings();
}

void M5StickS3_ENV_Hat_III::setBaseTempOffset(float t) {
  _baseTempOffset = t;
  saveSettings();
}

void M5StickS3_ENV_Hat_III::setHumOffset(float h) {
  _humOffset = h;
  saveSettings();
}

void M5StickS3_ENV_Hat_III::setPressOffset(float p) {
  _pressOffset = p;
  saveSettings();
}

// ============================================================
void M5StickS3_ENV_Hat_III::enableAutoComp(bool en) {
  _autoCompEnabled = en;
  saveSettings();
}

// ============================================================
// 実気温を入力すると、ベースオフセットとCPU-ENV基準差を再計算して保存
void M5StickS3_ENV_Hat_III::setActualTemp(float actualTemp) {
  _refTempAtCal = actualTemp;

  if (_sht_ok) {
    _baseTempOffset = actualTemp - _sht.cTemp;
    _refCpuEnvDiff  = temperatureRead() - _sht.cTemp;
  }
  saveSettings();
}

// ============================================================
void M5StickS3_ENV_Hat_III::_updateAutoComp(float rawTemp) {
  float cpuT = temperatureRead();
  float diff = cpuT - rawTemp;

  _diffEMA = 0.99f * _diffEMA + 0.01f * diff;

  float delta = fabs(_diffEMA - _lastDiff);
  if (delta < AUTOCOMP_DIFF_STABLE / 60.0f) {
    if (_diffStableSince == 0) _diffStableSince = millis();
    if (millis() - _diffStableSince > 60000UL) {
      _thermalEquilibrium = true;
    }
  } else {
    _diffStableSince = 0;
    _thermalEquilibrium = false;
  }
  _lastDiff = _diffEMA;
}

// ============================================================
float M5StickS3_ENV_Hat_III::_computeEffectiveTempOffset() const {
  // 自動補正OFF または 熱平衡前 → 固定オフセット
  if (!_autoCompEnabled || !_thermalEquilibrium) {
    return _baseTempOffset;
  }
  // 熱平衡後 → CPU-ENV差分の変化に応じて動的補正
  float diffShift = _diffEMA - _refCpuEnvDiff;
  return _baseTempOffset - diffShift * AUTOCOMP_GAIN;
}

// ============================================================
void M5StickS3_ENV_Hat_III::saveSettings() {
  _prefs.begin(NVS_NAMESPACE, false);
  _prefs.putFloat("t_off",    _baseTempOffset);
  _prefs.putFloat("h_off",    _humOffset);
  _prefs.putFloat("p_off",    _pressOffset);
  _prefs.putBool ("ac_en",    _autoCompEnabled);
  _prefs.putFloat("ref_temp", _refTempAtCal);
  _prefs.putFloat("ref_diff", _refCpuEnvDiff);
  _prefs.end();
}

void M5StickS3_ENV_Hat_III::loadSettings() {
  _prefs.begin(NVS_NAMESPACE, true);
  _baseTempOffset  = _prefs.getFloat("t_off",   -9.0f);
  _humOffset       = _prefs.getFloat("h_off",    16.0f);
  _pressOffset     = _prefs.getFloat("p_off",    3.0f);
  _autoCompEnabled = _prefs.getBool ("ac_en",    true);
  _refTempAtCal    = _prefs.getFloat("ref_temp", 25.0f);
  _refCpuEnvDiff   = _prefs.getFloat("ref_diff", 0.0f);
  _prefs.end();
}

void M5StickS3_ENV_Hat_III::resetSettings() {
  _prefs.begin(NVS_NAMESPACE, false);
  _prefs.clear();
  _prefs.end();
  _baseTempOffset  = -9.0f;
  _humOffset       = 16.0f;
  _pressOffset     = 3.0f;
  _autoCompEnabled = true;
  _refTempAtCal    = 25.0f;
  _refCpuEnvDiff   = 0.0f;
}