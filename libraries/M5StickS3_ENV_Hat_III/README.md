<!-- ファイル名: README.md -->
# M5StickS3 ENV Hat III

M5StickS3 + ENV III HAT（SHT30 + QMP6988）用のArduinoライブラリ。
CPU由来の熱影響を**自動補正**し、正確な気温・湿度・気圧を取得します。

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## ✨ 特徴

- **自動熱補正**：CPU温度センサーを利用し、ENV IIIが受ける排熱の影響を動的に補正
- **熱平衡検出**：起動後の温度上昇が落ち着くまで固定オフセットで動作し、安定後に動的補正へ移行
- **NVS永続化**：オフセット・自動補正設定・基準気温をデバイス内に保存
- **実気温からの自動キャリブレーション**：正確な温度計で測った実気温を入力するだけでオフセットを自動導出
- **シンプルAPI**：`begin()` / `update()` / `getTemperatureSHT()` の3つで完結

---

## 📦 インストール

### 方法1：手動インストール

1. 本リポジトリをクローンまたはZIPでダウンロード
2. `M5StickS3_ENV_Hat_III` フォルダを以下に配置：
   - **Windows**: `Documents/Arduino/libraries/`
   - **macOS**: `~/Documents/Arduino/libraries/`
   - **Linux**: `~/Arduino/libraries/`
3. Arduino IDEを再起動

### 方法2：ZIPライブラリとして追加

1. 本リポジトリをZIPでダウンロード
2. Arduino IDE → `スケッチ` → `ライブラリをインクルード` → `.ZIP形式のライブラリを追加`

---

## 🔧 依存ライブラリ

以下のライブラリをArduino IDEのライブラリマネージャからインストールしてください：

- **M5Unified**（0.2.12以上）
- **M5Unit-ENV**（M5Stack公式）

---

## 🚀 基本的な使い方

```cpp
#include <M5Unified.h>
#include <M5StickS3_ENV_Hat_III.h>

M5StickS3_ENV_Hat_III env;

void setup() {
    M5.begin();
    if (!env.begin()) {
        Serial.println("Sensor init failed");
        while (1);
    }
}

void loop() {
    env.update();

    Serial.printf("Temp: %.1f C\n",  env.getTemperatureSHT());
    Serial.printf("Hum:  %.1f %%\n", env.getHumidity());
    Serial.printf("Press:%.1f hPa\n",env.getPressure());

    delay(1000);
}
```

---

## 📚 API リファレンス

### 初期化・更新

| メソッド | 説明 |
|---|---|
| `bool begin()` | センサー初期化、NVSから設定ロード |
| `bool update()` | センサー値更新、自動補正を計算 |

### 値の取得（補正済み）

| メソッド | 戻り値 |
|---|---|
| `getTemperatureSHT()` | 気温 [℃]（自動補正適用） |
| `getHumidity()` | 湿度 [%RH]（オフセット適用） |
| `getPressure()` | 気圧 [hPa]（オフセット適用） |

### 値の取得（生値）

| メソッド | 戻り値 |
|---|---|
| `getRawTemperatureSHT()` | 気温 [℃]（補正なし） |
| `getRawHumidity()` | 湿度 [%RH]（補正なし） |
| `getRawPressure()` | 気圧 [hPa]（補正なし） |

### オフセット設定

| メソッド | 説明 |
|---|---|
| `setBaseOffsets(t, h, p)` | 気温・湿度・気圧オフセットを一括設定（NVS保存） |
| `setBaseTempOffset(t)` | 気温オフセットのみ設定 |
| `setHumOffset(h)` | 湿度オフセットのみ設定 |
| `setPressOffset(p)` | 気圧オフセットのみ設定 |
| `getBaseTempOffset()` | 気温オフセット取得 |
| `getEffectiveTempOffset()` | 現在適用中の実効オフセット取得 |

### 自動補正

| メソッド | 説明 |
|---|---|
| `enableAutoComp(bool)` | 自動補正のON/OFF |
| `isAutoCompEnabled()` | 自動補正が有効か |
| `isThermalEquilibrium()` | 熱平衡に達したか |
| `setActualTemp(t)` | 実気温を入力してオフセットを自動導出 |
| `getActualTempRef()` | 基準気温（キャリブレーション時の実気温） |
| `getCpuEnvDiff()` | CPU-ENV温度差のEMA値 |

### NVS操作

| メソッド | 説明 |
|---|---|
| `loadSettings()` | NVSから設定を読み込み |
| `saveSettings()` | NVSに設定を保存 |
| `resetSettings()` | NVSをクリアしてデフォルトに戻す |

---

## 🌡️ 自動補正の仕組み

ENV IIIはM5StickS3のHATとして直上に載るため、CPU・ディスプレイ・バッテリーの排熱を直接受けます。本ライブラリは、ESP32-S3の内部温度センサー（`temperatureRead()`）を発熱指標として利用し、以下の流れで補正します。

### 1. 熱平衡検出

```
CPU-ENV温度差のEMAを毎秒更新
  ↓
差分の変化量が閾値以下で1分継続
  ↓
熱平衡と判定（thermalEquilibrium = true）
```

### 2. 段階的補正

| フェーズ | 動作 |
|---|---|
| 起動〜熱平衡前 | 固定オフセットのみ適用（過補正防止） |
| 熱平衡後 | CPU-ENV差分の変化に応じてオフセットを動的調整 |

### 3. 実気温によるキャリブレーション

正確な温度計で測った実気温を `setActualTemp()` で入力すると：

- `baseTempOffset = 実気温 - 生値` を自動計算
- その時点のCPU-ENV差分を基準値として記録
- 以降、差分が基準からずれた分だけオフセットを動的補正

---

## 📝 使用例：自動補正の有効化

```cpp
#include <M5Unified.h>
#include <M5StickS3_ENV_Hat_III.h>

M5StickS3_ENV_Hat_III env;

void setup() {
    M5.begin();
    env.begin();

    // 自動補正を有効化（デフォルトで有効）
    env.enableAutoComp(true);

    // 実気温を入力（正確な温度計で測った値）
    // → baseTempOffsetとrefCpuEnvDiffが自動計算され、NVSに保存される
    env.setActualTemp(25.0f);
}

void loop() {
    env.update();
    Serial.printf("Temp: %.2f C (Equilibrium: %s)\n",
                  env.getTemperatureSHT(),
                  env.isThermalEquilibrium() ? "YES" : "NO");
    delay(1000);
}
```

---

## ⚠️ 注意事項

- **起動直後は不安定**：熱平衡に達するまで10〜15分かかります。この間は画面に「Warming up...」等を表示することを推奨
- **周囲温度依存**：真夏・真冬で最適なオフセットが変わります。季節ごとの再キャリブレーションを推奨
- **CPU周波数**：`setCpuFrequencyMhz(80)` で発熱を抑えると精度が向上します
- **画面輝度**：`M5.Lcd.setBrightness(20)` など低輝度設定で発熱を抑制

---

## 📄 ライセンス

MIT License — 詳細は [LICENSE](LICENSE) を参照。

---

## 👤 作者

**YNGMT**

- GitHub: [@jpeqtx](https://github.com/jpeqtx)
- Repository: [M5StickS3_ENV_Hat_III](https://github.com/jpeqtx/M5StickS3_ENV_Hat_III)

---

## 🤝 コントリビューション

バグ報告・機能提案は [Issues](https://github.com/jpeqtx/M5StickS3_ENV_Hat_III/issues) へ。
プルリクエストも歓迎します。