# M5StickS3 Seismometer

M5StickS3 + ENV III Hat による高精度震度計。
気象庁の計測震度算出法に準拠したリアルタイム震度演算を実装しています。

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Arduino](https://img.shields.io/badge/Arduino-2.x-blue.svg)](https://www.arduino.cc/)
[![M5Stack](https://img.shields.io/badge/M5Stack-StickS3-red.svg)](https://m5stack.com/)

---

## ✨ 特徴

- **功刀（Kunugi）2012フィルタ**を実装
  - 気象庁の計測震度算出に用いられる近似フィルタ
  - 参考文献：[功刀卓・他 (2013)](https://doi.org/10.4294/zisin.65.223)
- **0.3秒ルール**（JMA準拠のノイズ除去）
- **FFT周波数解析**による生活ノイズと地震波の識別
- **4票多数決**（STA/LTA + FFT + PGA + 計測震度）
- **ENV III Hatの自動熱補正**（CPU温度による動的補正）
- **Wi-Fi (AP+STA) + Web UI**（スマホから設定可能）
- **BEEP + WAV音声警告**（震度別）
- **FreeRTOSデュアルコア**による決定論的サンプリング

---

## 🧰 ハードウェア要件

| 部品 | 型番 |
|---|---|
| MCU | M5StickS3 (ESP32-S3) |
| 環境センサー | ENV III Hat (SHT30 + QMP6988) |
| スピーカー | M5StickS3内蔵 or 外部接続 |

---

## 📦 ソフトウェア要件

### Arduino IDE

- **Arduino IDE 2.3.6 以上**

### ボードパッケージ

- **M5Stack** ボードパッケージ（バージョン3.3.9以上）

### ライブラリ

| ライブラリ | バージョン | 用途 |
|---|---|---|
| **M5Unified** | 0.2.21以上 | M5StickS3制御 |
| **M5GFX** | 0.2.28以上 | ディスプレイ描画 |
| **M5Unit-ENV** | 1.7.1以上 | ENV III制御 |
| **arduinoFFT** (Enrique Condes) | 2.x | FFT解析 |

### 同梱ライブラリ

本リポジトリには、ENV III Hat用のカスタムライブラリ `M5StickS3_ENV_Hat_III` が `libraries/` フォルダに同梱されています。

---

## 🚀 セットアップ

### 1. リポジトリのクローン

```bash
git clone https://github.com/jpeqtx/M5StickS3-Seismometer.git
```

### 2. 同梱ライブラリのインストール

1. `libraries/M5StickS3_ENV_Hat_III/` フォルダをコピー
2. 以下のArduinoライブラリフォルダに貼り付け：
   - **Windows**: `Documents/Arduino/libraries/`
   - **macOS**: `~/Documents/Arduino/libraries/`
   - **Linux**: `~/Arduino/libraries/`
3. Arduino IDEを再起動

### 3. スケッチの書き込み

1. `M5StickS3_20260913_Seismometer.ino` をArduino IDEで開く
2. ボードに **M5StickS3** を選択
3. コンパイル＆書き込み

### 4. WAVファイルのアップロード（LittleFS）

1. **arduino-littlefs-upload** プラグインをインストール
   - [リリースページ](https://github.com/earlephilhower/arduino-littlefs-upload/releases)
2. `Ctrl + Shift + P` → **Upload LittleFS to Pico/ESP8266/ESP32** を実行
   （スケッチフォルダ内の `data/` の内容がLittleFSに書き込まれます）

---

## 📱 使い方

### 初回起動

1. M5StickS3の電源ON
2. 画面に `AP: M5Seismo` `URL: 192.168.4.1` が表示される
3. スマホのWiFi設定で **`M5Seismo`** に接続（パスワード：`seismo1234`）
4. ブラウザで **`http://192.168.4.1`** を開く
5. **Home WiFi Setting** で自宅WiFiを設定
6. **Manual Time Setting** で現在時刻を設定（NTP未接続時）
7. **Environmental Calibration** で実際の気温を入力 → Save & Reboot

### 日常使い

- 静置時：震度0が表示される
- 地震検知時：画面点灯 + BEEP + WAV再生
- ポーズ：青ボタン（BtnA）短押しで警告音を一時停止（5分後自動解除）

### 画面表示

```
┌─────────────────────┐
│  2026/09/13 (Sat)   │
│       14:30         │
│═════════════════════│
│ 25.3C  74%  1013    │
│ TEMP   HUM  hPa     │
│                     │
│  ┌─────────────┐    │
│  │  INTENSITY  │    │
│  │      0      │    │
│  │  Meas 0.00  │    │
│  └─────────────┘    │
│  ┌─ MONITOR 0/4 ─┐  │
│  └───────────────┘  │
│    PGA 0.5 gal      │
│      WiFi OK        │
└─────────────────────┘
```

---

## 🔧 技術仕様

### 計測震度算出

- **フィルタ**：功刀（Kunugi）2012フィルタ（2次×6段）
- **0.3秒ルール**：100サンプルリングバッファ + ソート
- **サンプリング**：100Hz（FreeRTOS Core 0固定）

### 検出器（4票多数決）

| # | 検出器 | 判定 |
|---|---|---|
| 1 | STA/LTA | 短時間平均 / 長時間平均 > 3.0 |
| 2 | FFT帯域 | 0.5〜10Hzエネルギー > 閾値 |
| 3 | PGA | 最大加速度 > 5.0 gal |
| 4 | 計測震度 | > 1.0 |

2票以上で「地震」と確定。

### 音声警告

| 震度 | BEEP | WAV |
|---|---|---|
| 2 | 短音×2 | i2.wav |
| 3 | 短音×3 | i3.wav |
| 4 | 短音×4 | i4.wav |
| 5以上 | 2秒連続 | i5.wav ×2 |

---

## 📚 参考文献

- 功刀卓・青井真・中村洋光・鈴木亘・森川信之・藤原広行 (2013)「震度のリアルタイム演算に用いられる近似フィルタの改良」地震 第2輯, 65巻, 223-230頁. https://doi.org/10.4294/zisin.65.223
- 功刀卓・青井真・中村洋光・藤原広行・森川信之 (2008)「震度のリアルタイム演算法」地震 第2輯, 60巻, 243-252頁.
- 気象庁「計測震度の算出方法」 https://www.jma.go.jp/jma/kishou/know/jishin/kyoshin/kaisetsu/calc_sindo.html

---

## ⚠️ 免責事項

- **本デバイスは人命に関わる用途には使用しないでください**
- 気象庁の公式震度計ではありません
- ホビー・研究目的での使用を想定しています
- 本ソフトウェアの使用によって生じた損害について、作者は一切の責任を負いません

---

## 📄 ライセンス

MIT License — 詳細は [LICENSE](LICENSE) を参照。

---

## 👤 作者

**YNGMT**

- GitHub: [@jpeqtx](https://github.com/jpeqtx)

---

## 🤝 コントリビューション

バグ報告・機能提案は [Issues](https://github.com/jpeqtx/M5StickS3-Seismometer/issues) へ。
プルリクエストも歓迎します。
