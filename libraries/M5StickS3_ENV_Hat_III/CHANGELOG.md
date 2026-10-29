<!-- ファイル名: CHANGELOG.md -->
# Changelog

本プロジェクトの主な変更点を記録します。
フォーマットは [Keep a Changelog](https://keepachangelog.com/ja/1.0.0/) に準拠。

## [1.2.0] - 2026-09-13

### Added
- 自動熱補正機能をライブラリ内に実装
  - CPU温度センサー（`temperatureRead()`）を利用した動的オフセット計算
  - 熱平衡検出（CPU-ENV温度差の安定判定）
  - `setActualTemp()` による実気温からのオフセット自動導出
- NVS永続化（Preferences）による設定保存
  - オフセット値（気温・湿度・気圧）
  - 自動補正のON/OFF
  - 基準気温・基準CPU-ENV差分
- `getEffectiveTempOffset()` で現在適用中のオフセットを取得可能
- `resetSettings()` でNVSをクリア

### Changed
- ライブラリ名を `StickS3_ENVIII` → `M5StickS3_ENV_Hat_III` に変更
- `begin()` の戻り値を「両センサー成功時のみtrue」に厳格化
- `update()` が `bool` を返すように変更
- `_qmp_ok` / `_sht_ok` を初期化（未定義動作を防止）

### Removed
- メインスケッチ側の `settings.h`（ライブラリに統合）

## [1.0.0] - 2026-08-01

### Added
- 初版リリース
- SHT30（温湿度）+ QMP6988（気圧）の読み取り
- HATポート（SDA=8, SCL=0）対応
- 外部電源制御（`M5.Power.setExtOutput(true)`）