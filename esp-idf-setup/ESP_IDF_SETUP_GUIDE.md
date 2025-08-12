# ESP-IDF v6.0 環境セットアップガイド

このドキュメントでは、isolation-sphereプロジェクト向けのESP-IDF v6.0開発環境のセットアップ手順を説明します。

## 概要

isolation-sphereプロジェクトは以下のハードウェア構成で動作します：
- **メインチップ**: ESP32S3-PICO-1 (LGA56)
- **Flash**: 8MB
- **PSRAM**: 8MB (OCTモード)
- **対象デバイス**: M5AtomS3R

## 前提条件

- Ubuntu 22.04 LTS (推奨)
- Python 3.10以上
- Git
- CMake 3.16以上

## 自動セットアップ（推奨）

### 1. 最新環境の自動セットアップ

```bash
cd /home/yakatano/work/isolation-sphere/esp-idf-setup
chmod +x update_esp_idf_v6.sh
./update_esp_idf_v6.sh
```

このスクリプトは以下を実行します：
- 既存のESP-IDF環境のバックアップ
- ESP-IDF v6.0のダウンロードとインストール
- 必要なツールチェーンのセットアップ
- 環境の検証

### 2. 環境のアクティベーション

```bash
cd /home/yakatano/work/isolation-sphere/esp-idf-setup
source activate_env.sh
```

便利なエイリアスが設定されます：
- `cd_project`: プロジェクトディレクトリに移動
- `build_test`: テストプロジェクトのビルド
- `flash_test`: フラッシュ書き込みとモニタリング

## 手動セットアップ

詳細な手動セットアップ手順については、[manual_setup.md](manual_setup.md) を参照してください。

## プロジェクト構成

```
isolation-sphere/
├── esp32/
│   ├── components/           # 共通コンポーネント
│   │   ├── bno055_driver/   # BNO055 IMUドライバー
│   │   └── config_manager/  # SPIFFS設定管理
│   ├── test_hello_world/    # 基本動作テストプロジェクト
│   └── partitions.csv       # パーティションテーブル
└── esp-idf-setup/          # セットアップスクリプト
    ├── update_esp_idf_v6.sh
    ├── activate_env.sh
    └── manual_setup.md
```

## テストプロジェクトの実行

### 1. ビルドテスト

```bash
cd /home/yakatano/work/isolation-sphere/esp32/test_hello_world
source /home/yakatano/esp/esp-idf/export.sh
idf.py build
```

### 2. フラッシュ書き込み

```bash
idf.py flash
```

### 3. シリアルモニター（オプション）

```bash
idf.py monitor
```

## 主要な設定項目

### sdkconfig.defaults の重要な設定

```ini
# ESP32S3 CPU設定
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240=y
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ=240

# PSRAM設定 - 8MB OCTモード
CONFIG_ESP32S3_SPIRAM_SUPPORT=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SIZE=-1
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_USE_MALLOC=y

# Flash設定 - 8MB
CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="8MB"

# パーティション設定
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

### パーティションテーブル (partitions.csv)

```csv
# ESP32S3 Test Hello World Partition Table - 8MB Flash
nvs,        data, nvs,     0x9000,    0x6000,
otadata,    data, ota,     0xf000,    0x2000,
app0,       app,  ota_0,   0x20000,   0x300000,
app1,       app,  ota_1,   0x320000,  0x300000,
spiffs,     data, spiffs,  0x620000,  0x1C0000,
user_data,  data, 0x40,    0x7E0000,  0x20000,
```

## コンポーネントの説明

### BNO055ドライバー
- I2C通信によるIMUセンサー制御
- Quaternionフィルタリング機能
- キャリブレーションデータ管理

### Config Manager
- SPIFFSベースの設定ファイル管理
- JSON形式での設定の保存/読み込み
- 動的設定変更サポート

## トラブルシューティング

### よくある問題

1. **PSRAM認識されない**
   - OCTモードの設定を確認
   - ハードウェア接続を確認
   - PSRAM初期化設定を確認

2. **ビルドエラー**
   - ESP-IDF v6.0の環境変数が正しく設定されているか確認
   - コンポーネントパスの設定を確認

3. **フラッシュ書き込み失敗**
   - デバイスの接続を確認 (`lsusb` でESP32デバイス確認)
   - 適切なUSBポートの選択
   - デバイスのリセット

### ログの確認

```bash
# ビルドログ
cat build/log/idf_py_stderr_output_*

# デバッグ出力
idf.py monitor --print-filter="*:D"
```

## バージョン情報

- **ESP-IDF**: v6.0-dev
- **ターゲット**: ESP32S3
- **Python**: 3.10+
- **CMake**: 3.16+

## 開発時の注意事項

1. **メモリ管理**
   - PSRAMの効率的な活用
   - メモリリークの監視

2. **パフォーマンス**
   - リアルタイム処理の最適化
   - タスク配置の最適化

3. **セキュリティ**
   - フラッシュ暗号化の検討
   - セキュアブートの検討

## サポート

問題が発生した場合は、以下を確認してください：

1. [ESP-IDF公式ドキュメント](https://docs.espressif.com/projects/esp-idf/)
2. プロジェクトのCLAUDE.mdファイル
3. 各コンポーネントのREADME

---

このガイドは isolation-sphere プロジェクト専用に作成されました。
最終更新: 2024年