# Isolation Sphere

球体LEDディスプレイ制御システム

## 概要

Isolation Sphereは、800個のLEDを搭載した直径110mmの球体ディスプレイを制御するシステムです。IMUセンサーによる姿勢検出により、ガジェット本体が回転しても映像が回転しない安定した表示を実現します。

## 主な特徴

- **球体ディスプレイ**: 直径110mm、800個のWS2812C-2020 LED
- **姿勢補正**: BNO055 IMUによるリアルタイム姿勢検出と画像変換
- **高速更新**: 30Hz目標の描画更新レート
- **動画再生**: 320x160解像度、10fpsの動画を球体表面に投影
- **リモート制御**: Web UIおよびゲームパッドによる操作

## システム構成

### ハードウェア構成

#### 1. コントローラー (Raspberry Pi)
- **OS**: Ubuntu 22.04
- **機能**:
  - FastAPI Webアプリケーション（UI制御）
  - 動画管理・再生
  - ROS2通信（Humble）
  - ゲームパッド接続

#### 2. ガジェット制御 (ESP32S3R)
- **機能**:
  - IMU (BNO055) データ取得
  - 画像変換処理（姿勢補正）
  - LED RGB値計算
  - SPI通信によるデータ送信

#### 3. LED制御 (RP2350)
- **機能**:
  - SPIデータ受信（ダブルバッファ）
  - 複数LEDストリップへの並列描画
  - PIOによる高速LED制御
  - フレーム同期

## 通信アーキテクチャ

```
Raspberry Pi
    ↓ (ROS2/WiFi)
ESP32S3R
    ↓ (SPI/DMA)
RP2350
    ↓ (PIO)
LED Strips (800 LEDs)
```

## プロジェクト構造

```
isolation-sphere/
├── esp32/          # ESP32ファームウェア
├── raspi/          # Raspberry Pi制御プログラム
├── rp2350/         # RP2350ファームウェア（予定）
├── CLAUDE.md       # 開発ガイドライン
└── README.md       # このファイル
```

## 開発環境

### 必要なツール
- ESP-IDF (ESP32開発)
- ROS2 Humble (通信)
- Python 3.x + FastAPI (Web UI)
- PlatformIO/Arduino IDE (RP2350開発)

### セットアップ

```bash
# リポジトリのクローン
git clone https://github.com/sastles-com/sastle.git
cd sastle/isolation-sphere

# ESP32開発環境
cd esp32
idf.py build

# Raspberry Pi環境
cd raspi
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 開発状況

- [x] プロジェクト初期化
- [x] 基本仕様定義
- [ ] ESP32ファームウェア実装
- [ ] Raspberry Pi制御プログラム実装
- [ ] RP2350ファームウェア実装
- [ ] システム統合テスト

## ライセンス

TBD

## 連絡先

GitHub: [@sastles-com](https://github.com/sastles-com)