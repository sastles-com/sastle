# ESP-IDF 手動セットアップガイド

## 必要な操作（ターミナルで実行）

### 1. 必要パッケージのインストール
```bash
sudo apt-get update
sudo apt-get install -y git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### 2. ESP-IDF のインストール
```bash
# ESP-IDFディレクトリ作成
mkdir -p ~/esp
cd ~/esp

# ESP-IDF v5.1.2 をクローン
git clone -b v5.1.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf

# ESP32S3用ツールチェーンをインストール
./install.sh esp32s3

# 環境変数を設定
source ./export.sh
```

### 3. インストール確認
```bash
idf.py --version
# 期待出力: ESP-IDF v5.1.2 以上
```

### 4. 環境変数の永続化（推奨）
```bash
# ~/.bashrc に追加
echo 'alias get_idf=". ~/esp/esp-idf/export.sh"' >> ~/.bashrc

# または毎回手動実行
source ~/esp/esp-idf/export.sh
```

## 次のステップ

### Hello World テスト
```bash
# ESP-IDF環境有効化
source ~/esp/esp-idf/export.sh

# テストプロジェクトに移動
cd /home/yakatano/work/isolation-sphere/esp32/test_hello_world

# ESP32S3ターゲット設定
idf.py set-target esp32s3

# ビルド
idf.py build

# デバイス確認（XIA ESP32S3の場合）
ls /dev/ttyACM* /dev/ttyUSB*

# フラッシュ書き込み（ポート名を確認して実行）
idf.py -p /dev/ttyACM0 flash

# シリアルモニター
idf.py -p /dev/ttyACM0 monitor
```

## トラブルシューティング

### デバイスが見つからない場合
```bash
# USB デバイス確認
lsusb | grep -i espressif

# 権限設定
sudo usermod -a -G dialout $USER
# ログアウト・ログイン後に有効

# 手動権限付与（一時的）
sudo chmod 666 /dev/ttyACM0
```

### ビルドエラーの場合
```bash
# クリーンビルド
idf.py fullclean
idf.py build

# 依存関係の更新
cd ~/esp/esp-idf
git submodule update --init --recursive
```

### PSRAMが認識されない場合
```bash
idf.py menuconfig
# Component config → ESP PSRAM → Support for external, SPI-connected RAM にチェック
```

## 環境が正常な場合の期待出力

### idf.py --version
```
ESP-IDF v5.1.2
```

### Hello World ビルド成功
```
Project build complete. To flash, run:
idf.py flash
```

### フラッシュ成功
```
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
```

### モニター出力例
```
I (xxx) HELLO_TEST: ESP32S3R Isolation Sphere Test
I (xxx) HELLO_TEST: Chip: esp32s3
I (xxx) HELLO_TEST: Cores: 2
I (xxx) HELLO_TEST: PSRAM: 8.00 MB
```