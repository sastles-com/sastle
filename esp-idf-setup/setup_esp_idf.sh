#!/bin/bash

# ESP-IDF v5.1.2 セットアップスクリプト
# XIAO ESP32S3 開発環境構築用

set -e

echo "=========================================="
echo "ESP-IDF v5.1.2 セットアップ開始"
echo "=========================================="

# 必要なパッケージのインストール
echo "1. 必要なパッケージをインストール中..."
sudo apt-get update
sudo apt-get install -y git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

# ESP-IDFディレクトリの作成
ESP_DIR="$HOME/esp"
echo "2. ESP-IDFを $ESP_DIR にインストール中..."
mkdir -p "$ESP_DIR"
cd "$ESP_DIR"

# ESP-IDFのクローン（既存チェック）
if [ ! -d "esp-idf" ]; then
    echo "ESP-IDFをクローン中（v5.1.2）..."
    git clone -b v5.1.2 --recursive https://github.com/espressif/esp-idf.git
else
    echo "ESP-IDFは既存です。更新中..."
    cd esp-idf
    git fetch origin
    git checkout v5.1.2
    git submodule update --init --recursive
    cd ..
fi

cd esp-idf

# インストールスクリプト実行
echo "3. ESP32S3用ツールチェーンをインストール中..."
./install.sh esp32s3

echo "4. 環境変数設定..."
source ./export.sh

# インストール確認
echo "5. インストール確認..."
idf.py --version

echo "=========================================="
echo "ESP-IDF セットアップ完了！"
echo ""
echo "使用方法："
echo "1. 毎回以下を実行してください："
echo "   source ~/esp/esp-idf/export.sh"
echo ""
echo "2. または、~/.bashrcに以下を追加："
echo "   alias get_idf='. ~/esp/esp-idf/export.sh'"
echo ""
echo "3. プロジェクトでのビルド例："
echo "   cd /home/yakatano/work/isolation-sphere/esp32/test_hello_world"
echo "   idf.py set-target esp32s3"
echo "   idf.py build"
echo "   idf.py -p /dev/ttyACM0 flash monitor"
echo "=========================================="