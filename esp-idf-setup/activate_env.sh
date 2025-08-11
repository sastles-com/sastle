#!/bin/bash

# Isolation Sphere Project - ESP-IDF Environment Activation Script

ESP_IDF_PATH="$HOME/esp/esp-idf"

if [ ! -d "$ESP_IDF_PATH" ]; then
    echo "ERROR: ESP-IDF not found at $ESP_IDF_PATH"
    echo "Please run update_esp_idf_v6.sh first"
    exit 1
fi

echo "Activating ESP-IDF environment..."
source "$ESP_IDF_PATH/export.sh"

# バージョン確認
ESP_IDF_VERSION=$(idf.py --version 2>/dev/null || echo "unknown")
echo "ESP-IDF Version: $ESP_IDF_VERSION"

# プロジェクト固有の環境変数
export ISOLATION_SPHERE_PROJECT_ROOT="/home/yakatano/work/isolation-sphere"
export ESP32_PROJECT_PATH="$ISOLATION_SPHERE_PROJECT_ROOT/esp32/test_hello_world"

echo "Environment activated!"
echo "Project root: $ISOLATION_SPHERE_PROJECT_ROOT"
echo "ESP32 project: $ESP32_PROJECT_PATH"
echo ""
echo "Quick commands:"
echo "  cd_project  - Change to ESP32 project directory"
echo "  build_test  - Build test_hello_world project"
echo "  flash_test  - Flash and monitor test_hello_world (adjust port as needed)"

# 便利なエイリアス
alias cd_project="cd $ESP32_PROJECT_PATH"
alias build_test="cd $ESP32_PROJECT_PATH && idf.py build"
alias flash_test="cd $ESP32_PROJECT_PATH && idf.py -p /dev/ttyACM0 flash monitor"
alias set_target="cd $ESP32_PROJECT_PATH && idf.py set-target esp32s3"

# プロジェクトディレクトリに移動
cd "$ESP32_PROJECT_PATH"
