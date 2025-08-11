#!/bin/bash

# ESP-IDF v6.0 セットアップスクリプト
# Isolation Sphere プロジェクト用 (ESP32S3R対応)

set -e

echo "=========================================="
echo "ESP-IDF v6.0 セットアップ開始"
echo "Isolation Sphere Project用"
echo "=========================================="

# 必要なパッケージのインストール
echo "1. 必要なパッケージをインストール中..."
sudo apt-get update
sudo apt-get install -y git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 \
    python3-setuptools

# ESP-IDFディレクトリの確認・更新
ESP_DIR="$HOME/esp"
echo "2. ESP-IDF v6.0に更新中..."
mkdir -p "$ESP_DIR"
cd "$ESP_DIR"

# 既存のESP-IDFをv6.0に更新
if [ -d "esp-idf" ]; then
    echo "既存のESP-IDFをv6.0に更新中..."
    cd esp-idf
    
    # 現在のバージョン確認
    current_version=$(git describe --tags --exact-match 2>/dev/null || echo "unknown")
    echo "現在のバージョン: $current_version"
    
    # v6.0ブランチに切り替え
    echo "v6.0にチェックアウト中..."
    git fetch origin
    git checkout v6.0
    git pull origin v6.0
    git submodule update --init --recursive
    
    cd ..
else
    echo "ESP-IDF v6.0をクローン中..."
    git clone -b v6.0 --recursive https://github.com/espressif/esp-idf.git
fi

cd esp-idf

# インストールスクリプト実行（ESP32S3用）
echo "3. ESP32S3用ツールチェーンをインストール中..."
./install.sh esp32s3

echo "4. 環境変数設定..."
source ./export.sh

# インストール確認
echo "5. インストール確認..."
idf_version=$(idf.py --version 2>/dev/null || echo "ERROR: idf.py not found")
echo "ESP-IDF Version: $idf_version"

# Python依存関係の確認
echo "6. Python依存関係確認..."
python -m pip list | grep -E "(esptool|esp-idf-monitor|kconfiglib)" || true

# isolation-sphereプロジェクト用の設定
echo "7. Isolation Sphere プロジェクト用環境設定..."

# .bashrcに環境変数設定を追加（重複チェック）
BASHRC_FILE="$HOME/.bashrc"
EXPORT_LINE="alias get_idf='. $HOME/esp/esp-idf/export.sh'"

if ! grep -q "get_idf" "$BASHRC_FILE" 2>/dev/null; then
    echo "" >> "$BASHRC_FILE"
    echo "# ESP-IDF v6.0 Environment" >> "$BASHRC_FILE"
    echo "$EXPORT_LINE" >> "$BASHRC_FILE"
    echo "echo \"ESP-IDF v6.0 environment available. Run 'get_idf' to activate.\"" >> "$BASHRC_FILE"
    echo ".bashrcに環境設定を追加しました"
else
    echo ".bashrcの環境設定は既に存在します"
fi

# プロジェクト固有の設定ファイル作成
PROJECT_DIR="/home/yakatano/work/isolation-sphere"
PROJECT_ENV_FILE="$PROJECT_DIR/esp-idf-setup/activate_env.sh"

cat > "$PROJECT_ENV_FILE" << 'EOF'
#!/bin/bash

# Isolation Sphere Project - ESP-IDF v6.0 Environment Activation Script

ESP_IDF_PATH="$HOME/esp/esp-idf"

if [ ! -d "$ESP_IDF_PATH" ]; then
    echo "ERROR: ESP-IDF not found at $ESP_IDF_PATH"
    echo "Please run setup_esp_idf_v6.sh first"
    exit 1
fi

echo "Activating ESP-IDF v6.0 environment..."
source "$ESP_IDF_PATH/export.sh"

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
echo "  flash_test  - Flash and monitor test_hello_world"

# 便利なエイリアス
alias cd_project="cd $ESP32_PROJECT_PATH"
alias build_test="cd $ESP32_PROJECT_PATH && idf.py build"
alias flash_test="cd $ESP32_PROJECT_PATH && idf.py -p /dev/ttyACM0 flash monitor"

# プロジェクトディレクトリに移動
cd "$ESP32_PROJECT_PATH"
EOF

chmod +x "$PROJECT_ENV_FILE"

echo "=========================================="
echo "ESP-IDF v6.0 セットアップ完了！"
echo ""
echo "使用方法："
echo "1. 環境をアクティベート："
echo "   source ~/work/isolation-sphere/esp-idf-setup/activate_env.sh"
echo ""
echo "2. または毎回以下を実行："
echo "   source ~/esp/esp-idf/export.sh"
echo ""
echo "3. プロジェクトでのビルド例："
echo "   cd $PROJECT_DIR/esp32/test_hello_world"
echo "   idf.py set-target esp32s3"
echo "   idf.py build"
echo "   idf.py -p /dev/ttyACM0 flash monitor"
echo ""
echo "4. 次回からのショートカット："
echo "   get_idf  # 新しいターミナルで実行"
echo "=========================================="

# 即座に環境をアクティベート（この端末用）
echo ""
echo "現在のターミナルでESP-IDF環境をアクティベート中..."
source ./export.sh
echo "ESP-IDF環境がアクティベートされました！"