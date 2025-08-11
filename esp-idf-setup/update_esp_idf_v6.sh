#!/bin/bash

# ESP-IDF v6.0 アップデートスクリプト (sudo不要版)
# 既存のESP-IDF環境をv6.0に更新

set -e

echo "=========================================="
echo "ESP-IDF v6.0 アップデート開始"
echo "Isolation Sphere Project用 (sudo不要版)"
echo "=========================================="

# ESP-IDFディレクトリの確認・更新
ESP_DIR="$HOME/esp"
echo "1. ESP-IDF v6.0に更新中..."

if [ ! -d "$ESP_DIR" ]; then
    echo "ERROR: ESP-IDF directory not found at $ESP_DIR"
    echo "Please run full setup first or create directory manually"
    exit 1
fi

cd "$ESP_DIR"

# 既存のESP-IDFをv6.0に更新
if [ -d "esp-idf" ]; then
    echo "既存のESP-IDFをv6.0に更新中..."
    cd esp-idf
    
    # 現在のバージョン確認
    current_version=$(git describe --tags --exact-match 2>/dev/null || git branch --show-current || echo "unknown")
    echo "現在のバージョン/ブランチ: $current_version"
    
    # v6.0が存在するか確認
    echo "利用可能なv6.0関連ブランチ/タグを確認中..."
    git fetch origin
    
    # v6.0のバリエーションを確認
    available_v6=$(git branch -r | grep -E "(v6\.0|release/v6)" || echo "not found")
    echo "利用可能なv6.0関連: $available_v6"
    
    # タグも確認
    available_tags=$(git tag | grep -E "^v6\." | sort -V | tail -5 || echo "no v6 tags found")
    echo "利用可能なv6.x tags: $available_tags"
    
    # 最新のv6.x系タグを探す
    latest_v6_tag=$(git tag | grep -E "^v6\." | sort -V | tail -1)
    
    if [ -n "$latest_v6_tag" ]; then
        echo "最新のv6.xタグ ($latest_v6_tag) にチェックアウト中..."
        git checkout "$latest_v6_tag"
        git submodule update --init --recursive
    else
        echo "WARNING: v6.0系が見つかりません。masterブランチを使用します。"
        git checkout master
        git pull origin master
        git submodule update --init --recursive
    fi
    
    cd ..
else
    echo "ESP-IDFディレクトリが見つかりません。新規クローン中..."
    # 最新の安定版を取得
    latest_tag=$(git ls-remote --tags --sort='version:refname' https://github.com/espressif/esp-idf.git | \
                 grep -E "refs/tags/v[0-9]+\.[0-9]+(\.[0-9]+)?$" | \
                 tail -1 | sed 's/.*refs\/tags\///')
    
    echo "最新の安定版 ($latest_tag) をクローン中..."
    git clone -b "$latest_tag" --recursive https://github.com/espressif/esp-idf.git
fi

cd esp-idf

# 現在のバージョンを表示
current_version=$(git describe --tags --exact-match 2>/dev/null || git branch --show-current || echo "unknown")
echo "更新後のバージョン: $current_version"

# インストールスクリプト実行（ESP32S3用）
echo "2. ESP32S3用ツールチェーンをインストール/更新中..."
if [ -f "./install.sh" ]; then
    ./install.sh esp32s3
else
    echo "WARNING: install.sh not found. Manual installation may be required."
fi

echo "3. 環境変数設定..."
if [ -f "./export.sh" ]; then
    source ./export.sh
else
    echo "WARNING: export.sh not found. Manual environment setup may be required."
fi

# インストール確認
echo "4. インストール確認..."
if command -v idf.py &> /dev/null; then
    idf_version=$(idf.py --version 2>/dev/null || echo "ERROR: idf.py failed")
    echo "ESP-IDF Version: $idf_version"
else
    echo "WARNING: idf.py not found in PATH"
fi

# isolation-sphereプロジェクト用の設定
echo "5. Isolation Sphere プロジェクト用環境設定..."

# プロジェクト固有の設定ファイル作成
PROJECT_DIR="/home/yakatano/work/isolation-sphere"
PROJECT_ENV_FILE="$PROJECT_DIR/esp-idf-setup/activate_env.sh"

cat > "$PROJECT_ENV_FILE" << 'EOF'
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
EOF

chmod +x "$PROJECT_ENV_FILE"

# .bashrcに環境変数設定を追加（重複チェック）
BASHRC_FILE="$HOME/.bashrc"
EXPORT_LINE="alias get_idf='. $HOME/esp/esp-idf/export.sh'"

if ! grep -q "get_idf" "$BASHRC_FILE" 2>/dev/null; then
    echo "" >> "$BASHRC_FILE"
    echo "# ESP-IDF Environment" >> "$BASHRC_FILE"
    echo "$EXPORT_LINE" >> "$BASHRC_FILE"
    echo "echo \"ESP-IDF environment available. Run 'get_idf' to activate.\"" >> "$BASHRC_FILE"
    echo ".bashrcに環境設定を追加しました"
else
    echo ".bashrcの環境設定は既に存在します"
fi

echo "=========================================="
echo "ESP-IDF アップデート完了！"
echo ""
echo "現在のバージョン: $current_version"
echo ""
echo "使用方法："
echo "1. 環境をアクティベート："
echo "   source ~/work/isolation-sphere/esp-idf-setup/activate_env.sh"
echo ""
echo "2. プロジェクトでのビルド例："
echo "   # 環境アクティベート後"
echo "   cd /home/yakatano/work/isolation-sphere/esp32/test_hello_world"
echo "   idf.py set-target esp32s3"
echo "   idf.py build"
echo "   idf.py -p /dev/ttyACM0 flash monitor"
echo ""
echo "=========================================="

# 即座に環境をアクティベート（この端末用）
echo ""
echo "現在のターミナルでESP-IDF環境をアクティベート中..."
if [ -f "./export.sh" ]; then
    source ./export.sh
    echo "ESP-IDF環境がアクティベートされました！"
    echo "idf.py version: $(idf.py --version 2>/dev/null || echo 'failed to get version')"
else
    echo "WARNING: export.sh not found, manual activation required"
fi