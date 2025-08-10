# 設定管理システム

## 概要

Isolation Sphereの設定を階層的に管理するシステム

## 設定の種類

### 1. ビルド時固定設定（sphere_config.h）
- **特徴**: コンパイル時に決定、実行時変更不可
- **用途**: ハードウェア仕様、GPIO割り当て、メモリサイズなど
- **変更方法**: ヘッダーファイル編集 → 再ビルド

### 2. 実行時設定（NVS保存）
- **特徴**: 実行中に変更可能、電源OFF後も保持
- **用途**: 明度、WiFi設定、動作モードなど
- **変更方法**: UIコマンド、Web UI、設定API

### 3. 一時設定（RAM保存）
- **特徴**: 実行中のみ有効、電源OFFで消去
- **用途**: デバッグフラグ、テストモードなど

## ファイル構成

```
config/
├── sphere_config.h           # ビルド時固定設定
├── sphere_config_runtime.json # 実行時設定のデフォルト値
├── config_manager.h          # 設定管理API
├── config_manager.c          # 設定管理実装（要作成）
└── README.md                 # このファイル
```

## 使用方法

### 基本的な使用例

```c
#include "config_manager.h"

void app_main() {
    // 初期化
    config_manager_init();
    
    // 現在の設定取得
    sphere_config_t config;
    config_get_current(&config);
    
    // 画像サイズを動的に取得
    uint16_t width = config.image.width;
    uint16_t height = config.image.height;
    
    // 明度を変更
    config.display.brightness = 200;
    config_update(&config, true);  // NVSに保存
}
```

### UIコマンドからの設定変更

```c
// JSONコマンド処理
void handle_ui_command(const char *json_cmd) {
    // 例: {"cmd": "brightness", "value": 150}
    
    cJSON *root = cJSON_Parse(json_cmd);
    if (cJSON_GetObjectItem(root, "cmd")) {
        const char *cmd = cJSON_GetObjectItem(root, "cmd")->valuestring;
        
        if (strcmp(cmd, "brightness") == 0) {
            int value = cJSON_GetObjectItem(root, "value")->valueint;
            config_set_value("display.brightness", value);
        }
    }
    cJSON_Delete(root);
}
```

## 設定項目の変更可能性

| カテゴリ | 項目 | ビルド時 | 実行時 | 備考 |
|---------|------|----------|--------|------|
| **ハードウェア** |
| | LED総数 | ✓ | - | 800固定 |
| | 球体直径 | ✓ | - | 110mm固定 |
| | GPIOピン | ✓ | - | 基板依存 |
| **画像** |
| | 解像度 | △ | ✓ | 320x160デフォルト、変更可能 |
| | FPS | △ | ✓ | 10fpsデフォルト、1-30fps |
| | JPEG品質 | △ | ✓ | 80デフォルト、0-100 |
| **表示** |
| | 明度 | - | ✓ | 0-255 |
| | ガンマ値 | - | ✓ | 0.1-3.0 |
| | 色温度 | - | ✓ | 2700-6500K |
| **ネットワーク** |
| | WiFi SSID | △ | ✓ | デフォルト設定可能 |
| | ROS2 Agent IP | △ | ✓ | 192.168.1.100デフォルト |
| **性能** |
| | タスク優先度 | ✓ | - | FreeRTOS制約 |
| | スタックサイズ | ✓ | - | メモリ制約 |

## 画像解像度の動的変更

### 実装方針

```c
// config_manager.c内部実装例

typedef struct {
    uint8_t *buffer[2];  // ダブルバッファ
    size_t buffer_size;
    uint16_t width;
    uint16_t height;
} dynamic_image_buffer_t;

esp_err_t image_buffer_resize(uint16_t new_width, uint16_t new_height) {
    size_t new_size = new_width * new_height * 3;
    
    // PSRAMから新しいバッファを確保
    uint8_t *new_buffer = heap_caps_malloc(new_size, MALLOC_CAP_SPIRAM);
    if (!new_buffer) {
        return ESP_ERR_NO_MEM;
    }
    
    // 古いバッファを解放
    heap_caps_free(old_buffer);
    
    // 設定を更新
    config.image.width = new_width;
    config.image.height = new_height;
    
    return ESP_OK;
}
```

### 制約事項

1. **メモリ制限**: PSRAMの空き容量に依存
2. **処理性能**: 解像度増加で処理時間増加
3. **通信帯域**: 大きい画像は転送時間増加

## 設定の優先順位

1. **UIコマンド** （最優先）
2. **NVS保存値**
3. **JSONデフォルト値**
4. **ヘッダーファイルのデフォルト値**

## 設定変更時の通知

```c
// コールバック登録例
void on_config_change(const char *key, void *user_data) {
    ESP_LOGI(TAG, "Config changed: %s", key);
    
    if (strcmp(key, "display.brightness") == 0) {
        // 明度変更時の処理
        update_led_brightness();
    }
}

// 登録
config_register_callback(on_config_change, NULL);
```

## メンテナンス

### 新しい設定項目の追加手順

1. `sphere_config.h`にデフォルト値を定義
2. `sphere_config_runtime.json`に実行時デフォルトを追加
3. `config_manager.h`の構造体に追加
4. NVSキーを定義
5. 設定変更ハンドラを実装

### 設定のバックアップ・リストア

```bash
# 設定エクスポート（UART経由）
idf.py monitor
> config export

# 設定インポート
> config import {"display":{"brightness":200},...}
```

## トラブルシューティング

| 問題 | 原因 | 対策 |
|------|------|------|
| 設定が保存されない | NVS初期化失敗 | NVSパーティション確認 |
| メモリ不足エラー | 解像度が大きすぎる | PSRAMサイズ確認、解像度削減 |
| 設定競合 | 複数タスクから同時アクセス | ミューテックス使用 |

---
最終更新: 2024年