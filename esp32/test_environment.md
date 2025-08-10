# ESP-IDF 環境確認テスト仕様書

## 目的
ESP32S3R開発環境の基本動作を確認し、開発準備が整っていることを検証する。

## テスト実施順序

### Phase 0: 環境構築確認テスト（最優先）

#### 0.1 ESP-IDF インストール確認

```bash
# テスト項目: ESP-IDF_001
# 目的: ESP-IDFが正しくインストールされているか確認
cd ~/esp
git clone -b v5.1.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
source ./export.sh

# 確認コマンド
idf.py --version
# 期待結果: ESP-IDF v5.1.2 以上が表示される
```

#### 0.2 Hello World プロジェクト作成

```bash
# テスト項目: ESP-IDF_002
# 目的: 基本プロジェクトのビルド確認
cd /home/yakatano/work/isolation-sphere/esp32
idf.py create-project hello_world
cd hello_world

# menuconfig でターゲット設定
idf.py set-target esp32s3
idf.py menuconfig
# Component config → ESP32S3-Specific → CPU frequency (240 MHz)
# Component config → ESP PSRAM → Support for external, SPI-connected RAM → ✓
```

#### 0.3 ビルド確認

```bash
# テスト項目: ESP-IDF_003
# 目的: コンパイル環境の動作確認
idf.py build

# 期待結果:
# - ビルドが成功する
# - build/hello_world.bin が生成される
# - サイズ情報が表示される
```

#### 0.4 USB接続確認

```bash
# テスト項目: ESP-IDF_004
# 目的: ESP32S3Rボードの認識確認

# デバイス接続前
ls /dev/ttyUSB* /dev/ttyACM*

# ESP32S3R接続後
ls /dev/ttyUSB* /dev/ttyACM*
# 期待結果: /dev/ttyUSB0 または /dev/ttyACM0 が出現

# 権限確認
sudo usermod -a -G dialout $USER
# ログアウト・ログイン後に再確認
```

#### 0.5 フラッシュ書き込み確認

```bash
# テスト項目: ESP-IDF_005
# 目的: フラッシュ書き込みの動作確認
idf.py -p /dev/ttyUSB0 flash

# 期待結果:
# - Connecting... 成功
# - Writing... 進行状況表示
# - Hash of data verified. 表示
```

#### 0.6 シリアルモニター確認

```bash
# テスト項目: ESP-IDF_006
# 目的: シリアル通信とログ出力確認
idf.py -p /dev/ttyUSB0 monitor

# 期待結果:
# - "Hello world!" メッセージ表示
# - チップ情報表示（ESP32-S3, features: WiFi, BLE）
# - リブートループせずに安定動作
# Ctrl+] で終了
```

### Phase 0.5: ハードウェア機能確認テスト

#### 1. PSRAM認識テスト

```c
// test_psram.c
#include <stdio.h>
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"

static const char *TAG = "PSRAM_TEST";

void app_main(void) {
    ESP_LOGI(TAG, "PSRAM Test Starting...");
    
    // PSRAM初期化確認
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM is initialized");
        
        size_t psram_size = esp_psram_get_size();
        ESP_LOGI(TAG, "PSRAM size: %d bytes (%.2f MB)", 
                 psram_size, psram_size / (1024.0 * 1024.0));
        
        // PSRAM動作テスト
        void *ptr = heap_caps_malloc(1024 * 1024, MALLOC_CAP_SPIRAM);
        if (ptr != NULL) {
            ESP_LOGI(TAG, "Successfully allocated 1MB in PSRAM");
            heap_caps_free(ptr);
        } else {
            ESP_LOGE(TAG, "Failed to allocate PSRAM");
        }
        
        // 空きメモリ確認
        ESP_LOGI(TAG, "Free PSRAM: %d bytes", 
                 heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    } else {
        ESP_LOGE(TAG, "PSRAM not found!");
    }
}
```

#### 2. GPIO/LED点滅テスト

```c
// test_gpio.c
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define LED_GPIO 2  // 内蔵LED（ボードにより異なる）

static const char *TAG = "GPIO_TEST";

void app_main(void) {
    ESP_LOGI(TAG, "GPIO Blink Test");
    
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    
    for (int i = 0; i < 10; i++) {
        gpio_set_level(LED_GPIO, i % 2);
        ESP_LOGI(TAG, "LED: %s", (i % 2) ? "ON" : "OFF");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "GPIO Test Complete");
}
```

#### 3. WiFi接続テスト

```c
// test_wifi.c
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define WIFI_SSID "test_network"
#define WIFI_PASS "test_password"

static const char *TAG = "WIFI_TEST";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi started, connecting...");
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected successfully!");
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "WiFi Connection Test");
    
    // NVS初期化
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                               &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization complete");
}
```

#### 4. FreeRTOSマルチタスクテスト

```c
// test_freertos.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "RTOS_TEST";

void task_core0(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Task on Core 0 - Priority: %d", 
                 uxTaskPriorityGet(NULL));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task_core1(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Task on Core 1 - Priority: %d", 
                 uxTaskPriorityGet(NULL));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "FreeRTOS Multi-Core Test");
    
    // Core 0にタスク作成
    xTaskCreatePinnedToCore(task_core0, "task0", 2048, NULL, 10, 
                            NULL, 0);
    
    // Core 1にタスク作成  
    xTaskCreatePinnedToCore(task_core1, "task1", 2048, NULL, 10, 
                            NULL, 1);
    
    ESP_LOGI(TAG, "Tasks created on both cores");
}
```

## テスト実施チェックリスト

### 必須確認項目
- [ ] ESP-IDF v5.1.2以上がインストールされている
- [ ] ESP32S3がターゲットとして設定できる
- [ ] Hello Worldがビルドできる
- [ ] USB経由でボードが認識される
- [ ] フラッシュ書き込みが成功する
- [ ] シリアルモニターでログが確認できる
- [ ] PSRAMが8MB認識される
- [ ] GPIO制御（LED点滅）が動作する
- [ ] WiFi接続が可能
- [ ] マルチコアでタスクが動作する

### トラブルシューティング

#### 問題: USB デバイスが見つからない
```bash
# USBドライバ確認
lsusb | grep -E "CP210|CH340|FTDI"

# 権限問題の解決
sudo chmod 666 /dev/ttyUSB0
```

#### 問題: PSRAMが認識されない
```bash
# menuconfigで設定確認
idf.py menuconfig
# Component config → ESP PSRAM → Support for external, SPI-connected RAM
# → SPI RAM config → PSRAM type: Auto-detect
```

#### 問題: ビルドエラー
```bash
# クリーンビルド
idf.py fullclean
idf.py build

# 依存関係更新
cd $IDF_PATH
git submodule update --init --recursive
```

## 成功基準

1. **環境構築**: 全てのツールが正しくインストールされ、パスが通っている
2. **ビルド**: サンプルプロジェクトが警告なしでビルドできる
3. **書き込み**: 5秒以内にフラッシュ書き込みが開始される
4. **動作確認**: 基本的なハードウェア機能が全て動作する
5. **安定性**: 10分間の連続動作でクラッシュしない

## 次のステップ

環境確認が完了したら：
1. I2C通信テスト（BNO055接続準備）
2. SPI通信テスト（ディスプレイ接続準備）
3. micro-ROS環境構築
4. 実際のプロジェクト構造作成

---
作成日: 2024年
バージョン: 1.0.0