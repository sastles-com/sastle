# Config Manager コンポーネント

このコンポーネントは、isolation-sphereプロジェクトの設定情報をSPIFFS上のJSONファイルで管理します。

## 機能

- JSONファイルでの設定管理
- デフォルト設定の自動生成
- 設定値の検証
- 設定ファイルのバックアップ・復元
- 個別設定項目の更新

## 設定項目

### 画像設定 (image)
- `width`: 画像幅 (64-1920)
- `height`: 画像高さ (64-1080)
- `fps`: フレームレート (1-60)
- `quality`: 画質 (1-100)
- `format`: 画像フォーマット ("JPEG", "PNG")

### LED設定 (led)
- `count`: 総LED数 (1-2000)
- `brightness`: 明度 (0-255)
- `strip_count`: ストリップ数 (1-8)
- `leds_per_strip`: ストリップあたりのLED数
- `update_rate_hz`: 更新レート (1-120Hz)

### IMU設定 (imu)
- `sample_rate_hz`: サンプリングレート (1-1000Hz)
- `calibration_enabled`: キャリブレーション有効/無効
- `orientation_offset`: 姿勢オフセット (quaternion)
- `filter_strength`: フィルタ強度 (0-10)

### 通信設定 (communication)
- `wifi_ssid`: WiFi SSID
- `wifi_password`: WiFi パスワード
- `ros2_domain_id`: ROS2 ドメインID
- `ros2_namespace`: ROS2 ネームスペース
- `update_rate_hz`: 通信更新レート

### システム設定 (system)
- `log_level`: ログレベル (0-5)
- `debug_mode`: デバッグモード有効/無効
- `watchdog_timeout_ms`: ウォッチドッグタイムアウト
- `performance_monitoring`: 性能監視有効/無効

## 使用方法

### 初期化
```c
#include "config_manager.h"

// 初期化（SPIFFSマウント、設定ファイル読み込み）
esp_err_t ret = config_manager_init();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Config manager init failed");
}
```

### 設定の取得
```c
// 現在の設定を取得
const sphere_config_t *config = config_get_current();

// 個別設定へのアクセス
uint16_t image_width = config->image.width;
uint16_t led_count = config->led.count;
uint8_t imu_rate = config->imu.sample_rate_hz;
```

### 設定の更新
```c
// 画像設定の更新
config_update_image_settings(640, 480, 15, 85);

// LED設定の更新
config_update_led_settings(800, 128, 30);
```

### 設定の表示
```c
// 現在の設定をログ出力
config_print_current();
```

### 終了処理
```c
// SPIFFSアンマウント
config_manager_deinit();
```

## サンプル設定ファイル

```json
{
  "image": {
    "width": 320,
    "height": 160,
    "fps": 10,
    "quality": 80,
    "format": "JPEG"
  },
  "led": {
    "count": 800,
    "brightness": 128,
    "strip_count": 4,
    "leds_per_strip": 200,
    "update_rate_hz": 30
  },
  "imu": {
    "sample_rate_hz": 100,
    "calibration_enabled": true,
    "orientation_offset": [1.0, 0.0, 0.0, 0.0],
    "filter_strength": 5
  },
  "communication": {
    "wifi_ssid": "",
    "wifi_password": "",
    "ros2_domain_id": 0,
    "ros2_namespace": "sphere",
    "update_rate_hz": 30
  },
  "system": {
    "log_level": 3,
    "debug_mode": false,
    "watchdog_timeout_ms": 5000,
    "performance_monitoring": true
  }
}
```

## ファイル構成

- `/spiffs/sphere_config.json`: メイン設定ファイル
- `/spiffs/sphere_config_backup.json`: バックアップファイル

## エラーハンドリング

設定ファイルが破損した場合、自動的にデフォルト設定が適用されます。
バックアップファイルが存在する場合は復元が可能です。

## パーティション設定

SPIFFSパーティションが必要です：

```csv
# partitions.csv
spiffs, data, spiffs, 0x190000, 0x200000,
```