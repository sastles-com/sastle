/**
 * @file config_manager.h
 * @brief 設定管理システムのヘッダーファイル
 * 
 * ビルド時設定（sphere_config.h）と実行時設定（NVS）を統合管理
 */

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "sphere_config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// 設定構造体定義
// ============================================================

/**
 * @brief ディスプレイ設定（実行時変更可能）
 */
typedef struct {
    uint8_t brightness;          // LED明度 (0-255)
    float gamma;                 // ガンマ補正値 (0.1-3.0)
    uint16_t color_temperature;  // 色温度 (2700-6500K)
    bool auto_brightness;        // 自動明度調整
} display_config_t;

/**
 * @brief 画像処理設定（実行時変更可能）
 */
typedef struct {
    uint16_t width;              // 画像幅（デフォルト: IMAGE_WIDTH）
    uint16_t height;             // 画像高さ（デフォルト: IMAGE_HEIGHT）
    uint8_t fps;                 // フレームレート (1-30)
    char interpolation[16];      // 補間方式 ("linear", "cubic")
    uint8_t jpeg_quality;        // JPEG品質 (0-100)
} image_config_t;

/**
 * @brief IMU設定（実行時変更可能）
 */
typedef struct {
    struct {
        float x, y, z, w;        // Quaternionオフセット
    } offset;
    bool auto_calibration;       // 自動キャリブレーション
    float filter_strength;       // フィルタ強度 (0.0-1.0)
} imu_config_t;

/**
 * @brief ネットワーク設定
 */
typedef struct {
    struct {
        char ssid[32];           // WiFi SSID
        char password[64];       // WiFiパスワード
        bool auto_connect;       // 自動接続
        uint8_t reconnect_interval; // 再接続間隔[秒]
    } wifi;
    
    struct {
        char agent_ip[16];       // ROS2 Agent IPアドレス
        uint16_t agent_port;     // ROS2 Agentポート
        char transport[8];       // トランスポート ("udp", "tcp")
    } ros2;
} network_config_t;

/**
 * @brief 統合設定構造体
 */
typedef struct {
    // 実行時変更可能な設定
    display_config_t display;
    image_config_t image;
    imu_config_t imu;
    network_config_t network;
    
    // ビルド時固定設定へのポインタ（読み取り専用）
    const struct {
        uint16_t total_leds;
        uint8_t strip_count;
        uint16_t leds_per_strip;
        float sphere_radius_mm;
    } *hardware;
    
    // 設定バージョン
    uint32_t version;
    
} sphere_config_t;

// ============================================================
// API関数
// ============================================================

/**
 * @brief 設定管理システムの初期化
 * 
 * NVSから設定を読み込み、存在しない場合はデフォルト値を使用
 * 
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t config_manager_init(void);

/**
 * @brief 現在の設定を取得
 * 
 * @param[out] config 設定を格納する構造体へのポインタ
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t config_get_current(sphere_config_t *config);

/**
 * @brief 設定を更新
 * 
 * @param[in] config 新しい設定
 * @param[in] save_to_nvs NVSに保存するか
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t config_update(const sphere_config_t *config, bool save_to_nvs);

/**
 * @brief 特定の設定項目を更新
 * 
 * @param[in] key 設定キー（例: "display.brightness"）
 * @param[in] value 設定値（文字列形式）
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t config_set_value(const char *key, const char *value);

/**
 * @brief 特定の設定項目を取得
 * 
 * @param[in] key 設定キー
 * @param[out] value 値を格納するバッファ
 * @param[in] value_size バッファサイズ
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t config_get_value(const char *key, char *value, size_t value_size);

/**
 * @brief 設定を工場出荷時に戻す
 * 
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t config_factory_reset(void);

/**
 * @brief 設定をJSONフォーマットでエクスポート
 * 
 * @param[out] json_buffer JSONを格納するバッファ
 * @param[in] buffer_size バッファサイズ
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t config_export_json(char *json_buffer, size_t buffer_size);

/**
 * @brief JSONフォーマットから設定をインポート
 * 
 * @param[in] json_string JSON文字列
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t config_import_json(const char *json_string);

/**
 * @brief 設定変更時のコールバック登録
 * 
 * @param[in] callback コールバック関数
 * @param[in] user_data ユーザーデータ
 * @return ESP_OK: 成功, その他: エラー
 */
typedef void (*config_change_callback_t)(const char *key, void *user_data);
esp_err_t config_register_callback(config_change_callback_t callback, void *user_data);

// ============================================================
// ヘルパーマクロ
// ============================================================

// ビルド時設定へのアクセス（読み取り専用）
#define CONFIG_GET_LED_COUNT()          TOTAL_LED_COUNT
#define CONFIG_GET_IMAGE_WIDTH()        (config.image.width)
#define CONFIG_GET_IMAGE_HEIGHT()       (config.image.height)
#define CONFIG_GET_IMAGE_SIZE()         (CONFIG_GET_IMAGE_WIDTH() * CONFIG_GET_IMAGE_HEIGHT() * IMAGE_CHANNELS)

// 設定範囲チェック
#define CONFIG_VALIDATE_BRIGHTNESS(x)   ((x) >= 0 && (x) <= 255)
#define CONFIG_VALIDATE_FPS(x)          ((x) >= 1 && (x) <= 30)
#define CONFIG_VALIDATE_GAMMA(x)        ((x) >= 0.1f && (x) <= 3.0f)

#ifdef __cplusplus
}
#endif

#endif // CONFIG_MANAGER_H