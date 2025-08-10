/**
 * @file spi_protocol.h
 * @brief ESP32→RP2350 SPI通信プロトコル定義
 * 
 * 800LED分の一括送信 + UI制御情報の統合プロトコル
 */

#ifndef SPI_PROTOCOL_H
#define SPI_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "sphere_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// プロトコル定数
// ============================================================

#define SPI_MAGIC_NUMBER            0xAA55
#define SPI_PACKET_VERSION          1

// 動作モード定義
typedef enum {
    SPI_MODE_NORMAL = 0,        // 通常動作（動画表示）
    SPI_MODE_DEMO = 1,          // デモモード
    SPI_MODE_TEST = 2,          // テストパターン
    SPI_MODE_PATTERN = 3,       // 固定パターン
    SPI_MODE_CALIBRATION = 4,   // キャリブレーション
    SPI_MODE_OFF = 5            // 消灯
} spi_mode_t;

// パターン種別定義
typedef enum {
    SPI_PATTERN_OFF = 0,        // 消灯
    SPI_PATTERN_AXIS = 1,       // XYZ軸表示
    SPI_PATTERN_TEXT = 2,       // 文字表示
    SPI_PATTERN_GRADIENT = 3,   // グラデーション
    SPI_PATTERN_GRID = 4,       // 緯度経度グリッド
    SPI_PATTERN_SOLID = 5       // 単色表示
} spi_pattern_t;

// ============================================================
// メインプロトコル構造体
// ============================================================

/**
 * @brief ESP32→RP2350 SPI送信パケット構造体
 * 
 * 全LED（800個）を一括送信し、RP2350側で4ストリップに分割処理
 */
typedef struct __attribute__((packed)) {
    // ===== ヘッダー部（24bytes） =====
    uint16_t magic;             // 0xAA55（パケット識別）
    uint8_t  frame_id;          // フレーム番号（0-255循環）
    
    // 表示制御情報
    uint8_t  brightness;        // 全体明度（0-255）
    uint8_t  mode;              // 動作モード（spi_mode_t）
    uint8_t  pattern_type;      // パターン種別（spi_pattern_t）
    uint16_t color_temp;        // 色温度（2700-6500K）
    
    // ガンマ補正
    union {
        float    gamma_float;   // ガンマ補正値（1.0-3.0）
        uint32_t gamma_raw;     // 4bytes raw data
    };
    
    // タイムスタンプ
    uint32_t timestamp;         // フレーム生成時刻（μs）
    
    // 将来拡張用
    uint8_t  reserved[8];       // 拡張フィールド
    
    // ===== データ部（2402bytes） =====
    uint16_t data_len;          // LEDデータ長（2400固定）
    uint8_t  led_data[2400];    // RGB×800LED（R,G,B,R,G,B...）
    
    // ===== 末尾（2bytes） =====
    uint16_t crc16;             // CRC-16-CCITT（ヘッダー+データ）
    
} spi_packet_t;

// コンパイル時サイズ確認
_Static_assert(sizeof(spi_packet_t) == SPI_PACKET_TOTAL_SIZE, 
               "SPI packet size mismatch");

// ============================================================
// ヘルパー構造体
// ============================================================

/**
 * @brief LED制御情報（ヘッダー情報の構造化）
 */
typedef struct {
    uint8_t brightness;         // 明度 (0-255)
    float gamma;                // ガンマ補正 (1.0-3.0)
    uint16_t color_temp;        // 色温度 (2700-6500K)
    spi_mode_t mode;            // 動作モード
    spi_pattern_t pattern;      // パターン種別
    uint32_t timestamp;         // タイムスタンプ
} led_control_info_t;

/**
 * @brief SPI送信統計情報
 */
typedef struct {
    uint32_t packets_sent;      // 送信パケット数
    uint32_t packets_acked;     // ACK受信数
    uint32_t packets_nacked;    // NACK受信数
    uint32_t crc_errors;        // CRCエラー数
    uint32_t timeouts;          // タイムアウト数
    uint32_t retries;           // リトライ数
    float    avg_transfer_time_us; // 平均転送時間[μs]
} spi_stats_t;

// ============================================================
// API関数
// ============================================================

/**
 * @brief SPIプロトコル初期化
 * 
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t spi_protocol_init(void);

/**
 * @brief SPIパケット作成
 * 
 * @param[out] packet 作成されるパケット
 * @param[in] led_data LED RGB データ（800×3 = 2400bytes）
 * @param[in] control_info 制御情報
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t spi_packet_create(spi_packet_t *packet, 
                            const uint8_t *led_data,
                            const led_control_info_t *control_info);

/**
 * @brief SPIパケット送信（DMA使用）
 * 
 * @param[in] packet 送信するパケット
 * @param[in] timeout_ms タイムアウト時間[ms]
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t spi_packet_transmit(const spi_packet_t *packet, 
                             uint32_t timeout_ms);

/**
 * @brief CRC-16-CCITT計算
 * 
 * @param[in] data データ
 * @param[in] length データ長
 * @return CRC値
 */
uint16_t spi_calculate_crc16(const uint8_t *data, size_t length);

/**
 * @brief SPIパケット検証
 * 
 * @param[in] packet 検証するパケット
 * @return true: 有効, false: 無効
 */
bool spi_packet_validate(const spi_packet_t *packet);

/**
 * @brief SPI統計情報取得
 * 
 * @param[out] stats 統計情報
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t spi_get_statistics(spi_stats_t *stats);

/**
 * @brief SPI統計情報リセット
 */
void spi_reset_statistics(void);

// ============================================================
// 高レベルAPI
// ============================================================

/**
 * @brief 800LEDデータを一括送信
 * 
 * @param[in] rgb_data RGB配列（800×3bytes）
 * @param[in] brightness 明度 (0-255)
 * @param[in] gamma ガンマ補正 (1.0-3.0)
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t spi_send_led_frame(const uint8_t rgb_data[2400],
                            uint8_t brightness,
                            float gamma);

/**
 * @brief テストパターン送信
 * 
 * @param[in] pattern パターン種別
 * @param[in] brightness 明度 (0-255)
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t spi_send_test_pattern(spi_pattern_t pattern,
                               uint8_t brightness);

/**
 * @brief 全LED消灯
 * 
 * @return ESP_OK: 成功, その他: エラー
 */
esp_err_t spi_send_all_off(void);

// ============================================================
// デバッグ機能
// ============================================================

/**
 * @brief パケット内容をダンプ出力
 * 
 * @param[in] packet ダンプするパケット
 */
void spi_packet_dump(const spi_packet_t *packet);

/**
 * @brief SPI通信のベンチマーク実行
 * 
 * @param[in] iterations 実行回数
 * @return 平均転送時間[μs]
 */
float spi_benchmark(uint32_t iterations);

#ifdef __cplusplus
}
#endif

#endif // SPI_PROTOCOL_H