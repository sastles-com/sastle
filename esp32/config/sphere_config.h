/**
 * @file sphere_config.h
 * @brief Isolation Sphere システム設定定数定義
 * 
 * このファイルでシステム全体の定数を一元管理
 * ビルド時に変更可能、実行時変更が必要な項目はNVSで管理
 */

#ifndef SPHERE_CONFIG_H
#define SPHERE_CONFIG_H

// ============================================================
// 1. ディスプレイ設定
// ============================================================

// LED構成
#define SPHERE_DIAMETER_MM          110        // 球体直径 [mm]
#define SPHERE_RADIUS_MM            55         // 球体半径 [mm]
#define TOTAL_LED_COUNT             800        // LED総数
#define LED_STRIP_COUNT             4          // LEDストリップ数
#define LEDS_PER_STRIP              (TOTAL_LED_COUNT / LED_STRIP_COUNT)  // 200

// LED型番仕様
#define LED_TYPE                    "WS2812C-2020"
#define LED_REFRESH_RATE_HZ         800        // LED更新周波数 [Hz]

// ============================================================
// 2. 画像処理設定
// ============================================================

// 入力画像仕様（パノラマ画像）
#define IMAGE_WIDTH                 320        // パノラマ画像幅 [pixel]
#define IMAGE_HEIGHT                160        // パノラマ画像高さ [pixel]
#define IMAGE_CHANNELS              3          // RGB
#define IMAGE_SIZE_BYTES            (IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS)
#define IMAGE_FPS                   10         // 動画フレームレート [fps]
#define IMAGE_TYPE                  "equirectangular"  // 正距円筒図法

// 画像処理目標性能
#define TARGET_DISPLAY_FPS          30         // 表示更新レート [fps]
#define FRAME_TIME_MS               (1000 / TARGET_DISPLAY_FPS)  // 33ms

// 補間方式
#define INTERPOLATION_METHOD        "nearest"  // 最近傍補間（速度優先）

// JPEG設定
#define JPEG_MAX_SIZE_KB            64         // 最大JPEGサイズ [KB]
#define JPEG_QUALITY                80         // JPEG品質 (0-100)

// ダブルバッファリング
#define IMAGE_BUFFER_COUNT          2          // ダブルバッファ
#define PSRAM_IMAGE_BUFFER_SIZE     (IMAGE_SIZE_BYTES * IMAGE_BUFFER_COUNT)

// ============================================================
// 3. IMUセンサー設定
// ============================================================

// BNO055設定
#define IMU_I2C_ADDR_PRIMARY        0x28       // プライマリI2Cアドレス
#define IMU_I2C_ADDR_SECONDARY      0x29       // セカンダリI2Cアドレス（ADR=HIGH）
#define IMU_I2C_FREQ_HZ             400000     // I2C通信速度 [Hz] (Fast Mode)
#define IMU_SAMPLE_RATE_HZ          100        // BNO055内部更新レート [Hz]
#define IMU_POLL_RATE_HZ            TARGET_DISPLAY_FPS  // ポーリングレート [Hz]

// BNO055動作モード
#define IMU_MODE                    NDOF_MODE  // Nine Degrees of Freedom

// 精度要件
#define IMU_STATIC_ACCURACY_DEG     0.5        // 静的精度 [度]
#define IMU_DYNAMIC_ACCURACY_DEG    1.0        // 動的精度 [度]
#define IMU_MAX_ROTATION_SPEED_DPS  100        // 最大回転速度 [度/秒]

// キャリブレーション
#define IMU_CALIB_MIN_SYSTEM        3          // 最小システムキャリブレーション値
#define IMU_CALIB_MIN_GYRO          3          // 最小ジャイロキャリブレーション値

// ============================================================
// 4. 通信設定
// ============================================================

// WiFi設定（デフォルト値、NVSで上書き可能）
#define WIFI_SSID_DEFAULT           "isolation_sphere"
#define WIFI_PASSWORD_DEFAULT       "sphere_password"
#define WIFI_MAX_RETRY              5          // 最大再接続試行回数
#define WIFI_RECONNECT_INTERVAL_S   10         // 再接続間隔 [秒]

// ROS2/micro-ROS設定
#define ROS2_AGENT_IP               "192.168.1.100"  // Raspberry PiのIPアドレス
#define ROS2_AGENT_PORT             8888             // micro-ROS Agentポート
#define ROS2_TRANSPORT              "udp"            // トランスポート層

// トピック名
#define TOPIC_IMU_QUATERNION        "/imu/quaternion"
#define TOPIC_VIDEO_FRAME           "/video/frame"
#define TOPIC_UI_COMMAND            "/ui/command"

// QoS設定
#define QOS_HISTORY_DEPTH           1          // Keep Last設定

// ============================================================
// 5. SPI通信設定（RP2350向け）
// ============================================================

// SPIハードウェア設定
#define SPI_CLOCK_FREQ_MHZ          40         // SPIクロック周波数 [MHz]
#define SPI_MODE                    0          // SPI Mode 0 (CPOL=0, CPHA=0)
#define SPI_DMA_CHANNEL             2          // DMAチャンネル番号
#define SPI_DMA_BURST_SIZE          64         // DMAバーストサイズ [bytes]

// プロトコル設定
#define SPI_PACKET_MAGIC            0xAA55     // パケット識別子
#define SPI_PACKET_HEADER_SIZE      24         // ヘッダーサイズ [bytes]
#define SPI_PACKET_LED_DATA_SIZE    2400       // LED RGBデータサイズ (800*3)
#define SPI_PACKET_TOTAL_SIZE       (SPI_PACKET_HEADER_SIZE + SPI_PACKET_LED_DATA_SIZE + 2)  // +CRC
#define SPI_RETRY_COUNT             3          // エラー時リトライ回数
#define SPI_TIMEOUT_MS              100        // タイムアウト時間 [ms]

// 拡張ヘッダー情報
#define SPI_HEADER_RESERVED_SIZE    8          // 将来拡張用バイト数

// ============================================================
// 6. メモリ管理設定
// ============================================================

// 内部RAM割り当て
#define INTERNAL_RAM_STATIC_KB      150        // 静的割り当て [KB]
#define INTERNAL_RAM_DYNAMIC_KB     50         // 動的割り当て上限 [KB]

// PSRAM割り当て
#define PSRAM_STATIC_MB             4          // 静的割り当て [MB]
#define PSRAM_DYNAMIC_MB            2          // 動的割り当て上限 [MB]
#define PSRAM_TOTAL_MB              8          // PSRAM総容量 [MB]

// フラッシュ割り当て
#define FLASH_PROGRAM_MB            2          // プログラム領域 [MB]
#define FLASH_SPIFFS_MB             1          // SPIFFSファイルシステム [MB]
#define FLASH_NVS_KB                16         // NVS領域 [KB]

// ============================================================
// 7. タスク設定（FreeRTOS）
// ============================================================

// Core 0タスク（通信・センサー系）
#define IMU_TASK_PRIORITY           24
#define IMU_TASK_STACK_SIZE         4096
#define IMU_TASK_CORE               0

#define ROS2_RX_TASK_PRIORITY       20
#define ROS2_RX_TASK_STACK_SIZE     8192
#define ROS2_RX_TASK_CORE           0

#define ROS2_TX_TASK_PRIORITY       18
#define ROS2_TX_TASK_STACK_SIZE     4096
#define ROS2_TX_TASK_CORE           0

// Core 1タスク（画像処理・表示系）
#define IMAGE_TASK_PRIORITY         24
#define IMAGE_TASK_STACK_SIZE       8192
#define IMAGE_TASK_CORE             1

#define SPI_TASK_PRIORITY           22
#define SPI_TASK_STACK_SIZE         4096
#define SPI_TASK_CORE               1

#define DISPLAY_TASK_PRIORITY       16
#define DISPLAY_TASK_STACK_SIZE     4096
#define DISPLAY_TASK_CORE           1

// ============================================================
// 8. 性能要件
// ============================================================

// 処理時間目標 [ms]
#define TARGET_IMU_READ_MS          1
#define TARGET_JPEG_DECODE_MS       5
#define TARGET_SPHERE_MAP_MS        10
#define TARGET_SPI_TRANSFER_MS      2
#define TARGET_ROS2_RECEIVE_MS      3
#define TARGET_TOTAL_PROCESS_MS     25

// CPU使用率上限 [%]
#define MAX_CPU_CORE0_AVG           60
#define MAX_CPU_CORE0_PEAK          80
#define MAX_CPU_CORE1_AVG           70
#define MAX_CPU_CORE1_PEAK          85

// ============================================================
// 9. エラー処理・フェイルセーフ
// ============================================================

// タイムアウト設定
#define WATCHDOG_TIMEOUT_S          1          // ウォッチドッグタイマー [秒]
#define COMM_LOSS_TIMEOUT_S         5          // 通信断絶判定時間 [秒]
#define RECONNECT_INTERVAL_S        10         // 再接続試行間隔 [秒]

// ログ設定
#define LOG_BUFFER_SIZE             1000       // ログバッファサイズ [件]
#define LOG_LEVEL_DEFAULT           LOG_LEVEL_INFO

// ============================================================
// 10. テストパターン設定
// ============================================================

// パターン切り替え
#define PATTERN_AUTO_SWITCH_S       10         // 自動切り替え間隔 [秒]
#define PATTERN_ANIMATION_FPS       30         // アニメーションFPS

// 軸表示
#define AXIS_LINE_WIDTH             3          // 軸線の太さ [LED]
#define AXIS_COLOR_X                {255, 0, 0}    // X軸色（赤）
#define AXIS_COLOR_Y                {0, 255, 0}    // Y軸色（緑）
#define AXIS_COLOR_Z                {0, 0, 255}    // Z軸色（青）

// グリッド表示
#define GRID_LAT_SPACING_DEG        10         // 緯線間隔 [度]
#define GRID_LON_SPACING_DEG        15         // 経線間隔 [度]
#define GRID_LINE_BRIGHTNESS        128        // グリッド線明度

// ============================================================
// 11. GPIO設定
// ============================================================

// I2C (BNO055)
#define GPIO_I2C_SDA                21
#define GPIO_I2C_SCL                22

// SPI0 (RP2350)
#define GPIO_SPI0_MOSI              23
#define GPIO_SPI0_MISO              19
#define GPIO_SPI0_SCK               18
#define GPIO_SPI0_CS                5

// SPI1 (Round Display)
#define GPIO_SPI1_MOSI              13
#define GPIO_SPI1_SCK               14
#define GPIO_SPI1_CS                15
#define GPIO_SPI1_DC                2

// その他
#define GPIO_VSYNC                  25         // フレーム同期信号
#define GPIO_LED_BUILTIN            2          // 内蔵LED（ボード依存）

// ============================================================
// 12. バージョン情報
// ============================================================

#define FIRMWARE_VERSION            "1.0.0"
#define HARDWARE_VERSION            "ESP32S3R-1.0"
#define CONFIG_VERSION              1          // 設定フォーマットバージョン

#endif // SPHERE_CONFIG_H