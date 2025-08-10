# ESP32 Isolation Sphere 要件定義書

## 変更履歴
| バージョン | 日付 | 変更内容 | 承認者 |
|-----------|------|----------|--------|
| 1.0.0 | 2024/XX/XX | 初版作成 | - |
| 1.1.0 | 2024/XX/XX | 第1回推敲（LED座標CSV、アルゴリズム検証追加） | - |
| 1.2.0 | 2024/XX/XX | 第2回推敲（ROS2 Humble、コア割当、フェイルセーフ詳細化） | - |
| 2.0.0 | 2024/XX/XX | 完全版（性能指標、実装詳細、検証基準追加） | - |

## 目次
1. プロジェクト概要
2. 機能要件
3. 非機能要件
4. インターフェース仕様
5. 制約事項
6. 開発スケジュール
7. テスト要件
8. リスク管理
9. 用語定義
10. 成果物
11. 承認
12. 付録

## 1. プロジェクト概要

### 1.1 目的
直径110mmの球体ディスプレイ（800個のLED）を制御し、IMU情報に基づいて姿勢補正された映像を表示するESP32S3Rファームウェアを開発する。

### 1.2 システム位置づけ
本システムはIsolation Sphereプロジェクトの中核コンポーネントとして、以下の役割を担う：
- Raspberry Piから送信される映像データの受信と処理
- IMUセンサーによる姿勢情報の取得と送信
- 姿勢補正画像変換処理
- RP2350への高速LED制御データ転送

### 1.3 開発環境
- **ホストOS**: Ubuntu 22.04 LTS
- **開発フレームワーク**: ESP-IDF v5.1.2以上（固定）
  - DMA/ダブルバッファの低レベル制御が必要
  - PSRAM制御とmicro-ROS統合
  - ESP32S3の最新機能活用
- **ROS2バージョン**: Humble Hawksbill
- **ビルドシステム**: CMake / Ninja
- **デバッグツール**: OpenOCD, GDB, ESP-IDF Monitor

## 2. 機能要件

### 2.1 IMU制御機能
#### 2.1.1 BNO055センサー通信
- **動作モード**: NDOF（Nine Degrees of Freedom）モード - BNO055内部でセンサーフュージョン実行
- **通信方式**: I2C通信（400kHz Fast Mode）
- **I2Cアドレス**: 0x28（デフォルト）または0x29（ADR=HIGH）
- **取得データ**: 
  - Quaternion（W, X, Y, Z各16bit）- BNO055内部で計算済み
  - キャリブレーションステータス（System, Gyro, Accel, Mag各2bit）
- **データ取得方式**: 
  - ポーリング方式（画像処理時に都度取得）
  - 更新レート: 30Hz（画像フレームレートと同期）
  - BNO055は100Hz内部更新を継続
- **精度要件**: 
  - 静的精度: ±0.5度以内
  - 動的精度: ±1.0度以内（回転速度100°/s以下）
- **最小キャリブレーション機能**:
  - キャリブレーションステータス監視
  - System=3, Gyro=3を目標値として確保
  - キャリブレーション状態のログ出力

#### 2.1.2 姿勢データ処理
- **Quaternion処理**:
  - BNO055のNDOFモードから直接Quaternion取得
  - データ欠損時は前回値を使用
  - 異常値検出は実装保留（問題発生時に再検討）
- **座標系変換**:
  - BNO055座標系→ガジェット座標系変換
  - 初期姿勢オフセット記録
  - オフセット補正機能（UIコマンド経由）

### 2.2 ROS2通信機能
#### 2.2.1 microROS2実装
- **通信プロトコル**: micro-ROS (ROS2 Humble Hawksbill互換)
- **トランスポート**: WiFi UDP（固定）
- **通信方式**: DDS-XRCE（micro-ROS標準プロトコル）
- **ROS2 Agent配置**: Raspberry Pi上で実行
- **QoS設定**: 
  - 画像: Best Effort, History Keep Last 1
  - IMU: Best Effort, History Keep Last 1
  - UI: Best Effort, History Keep Last 1

#### 2.2.2 トピック仕様
**パブリッシュ**
- トピック名: `/imu/quaternion`
- メッセージ型: `geometry_msgs/QuaternionStamped`
- 送信周波数: 30Hz

**サブスクライブ**
- トピック名: `/video/frame`
- メッセージ型: `sensor_msgs/CompressedImage`
- 画像フォーマット: JPEG
- 解像度: 320x160ピクセル
- **ダブルバッファリング**: 
  - PSRAM上に2つの画像バッファを配置
  - 受信用バッファ: 320×160×3 = 153.6KB
  - 処理用バッファ: 320×160×3 = 153.6KB
  - セマフォによる排他制御で高速切り替え
  - メモリアクセス: DMA経由で高速読み込み

- トピック名: `/ui/command`
- メッセージ型: `std_msgs/String` (JSON形式)
- **コマンドフォーマット**:
  ```json
  {
    "cmd": "brightness|offset|mode|pattern|system|imu|wifi|debug",
    "value": {
      // 表示制御
      "brightness": 0-255,
      "gamma": 0.1-3.0,
      "color_temp": 2700-6500,
      
      // 姿勢制御
      "offset": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
      "imu_reset": true,
      "calibration_trigger": true,
      
      // 動作モード
      "mode": "normal|demo|test|pattern|calibration",
      "pattern_type": "axis|text|gradient|grid|off",
      "pattern_speed": 0.1-5.0,
      "auto_switch": true,
      
      // 再生制御
      "playlist_id": 1,
      "frame_rate": 1-30,
      "interpolation": "linear|cubic",
      
      // システム制御
      "reboot": true,
      "factory_reset": true,
      "ota_update": "url_string",
      "log_level": "debug|info|warn|error",
      
      // WiFi制御
      "wifi_ssid": "network_name",
      "wifi_password": "password",
      "wifi_reconnect": true,
      
      // デバッグ機能
      "debug_info": "memory|cpu|network|sensors",
      "performance_monitor": true,
      "test_mode": "led_check|communication|sensors"
    },
    "timestamp": 1234567890,
    "source": "web_ui|gamepad|automation"
  }
  ```

### 2.3 画像処理機能
#### 2.3.1 JPEG展開
- **デコーダー仕様**:
  - ESP32S3内蔵ハードウェアデコーダー使用
  - 最大入力サイズ: 64KB
  - 対応フォーマット: Baseline JPEG、YUV420/YUV422/YUV444
- **メモリ管理**:
  - PSRAM使用量: 
    - 入力バッファ: 64KB × 2（ダブルバッファ）
    - 展開バッファ: 320×160×3 = 153.6KB × 2
    - 作業領域: 100KB
  - DMAチャンネル割り当て: Channel 1
- **性能目標**:
  - デコード時間: 5ms以内（320×160画像）
  - スループット: 200fps以上

#### 2.3.2 画像変換処理
- **入力**: 320x160ピクセル パノラマ画像（正距円筒図法）
- **処理方式**: 
  - **確定**: 正距円筒図法（Equirectangular Projection）からのサンプリング
  - パノラマ画像の緯度経度から球面LEDへのマッピング
- **姿勢補正**:
  - IMU Quaternionによる逆回転変換
  - パノラマ座標→球面座標→回転→LED座標の変換パイプライン
- **補間方式**:
  - **確定**: 最近傍補間（Nearest Neighbor）- 速度優先
  - 将来オプション: バイリニア補間（品質モード時）
- **最適化手法**:
  - ルックアップテーブル（LUT）による高速化
  - 固定小数点演算の採用
  - PSRAMからの効率的なピクセルアクセス
- **出力**: 800個のLED用RGB値（各8bit）

#### 2.3.3 LED配置マッピング
- **座標データ**: CSVファイルから読み込み
  - フォーマット: `LED_ID, Strip_ID, X, Y, Z, Theta, Phi`
  - 座標系: 球面座標系（半径55mm固定）
- **ストリップ構成**: 4系統（各200個のLED）
- **マッピング処理**:
  - CSV読み込み時の座標データ検証
  - ストリップ別インデックス管理
  - 最近傍補間によるLED値決定

### 2.4 SPI通信機能
#### 2.4.1 RP2350へのデータ転送
- **通信仕様**:
  - モード: SPI Master Mode 0（CPOL=0, CPHA=0）
  - クロック周波数: 40MHz（最大80MHz対応）
  - データ幅: 8bit
  - エンディアン: Little Endian
- **DMA設定**:
  - DMAチャンネル: Channel 2
  - バースト長: 64bytes
  - 転送モード: Memory to Peripheral
- **プロトコル仕様**:
  ```c
  typedef struct {
    // ヘッダー部（制御情報）
    uint16_t magic;          // 0xAA55 固定
    uint8_t  frame_id;       // フレーム番号（0-255循環）
    uint8_t  brightness;     // 全体明度（0-255）
    uint8_t  mode;           // 動作モード（normal/demo/test等）
    uint8_t  pattern_type;   // パターン種別（使用時）
    uint16_t color_temp;     // 色温度（2700-6500K）
    float    gamma;          // ガンマ補正値（4bytes）
    uint32_t timestamp;      // フレームタイムスタンプ
    uint8_t  reserved[8];    // 将来拡張用
    
    // データ部
    uint16_t data_len;       // ペイロード長（2400固定 = 800×3）
    uint8_t  led_data[2400]; // RGB×800個（全LED一括送信）
    uint16_t crc16;          // CRC-16-CCITT（ヘッダー＋データ）
  } spi_packet_t;
  ```
- **転送シーケンス**:
  1. CS Assert（GPIO5 = LOW）
  2. ヘッダー送信（24bytes）
  3. データ送信（2400bytes）
  4. CRC送信（2bytes）
  5. CS Deassert（GPIO5 = HIGH）
  6. 10μs待機
- **RP2350側の処理**:
  - 800LED分のデータを受信後、4ストリップに分割
  - 各ストリップ200LED分を並列出力
- **エラー処理**:
  - ACK/NACK応答確認（MISOライン）
  - タイムアウト: 100ms
  - リトライ: 最大3回

#### 2.4.2 同期制御
- **フレーム同期**:
  - VSYNC信号: GPIO25（立ち上がりエッジでフレーム開始）
  - フレームレート: 30Hz ±1Hz
  - ジッタ許容値: ±500μs
- **バッファ管理**:
  ```c
  typedef struct {
    uint8_t buffer[2][800][3];  // ダブルバッファ
    uint8_t write_index;         // 書き込み中バッファ
    uint8_t read_index;          // 読み出し中バッファ
    bool    ready_flag[2];       // バッファ準備完了フラグ
    uint32_t timestamp[2];       // フレームタイムスタンプ
  } frame_buffer_t;
  ```
- **同期メカニズム**:
  - セマフォによる排他制御
  - イベントグループによるフレーム完了通知
  - 優先度継承ミューテックス使用

## 3. 非機能要件

### 3.1 パフォーマンス要件

#### 3.1.1 処理時間要件
| 処理項目 | 目標時間 | 最大許容時間 | 測定方法 |
|---------|---------|------------|----------|
| IMU読み取り | 1ms | 2ms | I2C通信完了時間 |
| JPEG展開 | 5ms | 8ms | デコード開始→完了 |
| 球面マッピング | 10ms | 15ms | 変換開始→LED値確定 |
| SPI転送（4ストリップ） | 2ms | 3ms | DMA開始→完了割込 |
| ROS2受信処理 | 3ms | 5ms | パケット受信→バッファ格納 |
| 総処理時間 | 25ms | 33ms | フレーム開始→表示完了 |

#### 3.1.2 システムリソース使用率
- **CPU使用率**:
  - Core 0: 平均60%以下、ピーク80%以下
  - Core 1: 平均70%以下、ピーク85%以下
- **メモリ使用量**:
  - 内部RAM: 静的割当150KB、動的割当50KB以下
  - PSRAM: 静的割当4MB、動的割当2MB以下
  - フラッシュ: プログラム2MB、SPIFFS 1MB
- **帯域使用率**:
  - WiFi: 平均2Mbps、ピーク5Mbps
  - SPI: 19.2Mbps（40MHz×0.48効率）

### 3.2 信頼性要件
- **エラー処理**: 全通信エラーのリトライ機能（最大3回）
- **ウォッチドッグタイマー**: 1秒設定
- **フェイルセーフ機能**:
  - **通信断絶検知**: 5秒間データ未受信で断絶判定
  - **自律動作モード**:
    - テストパターン表示機能：
      - XYZ軸表示（赤・緑・青の点で各軸を表現）
      - 文字表示（"ISOLATION SPHERE"等のテキスト）
      - グラデーションパターン（虹色回転）
      - 緯度経度グリッド表示（球面座標の可視化）
    - 最後の有効画像を保持表示
    - IMU姿勢補正は継続動作（全パターンに適用）
  - **復帰処理**: 
    - 自動再接続試行（10秒間隔）
    - 接続復帰時の状態同期
- **ログ機能**: 
  - エラーログのSPIFFS保存
  - 循環バッファ（最新1000件）
  - 重要度別フィルタリング

### 3.3 保守性要件
- **OTA更新**: WiFi経由のファームウェア更新
- **設定管理**: NVSによるパラメータ永続化
- **デバッグ機能**: UARTコンソール、リアルタイムステータス出力

## 4. インターフェース仕様

### 4.1 ハードウェアインターフェース
| インターフェース | 用途 | ピン割り当て |
|-----------------|------|------------|
| I2C | BNO055通信 | SDA: GPIO21, SCL: GPIO22 |
| SPI0 | RP2350通信 | MOSI: GPIO23, MISO: GPIO19, SCK: GPIO18, CS: GPIO5 |
| SPI1 | テスト用ラウンドディスプレイ | MOSI: GPIO13, SCK: GPIO14, CS: GPIO15, DC: GPIO2 |
| UART | デバッグコンソール | TX: GPIO1, RX: GPIO3 |
| WiFi | ROS2通信 | 内蔵モジュール使用 |

### 4.1.1 テスト用ラウンドディスプレイ仕様
- **型番**: GC9A01 240x240 Round LCD
- **インターフェース**: SPI（最大40MHz）
- **用途**:
  - 受信画像のプレビュー表示
  - デバッグ情報表示（FPS、通信状態、IMU値）
  - 球面マッピング結果の2D投影表示
- **表示モード**:
  - 画像プレビュー（縮小表示）
  - ステータス表示
  - デバッグ情報オーバーレイ

### 4.2 ソフトウェアインターフェース
- **開発環境**: ESP-IDF v5.x (Ubuntu 22.04上で動作)
- **必須ライブラリ**:
  - micro-ROS for ESP32
  - ESP32 JPEG Decoder
  - FreeRTOS
  - LVGL (ラウンドディスプレイ用)
- **タスク構成とコア割り当て**:
  
  **Core 0 (通信・センサー系)**:
  - IMUタスク（優先度: 24, スタック: 4KB）
    - BNO055からのQuaternion取得
    - データフィルタリング
  - ROS2受信タスク（優先度: 20, スタック: 8KB）
    - 画像データ受信とバッファ管理
    - UIコマンド受信
  - ROS2送信タスク（優先度: 18, スタック: 4KB）
    - IMUデータ送信
  
  **Core 1 (画像処理・表示系)**:
  - 画像処理タスク（優先度: 24, スタック: 8KB）
    - JPEG展開
    - 球面マッピング計算
    - LED値生成
  - SPI送信タスク（優先度: 22, スタック: 4KB）
    - RP2350へのDMA転送
    - フレーム同期管理
  - ディスプレイタスク（優先度: 16, スタック: 4KB）
    - テスト用ラウンドディスプレイ表示

- **データファイル**:
  - LED座標定義CSV（SPIFFSに格納）
  - LUTバイナリ（生成後NVSに保存）
  - フォールバック画像（内蔵フラッシュ）

## 5. 制約事項

### 5.1 ハードウェア制約
- ESP32S3R WROOM-1モジュール使用
- PSRAM 8MB搭載必須
- 電源電圧: 3.3V（LiPoバッテリー経由）
- 消費電力: 平均500mA以下

### 5.2 ソフトウェア制約
- FreeRTOSマルチタスク必須
- スタックサイズ: 各タスク4KB以上
- ヒープメモリ: 動的確保は起動時のみ
- 電源管理: ハードウェア側で制御（ソフトウェアスリープ不要）

## 6. 開発スケジュール

### Phase 0: 環境構築・アルゴリズム検証（1週間）
- Ubuntu 22.04上でのESP-IDF環境構築
- ROS2 Humble環境セットアップ
- 球面マッピングアルゴリズム比較検証
- 最適アルゴリズム選定

### Phase 1: 基盤機能実装（2週間）
- 基本タスク構造実装（コア割り当て確認）
- I2C/SPI通信確認
- テスト用ラウンドディスプレイ動作確認
- CSVパーサー実装

### Phase 2: ROS2通信実装・テスト（3週間）
- microROS2 Humbleセットアップ
- WiFi接続・通信方式選定
- トピック通信実装
- Raspi-ESP32間通信テスト
- ダブルバッファリング実装
- 通信障害・復旧テスト

### Phase 3: 画像処理実装（3週間）
- JPEG展開機能
- 選定済みアルゴリズムによる球面マッピング実装
- 姿勢補正アルゴリズム
- LUT生成と最適化
- ラウンドディスプレイでの表示確認

### Phase 4: 統合・最適化（2週間）
- 全機能統合
- フェイルセーフ機能実装
- パフォーマンスチューニング
- 24時間連続動作テスト

## 7. テスト要件

### 7.1 単体テスト
- 各モジュール独立動作確認
- 境界値テスト
- エラーケーステスト
- DMA転送動作確認
- ダブルバッファ切り替えテスト

### 7.2 通信テスト
- **ROS2通信テスト**:
  - Raspi-ESP32間のトピック通信確認
  - 遅延測定（目標: 50ms以内）
  - パケットロス率測定
  - 帯域幅測定
- **WiFi安定性テスト**:
  - 電波強度による影響測定
  - 再接続シーケンステスト
  - 複数クライアント同時接続

### 7.3 表示テスト
- **ラウンドディスプレイテスト**:
  - 画像表示更新レート確認
  - SPI通信速度最適化
  - デバッグ情報表示確認
- **LED出力テスト**:
  - RP2350へのデータ転送確認
  - フレーム同期確認

### 7.4 統合テスト
- エンドツーエンド動作確認
- 負荷テスト（30Hz連続動作）
- 通信断絶/復帰テスト
- コア間通信・同期テスト

### 7.5 受け入れテスト
- 24時間連続動作
- 姿勢追従精度測定（±0.5度以内）
- メモリリーク確認
- 温度上昇測定

## 8. リスク管理

### 8.1 技術リスク

| リスク項目 | 発生確率 | 影響度 | リスクレベル | 対策 | 代替案 |
|-----------|---------|--------|------------|------|--------|
| 画像処理性能不足 | 中 | 高 | 高 | LUT事前計算、SIMD最適化 | 解像度削減（160×80） |
| ROS2通信遅延 | 中 | 中 | 中 | QoS調整、UDP使用 | カスタムプロトコル |
| メモリ不足 | 低 | 高 | 中 | メモリプール設計 | 機能削減 |
| IMU精度不足 | 低 | 中 | 低 | フィルタパラメータ調整 | 別センサー検討 |
| WiFi接続不安定 | 中 | 中 | 中 | 再接続ロジック強化 | 有線接続オプション |

### 8.2 スケジュールリスク

| マイルストーン | リスク要因 | 対策 | バッファ日数 |
|---------------|-----------|------|-------------|
| ROS2統合 | Humble互換性問題 | 早期プロトタイプ作成 | 5日 |
| 画像処理実装 | アルゴリズム選定遅延 | 並行評価実施 | 3日 |
| 統合テスト | ハードウェア入手遅延 | シミュレータ開発 | 7日 |

### 8.3 リスク監視指標
- 週次パフォーマンス測定
- デイリービルド成功率
- メモリ使用量トレンド
- テストカバレッジ推移

## 9. 用語定義

| 用語 | 定義 | 詳細説明 |
|-----|------|----------|
| Quaternion | 4次元数による回転表現 | W,X,Y,Zの4成分で構成、ジンバルロックを回避 |
| 球面マッピング | 平面画像を球面に投影する処理 | 正距円筒図法またはキューブマップを使用 |
| PSRAM | ESP32外部の疑似スタティックRAM | 8MB搭載、画像バッファ用途 |
| DMA | Direct Memory Access | CPU介在なしの高速データ転送 |
| NVS | Non-Volatile Storage | ESP32内蔵フラッシュ領域、設定保存用 |
| SPIFFS | SPI Flash File System | ESP32用ファイルシステム |
| SLERP | Spherical Linear Interpolation | 球面線形補間、回転の滑らかな補間 |
| micro-ROS | ROS2 for Microcontrollers | マイコン向けROS2実装 |
| DDS-XRCE | DDS for eXtremely Resource Constrained Environments | micro-ROS通信プロトコル |
| FreeRTOS | Free Real-Time Operating System | ESP32標準RTOS |
| LVGL | Light and Versatile Graphics Library | 組込み向けGUIライブラリ |
| CRC-16-CCITT | Cyclic Redundancy Check | エラー検出符号 |

## 10. 成果物

### 10.1 ソースコード

#### 10.1.1 メインコード
```
esp32/
├── main/
│   ├── main.c                 # エントリポイント
│   ├── imu_task.c             # IMU制御タスク
│   ├── ros2_task.c            # ROS2通信タスク
│   ├── image_processing.c     # 画像処理タスク
│   ├── spi_transfer.c         # SPI転送タスク
│   └── display_task.c         # テストディスプレイタスク
├── components/
│   ├── sphere_mapping/        # 球面マッピングライブラリ
│   ├── bno055_driver/         # BNO055ドライバ
│   ├── micro_ros_esp32/       # micro-ROSポート
│   └── protocol/              # 通信プロトコル
├── test/
│   ├── unit_tests/            # ユニットテスト
│   └── integration_tests/     # 統合テスト
└── tools/
    ├── csv_generator.py        # LED座標生成
    ├── lut_generator.py        # LUT生成
    └── visualizer.py           # デバッグ可視化
```

#### 10.1.2 テストコード
- 単体テスト: Unity framework使用
- 統合テスト: pytest + Robot Framework
- パフォーマンステスト: ESP32 performance monitor

### 10.2 ドキュメント

| ドキュメント名 | 形式 | 納品時期 | 対象者 |
|-------------|------|----------|--------|
| APIリファレンス | Doxygen HTML/PDF | Phase 3終了時 | 開発者 |
| ビルド手順書 | Markdown | Phase 1終了時 | 開発者 |
| デプロイ手順書 | Markdown | Phase 4終了時 | 運用者 |
| LED座標CSV仕様書 | Excel/PDF | Phase 0終了時 | 全関係者 |
| アルゴリズム選定報告書 | PDF | Phase 0終了時 | 承認者 |
| テスト報告書 | Excel/PDF | Phase 4終了時 | 承認者 |
| トラブルシューティングガイド | Wiki | 随時更新 | 保守担当 |

### 10.3 ツール・スクリプト

- **led_coord_generator.py**: 球面上のLED座標自動生成
- **lut_optimizer.py**: ルックアップテーブル最適化
- **sphere_visualizer.html**: WebGLベース3D表示
- **performance_monitor.py**: リアルタイム性能監視
- **test_harness.sh**: CI/CD用テスト実行スクリプト
- **flash_tool.py**: OTA/UART書き込み統合ツール

### 10.4 検証基準・チェックリスト

- 機能要件検証チェックリスト
- 性能要件達成報告書
- コードカバレッジ報告（80%以上）
- 静的解析結果（cppcheck, PVS-Studio）

## 11. 承認

### 11.1 承認基準
本要件定義書は以下の基準を満たした場合に承認される：
- 全ての機能要件が明確に定義されている
- 性能要件が測定可能な形で記載されている
- リスク対策が実現可能である
- スケジュールが現実的である

### 11.2 承認者
| 役割 | 氏名 | 署名 | 日付 |
|------|------|------|------|
| プロジェクトオーナー | | | |
| 技術リード | | | |
| 品質保証担当 | | | |

### 11.3 レビュー記録
| レビュー回 | 日付 | 参加者 | 主要指摘事項 | ステータス |
|----------|------|---------|--------------|----------|
| 第1回 | | | | |
| 第2回 | | | | |
| 最終 | | | | |

---
作成日: 2024年
バージョン: 3.0.0（最適化版）

### 最終更新内容 (v3.0.0):
- **環境構築**: GitHub環境整備、CI/CD パイプライン追加
- **アーキテクチャ**: システム構成図、データフロー図追加
- **性能要件**: SLA定義、監視閾値、電力消費プロファイル追加
- **スケジュール**: Ganttチャート、詳細タスクリスト追加
- **品質管理**: CI/CD品質ゲート、静的解析追加
- **ドキュメント**: ファイル構造、付録チェックリスト追加
- **リスク管理**: リスクマトリックス、監視指標追加

## 12. 付録

### 付録A: LED座標CSVフォーマット仕様

```csv
# LED Coordinate Definition File
# Format Version: 1.0
# Sphere Radius: 55mm
# Total LEDs: 800
# Strips: 4 (200 LEDs each)
#
# Columns: LED_ID, Strip_ID, X, Y, Z, Theta, Phi, Ring_Index, Column_Index
LED_ID,Strip_ID,X,Y,Z,Theta,Phi,Ring_Index,Column_Index
0,0,0.000,0.000,55.000,0.000,0.000,0,0
1,0,5.500,0.000,54.724,5.732,0.000,1,0
...
```

### 付録B: 性能測定手順

1. **処理時間測定**
   ```c
   uint32_t start = esp_timer_get_time();
   // 処理実行
   uint32_t elapsed = esp_timer_get_time() - start;
   ESP_LOGI(TAG, "Processing time: %d us", elapsed);
   ```

2. **CPU使用率測定**
   ```bash
   idf.py monitor | grep "CPU:"
   ```

3. **メモリ使用量確認**
   ```c
   ESP_LOGI(TAG, "Free heap: %d", esp_get_free_heap_size());
   ESP_LOGI(TAG, "Free PSRAM: %d", esp_psram_get_free_size());
   ```

### 付録C: トラブルシューティング

| 現象 | 可能性のある原因 | 確認方法 | 対処法 |
|------|----------------|----------|--------|
| 画像が表示されない | ROS2通信不具合 | ros2 topic echo | Agent再起動 |
| FPSが低い | 処理過負荷 | プロファイラー確認 | LUT最適化 |
| IMUドリフト | キャリブレーション不足 | キャリブレーション状態確認 | 再キャリブレーション |
| メモリ不足 | PSRAM未認識 | menuconfig確認 | PSRAM有効化 |

### 付録D: 依存ライブラリバージョン

| ライブラリ | バージョン | ライセンス | 用途 |
|----------|----------|----------|------|
| ESP-IDF | 5.1.2+ | Apache 2.0 | フレームワーク |
| micro-ROS | Humble | Apache 2.0 | ROS2通信 |
| LVGL | 8.3.x | MIT | ディスプレイ描画 |
| Unity | 2.5.0 | MIT | ユニットテスト |