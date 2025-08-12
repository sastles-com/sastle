# M5AtomS3R LCD バックライト制御 - 解決記録

## 概要

M5AtomS3RのLCDバックライト制御に関する問題解決の記録です。当初I2C制御と推測していましたが、実際は**GPIO16による直接制御**であることが判明しました。

## 解決済み問題

### 問題：LCDバックライトが全く点灯しない

**症状：**
- LCD表示コードは正常に動作
- SPI通信も成功
- しかしバックライトが点灯せず、画面が真っ暗

## 正しい解決方法

### バックライト制御仕様

| 項目 | 仕様 |
|------|------|
| **制御方式** | GPIO直接制御（**I2Cではない**） |
| **制御ピン** | **GPIO16** |
| **制御論理** | HIGH = 点灯, LOW = 消灯 |
| **PWM周波数** | 500Hz（明度制御時） |
| **電圧レベル** | 3.3V Logic |

### 間違った仕様（試行錯誤の記録）

以下は**間違った仕様**です：

❌ **I2C制御**（アドレス0x30, 0x48）
❌ **GPIO32, 46, 47, 48, 15, 2, 4, 5**による制御
❌ **M5Unified Light_M5StackAtomS3R実装**の模倣

## 正しい実装コード

### 1. ヘッダー定義（m5atoms3r_lcd.h）

```c
// M5AtomS3R LCD GPIO pins
#define LCD_PIN_MOSI     21     // SPI MOSI - G21
#define LCD_PIN_SCLK     15     // SPI SCLK - G15  
#define LCD_PIN_CS       14     // Chip Select - G14
#define LCD_PIN_DC       42     // Data/Command - G42
#define LCD_PIN_RST      48     // Reset - G48
#define LCD_PIN_BL       16     // Backlight control - GPIO16 ⭐重要⭐
```

### 2. バックライト初期化

```c
static esp_err_t backlight_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 16),    // GPIO16
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 初期状態：バックライトOFF
    gpio_set_level(16, 0);
    return ESP_OK;
}
```

### 3. バックライト制御関数

```c
// 基本ON/OFF制御
static esp_err_t backlight_set_enable(bool enable) {
    gpio_set_level(16, enable ? 1 : 0);
    return ESP_OK;
}

// PWM明度制御
static esp_err_t backlight_set_brightness(uint8_t brightness) {
    // brightness: 0-100%
    uint32_t duty = (brightness * 1023) / 100;  // 10-bit PWM
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    return ESP_OK;
}
```

### 4. PWM設定（明度制御用）

```c
static esp_err_t backlight_pwm_init(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 500,                    // 500Hz PWM
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = 16,                    // GPIO16
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);
    
    return ESP_OK;
}
```

## 検証方法

### テストシーケンス

1. **基本ON/OFFテスト**
   ```c
   backlight_set_enable(true);   // 点灯
   vTaskDelay(pdMS_TO_TICKS(2000));
   backlight_set_enable(false);  // 消灯  
   vTaskDelay(pdMS_TO_TICKS(2000));
   ```

2. **PWM明度テスト**
   ```c
   for (int brightness = 0; brightness <= 100; brightness += 5) {
       backlight_set_brightness(brightness);
       vTaskDelay(pdMS_TO_TICKS(100));
   }
   ```

3. **LCD表示との組み合わせテスト**
   - LCD初期化
   - テストパターン表示
   - バックライト制御による視認性確認

## 技術的詳細

### LCD仕様
- **LCD Controller**: GC9107
- **解像度**: 128x128ピクセル
- **色深度**: 16-bit RGB565
- **SPI周波数**: 1MHz（安定動作）

### GPIO配置
```
M5AtomS3R LCD接続:
┌─────────────────┐
│ ESP32S3         │
│                 │
│ GPIO21 → MOSI   │
│ GPIO15 → SCLK   │  
│ GPIO14 → CS     │
│ GPIO42 → DC     │
│ GPIO48 → RST    │
│ GPIO16 → BL     │ ⭐バックライト⭐
└─────────────────┘
```

## トラブルシューティング履歴

### 試行1: 間違ったGPIOピン
- 試行ピン: GPIO46, 32, 47, 48, 15, 2, 4, 5
- 結果: 全て失敗（GPIO32はWDTリセット発生）

### 試行2: I2C制御の誤解
- 試行内容: I2Cアドレス0x30でLP5562チップ制御を模擬
- 結果: I2C通信は成功するが、バックライト点灯せず
- 原因: M5AtomS3RはI2C制御ではない

### 試行3: M5Unified実装の誤解釈
- 試行内容: M5Unified Light_M5StackAtomS3R クラスの実装を模倣
- 結果: 複雑な初期化シーケンスを実装したが効果なし
- 原因: 該当実装は別のハードウェア用

### 解決: LovyanGFXライブラリ調査
- **情報源**: LovyanGFX, M5Stack-IDF ライブラリ
- **発見**: GPIO16が正しいバックライト制御ピン
- **実装**: 直接GPIO制御
- **結果**: ✅ **成功！バックライト点灯**

## 教訓

1. **公式ライブラリの調査が重要**
   - 推測ではなく、実績のあるライブラリの実装を参照

2. **ハードウェア仕様の確認**
   - 同名デバイスでも世代・バリエーションで仕様が異なる

3. **段階的テスト**
   - 最も単純な制御方法（GPIO直接制御）から試すべき

4. **コミュニティ情報の活用**
   - GitHub Issues, フォーラム, ドキュメントを総合的に調査

## 参考資料

- [LovyanGFX Library](https://github.com/lovyan03/LovyanGFX)
- [M5Stack-IDF Library](https://github.com/m5stack/M5Stack-IDF)  
- [M5Stack AtomS3R Documentation](https://docs.m5stack.com/ja/core/AtomS3R)

## ファイル構成

成功時の実装ファイル：
```
esp32/test_hello_world/main/
├── gpio_backlight_test.c      # GPIO16バックライトテスト（成功版）
├── gpio_backlight_test.h      # ヘッダー
├── m5atoms3r_lcd.h            # GPIO16定義（#define LCD_PIN_BL 16）
├── m5atoms3r_lcd.c            # LCD制御実装
└── main.c                     # run_gpio_backlight_test() 呼び出し
```

---
**最終更新**: 2025-08-12  
**ステータス**: ✅ 解決済み - GPIO16による制御成功  
**重要度**: 🔴 CRITICAL - プロジェクト基盤となる制御方式