#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "LovyanGFX.hpp"

static const char *TAG = "LOVYAN_GFX_TEST";

// M5AtomS3R用のLovyanGFX設定
class LGFX : public lgfx::LGFX_Device
{
public:
  lgfx::Panel_GC9107      _panel_instance;
  lgfx::Bus_SPI           _bus_instance;
  lgfx::Light_PWM         _light_instance;

  LGFX(void)
  {
    // バス設定
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;     // 使用するSPIを選択
      cfg.spi_mode = 0;             // SPI通信モード (0 ~ 3)
      cfg.freq_write = 80000000;    // 送信時のSPIクロック (最大80MHz)
      cfg.freq_read  = 16000000;    // 受信時のSPIクロック
      cfg.spi_3wire  = true;        // 受信をMOSIピンで行う場合はtrueを設定
      cfg.use_lock   = true;        // トランザクションロックを使用
      cfg.dma_channel = SPI_DMA_CH_AUTO; // 使用するDMAチャンネル
      cfg.pin_sclk = 17;            // SPIのSCLKピン番号を設定
      cfg.pin_mosi = 21;            // SPIのMOSIピン番号を設定
      cfg.pin_miso = -1;            // SPIのMISOピン番号を設定 (-1 = disable)
      cfg.pin_dc   = 15;            // SPIのD/Cピン番号を設定
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    // 表示パネル設定
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           =    6;  // CSが接続されているピン番号
      cfg.pin_rst          =   34;  // RSTが接続されているピン番号
      cfg.pin_busy         =   -1;  // BUSYが接続されているピン番号 (-1 = disable)

      cfg.panel_width      =  128;  // 実際に表示可能な幅
      cfg.panel_height     =  128;  // 実際に表示可能な高さ
      cfg.offset_x         =    0;  // パネルのX方向オフセット量
      cfg.offset_y         =    0;  // パネルのY方向オフセット量
      cfg.offset_rotation  =    2;  // 回転方向の値のオフセット 0~7
      cfg.dummy_read_pixel =    8;  // ピクセル読出し前のダミーリードのビット数
      cfg.dummy_read_bits  =    1;  // ピクセル以外のデータ読出し前のダミーリードのビット数
      cfg.readable         = false; // データ読出しが可能な場合 trueに設定
      cfg.invert           = true;  // パネルの明暗が反転してしまう場合 trueに設定
      cfg.rgb_order        = false; // パネルの赤と青が入れ替わってしまう場合 trueに設定
      cfg.dlen_16bit       = false; // 16bitパラレルやSPIでデータ長を16bit単位で送信するパネル
      cfg.bus_shared       = false; // SDカードとバスを共有している場合 trueに設定

      _panel_instance.config(cfg);
    }

    // バックライト制御設定
    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = 16;              // バックライトが接続されているピン番号
      cfg.invert = false;           // バックライトの輝度を反転させる場合 true
      cfg.freq   = 500;            // バックライトのPWM周波数 (M5AtomS3R推奨: 500Hz)
      cfg.pwm_channel = 0;          // 使用するPWMのチャンネル番号

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    setPanel(&_panel_instance);
  }
};

// LovyanGFXインスタンス作成
LGFX lcd;

extern "C" void app_main(void)
{
  ESP_LOGI(TAG, "M5AtomS3R LovyanGFX Test Starting...");
  
  // 液晶初期化
  lcd.init();
  ESP_LOGI(TAG, "LCD initialized");
  
  // バックライト輝度設定 (0～255)
  lcd.setBrightness(128);
  ESP_LOGI(TAG, "Backlight set to 128/255");
  
  // 背景色設定
  lcd.fillScreen(0x0000); // 黒色
  ESP_LOGI(TAG, "Screen cleared");
  
  // シンプルなテストパターン
  ESP_LOGI(TAG, "Drawing test patterns");
  
  // まず全画面赤色
  lcd.fillScreen(0xF800); // 赤
  ESP_LOGI(TAG, "Red screen displayed");
  
  uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFFF, 0x0000, 0xFFE0};
  const char* color_names[] = {"RED", "GREEN", "BLUE", "WHITE", "BLACK", "YELLOW"};
  int counter = 0;
  
  while (true) {
    int color_index = counter % 6;
    lcd.fillScreen(colors[color_index]);
    
    ESP_LOGI(TAG, "Display color: %s (counter: %d)", color_names[color_index], counter);
    counter++;
    
    // 2秒待機
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}