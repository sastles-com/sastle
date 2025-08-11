#ifndef M5ATOMS3R_LCD_H
#define M5ATOMS3R_LCD_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// M5AtomS3R LCD specifications
#define LCD_WIDTH        128
#define LCD_HEIGHT       128
#define LCD_BPP          16     // 16-bit RGB565

// M5AtomS3R LCD GPIO pins
#define LCD_PIN_MOSI     21
#define LCD_PIN_SCLK     15
#define LCD_PIN_CS       14
#define LCD_PIN_DC       42     // Data/Command
#define LCD_PIN_RST      48     // Reset
#define LCD_PIN_BL       -1     // Backlight control (not available)

// SPI configuration
#define LCD_SPI_HOST     SPI2_HOST
#define LCD_SPI_FREQ     26000000    // 26MHz
#define LCD_SPI_MODE     0

// Colors (RGB565)
#define LCD_COLOR_BLACK     0x0000
#define LCD_COLOR_WHITE     0xFFFF
#define LCD_COLOR_RED       0xF800
#define LCD_COLOR_GREEN     0x07E0
#define LCD_COLOR_BLUE      0x001F
#define LCD_COLOR_YELLOW    0xFFE0
#define LCD_COLOR_CYAN      0x07FF
#define LCD_COLOR_MAGENTA   0xF81F
#define LCD_COLOR_GRAY      0x8410
#define LCD_COLOR_DARKGRAY  0x4208
#define LCD_COLOR_ORANGE    0xFD20
#define LCD_COLOR_PURPLE    0x801F

// GC9107 LCD controller commands
#define GC9107_SWRESET     0x01
#define GC9107_RDDID       0x04
#define GC9107_RDDST       0x09
#define GC9107_SLPIN       0x10
#define GC9107_SLPOUT      0x11
#define GC9107_PTLON       0x12
#define GC9107_NORON       0x13
#define GC9107_INVOFF      0x20
#define GC9107_INVON       0x21
#define GC9107_DISPOFF     0x28
#define GC9107_DISPON      0x29
#define GC9107_CASET       0x2A
#define GC9107_RASET       0x2B
#define GC9107_RAMWR       0x2C
#define GC9107_RAMRD       0x2E
#define GC9107_PTLAR       0x30
#define GC9107_COLMOD      0x3A
#define GC9107_MADCTL      0x36
#define GC9107_FRMCTR1     0xB1
#define GC9107_FRMCTR2     0xB2
#define GC9107_FRMCTR3     0xB3
#define GC9107_INVCTR      0xB4
#define GC9107_PWCTR1      0xC0
#define GC9107_PWCTR2      0xC1
#define GC9107_PWCTR3      0xC2
#define GC9107_PWCTR4      0xC3
#define GC9107_PWCTR5      0xC4
#define GC9107_VMCTR1      0xC5
#define GC9107_GMCTRP1     0xE0
#define GC9107_GMCTRN1     0xE1

// LCD handle structure
typedef struct {
    spi_device_handle_t spi_device;
    bool initialized;
    uint16_t width;
    uint16_t height;
    uint8_t rotation;
} m5atoms3r_lcd_t;

// Function prototypes
esp_err_t m5atoms3r_lcd_init(m5atoms3r_lcd_t* lcd);
esp_err_t m5atoms3r_lcd_deinit(m5atoms3r_lcd_t* lcd);
esp_err_t m5atoms3r_lcd_clear(m5atoms3r_lcd_t* lcd, uint16_t color);
esp_err_t m5atoms3r_lcd_fill_rect(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
esp_err_t m5atoms3r_lcd_draw_pixel(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, uint16_t color);
esp_err_t m5atoms3r_lcd_draw_char(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, char c, uint16_t color, uint16_t bg_color, uint8_t size);
esp_err_t m5atoms3r_lcd_print_string(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, const char* str, uint16_t color, uint16_t bg_color, uint8_t size);
esp_err_t m5atoms3r_lcd_printf(m5atoms3r_lcd_t* lcd, int16_t x, int16_t y, uint16_t color, uint16_t bg_color, uint8_t size, const char* format, ...);

// Utility functions
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b);

#endif // M5ATOMS3R_LCD_H