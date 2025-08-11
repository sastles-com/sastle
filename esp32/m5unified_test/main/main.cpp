#include <M5Unified.h>

extern "C" void app_main()
{
    auto cfg = M5.config();
    
    // M5AtomS3R specific configuration
    cfg.external_display.atom_display = true;  // Enable AtomDisplay
    cfg.external_display.module_display = false;
    cfg.external_display.unit_glass = false;
    cfg.external_display.unit_glass2 = false;
    cfg.external_display.unit_oled = false;
    cfg.external_display.unit_mini_oled = false;
    cfg.external_display.unit_lcd = false;
    cfg.external_display.unit_rca = false;
    cfg.external_display.module_rca = false;
    
    M5.begin(cfg);
    
    // Clear screen
    M5.Display.fillScreen(BLACK);
    
    // Set text properties
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(1);
    
    // Display test patterns similar to Arduino M5Unified Display sample
    int counter = 0;
    uint16_t colors[] = {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE};
    const char* color_names[] = {"RED", "GREEN", "BLUE", "YELLOW", "CYAN", "MAGENTA", "WHITE"};
    
    while (1) {
        int color_index = counter % 7;
        
        // Fill screen with color
        M5.Display.fillScreen(colors[color_index]);
        
        // Draw text
        M5.Display.setTextColor(colors[color_index] == WHITE ? BLACK : WHITE);
        M5.Display.setCursor(10, 10);
        M5.Display.printf("M5AtomS3R");
        
        M5.Display.setCursor(10, 30);
        M5.Display.printf("Display Test");
        
        M5.Display.setCursor(10, 50);
        M5.Display.printf("Color: %s", color_names[color_index]);
        
        M5.Display.setCursor(10, 70);
        M5.Display.printf("Count: %d", counter);
        
        // Draw some graphics
        M5.Display.drawCircle(64, 100, 20, colors[color_index] == WHITE ? BLACK : WHITE);
        M5.Display.fillCircle(64, 100, 10, colors[color_index] == WHITE ? BLACK : WHITE);
        
        M5.update();
        
        counter++;
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2 seconds delay
    }
}