idf_component_register(
    SRCS "main.c" "imu_integration_test.c" "simple_bno055.c" "m5atoms3r_lcd.c" "quaternion_visualizer.c" "simple_test.c" "direct_imu_lcd_test.c"
    INCLUDE_DIRS "."
    REQUIRES "esp_psram" "spi_flash" "driver"
)