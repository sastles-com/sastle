/**
 * @file lcd_unit_test.h
 * @brief LCD unit test header
 */

#ifndef LCD_UNIT_TEST_H
#define LCD_UNIT_TEST_H

#include "esp_err.h"

/**
 * @brief Run LCD unit test
 * @return ESP_OK on success
 */
esp_err_t run_lcd_unit_test(void);

#endif // LCD_UNIT_TEST_H