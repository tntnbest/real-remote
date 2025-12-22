#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define LCD_I2C_ADDR (0x27 << 1)  

void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_send_data(uint8_t data);
void lcd_print(const char *str);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_create_char(uint8_t location, uint8_t charmap[]);
void lcd_welcome(void);
void lcd_update_menu(uint8_t selected_index);
void lcd_update_use(uint8_t use_index, uint8_t total, char names[][16]);
void lcd_update_name(void);