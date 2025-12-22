#include "lcd.h"
#include "string.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
static I2C_HandleTypeDef *lcd_i2c = NULL;

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define LCD_RW 0x00
#define LCD_RS 0x01

static const char *menu_items[] = {
    "1. Use Mode",
    "2. Save Mode",
    "3. Just ON",
    "4. Just OFF",
	"5. Reset"
};

static const uint8_t menu_count = sizeof(menu_items) / sizeof(menu_items[0]);

void lcd_send_raw_cmd(uint8_t cmd);
void lcd_send_internal(uint8_t data, uint8_t flags);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_byte(uint8_t data);

void lcd_init(I2C_HandleTypeDef *hi2c) {
	lcd_i2c = hi2c;
	HAL_Delay(100);
	
	for (int i = 0; i < 3; i++)
	{
		lcd_send_raw_cmd(0x30);
		HAL_Delay(i == 0 ? 5 : 1);
	}
	lcd_send_raw_cmd(0x20); 
	HAL_Delay(1);

	lcd_send_cmd(0x28); 
	lcd_send_cmd(0x08); 
	lcd_send_cmd(0x01); 
	HAL_Delay(2);
	lcd_send_cmd(0x06); 
	lcd_send_cmd(0x0C); 
}

void lcd_send_cmd(uint8_t cmd) {
	lcd_send_internal(cmd, 0);
}

void lcd_send_data(uint8_t data) {
	lcd_send_internal(data, LCD_RS);
}

void lcd_send_raw_cmd(uint8_t cmd) {
	uint8_t high_nibble = cmd & 0xF0;
	uint8_t buf[2];

	buf[0] = high_nibble | LCD_BACKLIGHT | LCD_ENABLE;
	buf[1] = high_nibble | LCD_BACKLIGHT;

	HAL_I2C_Master_Transmit(lcd_i2c, LCD_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
}

void lcd_send_internal(uint8_t data, uint8_t flags) {
	uint8_t high_nibble = data & 0xF0;
	uint8_t low_nibble = (data << 4) & 0xF0;
	uint8_t buf[4];

	buf[0] = high_nibble | flags | LCD_BACKLIGHT | LCD_ENABLE;
	buf[1] = high_nibble | flags | LCD_BACKLIGHT;
	buf[2] = low_nibble | flags | LCD_BACKLIGHT | LCD_ENABLE;
	buf[3] = low_nibble | flags | LCD_BACKLIGHT;

	HAL_I2C_Master_Transmit(lcd_i2c, LCD_I2C_ADDR, buf, 4, HAL_MAX_DELAY);
}

void lcd_print(const char *str) {
	while (*str) {
		lcd_send_data((uint8_t)(*str));
		str++;
	}
}

void print_status(const char *msg)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print(msg);
    HAL_Delay(500);
}

void lcd_clear(void) {
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};	
	lcd_send_cmd(0x80 | (col + row_offsets[row]));
}

void lcd_create_char(uint8_t location, uint8_t charmap[])
{
	location &= 0x07; 
	lcd_send_cmd(0x40 | (location << 3)); 

	for (int i = 0; i < 8; i++) {
			lcd_send_data(charmap[i]);
	}
}

void lcd_welcome(void)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("  <MY REMOTE>");
    HAL_Delay(3000);
    lcd_clear();
}

void lcd_update_menu(uint8_t menu_index)
{
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print(">");

    lcd_set_cursor(0, 2);
    lcd_print((char *)menu_items[menu_index]);

    if (menu_index + 1 < menu_count) {
        lcd_set_cursor(1, 2);
        lcd_print((char *)menu_items[menu_index + 1]);
    }
}

void lcd_update_use(uint8_t use_index, uint8_t total, char names[][16]) {
    lcd_clear();
    lcd_set_cursor(0,0);
    lcd_print(">");
    lcd_set_cursor(0,2);
	
    if (total == 0) {
        lcd_print("Empty");
    } else {
        lcd_print(names[use_index]);
    }
		
    lcd_set_cursor(1,2);
    if (use_index + 1 < total) {
        lcd_print(names[use_index+1]);
    }
}
