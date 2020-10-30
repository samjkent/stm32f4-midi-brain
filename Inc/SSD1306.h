#include "stm32f4xx_hal.h"

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_BUFFER_SIZE ((SSD1306_WIDTH * SSD1306_HEIGHT) / 8)

#define SSD1306_BLACK 0x00
#define SSD1306_WHITE 0xFF

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint16_t addr;
    uint8_t buffer[SSD1306_BUFFER_SIZE];
} SSD1306_HandleTypeDef;

HAL_StatusTypeDef ssd1306_init(SSD1306_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr);
HAL_StatusTypeDef ssd1306_update_screen(SSD1306_HandleTypeDef *hdev);
HAL_StatusTypeDef ssd1306_fill_screen(SSD1306_HandleTypeDef *hdev, uint8_t colour);
HAL_StatusTypeDef ssd1306_draw_pixel(SSD1306_HandleTypeDef *hdev, uint8_t x, uint8_t y, uint8_t colour);
HAL_StatusTypeDef ssd1306_put_char(SSD1306_HandleTypeDef *hdev, uint8_t x, uint8_t y, char c);
HAL_StatusTypeDef ssd1306_put_string(SSD1306_HandleTypeDef *hdev, uint8_t x, uint8_t y, char* s);
