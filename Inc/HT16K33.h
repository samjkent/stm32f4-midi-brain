#include "stm32f4xx_hal.h"

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint16_t addr;
    uint16_t buffer[8];
} HT16K33_HandleTypeDef;

#define HT16K33_REGISTER_SYSTEM_SETUP  0x20
#define HT16K33_REGISTER_DISPLAY_SETUP 0x80
#define HT16K33_REGISTER_DIMMING       0xE0

#define HT16K33_BLINKRATE_OFF          0x00
#define HT16K33_BLINKRATE_2HZ          0x01
#define HT16K33_BLINKRATE_1HZ          0x02
#define HT16K33_BLINKRATE_HALFHZ       0x03

#define HT16K33_RAM_ADDRESS            0x00

#define HT16K33_BASE_ADDRESS           0x70

HAL_StatusTypeDef ht16k33_init(HT16K33_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr);
HAL_StatusTypeDef ht16k33_set_brightness(HT16K33_HandleTypeDef *hdev, uint8_t brightness);
HAL_StatusTypeDef ht16k33_set_blink_rate(HT16K33_HandleTypeDef *hdev, uint8_t blinkRate);
HAL_StatusTypeDef ht16k33_write_display(HT16K33_HandleTypeDef *hdev);
HAL_StatusTypeDef ht16k33_clear(HT16K33_HandleTypeDef *hdev);
