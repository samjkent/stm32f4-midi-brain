#include "ht16k33.h"

HAL_StatusTypeDef status;
uint16_t ht16k33_Buffer[8];

HAL_StatusTypeDef ht16k33_init(HT16K33_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr)
{
	hdev->hi2c = hi2c;
	hdev->addr = addr << 1;

    status = HAL_I2C_IsDeviceReady(hdev->hi2c, hdev->addr, 10, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }

    hdev->buffer[0] = 0x0F0F;
    hdev->buffer[1] = 0xFFFF;
    hdev->buffer[2] = 0x0000;
    hdev->buffer[3] = 0xFFFF;
    hdev->buffer[4] = 0x0000;
    hdev->buffer[5] = 0xFFFF;
    hdev->buffer[6] = 0x0000;
    hdev->buffer[7] = 0xFFFF;

    // Turn the oscillator on
    uint8_t data = 0x01;
	status = HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, HT16K33_REGISTER_SYSTEM_SETUP, 1, &data, 1, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }

    // Turn blink off
    status = ht16k33_set_blink_rate(hdev, HT16K33_BLINKRATE_OFF);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }

    // Set max brightness
    status = ht16k33_set_brightness(hdev, 15);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }

    return HAL_OK;
}

HAL_StatusTypeDef ht16k33_set_brightness(HT16K33_HandleTypeDef *hdev, uint8_t brightness)
{
    if (brightness > 15) brightness = 15;
    status = HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, HT16K33_REGISTER_DIMMING, 1, &brightness, 1, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }
    return status;
}

HAL_StatusTypeDef ht16k33_set_blink_rate(HT16K33_HandleTypeDef *hdev, uint8_t blinkRate)
{
    if (blinkRate > HT16K33_BLINKRATE_HALFHZ) blinkRate = HT16K33_BLINKRATE_OFF;
    uint8_t data = 0x1 | (blinkRate << 1);
    status = HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, HT16K33_REGISTER_DISPLAY_SETUP, 1, &data, 1, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }
    return status;
}

HAL_StatusTypeDef ht16k33_write_display(HT16K33_HandleTypeDef *hdev)
{
    // Send 8x16-bits of data
    uint8_t data[16] = { 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0 };
    status = HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, HT16K33_RAM_ADDRESS, 1, (uint8_t *)data, 16, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }
    return status;
}

HAL_StatusTypeDef ht16k33_clear(HT16K33_HandleTypeDef *hdev)
{
  uint8_t i;
  for (i=0; i<8; i++)
  {
    hdev->buffer[i] = 0xFFFF;
  }
  return HAL_OK;
}

HAL_StatusTypeDef ht16k33_set_led(HT16K33_HandleTypeDef hdev, uint16_t led, uint8_t value) {
    // hdev->buffer 
    return HAL_OK;
}
