#include "ht16k33.h"

HAL_StatusTypeDef status;

HAL_StatusTypeDef ht16k33_init(HT16K33_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr)
{
	hdev->hi2c = hi2c;
	hdev->addr = addr << 1;
    for(int i=0; i<8; i++) {
        hdev->buffer[i] = 0x0000;
    }

    status = HAL_I2C_IsDeviceReady(hdev->hi2c, hdev->addr, 10, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }

    // Turn the oscillator on
    uint8_t data = HT16K33_REGISTER_SYSTEM_SETUP | 0x01;
    status = HAL_I2C_Master_Transmit(hdev->hi2c, hdev->addr, &data, 1, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }
    
    // Blink rate
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
    if(brightness > 15) brightness = 15;
    uint8_t data = HT16K33_REGISTER_DIMMING | brightness;
    status = HAL_I2C_Master_Transmit(hdev->hi2c, hdev->addr, &data, 1, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }
    return status;
}

HAL_StatusTypeDef ht16k33_set_blink_rate(HT16K33_HandleTypeDef *hdev, uint8_t blinkRate)
{
    if (blinkRate > HT16K33_BLINKRATE_HALFHZ) blinkRate = HT16K33_BLINKRATE_OFF;
    uint8_t data = HT16K33_REGISTER_DISPLAY_SETUP | HT16K33_DISPLAY_ON | (blinkRate << 1);
    status = HAL_I2C_Master_Transmit(hdev->hi2c, hdev->addr, &data, 1, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }
    return status;
}

HAL_StatusTypeDef ht16k33_write_display(HT16K33_HandleTypeDef *hdev)
{
    uint8_t buffer[17];
    buffer[0] = 0x00;
    for(int i = 0; i < 8; i++) {
        buffer[2*i+1] = hdev->buffer[i] & 0xFF;
        buffer[2*i+2] = hdev->buffer[i] >> 8;
    }
    // status = HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, HT16K33_RAM_ADDRESS, 1, buffer, sizeof buffer, HAL_TIMEOUT);
    status = HAL_I2C_Master_Transmit(hdev->hi2c, hdev->addr, buffer, 17, HAL_TIMEOUT);
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
    hdev->buffer[i] = 0x0000;
  }
  return HAL_OK;
}

HAL_StatusTypeDef ht16k33_set_led(HT16K33_HandleTypeDef *hdev, uint16_t led)
{
  hdev->buffer[led / 16] |= 0x1 << (led % 16);
  return HAL_OK;
}
