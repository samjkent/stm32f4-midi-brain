#include "SSD1306.h"
#include "font.h"

HAL_StatusTypeDef ssd1306_write(SSD1306_HandleTypeDef *hdev, uint8_t command)
{
    uint8_t data[2] = {0x00, command};
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdev->hi2c, hdev->addr, &data, 2, HAL_TIMEOUT);
    
    if(status != HAL_OK) {
        Error_Handler(__FILE__,__LINE__);
    }

    return status;
}


HAL_StatusTypeDef ssd1306_init(SSD1306_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr)
{	

	hdev->hi2c = hi2c;
	hdev->addr = addr << 1;

	// Wait for the screen to boot
	HAL_Delay(500);

    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hdev->hi2c, hdev->addr, 10, HAL_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }
	
	ssd1306_write(hdev, 0xAE); //display off
	ssd1306_write(hdev, 0x20); //Set Memory Addressing Mode   
	ssd1306_write(hdev, 0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	ssd1306_write(hdev, 0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	ssd1306_write(hdev, 0xC8); //Set COM Output Scan Direction
	ssd1306_write(hdev, 0x00); //---set low column address
	ssd1306_write(hdev, 0x10); //---set high column address
	ssd1306_write(hdev, 0x40); //--set start line address
	ssd1306_write(hdev, 0x81); //--set contrast control register
	ssd1306_write(hdev, 0xFF);
	ssd1306_write(hdev, 0xA1); //--set segment re-map 0 to 127
	ssd1306_write(hdev, 0xA6); //--set normal display
	ssd1306_write(hdev, 0xA8); //--set multiplex ratio(1 to 64)
	ssd1306_write(hdev, 0x3F); //
	ssd1306_write(hdev, 0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	ssd1306_write(hdev, 0xD3); //-set display offset
	ssd1306_write(hdev, 0x00); //-not offset
	ssd1306_write(hdev, 0xD5); //--set display clock divide ratio/oscillator frequency
	ssd1306_write(hdev, 0xF0); //--set divide ratio
	ssd1306_write(hdev, 0xD9); //--set pre-charge period
	ssd1306_write(hdev, 0x22); //
	ssd1306_write(hdev, 0xDA); //--set com pins hardware configuration
	ssd1306_write(hdev, 0x12);
	ssd1306_write(hdev, 0xDB); //--set vcomh
	ssd1306_write(hdev, 0x20); //0x20,0.77xVcc
	ssd1306_write(hdev, 0x8D); //--set DC-DC enable
	ssd1306_write(hdev, 0x14); //
	ssd1306_write(hdev, 0xAF); //--turn on SSD1306 panel
	
	// Clear screen
	ssd1306_fill_screen(hdev, SSD1306_BLACK);
	
	// Flush buffer to screen
	ssd1306_update_screen(hdev);
	
	return HAL_OK;
}

HAL_StatusTypeDef ssd1306_fill_screen(SSD1306_HandleTypeDef *hdev, uint8_t colour) 
{
	for(uint16_t i = 0; i < SSD1306_BUFFER_SIZE; i++)
	{
		hdev->buffer[i] = colour;
	}
}

HAL_StatusTypeDef ssd1306_update_screen(SSD1306_HandleTypeDef *hdev) 
{
    volatile HAL_StatusTypeDef status;

    uint8_t buffer[17];
    buffer[0] = 0x40;

    for(int i = 0; i < 64; i++) {
        // Send in 16 byte chunks
        for(int j = 0; j < 16; j++) {
            buffer[1 + j] = hdev->buffer[(i * 16) + j];
        }

        // Send
        status = HAL_I2C_Master_Transmit(hdev->hi2c, hdev->addr, buffer, 17, HAL_TIMEOUT);
    }
    
    if(status != HAL_OK) {
        Error_Handler(__FILE__,__LINE__);
    }

    return status;
}

HAL_StatusTypeDef ssd1306_draw_pixel(SSD1306_HandleTypeDef *hdev, uint8_t x, uint8_t y, uint8_t colour)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) 
		return;
	
	if(colour == SSD1306_WHITE) {
		hdev->buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		hdev->buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

HAL_StatusTypeDef ssd1306_put_string(SSD1306_HandleTypeDef *hdev, uint8_t x, uint8_t y, char* s) {
    int i = 0;
    while(s[i] != '\0') {
        ssd1306_put_char(hdev, x + i, y, s[i]);
        i++;
    }
}

HAL_StatusTypeDef ssd1306_put_char(SSD1306_HandleTypeDef *hdev, uint8_t x, uint8_t y, char c) {
    for(int _y = 0; _y <24; _y++) {
        for(int _x = 0; _x < 8; _x++) {
            ssd1306_draw_pixel(hdev,(x*16) + _x,(y*26) + _y, ((font[c][2 * _y] >> _x) & 0x01) ? SSD1306_WHITE : SSD1306_BLACK);
            ssd1306_draw_pixel(hdev,(x*16) + _x + 8,(y*26) + _y, ((font[c][2 * _y + 1] >> _x) & 0x01) ? SSD1306_WHITE : SSD1306_BLACK);
        }
    }

    return HAL_OK;
}
