// Copyright (c) 2018 Rudá Moura <ruda.moura@gmail.com>
// License: The MIT License (MIT)

#include "MCP23017.h"

// Registers
#define REGISTER_IODIRA		0x00
#define REGISTER_IODIRB		0x01
#define REGISTER_IPOLA		0x02
#define REGISTER_IPOLB		0x03
#define REGISTER_GPINTENA	0x04
#define REGISTER_GPINTENB	0x05
#define REGISTER_DEFVALA	0x06
#define REGISTER_DEFVALB	0x07
#define REGISTER_INTCONA	0x08
#define REGISTER_INTCONB	0x09
#define REGISTER_IOCONA 	0x0A
#define REGISTER_IOCONB 	0x0B
#define REGISTER_GPPUA		0x0C
#define REGISTER_GPPUB		0x0D
#define REGISTER_INTFA		0x0E
#define REGISTER_INTFB		0x0F
#define REGISTER_INTCAPA	0x10
#define REGISTER_INTCAPB	0x11
#define REGISTER_GPIOA		0x12
#define REGISTER_GPIOB		0x13
#define REGISTER_OLATA		0x14
#define REGISTER_OLATB		0x15

#define I2C_TIMEOUT		10

void mcp23017_init(MCP23017_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr)
{
	hdev->hi2c = hi2c;
	hdev->addr = addr << 1;

	HAL_StatusTypeDef status;
    status = HAL_I2C_IsDeviceReady(hdev->hi2c, hdev->addr, 10, I2C_TIMEOUT);
    if(status != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }

}

HAL_StatusTypeDef mcp23017_read(MCP23017_HandleTypeDef *hdev, uint16_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, reg, 1, data, 1, I2C_TIMEOUT);
}

HAL_StatusTypeDef mcp23017_write(MCP23017_HandleTypeDef *hdev, uint16_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, reg, 1, data, 1, I2C_TIMEOUT);
}

HAL_StatusTypeDef mcp23017_iodir(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t iodir)
{
	uint8_t data[1] = {iodir};
	return mcp23017_write(hdev, REGISTER_IODIRA|port, data);
}

HAL_StatusTypeDef mcp23017_ipol(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t ipol)
{
	uint8_t data[1] = {ipol};
	return mcp23017_write(hdev, REGISTER_IPOLA|port, data);
}

HAL_StatusTypeDef mcp23017_ggpu(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t pu)
{
	uint8_t data[1] = {pu};
	return mcp23017_write(hdev, REGISTER_GPPUA|port, data);
}

HAL_StatusTypeDef mcp23017_read_gpio(MCP23017_HandleTypeDef *hdev, uint8_t port)
{
	uint8_t data[1];
	HAL_StatusTypeDef status;
	status = mcp23017_read(hdev, REGISTER_GPIOA|port, data);
	if (status == HAL_OK)
		hdev->gpio[port] = data[0];
	return status;
}

HAL_StatusTypeDef mcp23017_read_intf(MCP23017_HandleTypeDef *hdev, uint8_t port)
{
	uint8_t data[1];
	HAL_StatusTypeDef status;
	status = mcp23017_read(hdev, REGISTER_INTFA|port, data);
	if (status == HAL_OK)
		hdev->intf[port] = data[0];
	return status;
}

HAL_StatusTypeDef mcp23017_read_intcap(MCP23017_HandleTypeDef *hdev, uint8_t port)
{
	uint8_t data[1];
	HAL_StatusTypeDef status;
	status = mcp23017_read(hdev, REGISTER_INTCAPA|port, data);
	if (status == HAL_OK)
		hdev->intcap[port] = data[0];
	return status;
}

HAL_StatusTypeDef mcp23017_write_gpio(MCP23017_HandleTypeDef *hdev, uint8_t port)
{
	uint8_t data[1] = {hdev->gpio[port]};
	return mcp23017_write(hdev, REGISTER_GPIOA|port, data);
}

HAL_StatusTypeDef mcp23017_iocon(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t iocon)
{
    uint8_t data[1] = {iocon};
	return mcp23017_write(hdev, REGISTER_IOCONA|port, data);
}

HAL_StatusTypeDef mcp23017_gpinten(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t gpinten )
{
    uint8_t data[1] = {gpinten};
	return mcp23017_write(hdev, REGISTER_GPINTENA|port, data);
}

HAL_StatusTypeDef mcp23017_defval(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t defval)
{
    uint8_t data[1] = {defval};
	return mcp23017_write(hdev, REGISTER_DEFVALA|port, data);
}

HAL_StatusTypeDef mcp23017_intcon(MCP23017_HandleTypeDef *hdev, uint8_t port, uint8_t intcon)
{
    uint8_t data[1] = {intcon};
	return mcp23017_write(hdev, REGISTER_INTCONA|port, data);
}
