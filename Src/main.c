/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_midi_if.h"
#include "MCP23017.h"
#include "HT16K33.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const uint8_t encoderMap[256] = {
  0xFF,0x08,0x18,0x17,0x28,0xFF,0x27,0x24,0x38,0x39,0xFF,0xFF,0x37,0xFF,0x34,0x0D,
  0x78,0x07,0xFF,0x14,0x29,0xFF,0xFF,0x7D,0xFF,0xFF,0xFF,0x13,0x36,0xFF,0x35,0x12,
  0x68,0xFF,0x19,0xFF,0xFF,0xFF,0x26,0x25,0x69,0x3A,0x6A,0xFF,0xFF,0xFF,0xFF,0x0E,
  0x77,0x04,0xFF,0x6D,0xFF,0x03,0xFF,0x02,0x76,0xFF,0x6B,0x6C,0xFF,0xFF,0xFF,0x01,
  0x48,0xFF,0x49,0xFF,0xFF,0xFF,0xFF,0x23,0x47,0x46,0xFF,0xFF,0x44,0x45,0x1D,0x22,
  0x79,0x06,0x4A,0xFF,0x7A,0x7B,0xFF,0x7C,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x1E,0x11,
  0xFF,0xFF,0x1A,0xFF,0xFF,0xFF,0x1B,0xFF,0xFF,0x3B,0xFF,0xFF,0x43,0xFF,0x1C,0x0F,
  0xFF,0x05,0xFF,0x6E,0xFF,0xFF,0xFF,0x6F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x10,
  0x58,0x09,0xFF,0x16,0x59,0x5A,0xFF,0xFF,0xFF,0x0A,0xFF,0x0B,0xFF,0xFF,0x33,0x0C,
  0xFF,0xFF,0xFF,0x15,0x2A,0xFF,0xFF,0x7E,0xFF,0xFF,0xFF,0xFF,0x2B,0xFF,0xFF,0x7F,
  0x67,0xFF,0xFF,0xFF,0x66,0x5B,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
  0x74,0x5D,0x73,0x72,0xFF,0x5C,0xFF,0x71,0x75,0x5E,0xFF,0x5F,0xFF,0xFF,0xFF,0x00,
  0x57,0xFF,0x56,0xFF,0xFF,0xFF,0xFF,0xFF,0x54,0xFF,0x55,0xFF,0x2D,0x2E,0x32,0x21,
  0xFF,0xFF,0x4B,0xFF,0xFF,0xFF,0xFF,0xFF,0x53,0xFF,0xFF,0xFF,0x2C,0xFF,0x1F,0x20,
  0x64,0x63,0xFF,0xFF,0x65,0xFF,0xFF,0xFF,0x3D,0x3C,0x3E,0xFF,0x42,0x2F,0x31,0x30,
  0x4D,0x62,0x4C,0x61,0x4E,0x4F,0xFF,0x70,0x52,0x51,0x3F,0x60,0x41,0x50,0x40,0xFF
 };
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEMO 0
#define NUM_ENCS 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
extern struct tUsbMidiCable usbmidicable1;
MCP23017_HandleTypeDef enc1;
MCP23017_HandleTypeDef enc2;
MCP23017_HandleTypeDef enc3;
MCP23017_HandleTypeDef enc4;

MCP23017_HandleTypeDef enc1_2;
MCP23017_HandleTypeDef enc3_4;

HT16K33_HandleTypeDef led1;
HT16K33_HandleTypeDef led2;
HT16K33_HandleTypeDef led3;
HT16K33_HandleTypeDef led4;
HT16K33_HandleTypeDef led5;

uint8_t enc_values[8];

uint8_t banks[1][8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void demo(int n) {
    uint16_t values[5] = {0, 32, 64, 96, 0};
    int n_runs = 0;

    while(1) {
        // Clear LEDs
        ht16k33_clear(&led1);
        ht16k33_clear(&led2);
        ht16k33_clear(&led3);
        ht16k33_clear(&led4);

        if(!values[4]) 
            ht16k33_clear(&led5);

        // Set real value
        ht16k33_set_led(&led1, values[0]);
        ht16k33_set_led(&led2, values[1]);
        ht16k33_set_led(&led3, values[2]);
        ht16k33_set_led(&led4, values[3]);
        ht16k33_set_led(&led5, values[4]);
    
        ht16k33_write_display(&led1);
        ht16k33_write_display(&led2);
        ht16k33_write_display(&led3);
        ht16k33_write_display(&led4);
        ht16k33_write_display(&led5);

        for(int i=0; i < 5; i++) {
            values[i] = (values[i] + 1) % 128;
        }
        
        HAL_Delay(5);

        if(n_runs > n && n != 0) return; 
        n_runs++;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  for (uint8_t i=1; i<128; i++)
 	{
 	  int result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
 	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
 	  {
          // No I2C
 	  }
 	  if (result == HAL_OK)
 	  {
 		  printf("Device detected. Address: %x", i);
 	  }
  }

  for (uint8_t i=1; i<128; i++)
 	{
 	  int result = HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(i<<1), 2, 2);
 	  if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
 	  {
          // No I2C
 	  }
 	  if (result == HAL_OK)
 	  {
 		  printf("Device detected. Address: %x", i);
 	  }
  }

  // Enc 1
  mcp23017_init(&enc1, &hi2c3, MCP23017_ADDRESS_20);
  mcp23017_iocon(&enc1, MCP23017_PORTA, MCP23017_MIRROR);
  mcp23017_iodir(&enc1, MCP23017_PORTA, MCP23017_IODIR_ALL_INPUT);
  mcp23017_iodir(&enc1, MCP23017_PORTB, MCP23017_IODIR_ALL_INPUT);
  mcp23017_ggpu(&enc1, MCP23017_PORTA, MCP23017_GPPU_ALL_ENABLED);
  mcp23017_ggpu(&enc1, MCP23017_PORTB, MCP23017_GPPU_ALL_ENABLED);
  ht16k33_init(&led1, &hi2c3, HT16K33_BASE_ADDRESS);

  // Enc 2
  mcp23017_init(&enc2, &hi2c3, MCP23017_ADDRESS_20 + 4);
  mcp23017_iocon(&enc2, MCP23017_PORTA, MCP23017_MIRROR);
  mcp23017_iodir(&enc2, MCP23017_PORTA, MCP23017_IODIR_ALL_INPUT);
  mcp23017_iodir(&enc2, MCP23017_PORTB, MCP23017_IODIR_ALL_INPUT);
  mcp23017_ggpu(&enc2, MCP23017_PORTA, MCP23017_GPPU_ALL_ENABLED);
  mcp23017_ggpu(&enc2, MCP23017_PORTB, MCP23017_GPPU_ALL_ENABLED);
  ht16k33_init(&led2, &hi2c3, HT16K33_BASE_ADDRESS + 2);

  // Enc 3
  mcp23017_init(&enc3, &hi2c3, MCP23017_ADDRESS_20 + 2);
  mcp23017_iocon(&enc3, MCP23017_PORTA, MCP23017_MIRROR);
  mcp23017_iodir(&enc3, MCP23017_PORTA, MCP23017_IODIR_ALL_INPUT);
  mcp23017_iodir(&enc3, MCP23017_PORTB, MCP23017_IODIR_ALL_INPUT);
  mcp23017_ggpu(&enc3, MCP23017_PORTA, MCP23017_GPPU_ALL_ENABLED);
  mcp23017_ggpu(&enc3, MCP23017_PORTB, MCP23017_GPPU_ALL_ENABLED);
  ht16k33_init(&led3, &hi2c3, HT16K33_BASE_ADDRESS + 4);
  
  // Enc 4
  mcp23017_init(&enc4, &hi2c3, MCP23017_ADDRESS_20 + 6);
  mcp23017_iocon(&enc4, MCP23017_PORTA, MCP23017_MIRROR);
  mcp23017_iodir(&enc4, MCP23017_PORTA, MCP23017_IODIR_ALL_INPUT);
  mcp23017_iodir(&enc4, MCP23017_PORTB, MCP23017_IODIR_ALL_INPUT);
  mcp23017_ggpu(&enc4, MCP23017_PORTA, MCP23017_GPPU_ALL_ENABLED);
  mcp23017_ggpu(&enc4, MCP23017_PORTB, MCP23017_GPPU_ALL_ENABLED);
  ht16k33_init(&led4, &hi2c3, HT16K33_BASE_ADDRESS + 6);

  // Small Enc LED 
  ht16k33_init(&led5, &hi2c1, HT16K33_BASE_ADDRESS);

  // Enc 1 & 2
  mcp23017_init(&enc1_2, &hi2c1, MCP23017_ADDRESS_20 + 6);
  mcp23017_iocon(&enc1_2, MCP23017_PORTA, MCP23017_MIRROR);
  mcp23017_iodir(&enc1_2, MCP23017_PORTA, MCP23017_IODIR_ALL_INPUT);
  mcp23017_iodir(&enc1_2, MCP23017_PORTB, MCP23017_IODIR_ALL_INPUT);
  mcp23017_ggpu(&enc1_2, MCP23017_PORTA, MCP23017_GPPU_ALL_ENABLED);
  mcp23017_ggpu(&enc1_2, MCP23017_PORTB, MCP23017_GPPU_ALL_ENABLED);

  // Enc 3 & 4
  mcp23017_init(&enc3_4, &hi2c1, MCP23017_ADDRESS_20 + 7);
  mcp23017_iocon(&enc3_4, MCP23017_PORTA, MCP23017_MIRROR);
  mcp23017_iodir(&enc3_4, MCP23017_PORTA, MCP23017_IODIR_ALL_INPUT);
  mcp23017_iodir(&enc3_4, MCP23017_PORTB, MCP23017_IODIR_ALL_INPUT);
  mcp23017_ggpu(&enc3_4, MCP23017_PORTA, MCP23017_GPPU_ALL_ENABLED);
  mcp23017_ggpu(&enc3_4, MCP23017_PORTB, MCP23017_GPPU_ALL_ENABLED);

  if(DEMO) {
    demo(0);
  } else {
    demo(128);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    // Clear LEDs
    ht16k33_clear(&led1);
    ht16k33_clear(&led2);
    ht16k33_clear(&led3);
    ht16k33_clear(&led4);
    ht16k33_clear(&led5);

    // Read Encs
    mcp23017_read_gpio(&enc1, 0);
    mcp23017_read_gpio(&enc1, 1);
    mcp23017_read_gpio(&enc2, 0);
    mcp23017_read_gpio(&enc2, 1);
    mcp23017_read_gpio(&enc3, 0);
    mcp23017_read_gpio(&enc3, 1);
    mcp23017_read_gpio(&enc4, 0);
    mcp23017_read_gpio(&enc4, 1);
   
    mcp23017_read_gpio(&enc1_2, 0);
    mcp23017_read_gpio(&enc1_2, 1);
    mcp23017_read_gpio(&enc3_4, 0);
    mcp23017_read_gpio(&enc3_4, 1);

    // Get real value
    enc_values[0] = encoderMap[enc1.gpio[1]];
    enc_values[1] = encoderMap[enc2.gpio[1]];
    enc_values[2] = encoderMap[enc3.gpio[1]];
    enc_values[3] = encoderMap[enc4.gpio[1]];
   
    enc_values[4] = encoderMap[enc1_2.gpio[1]] / 4;
    enc_values[5] = encoderMap[enc1_2.gpio[0]] / 4;
    enc_values[6] = encoderMap[enc3_4.gpio[1]] / 4;
    enc_values[7] = encoderMap[enc3_4.gpio[0]] / 4;

    for(int i = 0; i < NUM_ENCS; i++) {
        if(banks[0][i] != enc_values[i]) {
            banks[0][i] = enc_values[i];
            USBD_AddCC(0, 0, i, banks[0][i]);
        }
    }

    // Set real value
    ht16k33_set_led(&led1, banks[0][0]);
    ht16k33_set_led(&led2, banks[0][1]);
    ht16k33_set_led(&led3, banks[0][2]);
    ht16k33_set_led(&led4, banks[0][3]);

    ht16k33_set_led(&led5,  0 + banks[0][4]);
    ht16k33_set_led(&led5, 32 + banks[0][5]);
    ht16k33_set_led(&led5, 64 + banks[0][6]);
    ht16k33_set_led(&led5, 96 + banks[0][7]);
    
    ht16k33_write_display(&led1);
    ht16k33_write_display(&led2);
    ht16k33_write_display(&led3);
    ht16k33_write_display(&led4);
    ht16k33_write_display(&led5);

    USBD_SendMidiMessages();

    HAL_Delay(5);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
