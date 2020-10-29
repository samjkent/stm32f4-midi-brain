/**
  ******************************************************************************
  * @file    usbd_midi_if.c
  * @author  Sam Kent
  * @brief   USB MIDI Access Layer
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi_if.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_MIDI
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_MIDI_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_MIDI_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_MIDI_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_MIDI_Private_FunctionPrototypes
  * @{
  */

static int8_t MIDI_Init     (void);
static int8_t MIDI_DeInit   (void);
static uint16_t MIDI_Receive  (uint8_t* pbuf, uint16_t Len);
static uint16_t MIDI_Send     (uint8_t* pbuf, uint16_t Len);

USBD_MIDI_ItfTypeDef USBD_MIDI_fops =
{
  MIDI_Init,
  MIDI_DeInit,
  MIDI_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  MIDI_Init
  *         Initializes the MIDI media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t MIDI_Init(void)
{
  /*
     Add your initialization code here
  */
  return (0);
}

/**
  * @brief  MIDI_DeInit
  *         DeInitializes the MIDI media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t MIDI_DeInit(void)
{
  /*
     Add your deinitialization code here
  */
  return (0);
}

/**
  * @brief  MIDI_Send
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t MIDI_Send (uint8_t* Buf, uint16_t Len)
{
  uint8_t ret = USBD_OK;

  USBD_MIDI_SetTxBuffer(&hUsbDeviceFS,Buf,Len);
  ret = USBD_MIDI_TransmitPacket(&hUsbDeviceFS);

  return (ret);
}

/**
  * @brief  MIDI_Receive
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static uint16_t MIDI_Receive (uint8_t* Buf, uint16_t Len)
{

  uint8_t chan = Buf[1] & 0xf;
  uint8_t msgtype = Buf[1] & 0xf0;
  uint8_t b1 =  Buf[2];
  uint8_t b2 =  Buf[3];
  uint16_t b = ((b2 & 0x7f) << 7) | (b1 & 0x7f);
  
  switch (msgtype) {
  case 0xF0:
    if(chan == 0xFF) {
        NVIC_SystemReset(); // Reset into DFU mode
    }
  	break;
  default:
  	break;
  }
  
  return (0);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

void MIDI_note_on(uint8_t note, uint8_t velocity) {
    
    uint8_t b[3];
    b[0] = 0x90;
    b[1] = note;
    b[2] = velocity;

    MIDI_Send(b, 3);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

