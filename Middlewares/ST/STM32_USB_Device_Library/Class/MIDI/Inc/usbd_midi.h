/**
  ******************************************************************************
  * @file    usbd_midi.h
  * @author  Sam Kent
  * @brief   header file for the usbd_midi.c file.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_MIDI_H
#define __USB_MIDI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_MIDI
  * @brief This file is the Header file for usbd_midi.c
  * @{
  */


/** @defgroup USBD_MIDI_Exported_Defines
  * @{
  */
#define MIDI_IN_EP                                   0x81U
#define MIDI_OUT_EP                                  0x01U
#define USB_MIDI_CONFIG_DESC_SIZ                     0x85U

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

typedef struct
{
  uint8_t                    rx_buffer[USB_FS_MAX_PACKET_SIZE];
  uint8_t                    tx_buffer[USB_FS_MAX_PACKET_SIZE];
  uint8_t                    tx_busy;
  uint8_t                    tx_length;
}
USBD_MIDI_HandleTypeDef;


typedef struct
{
    int8_t  (*Init)         (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
    int8_t  (*DeInit)       (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
    int8_t  (*Receive)      (uint8_t* pbuf, uint32_t size);
    int8_t  (*Send)      (uint8_t* pbuf, uint32_t size);
}USBD_MIDI_ItfTypeDef;
/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_MIDI;
#define USBD_MIDI_CLASS    &USBD_MIDI
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_MIDI_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                        USBD_MIDI_ItfTypeDef *fops);

uint8_t  USBD_MIDI_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *buff,
                                uint16_t length);
uint8_t  USBD_MIDI_TransmitPacket(USBD_HandleTypeDef *pdev);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_MIDI_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
