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

/** @defgroup usbd_midi
  * @brief This file is the Header file for usbd_midi.c
  * @{
  */


/** @defgroup usbd_midi_Exported_Defines
  * @{
  */
#define MIDI_IN_EP                                   0x81U  /* EP1 for data IN */
#define MIDI_OUT_EP                                  0x01U  /* EP1 for data OUT */

/* MIDI Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define MIDI_DATA_MAX_PACKET_SIZE                    64U  /* Endpoint IN & OUT Packet size */
#define MIDI_CMD_PACKET_SIZE                         8U  /* Control Endpoint Packet size */

#define USB_MIDI_CONFIG_DESC_SIZ                     0x85U
#define MIDI_DATA_IN_PACKET_SIZE                     MIDI_DATA_MAX_PACKET_SIZE
#define MIDI_DATA_OUT_PACKET_SIZE                    MIDI_DATA_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  MIDI definitions                                                    */
/*---------------------------------------------------------------------*/

/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */

typedef struct _USBD_MIDI_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  uint16_t (*Receive)        (uint8_t *msg, uint16_t length);
}USBD_MIDI_ItfTypeDef;


typedef struct
{
  uint32_t data[MIDI_DATA_MAX_PACKET_SIZE / 4U];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
}
USBD_MIDI_HandleTypeDef;



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

uint8_t  USBD_MIDI_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_MIDI_TransmitPacket     (USBD_HandleTypeDef *pdev);
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

/***************************************************************END OF FILE****/
