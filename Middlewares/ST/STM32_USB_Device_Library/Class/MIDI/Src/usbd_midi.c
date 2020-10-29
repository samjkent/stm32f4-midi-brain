/**
  ******************************************************************************
  * @file    usbd_midi.c
  * @author  Sam Kent
  * @brief   This file provides the high layer firmware functions to manage the
  *          following functionalities of the USB MIDI Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as MIDI Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Error management
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi.h"
#include "usbd_ctlreq.h"


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


static uint8_t  USBD_MIDI_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx);

static uint8_t  USBD_MIDI_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx);

static uint8_t  USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev,
                                 uint8_t epnum);

static uint8_t  USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev,
                                 uint8_t epnum);

static uint8_t  *USBD_MIDI_GetCfgDesc (uint16_t *length);

/* USB Standard Device Descriptor */

/**
  * @}
  */

/** @defgroup USBD_MIDI_Private_Variables
  * @{
  */


/* MIDI interface class callbacks structure */
USBD_ClassTypeDef  USBD_MIDI =
{
  USBD_MIDI_Init,
  USBD_MIDI_DeInit,
  NULL,
  NULL,
  NULL,
  USBD_MIDI_DataIn,
  USBD_MIDI_DataOut,
  NULL,
  NULL,
  NULL,
  NULL,
  USBD_MIDI_GetCfgDesc,
  NULL,
  NULL,
};

/* USB MIDI device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_MIDI_CfgDesc[USB_MIDI_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_MIDI_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x02,   /* bNumInterfaces: 2 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0x80,   /* bmAttributes: self powered */
  0x50,   /* MaxPower */

  /*---------------------------------------------------------------------------*/

  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x00,   /* bNumEndpoints: Zero endpoints used */
  0x01,   /* bInterfaceClass: Audio Class */
  0x01,   /* bInterfaceSubClass: MIDI */
  0x00,   /* bInterfaceProtocol: */
  0x02,   /* iInterface: */

  /* Class Specific Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  0x24,   /* bDescriptorType: CS_Interface */
  0x01,   /* bDescriptorSubType */
  0x00,   /* bcdADC */
  0x01,   
  0x09,   /* wTotalLength */
  0x00,   
  0x01,   /* binCollecton */
  0x01,   /* baInterfaceNr(1) */

  /* Standard MS Interface Descriptor */
  0x09,   /* bLength: Endpoint Descriptor size */
  0x04,   /* bDescriptorType: INTERFACE */
  0x01,   /* bInterfaceNumber */
  0x00,   /* bAlternateSetting */ 
  0x02,   /* bNumEndpoints */
  0x01,   /* bInterfaceClass: AUDIO */
  0x03,   /* MIDI Streaming */
  0x00,   /* bInterfaceProtocol */
  0x02,   /* iInterface */ 

  /* Class-specific MS Interface Descriptor */
  0x07,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bcdADC */
  0x01,
  0x41,   /* wTotalLength */
  0x00, 

  /* MIDI IN Jack Descriptor */
  0x06,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: MIDI_IN_JACK */
  0x01,   /* bJackType: EMBEDDED */
  0x01,   /* bJackID */
  0x00,   /* iJack */
  
  /* MIDI IN Jack Descriptor */
  0x06,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: MIDI_IN_JACK */
  0x02,   /* bJackType: EXTERNAL */
  0x02,   /* bJackID */
  0x00,   /* iJack */
  
  /* MIDI OUT Jack Descriptor */
  0x09,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x03,   /* bDescriptorSubtype: MIDI_OUT_JACK */
  0x01,   /* bJackType: EMBEDDED */
  0x03,   /* bJackID */
  0x01,   /* bNrInputPins */
  0x02,   /* BaSourceID(1) */
  0x01,   /* BaSourcePin(1) */
  0x00,   /* iJack */

  /* MIDI OUT Jack Descriptor */
  0x09,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x03,   /* bDescriptorSubtype: MIDI_OUT_JACK */
  0x02,   /* bJackType: EXTERNAL */
  0x04,   /* bJackID */
  0x01,   /* bNrInputPins */
  0x01,   /* BaSourceID(1) */
  0x01,   /* BaSourcePin(1) */
  0x00,   /* iJack */

  /* MIDI IN Jack Descriptor */
  0x06,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: MIDI_IN_JACK */
  0x01,   /* bJackType: EMBEDDED */
  0x05,   /* bJackID */
  0x00,   /* iJack */
  
  /* MIDI IN Jack Descriptor */
  0x06,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: MIDI_IN_JACK */
  0x02,   /* bJackType: EXTERNAL */
  0x06,   /* bJackID */
  0x00,   /* iJack */
  
  /* MIDI OUT Jack Descriptor */
  0x09,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x03,   /* bDescriptorSubtype: MIDI_OUT_JACK */
  0x01,   /* bJackType: EMBEDDED */
  0x07,   /* bJackID */
  0x01,   /* bNrInputPins */
  0x06,   /* BaSourceID(1) */
  0x01,   /* BaSourcePin(1) */
  0x00,   /* iJack */

  /* MIDI OUT Jack Descriptor */
  0x09,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x03,   /* bDescriptorSubtype: MIDI_OUT_JACK */
  0x02,   /* bJackType: EXTERNAL */
  0x08,   /* bJackID */
  0x01,   /* bNrInputPins */
  0x05,   /* BaSourceID(1) */
  0x01,   /* BaSourcePin(1) */
  0x00,   /* iJack */

  /*Endpoint OUT Descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  MIDI_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(MIDI_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(MIDI_DATA_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  0x00,                              /* bRefresh */
  0x00,                              /* bSynchAddress */

  0x06, /* bLength */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType */
  0x01, /* bDescriptorSubtype */
  0x02, /* bNumEmbMIDIJack */
  0x01, /* BaAssocJackID(1) */
  0x05, /* BaAssocJackID(2) */
  
  /*Endpoint IN Descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  MIDI_IN_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(MIDI_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(MIDI_DATA_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  0x00,                              /* bRefresh */
  0x00,                              /* bSynchAddress */

  0x06, /* bLength */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType */
  0x01, /* bDescriptorSubtype */
  0x02, /* bNumEmbMIDIJack */
  0x03, /* BaAssocJackID(1) */
  0x07, /* BaAssocJackID(2) */

} ;

/**
  * @}
  */

/** @defgroup USBD_MIDI_Private_Functions
  * @{
  */

/**
  * @brief  USBD_MIDI_Init
  *         Initialize the MIDI interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MIDI_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t ret = 0U;
  USBD_MIDI_HandleTypeDef   *hmidi;

  /* Open EP IN */
  USBD_LL_OpenEP(pdev, MIDI_IN_EP, USBD_EP_TYPE_BULK,
                 MIDI_DATA_IN_PACKET_SIZE);

  pdev->ep_in[MIDI_IN_EP & 0xFU].is_used = 1U;

  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, MIDI_OUT_EP, USBD_EP_TYPE_BULK,
                 MIDI_DATA_OUT_PACKET_SIZE);

  pdev->ep_out[MIDI_OUT_EP & 0xFU].is_used = 1U;

  pdev->pClassData = USBD_malloc(sizeof (USBD_MIDI_HandleTypeDef));

  if(pdev->pClassData == NULL)
  {
    ret = 1U;
  }
  else
  {
    hmidi = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

    /* Init  physical Interface components */
    ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->Init();

    /* Init Xfer states */
    hmidi->TxState = 0U;
    hmidi->RxState = 0U;

    /* Prepare Out endpoint to receive next packet */
    USBD_LL_PrepareReceive(pdev, MIDI_OUT_EP, hmidi->RxBuffer,
                           MIDI_DATA_OUT_PACKET_SIZE);
  
  }
  
  return ret;
}

/**
  * @brief  USBD_MIDI_Init
  *         DeInitialize the MIDI layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MIDI_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t ret = 0U;

  /* Close EP IN */
  USBD_LL_CloseEP(pdev, MIDI_IN_EP);
  pdev->ep_in[MIDI_IN_EP & 0xFU].is_used = 0U;

  /* Close EP OUT */
  USBD_LL_CloseEP(pdev, MIDI_OUT_EP);
  pdev->ep_out[MIDI_OUT_EP & 0xFU].is_used = 0U;

  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return ret;
}

/**
  * @brief  USBD_MIDI_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_MIDI_HandleTypeDef *hmidi = (USBD_MIDI_HandleTypeDef*)pdev->pClassData;
  PCD_HandleTypeDef *hpcd = pdev->pData;

  if(pdev->pClassData != NULL)
  {
    if((pdev->ep_in[epnum].total_length > 0U) && ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U))
    {
      /* Update the packet total length */
      pdev->ep_in[epnum].total_length = 0U;

      /* Send ZLP */
      USBD_LL_Transmit (pdev, epnum, NULL, 0U);
    }
    else
    {
      hmidi->TxState = 0U;
    }
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_MIDI_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  
  USBD_MIDI_HandleTypeDef   *hmidi = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  /* Get the received data length */
  hmidi->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);

  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */
  if(pdev->pClassData != NULL)
  {

    ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->Receive(hmidi->RxBuffer, &hmidi->RxLength);
    
    /* Prepare Out endpoint to receive next packet */
    USBD_LL_PrepareReceive(pdev,
                           MIDI_OUT_EP,
                           hmidi->RxBuffer,
                           MIDI_DATA_OUT_PACKET_SIZE);

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_MIDI_GetCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_MIDI_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_CfgDesc);
  return USBD_MIDI_CfgDesc;
}

/**
* @brief  USBD_MIDI_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t  USBD_MIDI_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                      USBD_MIDI_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;

  if(fops != NULL)
  {
    pdev->pUserData= fops;
    ret = USBD_OK;
  } else {
  }

  return ret;
}

/**
  * @brief  USBD_MIDI_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_MIDI_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length)
{
  USBD_MIDI_HandleTypeDef   *hmidi = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  hmidi->TxBuffer = pbuff;
  hmidi->TxLength = length;

  return USBD_OK;
}


/**
  * @brief  USBD_MIDI_TransmitPacket
  *         Transmit packet on IN endpoint
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_MIDI_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_MIDI_HandleTypeDef   *hmidi = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  if(pdev->pClassData != NULL)
  {
    if(hmidi->TxState == 0U)
    {
      /* Tx Transfer in progress */
      hmidi->TxState = 1U;

      /* Update the packet total length */
      pdev->ep_in[MIDI_IN_EP & 0xFU].total_length = hmidi->TxLength;

      /* Transmit next packet */
      USBD_LL_Transmit(pdev, MIDI_IN_EP, hmidi->TxBuffer,
                       (uint16_t)hmidi->TxLength);

      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
