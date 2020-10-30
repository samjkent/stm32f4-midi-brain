/**
  ******************************************************************************
  * @file    usbd_midi.c
  * @author  Sam Kent
  * @brief   This file provides the MIDI core functions.
  *
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

static uint8_t  USBD_MIDI_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_MIDI_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_MIDI_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_MIDI_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_MIDI_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_MIDI_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_MIDI_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_MIDI_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

/**
  * @}
  */

/** @defgroup USBD_MIDI_Private_Variables
  * @{
  */

USBD_ClassTypeDef  USBD_MIDI =
{
  USBD_MIDI_Init,
  USBD_MIDI_DeInit,
  USBD_MIDI_Setup,
  USBD_MIDI_EP0_TxReady,
  USBD_MIDI_EP0_RxReady,
  USBD_MIDI_DataIn,
  USBD_MIDI_DataOut,
  USBD_MIDI_SOF,
  USBD_MIDI_IsoINIncomplete,
  USBD_MIDI_IsoOutIncomplete,
  USBD_MIDI_GetCfgDesc,
  USBD_MIDI_GetCfgDesc,
  USBD_MIDI_GetCfgDesc,
  USBD_MIDI_GetDeviceQualifierDesc,
};

/* USB MIDI device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_CfgDesc[USB_MIDI_CONFIG_DESC_SIZ] __ALIGN_END =
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
  LOBYTE(USB_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(USB_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  0x00,                              /* bRefresh */
  0x00,                              /* bSynchAddress */

  0x06, /* bLength */
  0x25, /* bDescriptorType */
  0x01, /* bDescriptorSubtype */
  0x02, /* bNumEmbMIDIJack */
  0x01, /* BaAssocJackID(1) */
  0x05, /* BaAssocJackID(2) */
  
  /*Endpoint IN Descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  MIDI_IN_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(USB_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(USB_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  0x00,                              /* bRefresh */
  0x00,                              /* bSynchAddress */

  0x06, /* bLength */
  0x25, /* bDescriptorType */
  0x01, /* bDescriptorSubtype */
  0x02, /* bNumEmbMIDIJack */
  0x03, /* BaAssocJackID(1) */
  0x07, /* BaAssocJackID(2) */
} ;

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

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
  USBD_MIDI_HandleTypeDef   *hmidi;

  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, MIDI_OUT_EP, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE);
  pdev->ep_out[MIDI_OUT_EP & 0xFU].is_used = 1U;
  
  /* Open EP IN */
  USBD_LL_OpenEP(pdev, MIDI_IN_EP, USBD_EP_TYPE_BULK, USB_FS_MAX_PACKET_SIZE);
  pdev->ep_in[MIDI_IN_EP & 0xFU].is_used = 1U;

  /* Allocate MIDI structure */
  pdev->pClassData = USBD_malloc(sizeof (USBD_MIDI_HandleTypeDef));

  if(pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }
  else
  {
    hmidi = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

    hmidi->tx_busy = 0;

    /* Prepare Out endpoint to receive 1st packet */
    USBD_LL_PrepareReceive(pdev, MIDI_OUT_EP, hmidi->rx_buffer,
                           USB_FS_MAX_PACKET_SIZE);
  }
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_Init
  *         DeInitialize the MIDI layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MIDI_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{

  /* Open EP OUT */
  USBD_LL_CloseEP(pdev, MIDI_OUT_EP);
  pdev->ep_out[MIDI_OUT_EP & 0xFU].is_used = 0U;
  
  /* Open EP IN */
  USBD_LL_CloseEP(pdev, MIDI_IN_EP);
  pdev->ep_in[MIDI_IN_EP & 0xFU].is_used = 0U;

  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_Setup
  *         Handle the MIDI specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_MIDI_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  return USBD_OK;
}


/**
  * @brief  USBD_MIDI_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor rx_buffer
  */
static uint8_t  *USBD_MIDI_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_CfgDesc);
  return USBD_MIDI_CfgDesc;
}

/**
  * @brief  USBD_MIDI_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev,
                              uint8_t epnum)
{

  USBD_MIDI_HandleTypeDef   *hmidi;
  hmidi = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;
  hmidi->tx_busy = 0;
  
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_MIDI_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  return USBD_OK;
}
/**
  * @brief  USBD_MIDI_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_MIDI_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}
/**
  * @brief  USBD_MIDI_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_MIDI_SOF (USBD_HandleTypeDef *pdev)
{
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
void  USBD_MIDI_Sync (USBD_HandleTypeDef *pdev)
{
  return;
}

/**
  * @brief  USBD_MIDI_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  return USBD_OK;
}
/**
  * @brief  USBD_MIDI_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_MIDI_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev,
                              uint8_t epnum)
{
  USBD_MIDI_HandleTypeDef   *hmidi;
  hmidi = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  if (epnum == MIDI_OUT_EP)
  {
    /* Get data length and actual data */
    size_t length = USBD_LL_GetRxDataSize(pdev, epnum);
    uint8_t  *rx_buffer = hmidi->rx_buffer;

    /* Pass data to Receive() */
    for (uint32_t i=0; i < length; i+=4) {
        ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->Receive(rx_buffer + i, length);
    }

    /* Prepare Out endpoint to receive next midi packet */
    USBD_LL_PrepareReceive(pdev, MIDI_OUT_EP, hmidi->rx_buffer,
                           USB_FS_MAX_PACKET_SIZE);
  }

  return USBD_OK;
}

/**
  * @brief  MIDI_Req_GetCurrent
  *         Handles the GET_CUR MIDI control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void MIDI_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  return;
}

/**
  * @brief  MIDI_Req_SetCurrent
  *         Handles the SET_CUR MIDI control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void MIDI_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  return;
}


/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor rx_buffer
*/
static uint8_t  *USBD_MIDI_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_DeviceQualifierDesc);
  return USBD_MIDI_DeviceQualifierDesc;
}

/**
* @brief  USBD_MIDI_RegisterInterface
* @param  fops: MIDI interface callback
* @retval status
*/
uint8_t  USBD_MIDI_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                        USBD_MIDI_ItfTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pUserData= fops;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_SetTxBuffer
  * @param  pdev: instance
  * @param  buffer
  * @retval status
  */
uint8_t  USBD_MIDI_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *buff,
                                uint16_t length)
{
  USBD_MIDI_HandleTypeDef   *hmidi = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  // hmidi->tx_buffer = buff;
  memcpy(hmidi->tx_buffer, buff, length);
  hmidi->tx_length = length;

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

  if(hmidi->tx_busy == 0) {

    hmidi->tx_busy = 1;
    pdev->ep_in[MIDI_IN_EP & 0xFU].total_length = hmidi->tx_length;

    /* Transmit */
    USBD_LL_Transmit(pdev, MIDI_IN_EP, hmidi->tx_buffer,
                     (uint8_t)hmidi->tx_length);
    
    return USBD_OK;
  } else {
    return USBD_BUSY;
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
