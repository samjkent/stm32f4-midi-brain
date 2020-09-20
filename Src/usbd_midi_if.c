/**
  ******************************************************************************
  * @file           : usbd_midi_if.c
  * @brief          :
  ******************************************************************************

    (CC at)2016 by D.F.Mac. @TripArts Music

*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi_if.h"

#define NEXTBYTE(idx, mask) (mask & (idx + 1))


struct tUsbMidiCable usbmidicable1;
struct tUsbMidiCable usbmidicable2;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;

// basic midi rx/tx functions
static uint16_t MIDI_DataRx(uint8_t *msg, uint16_t length);
static uint16_t MIDI_DataTx(uint8_t *msg, uint16_t length);

USBD_MIDI_ItfTypeDef USBD_Interface_fops_FS =
{
  MIDI_DataRx,
  MIDI_DataTx
};


//static uint8_t buf[4];

void USBD_AddNoteOn(uint8_t cable, uint8_t ch, uint8_t note, uint8_t vel)
{
  //uint8_t cable = 0;
  uint8_t txbuf[4];
  
  cable <<= 4;
  txbuf[0] = cable + 0x9;
  txbuf[1] = 0x90 | ch;
  txbuf[2] = 0x7F & note;
  txbuf[3] = 0x7F & vel;
  MIDI_DataTx(txbuf, 4);
}

void USBD_AddCC(uint8_t cable, uint8_t ch, uint8_t mode, uint8_t val)
{
  //uint8_t cable = 0;
  uint8_t txbuf[4];
  
  cable <<= 4;
  txbuf[0] = cable + 0xB;
  txbuf[1] = 0xB0 | ch;
  txbuf[2] = 0x7F & mode;
  txbuf[3] = 0x7F & val;
  MIDI_DataTx(txbuf, 4);
}

void USBD_AddNoteOff(uint8_t cable, uint8_t ch, uint8_t note)
{
  //uint8_t cable = 0;
  uint8_t txbuf[4];
  
  cable <<= 4;
  txbuf[0] = cable + 0x8;
  txbuf[1] = 0x80 | ch;
  txbuf[2] = 0x7F & note;
  txbuf[3] = 0;
  MIDI_DataTx(txbuf, 4);
}


void USBD_AddSysExMessage(uint8_t cable, uint8_t *msg, uint8_t length)
{
  uint8_t txbuf[4];
  int8_t bytes_remain = length;
  uint8_t i = 0;
  
  cable <<= 4;
  
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
  
  while ( bytes_remain > 0 )
  {
    if (bytes_remain > 3)
    {
      txbuf[0] = cable + 0x4; //SysEx start or continue with 3 bytes
      txbuf[1] = msg[i++];
      txbuf[2] = msg[i++];
      txbuf[3] = msg[i++];   
      bytes_remain = length - i;
    }
    else
    {
      switch ( bytes_remain ) {
      case 3:
        txbuf[0] = cable + 0x7; //SysEx end or continue with 3 bytes
        txbuf[1] = msg[i++];
        txbuf[2] = msg[i++];
        txbuf[3] = msg[i];
        break;
      case 2:
        txbuf[0] = cable + 0x6; //SysEx end or continue with 2 bytes
        txbuf[1] = msg[i++];
        txbuf[2] = msg[i];
        txbuf[3] = 0;
        break;
      case 1:
        txbuf[0] = cable + 0x5; //SysEx end or continue with 1 byte
        txbuf[1] = msg[i];
        txbuf[2] = 0;
        txbuf[3] = 0;
        break;
      }
      bytes_remain = 0;
    }
    
    MIDI_DataTx(txbuf, 4);
  }
}

//Start transfer
void USBD_SendMidiMessages(void)
{
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
  {
    if (!USB_Tx_State)
      USBD_MIDI_SendPacket();
    else
      USB_Tx_State = USB_TX_CONTINUE;
  }
}

//fill midi tx buffer
static uint16_t MIDI_DataTx(uint8_t *msg, uint16_t length)
{
  uint16_t i = 0;
  while (i < length) {
    APP_Rx_Buffer[APP_Rx_ptr_in] = *(msg + i);
    APP_Rx_ptr_in++;
    i++;
    if (APP_Rx_ptr_in == APP_RX_DATA_SIZE) {
      APP_Rx_ptr_in = 0;
    }
  }
  return USBD_OK;
}


static uint16_t MIDI_DataRx(uint8_t* msg, uint16_t length) {
    uint8_t chan = msg[1] & 0xf;
	uint8_t b1 =  msg[2];
	uint8_t b2 =  msg[3];
	uint16_t b = ((b2 & 0x7f) << 7) | (b1 & 0x7f);

	switch (msg[1] & 0xF0) {
	case 0xB0:
        if(b1 == 0x00) { // Bank Select
		    controller_set_bank(b2);
        } else {
		    controller_set_cc(chan, b1, b2);
        }
		break;
    case 0xF0:
        if(msg[1] == 0xFF) {
            NVIC_SystemReset();
        }
        break;
	default:
		break;
	}

	return 0;
}

