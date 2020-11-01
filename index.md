## STM32 MIDI Brain

Example schematics and firmware for a USB MIDI device based on an STM32F4 micrcontroller using the ST HAL

![Example Application](https://raw.githubusercontent.com/samjkent/stm32f4-midi-brain/master/demo.gif)

### Includes drivers for

- HT16K33
- SS1306
- MCP23017
- USB-MIDI Class

### USB-MIDI

Example USB-MIDI Class is based on Appendix B. of [Universal Serial BusDevice Class Definition for MIDI Devices 1.0](https://www.usb.org/sites/default/files/midi10.pdf), and it's implementation is written in the style of the STM32_USB_Device_Library Audio Class.

Annotated `usbd_midi.h`, `usbd_midi.c`, `usbd_midi_if.h`, `usbd_midi_if.c` [here](stm32f4-midi-brain/usbd_midi)
