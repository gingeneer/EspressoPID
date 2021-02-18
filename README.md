# EspressoPID
A PID for the Gaggia Classic, using an ESP32 and a PT100 sensor to control the heater. 

The display outputs a nice graph of the temperature over time and allows for PID tuning directly on the device, without connecting to a PC.

## Components: 

- ESP32 (I used a DoIt devkit V1)
- PT100 temperature sensor with M4 thread to replace the original thermostat
- MAX31865 PT100 to SPI module
- 240x320 SPI display with ST7789 driver (multiple displays would be compatible with the used library)
- A rotary encoder and a push button for the UI
