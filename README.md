Following repository contains HAL C code for developing a one-wire bus communication between STM32U545RE-Q and DHT-11 sensor.
DHT11 sensor is a temperature and humidity sensor. the details of the sensor are provided in the datasheet with which the sensor is interfaced to the microcontroller. The link for the datasheet is given as:
https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf
Only the main source files are added in this repository as the rest of the files are generated automatically by the STMCubeIDE.
The target microcontroller - STM32U545RE-Q is selected in the STMCubeMX, matching the hardware.
Target microcontroller is configured for enabling RCC, timer and clock configurations.
The following repository shows that the DHT11 sensor communicating with the microcontroller and showing the temperature and humidity at real time conditions.
The resulting temperature and humidity readings are displayed using Serial console through UART protocol.
