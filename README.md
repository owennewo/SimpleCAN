# SimpleCAN (a CAN library for ARDUINO)

This library if for microcontrollers with an onboard CAN Controller.  

Esp32 and high-end STM32 have Can Controllers, but its worth checking on datasheets as it isn't always obvious e.g. blackpill_f411ce doesn't support CAN byt f405 often does.

Partially tested on stm (using can and fdcan hal lib) and esp32 (using TWAI lib):
 - stm_fdcan
   - g0 - nucleo_g0b1re 
   - g4 - g431
   - h7 - portenta_h7_m7
 - stm_can
   - f4 - stm32f405rg_vesc
   - f1 - untested
   - l0 - untested

checkout the platformio.ini for boards that have been (partially) tested.

The API is supposed aims to be similar to the Arduino Uno R4 (Renesas) API.
There are some small areas where this is not possible (E.g. the use of pre-initialised CAN - stm sometimes uses this as a CAN_TypeDef e.g blackpill)

This aims to be a true Hardware Abstraction across multiple vendor/series.


By true, I mean that you could build Messages on this that would work across esp32, stm can2b, stm fdcan. Same API, different hardware.

## Unit test

A bit ununsual but there are 'onboard' unit tests, that you can try to run.


