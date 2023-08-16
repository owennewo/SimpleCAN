# SimpleCAN (CanBus Library for Arduino)


This is aims to be a true Hardware Abstraction across multiple vendor/series.

By true, I mean that you could build Messages on this that would work across esp32, stm can2b, stm fdcan. Same API, different hardware.

  
| vendor | API | series  | variant/board | tx | rx | status |
|--|--|--|--|--|--|--|
| stm | G4 | fdcan | std, ext | polling ✅<br> irq ✅  | compiles, manual loopback tested, unit tested |  |
| stm | H7| fdcan | std, ext| polling ✅<br> irq ❌ | compiles, manual loopback tested, unit tested | stm | fdcan | L0 | | ? | | |
| stm | f4 | can2b | std, ext | ✅<br> irq ✅ | f405, f407 tested | | |
| stm | f1 | can2b | std, ext | ?| not tested | | |
| stm | l1 | can2b | std, ext | ? | not tested | | |
| esp | esp32S |twai |  | | was working (refactoring) | | | 
| arduino | R4 | ? |  | | not started | | | 



## Heierachy

You only need to include simplecan.h

Mostly you will use functions defined in `base_can.h``

But there are some addional functions (e.g. subscribe) in some of the lower interfaces e.g. `stm/fdcan/can.h``


And this will produce a flow chart:

```mermaid
graph LR
base[base_can] --> stm_can((stm/can.h))
base --> stm_fdcan((stm/fdcan/can.h))
base --> esp_twai((esp/twai/can.h))

```
