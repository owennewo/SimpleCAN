# Welcome to StackEdit!

# SimpleCAN (for STM32DUINO)

  

This is aims to be a true Hardware Abstraction across multiple vendor/series.


By true, I mean that you could build Messages on this that would work across esp32, stm can2b, stm fdcan. Same API, different hardware.
|  |  |
|--|--|
|  |  |

  
| vendor | API | series  | variant/board | tx | rx | status |
|--|--|--|--|--|--|--|
| stm | G4 | fdcan | std, ext | polling ✅<br> irq ✅  | compiles, manual loopback tested, unit tested |  |
| stm | H7| fdcan | std, ext| polling ✅<br> irq ❌ | compiles, manual loopback tested, unit tested | stm | fdcan | L0 | | ? | | |
| stm | G0 | fdcan |  | | ? | | | 
| esp | esp32S |twai |  | | was working (refactoring) | | | 
| arduino | R4 | ? |  | | not started | | | 






## Heierachy

You only need to include simplecan.h

Mostly you will use functions defined in `base_can.h``

But there are some addional functions (e.g. subscribe) in some of the lower interfaces e.g. `stm/fdcan/can.h``


And this will produce a flow chart:

```mermaid
graph LR
base_can[Square Rect] -- Link text --> stm/can((Circle))
base_can --> stm/fdcan(Round Rect)
base_can --> esp/twai/{Rhombus}

```