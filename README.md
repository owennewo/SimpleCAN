# SimpleCAN

This is a wrapper for using CAN bus on stm32f4XX chips (tested on stm32f405) but as it uses HAL_CAN it may work on other stm32 chips.

You can transmit and receive CAN message.  For receive it is possible to poll (i.e. call can.receive in loop()) or use interrupts which require can.subscribe(callback). 

For

Perhaps the most brittle part of this library is that it assumes your board has a 
42MHz PCKL1 which is common for 168Mhz Boards such as stm32f4xx

If you'r PCKL1 is a different speed then you won't be running at the speed stated.  Updating the table below in SimpleCAN.h will be necessary.
The printCanSpeed() method might help show what PCKL1 speed the board thinks it is using.
```
const can_timings[] {
	{.Prescaler=3,.TimeSeg1=11,.TimeSeg2=2}, // BAUD_1M
	{.Prescaler=6,.TimeSeg1=11,.TimeSeg2=2}, // BAUD_500K
	{.Prescaler=12,.TimeSeg1=11,.TimeSeg2=2}, // BAUD_250K
	{.Prescaler=21,.TimeSeg1=13,.TimeSeg2=2}, // BAUD_125K
	{.Prescaler=28,.TimeSeg1=12,.TimeSeg2=2}, // BAUD_100K
	{.Prescaler=56,.TimeSeg1=12,.TimeSeg2=2}, // BAUD_50K
	{.Prescaler=140,.TimeSeg1=12,.TimeSeg2=2}, // BAUD_20K
};
```
