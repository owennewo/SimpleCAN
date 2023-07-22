/*  Status
I've tested this in LOOPBACK only with my portenta H7 M7.
I've not tried CAN_STANDARD as my board lacks a traceiver chip.
It sends and receives (using available/readFrame in loop)
It seems to boot loop if you try the subscribe method (which uses interrupts)
*/

#include <Arduino.h>

#include "stm32fd/CanBusStm32FD.h"

CanBusStm32FD can1(GPIO_PIN_11, GPIO_PIN_12);

// void onReceive()
// {
// 	// // this is an called from an interrupt function, so be quick!  e.g. Serial prints may crash here
// 	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

// 	RxFrame frame = can1.readFrame();
// 	Serial.print("."); // frame.identifier); // frame: %d %d\n", frame.identifier, frame.isRemoteRequest);
// }

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115'200);
    delay(4000);
    Serial.println("SETUP CAN");
    can1.begin(125'000, CAN_LOOPBACK);
    can1.configureFilter(0x321, 0x7FFF, 0);
    // can1.subscribe(&onReceive);
    can1.start();
}

float count = 5.0;

void loop()
{
    delay(1000);
    // Serial.println(count);
    can1.writeRemoteFrame(0x321, 2);
    // can1.writeDataFrameFloat(0x321, count);

    while (can1.available() > 0)
    {
        RxFrame frame = can1.readFrame();
        Serial.println(frame.identifier); // frame: %d %d\n", frame.identifier, frame.isRemoteRequest);
    }

    count--;
}
