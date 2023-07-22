/*  Status
I've tested this in LOOPBACK only with my custom stm32g4 board.
I've alos used CAN_STANDARD so this is properly tested.
It sends and receives (using available/readFrame in loop)

The suscribe approach works fine too, but you need to be careful
with the callback function as it is an interrupt. I avoid it.
*/

#include <Arduino.h>

#include "stm32fd/CanBusStm32FD.h"

CanBusStm32FD can1(GPIO_PIN_11, GPIO_PIN_12, CAN_SHDN);

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
