#include "stm/can/can_stm.h"

CanFrame rx = {};

// DRIVA
// CanStm can(PA_11, PA_12, PB_4);

// ESC1
CanStm can(PA_11, PB_9, PC_11);

void handleCanFrame(CanFrame *rx)
{
    // can.readFrame(&rx);
    Serial.print("message can received!: ");
    Serial.println(rx->msgID, HEX);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void setup()
{
    // TIP1:  Don't use subscribe unless you respect IRQ timngs (return quickly, e.g. no Serial.prints).
    // TIP2:  If you must Serial.print in an IRQ then increase buffer using SERIAL_TX_BUFFER_SIZE in platformio.ini
    // TIP3:  Start with testing in CAN_LOOPBACK mode

    // digitalWrite(CAN_SHDN, LOW);

    Serial.begin(115200);
    delay(2000);
    Serial.println("Setup");
    Serial.print("tx size");
    Serial.println(SERIAL_TX_BUFFER_SIZE);
    Serial.println(PA_11);
    PinName rx = PA_11;
    Serial.println(rx);

    pinMode(PC13, OUTPUT);
    can.init(1'000'000, CanMode::CAN_LOOPBACK);

    // can.configureFilter(FilterType::FILTER_16BIT_1, 0b11000000000, 0b11100000000);
    // can.configureFilter(FilterType::FILTER_16BIT_2, 0b00100000000, 0b11100000000);
    can.configureFilter(FilterType::FILTER_ACCEPT_ALL);
    // can.configureFilter(FilterType::FILTER_16BIT_2, 0b00100000000, 0b11100000000);

    can.subscribe(&handleCanFrame);
    can.start();
}

int identifier = 0b11111111000; //<- bit smaller than the largest 11bit identify

uint32_t count = 3;

void loop()
{
    // static uint8_t data[2];
    // data[0]++;
    // data[1]--;

    // delay(1000);
    // // digitalWrite(LED_RED, !digitalRead(LED_RED));

    // if (can.available())
    // {
    //   can.readFrame(&rx);
    //   Serial.print(rx.msgID, HEX);
    // }
    // // can.writeRemoteFrame(0x234, 2);
    // can.writeDataFrame(identifier, data, 2);
    // identifier++;

    delay(1000);
    // can.writeDataFrameFloat(0x321, count);
    can.writeRemoteFrame(0x321, count);

    count--;
}
