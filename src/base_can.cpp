#include "base_can.h"

BaseCan::BaseCan(uint8_t pinRX, uint8_t pinTX, uint8_t pinSHDN) : _pinRX(pinRX), _pinTX(pinTX), _pinSHDN(pinSHDN)
{
    if (pinSHDN != NC)
    {
        // Serial.println("SHDN pin is not NC");
        pinMode(pinSHDN, OUTPUT);
        // digitalWrite(pinSHDN, HIGH);
    }
}