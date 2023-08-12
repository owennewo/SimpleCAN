#include "base_can.h"

BaseCan::BaseCan(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN) : _pinRX(pinRX), _pinTX(pinTX), _pinSHDN(pinSHDN)
{
    if (pinSHDN != NC)
    {
        // Serial.println("SHDN pin is not NC");
        pinMode(pinSHDN, OUTPUT);
        // digitalWrite(pinSHDN, HIGH);
    }
}

CanTiming BaseCan::solveCanTiming(uint32_t clockFreq, uint32_t bitrate)
{

    CanTiming timing = {};
    // Looking for a timeQuanta of between 8 and 25.
    // start at 16 and work outwards
    // this algo is inspired by: http://www.bittiming.can-wiki.info/

    uint32_t baseQuanta = 16;
    uint32_t timeQuanta = baseQuanta;

    uint32_t offset = 0;
    bool found = false;

    while (offset <= 9)
    {
        timeQuanta = baseQuanta - offset;
        if (clockFreq % (bitrate * timeQuanta) == 0)
        {
            found = true;
            break;
        }
        timeQuanta = baseQuanta + offset;
        if (clockFreq % (bitrate * timeQuanta) == 0)
        {
            found = true;
            break;
        }
        offset += 1;
    }
    if (!found)
    {
#ifdef CAN_DEBUG
        Serial.println("timeQuanta out of range");
#endif
        return timing;
    }

    timing.prescaler = clockFreq / (bitrate * timeQuanta);
    timing.sjw = 1;
    timing.tseg1 = uint32_t(0.875 * timeQuanta) - 1;

    float samplePoint = (1.0 + timing.tseg1) / timeQuanta;
    float samplePoint2 = (1.0 + timing.tseg1 + 1) / timeQuanta;

    if (abs(samplePoint2 - 0.875) < abs(samplePoint - 0.875))
    {
        timing.tseg1 += 1;
        samplePoint = samplePoint2;
    }

    timing.tseg2 = timeQuanta - timing.tseg1 - 1;
#ifdef CAN_DEBUG
    Serial.print("clockFreq:");
    Serial.print(clockFreq);
    Serial.print(", bitrate:");
    Serial.print(bitrate);
    Serial.print(", core:");
    Serial.print(clockFreq);
    Serial.print(", prescaler:");
    Serial.print(timing.prescaler);
    Serial.print(", timeQuanta:");
    Serial.print(timeQuanta);
    Serial.print(", nominalTimeSeg1:");
    Serial.print(timing.tseg1);
    Serial.print(", nominalTimeSeg2:");
    Serial.print(timing.tseg2);
    Serial.print(", samplePoint:");
    Serial.println(samplePoint);

#endif
    return timing;
}