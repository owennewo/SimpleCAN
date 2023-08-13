#include "base_can.h"

BaseCan::BaseCan(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN) : _pinRX(pinRX), _pinTX(pinTX), _pinSHDN(pinSHDN)
{
    if (pinSHDN != NC)
    {
        pinMode(pinSHDN, OUTPUT);
    }
}

CanTiming BaseCan::solveCanTiming(uint32_t clockFreq, uint32_t bitrate, uint8_t multiplier)
{
    // this algo is inspired by: http://www.bittiming.can-wiki.info/
    CanTiming timing = {};
    uint32_t baseQuanta = 16;
    uint32_t timeQuanta = baseQuanta;

    uint32_t offset = 0;
    bool found = false;

    // start at 16 and work outwards
    while (offset <= 9)
    {
        // Looking for a timeQuanta of between 8 and 25.
        timeQuanta = baseQuanta - offset;
        if (clockFreq % (bitrate * timeQuanta * multiplier) == 0)
        {
            found = true;
            break;
        }
        timeQuanta = baseQuanta + offset;
        if (clockFreq % (bitrate * timeQuanta * multiplier) == 0)
        {
            found = true;
            break;
        }
        offset += 1;
    }

    if (!found)
    {
        failAndBlink(CAN_ERROR_TIMING);
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

void BaseCan::logFrame(CanFrame *frame)
{
    Serial.print(frame->identifier, HEX);
    Serial.print(" [");

    // uint8_t length = dlcToLength(dataLength);
    Serial.print(frame->dataLength);
    Serial.print("] ");
    if (frame->isRTR)
    {
        Serial.print("R");
    }
    else
    {
        for (uint32_t byte_index = 0; byte_index < frame->dataLength; byte_index++)
        {
            Serial.print(frame->data[byte_index], HEX);
            Serial.print(" ");
        }
    }
    Serial.println();
}

void BaseCan::failAndBlink(CanErrorType errorType)
{
#ifdef CAN_DEBUG
    Serial.print("fatal error: ");
    Serial.println(errorType, HEX);
#endif
    while (1)
    {
        for (uint8_t i = 0; i < errorType; i++)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
        delay(1000);
    }
}