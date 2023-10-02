#include "BaseCAN.h"

BaseCAN::BaseCAN()
{
    // if (pinSHDN != NC)
    // {
    //     pinMode(pinSHDN, OUTPUT);
    // }
    _Serial = &Serial;
    mode = CAN_STANDARD;
}

bool BaseCAN::begin(int can_bitrate)
{
    return begin(static_cast<CanBitRate>(can_bitrate));
}

int BaseCAN::enableInternalLoopback()
{
    BaseCAN::mode = CAN_LOOPBACK;
    return 1;
}

int BaseCAN::disableInternalLoopback()
{
    BaseCAN::mode = CAN_STANDARD;
    return 1;
}

CanTiming BaseCAN::solveCanTiming(uint32_t clockFreq, uint32_t bitrate, uint8_t multiplier)
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
    _Serial->print("clockFreq:");
    _Serial->print(clockFreq);
    _Serial->print(", bitrate:");
    _Serial->print(bitrate);
    _Serial->print(", prescaler:");
    _Serial->print(timing.prescaler);
    _Serial->print(", timeQuanta:");
    _Serial->print(timeQuanta);
    _Serial->print(", nominalTimeSeg1:");
    _Serial->print(timing.tseg1);
    _Serial->print(", nominalTimeSeg2:");
    _Serial->print(timing.tseg2);
    // _Serial->print(", samplePoint:");
    // _Serial->println(samplePoint);

#endif
    return timing;
}

void BaseCAN::logMessage(CanMsg const *msg)
{
    msg->printTo(*_Serial);
    // _Serial->print(frame->identifier, HEX);
    // _Serial->print(" [");

    // // uint8_t length = dlcToLength(dataLength);
    // _Serial->print(frame->dataLength);
    // _Serial->print("] ");
    // if (frame->isRTR)
    // {
    //     _Serial->print("R");
    // }
    // else
    // {
    //     for (uint32_t byte_index = 0; byte_index < frame->dataLength; byte_index++)
    //     {
    //         _Serial->print(frame->data[byte_index], HEX);
    //         _Serial->print(" ");
    //     }
    // }
    // _Serial->println();
}

void BaseCAN::failAndBlink(CanErrorType errorType)
{
#ifdef CAN_DEBUG
    _Serial->print("fatal error: ");
    _Serial->println(errorType, HEX);
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

void BaseCAN::logTo(Stream *serial)
{
    _Serial = serial;
}