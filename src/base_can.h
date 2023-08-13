#pragma once

#include "Arduino.h"

#ifndef NC
#define NC 0xFF
#endif

#ifndef MAX_FRAME_SIZE
#define MAX_FRAME_SIZE 64
#endif

#ifndef CAN_SHDN
#define CAN_SHDN NC
#endif

enum CanMode
{
    CAN_STANDARD = 0x00U,
    CAN_LOOPBACK = 0x01U,
    // CAN_LOOPBACK_EXTERNAL = 0x02U,  <-- some boards support this
};

enum CanErrorType
{
    CAN_ERROR_CLOCK = 0x03U,
    CAN_ERROR_TIMING = 0x04U,
    CAN_ERROR_BITRATE_TOO_HIGH = 0x05U,
};

enum CanStatus
{
    CAN_OK = 0x00U,
    CAN_ERROR = 0x01U,
    CAN_NO_DATA = 0x02U,
};

enum FilterType
{
    FILTER_DISABLE = 0x00U,
    FILTER_MASK_STANDARD = 0x01U,
    FILTER_MASK_EXTENDED = 0x02U,
    FILTER_ACCEPT_ALL_STANDARD = 0x03U,
    FILTER_ACCEPT_ALL_EXTENDED = 0x04U
};

struct CanFrame
{
    uint32_t identifier;
    bool isRTR;
    bool isExtended;
    uint32_t dataLength;
    uint8_t data[MAX_FRAME_SIZE];
};

struct CanTiming
{
    uint32_t prescaler;
    uint32_t sjw;
    uint32_t tseg1;
    uint32_t tseg2;
};

class BaseCan
{

public:
    BaseCan(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN = NC);

    // setup methods
    virtual CanStatus init(CanMode mode = CAN_STANDARD, uint32_t bitrate = 250000) = 0;
    virtual CanStatus deinit() = 0;
    virtual CanStatus filter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111, bool maskRtrBit = false, bool identifierRtrBit = false) = 0;
    virtual CanStatus start() = 0;
    virtual CanStatus stop() = 0;

    // tx methods
    virtual CanStatus writeFrame(CanFrame *txFrame) = 0;

    // rx methods
    virtual CanStatus readFrame(CanFrame *rxMessage) = 0;

protected:
    virtual CanTiming solveCanTiming(uint32_t clockFreq, uint32_t bitrate);
    void logFrame(CanFrame *txFrame);
    void failAndBlink(CanErrorType errorType);
    uint16_t _pinRX;
    uint16_t _pinTX;
    uint16_t _pinSHDN;
};
