#pragma once

#include "Arduino.h"
#include "HardwareCAN.h"

#ifndef NC
#define NC 0xFF
#endif

#ifndef MAX_FRAME_SIZE
#define MAX_FRAME_SIZE 64
#endif

#ifndef PIN_CAN0_SHDN
#define PIN_CAN0_SHDN NC
#endif

#ifndef PIN_CAN1_SHDN
#define PIN_CAN1_SHDN NC
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

struct CanTiming
{
    uint32_t prescaler;
    uint32_t sjw;
    uint32_t tseg1;
    uint32_t tseg2;
};

class BaseCAN //: public HardwareCAN
{

public:
    BaseCAN();

    // setup methods
    virtual bool begin(int can_bitrate) = 0;
    bool begin(CanBitRate const can_bitrate);
    virtual void end() = 0;

    virtual void filter(CanFilter filter) = 0;

    virtual int write(CanMsg const &msg) = 0;
    virtual size_t available() = 0;
    virtual CanMsg read() = 0;

    int enableInternalLoopback();
    int disableInternalLoopback();

    void logTo(Stream *serial);

protected:
    virtual CanTiming solveCanTiming(uint32_t clockFreq, uint32_t bitrate, uint8_t multiplier = 1);
    void logMessage(CanMsg const *msg);
    void failAndBlink(CanErrorType errorType);
    Print *_Serial;
    CanMode mode;
};
