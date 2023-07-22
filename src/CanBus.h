#ifndef CANBUS_H
#define CANBUS_H

#include "Arduino.h"

struct RxFrame
{
    uint32_t identifier;
    bool isRemoteRequest;
    uint32_t dataLength;
    uint8_t *buffer;
};

enum CanStatus
{
    CAN_OK = 0x00U,
    CAN_ERROR = 0x01U,
};

enum CanMode
{
    CAN_STANDARD = 0x00U,
    CAN_LOOPBACK = 0x01U,
    CAN_LOOPBACK_EXTERNAL = 0x02U,
};

class CanBus
{

public:
    // these are overridden by the hardware specific implementations
    virtual CanStatus begin(uint32_t bitrate = 1'000'000, CanMode mode = CAN_STANDARD) = 0;
    virtual CanStatus writeDataFrame(uint32_t identifier, byte buffer[], uint8_t length) = 0;
    virtual CanStatus writeRemoteFrame(uint32_t identifier, uint8_t length) = 0;
    virtual CanStatus configureFilter(uint32_t identifier, uint32_t mask = 0b11111111111, uint32_t filter = 0) = 0;
    virtual CanStatus subscribe(void (*onReceive)()) = 0;
    virtual CanStatus unsubscribe() = 0;

    virtual uint32_t available() = 0;
    virtual RxFrame readFrame() = 0;

    // helpers, perhaps easy to use than the above
    virtual CanStatus writeDataFrameByte(uint32_t identifier, byte data);
    virtual CanStatus writeDataFrameInt(uint32_t identifier, uint32_t data);
    virtual CanStatus writeDataFrameLong(uint32_t identifier, long data);
    virtual CanStatus writeDataFrameFloat(uint32_t identifier, float data);
};

#endif // CANBUS_H