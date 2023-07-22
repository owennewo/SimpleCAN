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
    virtual CanStatus begin(int bitrate = 1'000'000, CanMode mode = CAN_STANDARD) = 0;
    virtual CanStatus writeDataFrame(int identifier, byte buffer[], uint8_t length) = 0;
    virtual CanStatus writeRemoteFrame(int identifier, uint8_t length) = 0;
    virtual uint32_t available() = 0;
    virtual RxFrame readFrame() = 0;

    // helpers, perhaps easy to use than the above
    virtual CanStatus writeDataFrameByte(int identifier, byte data);
    virtual CanStatus writeDataFrameInt(int identifier, int data);
    virtual CanStatus writeDataFrameLong(int identifier, long data);
    virtual CanStatus writeDataFrameFloat(int identifier, float data);
};

#endif // CANBUS_H