#pragma once

#include "Arduino.h"

enum CanMode
{
    CAN_STANDARD = 0x00U,
    CAN_LOOPBACK = 0x01U,
    CAN_LOOPBACK_EXTERNAL = 0x02U,
};

enum CanStatus
{
    CAN_OK = 0x00U,
    CAN_ERROR = 0x01U,
    CAN_BUSY = 0x02U,
    CAN_TIMEOUT = 0x03U
};

enum FilterType
{
    FILTER_DISABLE = 0x00U,
    FILTER_MASK = 0x01U,
    FILTER_ACCEPT_ALL = 0x02U
};

struct CanFrame
{
    uint8_t dlc;
    uint32_t msgID;
    bool isRTR;
    bool isStandard;
    uint8_t data[8];
};

class Can
{

public:
    // setup methods
    virtual CanStatus init(uint32_t bitrate = 1'000'000, CanMode mode = CAN_STANDARD) = 0;
    virtual CanStatus configureFilter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111, uint32_t filterIndex = 0) = 0;
    virtual CanStatus start() = 0;
    virtual CanStatus stop() = 0;

    // tx methods
    virtual CanStatus writeDataFrame(uint32_t identifier, byte buffer[], uint8_t length) = 0;
    virtual CanStatus writeRemoteFrame(uint32_t identifier, uint8_t length) = 0;

    // rx methods
    virtual CanStatus subscribe(void (*_messageReceiveCallback)(CanFrame *rxMessage) = nullptr) = 0;
    virtual CanStatus unsubscribe() = 0;
    virtual uint32_t available() = 0;
    virtual CanStatus readFrame(CanFrame *rxMessage) = 0;
};
