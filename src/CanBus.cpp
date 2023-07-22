#include "CanBus.h"

CanStatus CanBus::writeDataFrameByte(uint32_t identifier, byte data)
{
    uint8_t buffer[] = {data};
    return writeDataFrame(identifier, buffer, sizeof(buffer));
}

CanStatus CanBus::writeDataFrameInt(uint32_t identifier, uint32_t data)
{
    uint8_t buffer[sizeof(uint32_t)];
    memcpy(buffer, &data, sizeof(uint32_t));
    return writeDataFrame(identifier, buffer, sizeof(uint32_t));
}

CanStatus CanBus::writeDataFrameLong(uint32_t identifier, long data)
{
    uint8_t buffer[sizeof(long)];
    memcpy(buffer, &data, sizeof(long));
    return writeDataFrame(identifier, buffer, sizeof(long));
}

CanStatus CanBus::writeDataFrameFloat(uint32_t identifier, float data)
{
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    uint8_t buffer[sizeof(float)];
    memcpy(buffer, &data, sizeof(float));
    return writeDataFrame(identifier, buffer, sizeof(float));
}
