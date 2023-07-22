#include "CanBus.h"

CanStatus CanBus::writeDataFrameByte(int identifier, byte data)
{
    uint8_t buffer[] = {data};
    return writeDataFrame(identifier, buffer, sizeof(buffer));
}

CanStatus CanBus::writeDataFrameInt(int identifier, int data)
{
    uint8_t buffer[sizeof(int)];
    memcpy(buffer, &data, sizeof(int));
    return writeDataFrame(identifier, buffer, sizeof(int));
}

CanStatus CanBus::writeDataFrameLong(int identifier, long data)
{
    uint8_t buffer[sizeof(long)];
    memcpy(buffer, &data, sizeof(long));
    return writeDataFrame(identifier, buffer, sizeof(long));
}

CanStatus CanBus::writeDataFrameFloat(int identifier, float data)
{
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    uint8_t buffer[sizeof(float)];
    memcpy(buffer, &data, sizeof(float));
    return writeDataFrame(identifier, buffer, sizeof(float));
}
