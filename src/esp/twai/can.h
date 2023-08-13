#pragma once

#include "Arduino.h"
#include "../../base_can.h"
#include "driver/twai.h"

#ifndef WEAK
#define WEAK __attribute__((weak))
#endif

class Can : BaseCan
{

public:
    // constructor
    Can(gpio_num_t pinRX, gpio_num_t pinTX, gpio_num_t pinSHDN = GPIO_NUM_NC);

    CanStatus init(uint32_t bitrate = 1000000, CanMode mode = CAN_STANDARD) override;
    CanStatus deinit() override;

    // esp does not support multiple filters (but does support a single dual filter)
    CanStatus filter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111, bool maskRtrBit = false, bool identifierRtrBit = false) override;

    CanStatus start() override;
    CanStatus stop() override;

    CanStatus writeFrame(CanFrame *txFrame) override;
    CanStatus readFrame(CanFrame *rxMessage) override;

    static void _messageReceive();
    static void (*receiveCallback)(CanFrame *rxMessage);

private:
    CanStatus writeFrame(uint32_t identifier, bool isRTR, uint32_t dataLength, uint8_t data[]);

    twai_general_config_t _general_config;
    twai_timing_config_t _timing_config;
    twai_filter_config_t _filter_config;

    CanMode _mode;
};
