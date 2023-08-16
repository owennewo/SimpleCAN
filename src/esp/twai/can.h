#pragma once

#if defined(ARDUINO_ARCH_ESP32)

#include "Arduino.h"
#include "../../base_can.h"
#include "driver/twai.h"

#ifndef WEAK
#define WEAK __attribute__((weak))
#endif

class Can : public BaseCan
{

public:
    Can(uint32_t pinRX, uint32_t pinTX, uint32_t pinSHDN = GPIO_NUM_NC);

    CanStatus init(CanMode mode = CAN_STANDARD, uint32_t bitrate = 250000) override;
    CanStatus deinit() override;

    CanStatus filter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111, bool maskRtrBit = false, bool identifierRtrBit = false) override;
    // subscribe is not possible on ESP32
    // unsubscribe is not possible on ESP32

    CanStatus start() override;
    CanStatus stop() override;

    CanStatus writeFrame(CanFrame *txFrame) override;
    CanStatus readFrame(CanFrame *rxFrame) override;
    uint32_t available() override;

    static void _messageReceive();
    static void (*receiveCallback)(CanFrame *rxMessage);
    static uint16_t _pinRX;
    static uint16_t _pinTX;
    static uint16_t _pinSHDN;

private:
    twai_general_config_t _general_config;
    twai_timing_config_t _timing_config;
    twai_filter_config_t _filter_config;
    twai_message_t _rxEspFrame;
    twai_message_t _txEspFrame;
    twai_status_info_t _statusInfo;
    CanMode _mode;
    CanStatus logStatus(char op, esp_err_t status);
};
#endif
