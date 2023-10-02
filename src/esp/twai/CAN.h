#pragma once

#if defined(ARDUINO_ARCH_ESP32)

#include "Arduino.h"
#include "BaseCAN.h"
#include "driver/twai.h"

#ifndef WEAK
#define WEAK __attribute__((weak))
#endif

class ESP_TWAI_CAN : public BaseCAN
{

public:
    ESP_TWAI_CAN(uint32_t pinRX, uint32_t pinTX, uint32_t pinSHDN = GPIO_NUM_NC);

    bool begin(int can_bitrate) override;
    void end() override;

    void filter(CanFilter filter) override;

    int write(CanMsg const &msg) override;
    CanMsg read() override;
    uint32_t available() override;

    static void _messageReceive();
    static void (*receiveCallback)(CanMsg *rxMessage);
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
    // CanMode _mode;
    CanStatus logStatus(char op, esp_err_t status);
};

#if CAN_HOWMANY > 0
extern ESP_TWAI_CAN CAN;
#endif

// #if CAN_HOWMANY > 1
// extern ESP_TWAI_CAN CAN1;
// #endif

#endif
