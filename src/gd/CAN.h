#pragma once

#if ARDUINO_ARCH_GD32

#include "Arduino.h"
#include "BaseCAN.h"
#include "gd32f30x.h"
#include "gd32f30x_can.h"

class GD_CAN : public BaseCAN
{

public:
    GD_CAN(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN = NC);

    bool begin(int can_bitrate) override;
    void end() override;

    void filter(CanFilter filter) override;

    CanStatus subscribe(void (*_messageReceiveCallback)() = nullptr);
    CanStatus unsubscribe();

    int write(CanMsg const &msg) override;
    CanMsg read() override;
    size_t available() override;

    static uint32_t hcan_;
    static void (*callbackFunction_)();

    static uint16_t pinRX_;
    static uint16_t pinTX_;
    static uint16_t pinSHDN_;

private:
    bool started_ = false;
    void applyFilter(); // filter is applied after begin() is called
    CanFilter filter_;
    can_receive_message_struct rxHeader_;
    can_trasnmit_message_struct txHeader_;
    CanStatus logStatus(char op, uint32_t status);
};

#if CAN_HOWMANY > 0
extern GD_CAN CAN;
#endif

// #if CAN_HOWMANY > 1
// extern GD_CAN CAN1;
// #endif

#endif