#pragma once

#if HAL_CAN_MODULE_ENABLED

#include "Arduino.h"
#include "BaseCAN.h"

class STM_CAN : public BaseCAN
{

public:
	STM_CAN(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN = NC);

	bool begin(int can_bitrate) override;
	void end() override;

	void filter(CanFilter filter) override;

	CanStatus subscribe(void (*_messageReceiveCallback)() = nullptr);
	CanStatus unsubscribe();

	int write(CanMsg const &msg) override;
	CanMsg read() override;
	size_t available() override;

	static CAN_HandleTypeDef hcan_;
	static void (*callbackFunction_)();

	static uint16_t pinRX_;
	static uint16_t pinTX_;
	static uint16_t pinSHDN_;

private:
	bool started_ = false;
	void applyFilter(); // filter is applied after begin() is called
	CanFilter filter_;
	CAN_RxHeaderTypeDef rxHeader_;
	CAN_TxHeaderTypeDef txHeader_;
	CanStatus logStatus(char op, HAL_StatusTypeDef status);
};

#if CAN_HOWMANY > 0
extern STM_CAN CAN;
#endif

// #if CAN_HOWMANY > 1
// extern STM_CAN CAN1;
// #endif

#endif