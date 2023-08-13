#pragma once

#if HAL_CAN_MODULE_ENABLED

#include "Arduino.h"
#include "../../base_can.h"

class Can : public BaseCan
{

public:
	Can(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN = NC);

	CanStatus init(CanMode mode = CAN_STANDARD, uint32_t bitrate = 250000) override;
	CanStatus deinit() override;

	CanStatus filter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111, bool maskRtrBit = false, bool identifierRtrBit = false) override;
	CanStatus subscribe(void (*_messageReceiveCallback)() = nullptr);
	CanStatus unsubscribe();

	CanStatus start() override;
	CanStatus stop() override;

	CanStatus writeFrame(CanFrame *txFrame) override;
	CanStatus readFrame(CanFrame *rxFrame) override;
	uint32_t available();

	static CAN_HandleTypeDef _hcan;
	static void (*_callbackFunction)();
	// static void _messageReceive();

private:
	CAN_RxHeaderTypeDef _rxHeader;
	CAN_TxHeaderTypeDef _txHeader;
};

#endif