#pragma once

#if HAL_CAN_MODULE_ENABLED

#include "Arduino.h"
#include "../../base_can.h"

class Can : BaseCan
{

public:
	// constructor
	Can(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN = NC);

	CanStatus init(CanMode mode = CAN_STANDARD, uint32_t bitrate = 250000) override;
	CanStatus deinit() override;

	CanStatus filter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111, bool maskRtrBit = false, bool identifierRtrBit = false) override;

	CanStatus start() override;
	CanStatus stop() override;

	CanStatus writeFrame(CanFrame *txFrame) override;

	uint32_t available();

	CanStatus subscribe(void (*_messageReceiveCallback)() = nullptr);
	CanStatus unsubscribe();

	CanStatus readFrame(CanFrame *rxFrame) override;

	static void _messageReceive();
	static void (*receiveCallback)();
	static CAN_HandleTypeDef *_hcan;

private:
	CAN_RxHeaderTypeDef _rxHeader;
	uint16_t *_rxData;
};

#endif