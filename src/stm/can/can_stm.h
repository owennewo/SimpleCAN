#pragma once

#if HAL_CAN_MODULE_ENABLED

#include "Arduino.h"
#include "../../can.h"

class CanStm : Can
{

public:
	// constructor
	CanStm(uint32_t rx_pin, uint32_t tx_pin, uint32_t shdnPin = -1);

	CanStatus init(uint32_t bitrate = 1'000'000, CanMode mode = CAN_STANDARD) override;

	CanStatus configureFilter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111) override;

	CanStatus start() override;
	CanStatus stop() override;

	CanStatus writeDataFrame(uint32_t identifier, byte buffer[], uint8_t length) override;
	CanStatus writeRemoteFrame(uint32_t identifier, uint8_t length) override;

	// uint32_t available() override;

	CanStatus subscribe(void (*_messageReceiveCallback)(CanFrame *rxMessage) = nullptr) override;
	CanStatus unsubscribe() override;

	CanStatus readFrame(CanFrame *rxMessage) override;

	static void _messageReceive();
	static CanStatus _readFrame(CAN_HandleTypeDef *hcan, CanFrame *rxMessage);
	static void (*receiveCallback)(CanFrame *rxMessage);
	static CAN_HandleTypeDef *_hcan;

private:
	CanStatus writeFrame(uint32_t identifier, , bool isRTR, uint32_t dataLength, uint8_t data[]);
	CAN_RxHeaderTypeDef _rxHeader;
	uint8_t *_rxData;
};

#endif