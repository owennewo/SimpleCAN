#pragma once

#if HAL_FDCAN_MODULE_ENABLED

#include "Arduino.h"
#include "../../can.h"

class CanStm : Can
{

public:
	// constructor
	CanStm(PinName rx_pin, PinName tx_pin, PinName shdnPin = NC);

	CanStatus init(uint32_t bitrate = 1'000'000, CanMode mode = CAN_STANDARD) override;

	CanStatus configureFilter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111, uint32_t filterIndex = 0) override;

	CanStatus start() override;
	CanStatus stop() override;

	CanStatus writeDataFrame(uint32_t identifier, byte buffer[], uint8_t length) override;
	CanStatus writeRemoteFrame(uint32_t identifier, uint8_t length) override;

	uint32_t available() override;

	CanStatus subscribe(void (*_messageReceiveCallback)(CanFrame *rxMessage) = nullptr) override;
	CanStatus unsubscribe() override;

	CanStatus readFrame(CanFrame *rxMessage) override;

	static void _messageReceive();
	static CanStatus _readFrame(FDCAN_HandleTypeDef *hcan, CanFrame *rxMessage);
	static void (*receiveCallback)(CanFrame *rxMessage);
	static FDCAN_HandleTypeDef *_hcan;

	static PinName _pinSHDN;

private:
	CanStatus writeFrame(uint32_t identifier, uint32_t frameType, uint32_t dataLength, uint8_t data[]);
	FDCAN_RxHeaderTypeDef _rxHeader;
	uint8_t *_rxData;
};

#endif
