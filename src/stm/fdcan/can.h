#pragma once

#include "Arduino.h"
#include "../../base_can.h"

class Can : BaseCan
{

public:
	// constructor
	Can(uint8_t rx_pin, uint8_t tx_pin, uint8_t shdnPin = NC);

	CanStatus init(uint32_t bitrate = 1000000, CanMode mode = CAN_STANDARD) override;
	CanStatus deinit() override;

	CanStatus filter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111) override;

	CanStatus start() override;
	CanStatus stop() override;

	CanStatus writeFrame(CanFrame *txFrame) override;
	// CanStatus writeRemoteFrame(uint32_t identifier, uint8_t length = 0) override;

	uint32_t available();

	CanStatus subscribe(void (*_messageReceiveCallback)(CanFrame *rxMessage) = nullptr);
	CanStatus unsubscribe();

	CanStatus readFrame(CanFrame *rxMessage) override;

	static void _messageReceive();
	// static CanStatus _readFrame(FDCAN_HandleTypeDef *hcan, CanFrame *rxMessage);
	static void (*receiveCallback)(CanFrame *rxMessage);
	static FDCAN_HandleTypeDef *_hcan;

	// static PinName _pinSHDN;

private:
	CanStatus writeFrame(uint32_t identifier, bool isRTR, uint8_t dataLength, byte *data);
	FDCAN_RxHeaderTypeDef _rxHeader;
	uint8_t *_rxData;
};
