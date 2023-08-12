#pragma once
#if defined(HAL_FDCAN_MODULE_ENABLED)
#include "Arduino.h"
#include "../../base_can.h"

class Can : public BaseCan
{

public:
	Can(uint16_t rxPin, uint16_t txPin, uint16_t shdnPin = NC);

	CanStatus init(uint32_t bitrate = 1000000, CanMode mode = CAN_STANDARD) override;
	CanStatus deinit() override;

	CanStatus writeFrame(CanFrame *txFrame) override;

	virtual CanStatus filter(FilterType filterType, uint32_t identifier = 0b11111111111, uint32_t mask = 0b11111111111, bool maskRtrBit = false, bool identifierRtrBit = false) override;
	CanStatus subscribe(void (*onReceive)());
	CanStatus unsubscribe();

	CanStatus readFrame(CanFrame *rxMessage) override;
	uint32_t available();

	CanStatus start() override;
	CanStatus stop() override;

	static FDCAN_HandleTypeDef _hfdcan1;

	// todo make this private
	uint32_t dlcToLength(uint32_t dlc);
	static uint32_t _rxPin;
	static uint32_t _txPin;
	static uint32_t _shdnPin;
	static void (*_callbackFunction)();

private:
	WEAK void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
	uint32_t lengthToDLC(uint32_t length);
	FDCAN_RxHeaderTypeDef _rxHeader;
	// uint8_t _rxData[8];
};
#endif