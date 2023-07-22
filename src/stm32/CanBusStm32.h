#pragma once
#if defined(HAL_CAN_MODULE_ENABLED)
#include "Arduino.h"
#include "../CanBus.h"

#if defined(STM32F4xx)
#include "stm32f4xx_hal_can.h"
#endif

class CanBusStm32 : public CanBus
{

public:
	CanBusStm32(uint32_t rxPin, uint32_t txPin, uint32_t shdnPin = -1);

	CanStatus begin(uint32_t bitrate = 1'000'000, CanMode mode = CAN_STANDARD) override;
	CanStatus writeFrame(uint32_t identifier, uint32_t frameType, uint32_t dataLength, uint8_t data[]);
	CanStatus writeDataFrame(uint32_t identifier, byte buffer[], uint8_t length) override;
	CanStatus writeRemoteFrame(uint32_t identifier, uint8_t length) override;
	CanStatus configureFilter(uint32_t identifier, uint32_t mask = 0b11111111111, uint32_t filter = 0) override;
	CanStatus subscribe(void (*onReceive)()) override;
	CanStatus unsubscribe() override;

	RxFrame readFrame() override;
	uint32_t available() override;

	CanStatus start();
	CanStatus stop();

	static CAN_HandleTypeDef _hcan;

	// todo make this private
	// uint32_t dlcToLength(uint32_t dlc);
	static uint32_t _rxPin;
	static uint32_t _txPin;
	static uint32_t _shdnPin;
	static void (*_callbackFunction)();

private:
	// WEAK void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
	uint32_t lengthToDLC(uint32_t length);
	CAN_RxHeaderTypeDef _rxHeader;
	uint8_t *_rxData;
};
#endif