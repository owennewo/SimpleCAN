#pragma once

#include "Arduino.h"
#include "../CanBus.h"
#include <stm32g4xx_hal_fdcan.h>

class CanBusStm32FD : public CanBus
{
public:
	// class RxHandler
	// {
	// public:
	// 	RxHandler(uint16_t dataLength, void (*callback)(RxFrame frame)); // FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData));
	// 	~RxHandler();

	// 	void notify(FDCAN_HandleTypeDef *hfdcan);

	// private:
	// 	FDCAN_RxHeaderTypeDef _rxHeader;
	// 	uint8_t *_rxData;
	// 	void (*_callback)(RxFrame frame); // FDCAN_RxHeaderTypeDef, uint8_t *);
	// };

public:
	CanBusStm32FD(uint32_t rxPin, uint32_t txPin, bool terminateTransceiver);

	CanStatus begin(int bitrate = 1'000'000, CanMode mode = CAN_STANDARD) override;
	CanStatus writeFrame(uint32_t identifier, uint32_t frameType, uint32_t dataLength, uint8_t data[]);
	CanStatus writeDataFrame(int identifier, byte buffer[], uint8_t length) override;
	CanStatus writeRemoteFrame(int identifier, uint8_t length) override;
	RxFrame readFrame() override;
	uint32_t available() override;
	CanStatus subscribe(void (*onReceive)(), uint32_t primaryIdentifier, uint32_t primaryIdentifierMask);
	CanStatus unsubscribe();

	CanStatus start();
	CanStatus stop();

	static FDCAN_HandleTypeDef _hfdcan1;

	// todo make this private
	int dlcToLength(uint32_t dlc);
	static uint32_t _rxPin;
	static uint32_t _txPin;
	static void (*_callbackFunction)();

private:
	WEAK void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
	uint32_t lengthToDLC(int length);
	FDCAN_RxHeaderTypeDef _rxHeader;
	uint8_t *_rxData;
};