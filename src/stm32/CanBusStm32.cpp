#include "CanBusStm32.h"
#include <string.h>
#include <Arduino.h>

// // will be called from: HAL_FDCAN_Init
// extern "C" void HAL_FDCAN_MspInit(CAN_HandleTypeDef *hcan);
// #if !defined(STM32H7xx)
// extern "C" void FDCAN1_IT0_IRQHandler();
// #endif
// extern "C" void HAL_FDCAN_RxFifo0Callback(CAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs);

uint32_t CanBusStm32::_rxPin;
uint32_t CanBusStm32::_txPin;
uint32_t CanBusStm32::_shdnPin;

CanStatus canBusInit(uint32_t bitrate, CAN_HandleTypeDef *hcan, CanMode mode);
void canBusPostInit(CAN_HandleTypeDef *hcan);
void (*CanBusStm32::_callbackFunction)() = nullptr;

CAN_HandleTypeDef CanBusStm32::_hcan = {};

CanBusStm32::CanBusStm32(uint32_t rxPin, uint32_t txPin, uint32_t shdnPin)
{
	if (_hcan.Instance != NULL)
	{
		// Error_Handler();
	}
	CanBusStm32::_rxPin = rxPin;
	CanBusStm32::_txPin = txPin;
	CanBusStm32::_shdnPin = shdnPin;
	_hcan.Instance = CAN1;

	if (shdnPin != -1)
	{
		pinMode(shdnPin, OUTPUT);
	}
}

CanStatus CanBusStm32::begin(uint32_t bitrate, CanMode mode)
{

	if (bitrate > 1000000)
	{
#ifdef CAN_DEBUG
		Serial.println("bitrate > 1Mbit/s, failing");
#endif
		return CAN_ERROR;
	}

	if (canBusInit(bitrate, &_hcan, mode) != CAN_OK)
	{
#ifdef CAN_DEBUG
		Serial.println("canBusInit failed");
#endif
		return CAN_ERROR;
	}

	return CAN_OK;
}

CanStatus CanBusStm32::writeFrame(uint32_t identifier, uint32_t frameType, uint32_t dataLength, uint8_t buffer[])
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.DLC = dataLength;
	TxHeader.StdId = identifier;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = frameType;

	uint32_t status = HAL_CAN_AddTxMessage(&_hcan, &TxHeader, buffer, &TxMailbox);

#ifdef CAN_DEBUG
	Serial.print("-> can ");
	Serial.print(identifier, HEX);
	Serial.print(" [");

	// uint8_t length = dlcToLength(dataLength);
	Serial.print(dataLength);
	Serial.print("] ");
	for (uint32_t byte_index = 0; byte_index < dataLength; byte_index++)
	{
		Serial.print(buffer[byte_index], HEX);
		Serial.print(" ");
	}
	Serial.println(status == HAL_OK ? "✅" : "❌");
#endif

	return status == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32::writeDataFrame(uint32_t identifier, byte buffer[], uint8_t length)
{
	// uint32_t dataLength = lengthToDLC(length);
	return writeFrame(identifier, CAN_RTR_DATA, length, buffer);
}

CanStatus CanBusStm32::writeRemoteFrame(uint32_t identifier, uint8_t length)
{
	return writeFrame(identifier, CAN_RTR_REMOTE, 0, nullptr);
}

CanStatus CanBusStm32::start(void)
{
	if (CanBusStm32::_shdnPin != -1)
	{
		digitalWrite(CanBusStm32::_shdnPin, LOW);
	}
	return HAL_CAN_Start(&_hcan) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32::stop(void)
{
	if (CanBusStm32::_shdnPin != -1)
	{
		digitalWrite(CanBusStm32::_shdnPin, HIGH);
	}
	return HAL_CAN_Stop(&_hcan) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32::configureFilter(uint32_t identifier, uint32_t mask, uint32_t filterIndex)
{

	CAN_FilterTypeDef filter = {
		.FilterIdLow = identifier << 5,
		.FilterMaskIdLow = mask << 5,
		//.FilterMaskIdHigh = mask << 5,
		//.FilterMaskIdLow = mask << 5
		.FilterFIFOAssignment = CAN_FILTER_FIFO0,
		.FilterMode = CAN_FILTERMODE_IDMASK,
		.FilterScale = CAN_FILTERSCALE_16BIT,
		.FilterActivation = ENABLE,
	};

	return HAL_CAN_ConfigFilter(&_hcan, &filter) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32::subscribe(void (*onReceive)())
{
	CanBusStm32::_callbackFunction = onReceive;

	return HAL_CAN_ActivateNotification(&_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32::unsubscribe()
{
	CanBusStm32::_callbackFunction = nullptr;
	return HAL_CAN_DeactivateNotification(&_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK ? CAN_OK : CAN_ERROR;
}

/* Implentaing STM32 weak function */
void HAL_FDCAN_MspInit(CAN_HandleTypeDef *hcan)
{
	canBusPostInit(hcan);
}

void FDCAN1_IT0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&CanBusStm32::_hcan);
}

void HAL_FDCAN_RxFifo0Callback(CAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & CAN_IT_RX_FIFO0_MSG_PENDING) != RESET)
	{
		if (CanBusStm32::_callbackFunction == nullptr)
		{
			return;
		}
		CanBusStm32::_callbackFunction();
	}
}

uint32_t CanBusStm32::available()
{
	return HAL_CAN_GetRxFifoFillLevel(&_hcan, CAN_RX_FIFO0);
}

RxFrame CanBusStm32::readFrame()
{
	if (!available())
	{
#ifdef CAN_DEBUG
		Serial.print("GetRxMessage failed");
#endif
		return RxFrame{}; // return empty frame
	}

	if (HAL_CAN_GetRxMessage(&_hcan, CAN_RX_FIFO0, &_rxHeader, _rxData) != HAL_OK)
	{
#ifdef CAN_DEBUG
		Serial.print("GetRxMessage failed");
#endif
		return RxFrame{}; // return empty frame
	}

	RxFrame frame = {
		.identifier = _rxHeader.StdId,
		.isRemoteRequest = _rxHeader.RTR == CAN_RTR_REMOTE,
		.dataLength = _rxHeader.DLC,
		.buffer = new uint8_t[_rxHeader.DLC]};
	memcpy(frame.buffer, _rxData, _rxHeader.DLC);

	return frame;
}

// uint32_t CanBusStm32::dlcToLength(uint32_t dlc)
// {
// 	uint32_t length = dlc >> 16;
// 	if (length >= 13)
// 	{
// 		return 32 + (13 - length) * 16;
// 	}
// 	else if (length == 12)
// 	{
// 		return 24;
// 	}
// 	else if (length >= 9)
// 	{
// 		return 12 + (9 - length) * 4;
// 	}
// 	return length;
// }

// uint32_t CanBusStm32::lengthToDLC(uint32_t length)
// {
// 	if (length <= 8)
// 	{
// 		return length << 16;
// 	}
// 	else if (length <= 12)
// 	{
// 		return CAN_DLC_BYTES_12;
// 	}
// 	else if (length <= 16)
// 	{
// 		return FDCAN_DLC_BYTES_16;
// 	}
// 	else if (length <= 20)
// 	{
// 		return FDCAN_DLC_BYTES_20;
// 	}
// 	else if (length <= 24)
// 	{
// 		return FDCAN_DLC_BYTES_24;
// 	}
// 	else if (length <= 32)
// 	{
// 		return FDCAN_DLC_BYTES_32;
// 	}
// 	else if (length <= 48)
// 	{
// 		return FDCAN_DLC_BYTES_48;
// 	}
// 	else if (length <= 64)
// 	{
// 		return FDCAN_DLC_BYTES_64;
// 	}
// 	else
// 	{
// 		// Error handling: Data length is out of range
// 		// You may want to handle this case appropriately for your application
// 		return 0;
// 	}
// }

uint32_t toMode(CanMode mode)
{
	switch (mode)
	{
	case CAN_LOOPBACK:
		return CAN_MODE_LOOPBACK;
	case CAN_LOOPBACK_EXTERNAL:
		return CAN_MODE_LOOPBACK; // NOT SUPPORTED, SO FALLBACK TO LOOPBACK
	default:
		return CAN_MODE_NORMAL;
	}
}

WEAK CanStatus canBusInit(uint32_t bitrate, CAN_HandleTypeDef *hcan, CanMode mode)
{

	// this depends on how we set FdcanClockSelection
	uint32_t clockFreq = HAL_RCC_GetPCLK1Freq(); // or use HAL_RCC_GetSysClockFreq();

	// Looking for a timeQuanta of between 8 and 25.
	// start at 16 and work outwards
	// this algo is inspired by: http://www.bittiming.can-wiki.info/

	uint32_t baseQuanta = 16;
	uint32_t timeQuanta = baseQuanta;

	uint32_t offset = 0;
	bool found = false;

	while (offset <= 9)
	{
		timeQuanta = baseQuanta - offset;
		if (clockFreq % (bitrate * timeQuanta) == 0)
		{
			found = true;
			break;
		}
		timeQuanta = baseQuanta + offset;
		if (clockFreq % (bitrate * timeQuanta) == 0)
		{
			found = true;
			break;
		}
		offset += 1;
	}
	if (!found)
	{
#ifdef CAN_DEBUG
		Serial.println("timeQuanta out of range");
#endif
		return CAN_ERROR;
	}

	uint32_t prescaler = clockFreq / (bitrate * timeQuanta);

	uint32_t nominalTimeSeg1 = uint32_t(0.875 * timeQuanta) - 1;

	float samplePoint = (1.0 + nominalTimeSeg1) / timeQuanta;
	float samplePoint2 = (1.0 + nominalTimeSeg1 + 1) / timeQuanta;

	if (abs(samplePoint2 - 0.875) < abs(samplePoint - 0.875))
	{
		nominalTimeSeg1 += 1;
		samplePoint = samplePoint2;
	}

	uint32_t nominalTimeSeg2 = timeQuanta - nominalTimeSeg1 - 1;

	CAN_InitTypeDef *init = &(hcan->Init);
#if defined(STM32G4xx)
	// TODO: This is not availanle for H7, should we set another options?
	init->ClockDivider = FDCAN_CLOCK_DIV1; //<- this is on G4 but not H7
#endif

	init->Prescaler = (uint16_t)prescaler;
	init->Mode = toMode(mode);
	init->SyncJumpWidth = CAN_SJW_1TQ;
	init->TimeSeg1 = (nominalTimeSeg1 - 1) << CAN_BTR_TS1_Pos;
	init->TimeSeg2 = (nominalTimeSeg2 - 1) << CAN_BTR_TS2_Pos;
	init->TimeTriggeredMode = DISABLE;
	init->AutoBusOff = DISABLE;
	init->AutoWakeUp = DISABLE;
	init->AutoRetransmission = DISABLE;
	init->ReceiveFifoLocked = DISABLE;
	init->TransmitFifoPriority = DISABLE;

#ifdef CAN_DEBUG
	Serial.print("clockFreq:");
	Serial.print(clockFreq);
	Serial.print(", bitrate:");
	Serial.print(bitrate);
	Serial.print(", core:");
	Serial.print(clockFreq);
	Serial.print(", prescaler:");
	Serial.print(prescaler);
	Serial.print(", timeQuanta:");
	Serial.print(timeQuanta);
	Serial.print(", nominalTimeSeg1:");
	Serial.print(nominalTimeSeg1);
	Serial.print(", nominalTimeSeg2:");
	Serial.print(nominalTimeSeg2);
	Serial.print(", samplePoint:");
	Serial.println(samplePoint);
	uint32_t ts1 = (init->TimeSeg1 >> CAN_BTR_TS1_Pos) + 1;
	uint32_t ts2 = (init->TimeSeg2 >> CAN_BTR_TS2_Pos) + 1;

	Serial.print("PCKL1: ");
	Serial.println(HAL_RCC_GetPCLK1Freq());
	Serial.print("Prescaler: ");
	Serial.println(init->Prescaler);
	Serial.print("TS1: ");
	Serial.println(ts1);
	Serial.print("TS2: ");
	Serial.println(ts2);
	Serial.print("CAN bitrate: ");
	Serial.println(bitrate);

#endif

	if (HAL_CAN_Init(hcan) != HAL_OK)
	{
#ifdef CAN_DEBUG
		Serial.println("CAN init failed");
#endif
		return CAN_ERROR;
	}

	// 	if (HAL_CAN_ConfigGlobalFilter(hcan, CAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
	// 	{
	// #ifdef CAN_DEBUG
	// 		Serial.println("CAN configGlobalFilter failed");
	// 		return CAN_ERROR;
	// #endif
	// 	}
	return CAN_OK;
}

WEAK void canBusPostInit(CAN_HandleTypeDef *hcan)
{
	if (hcan == NULL || hcan->Instance != CAN1)
	{
		return;
	}

	RCC_PeriphCLKInitTypeDef periphClkInit = {};

	HAL_RCCEx_GetPeriphCLKConfig(&periphClkInit);

	// Initializes the peripherals clocks
	// periphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_I2S;

#if defined(STM32G4xx)
	// TODO: This is not availanle for H7, should we set another options?
	// periphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
	periphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
#elif defined(STM32H7xx)
	periphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;

#endif
	if (HAL_RCCEx_PeriphCLKConfig(&periphClkInit) != HAL_OK)
	{
		// Error_Handler();
	}

	// Peripheral clock enable
	// __HAL_RCC_CAN_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// // FDCAN1 GPIO Configuration
	// // PA11 ------> FDCAN1_RX
	// GPIO_InitTypeDef GPIO_InitStruct = {0};
	// GPIO_InitStruct.Pin = CanBusStm32::_rxPin;
	// GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	// GPIO_InitStruct.Pull = GPIO_NOPULL;
	// GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	// GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	// HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// GPIO_InitStruct.Pin = CanBusStm32::_txPin;
	// GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	// GPIO_InitStruct.Pull = GPIO_NOPULL;
	// GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	// GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	// HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// // FDCAN1 interrupt Init
	// HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
	// HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

	PinName rx_name = digitalPinToPinName(CanBusStm32::_rxPin);
	PinName tx_name = digitalPinToPinName(CanBusStm32::_txPin);

	// these pin_functions enabe port clock and set correct alternative functions/speed
	pin_function(rx_name, pinmap_function(rx_name, PinMap_CAN_RD));
	pin_function(tx_name, pinmap_function(tx_name, PinMap_CAN_TD));

	if (CanBusStm32::_hcan.Instance == CAN1)
	{
		__HAL_RCC_CAN1_CLK_ENABLE();
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		// HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
	}
	else if (CanBusStm32::_hcan.Instance == CAN2)
	{
		__HAL_RCC_CAN2_CLK_ENABLE();
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	}
}