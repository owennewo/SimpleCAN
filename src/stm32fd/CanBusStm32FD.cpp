#if defined(HAL_FDCAN_MODULE_ENABLED)

#include "CanBusStm32FD.h"
#include <string.h>

// will be called from: HAL_FDCAN_Init
extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
#if !defined(STM32H7xx)
extern "C" void FDCAN1_IT0_IRQHandler();
#endif
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

uint32_t CanBusStm32FD::_rxPin;
uint32_t CanBusStm32FD::_txPin;
uint32_t CanBusStm32FD::_shdnPin;

CanStatus canBusInit(uint32_t bitrate, FDCAN_HandleTypeDef *hfdcan, CanMode mode);
void canBusPostInit(FDCAN_HandleTypeDef *hfdcan);
void (*CanBusStm32FD::_callbackFunction)() = nullptr;

FDCAN_HandleTypeDef CanBusStm32FD::_hfdcan1 = {};
// CanBusStm32FD::RxHandler *CanBusStm32FD::_rxHandler = NULL;

CanBusStm32FD::CanBusStm32FD(uint32_t rxPin, uint32_t txPin, uint32_t shdnPin)
{
	if (_hfdcan1.Instance != NULL)
	{
		// Error_Handler();
	}
	CanBusStm32FD::_rxPin = rxPin;
	CanBusStm32FD::_txPin = txPin;
	CanBusStm32FD::_shdnPin = shdnPin;
	_hfdcan1.Instance = FDCAN1;

	if (shdnPin != -1)
	{
		pinMode(shdnPin, OUTPUT);
	}
}

CanStatus CanBusStm32FD::begin(uint32_t bitrate, CanMode mode)
{

	if (bitrate > 1000000)
	{
#ifdef CAN_DEBUG
		Serial.println("bitrate > 1Mbit/s, failing");
#endif
		return CAN_ERROR;
	}

	if (canBusInit(bitrate, &_hfdcan1, mode) != CAN_OK)
	{
#ifdef CAN_DEBUG
		Serial.println("canBusInit failed");
#endif
		return CAN_ERROR;
	}

	return CAN_OK;
}

CanStatus CanBusStm32FD::writeFrame(uint32_t identifier, uint32_t frameType, uint32_t dataLength, uint8_t buffer[])
{
	FDCAN_TxHeaderTypeDef TxHeader = {
		.Identifier = identifier,
		.IdType = FDCAN_STANDARD_ID,
		.TxFrameType = frameType,
		.DataLength = dataLength,
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF,
		.FDFormat = FDCAN_CLASSIC_CAN,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0};

	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&_hfdcan1, &TxHeader, buffer);

#ifdef CAN_DEBUG
	Serial.print("-> can ");
	Serial.print(identifier, HEX);
	Serial.print(" [");

	uint8_t length = dlcToLength(dataLength);
	Serial.print(length);
	Serial.print("] ");
	for (uint32_t byte_index = 0; byte_index < length; byte_index++)
	{
		Serial.print(buffer[byte_index], HEX);
		Serial.print(" ");
	}
	Serial.println(status == HAL_OK ? "✅" : "❌");
#endif

	return status == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32FD::writeDataFrame(uint32_t identifier, byte buffer[], uint8_t length)
{
	uint32_t dataLength = lengthToDLC(length);
	return writeFrame(identifier, FDCAN_DATA_FRAME, dataLength, buffer);
}

CanStatus CanBusStm32FD::writeRemoteFrame(uint32_t identifier, uint8_t length)
{
	return writeFrame(identifier, FDCAN_REMOTE_FRAME, FDCAN_DLC_BYTES_0, nullptr);
}

CanStatus CanBusStm32FD::start(void)
{
	if (CanBusStm32FD::_shdnPin != -1)
	{
		digitalWrite(CanBusStm32FD::_shdnPin, LOW);
	}
	return HAL_FDCAN_Start(&_hfdcan1) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32FD::stop(void)
{
	if (CanBusStm32FD::_shdnPin != -1)
	{
		digitalWrite(CanBusStm32FD::_shdnPin, HIGH);
	}
	return HAL_FDCAN_Stop(&_hfdcan1) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32FD::configureFilter(uint32_t identifier, uint32_t mask, uint32_t filterIndex)
{
	FDCAN_FilterTypeDef filter = {
		.IdType = FDCAN_STANDARD_ID,
		.FilterIndex = filterIndex,
		.FilterType = FDCAN_FILTER_MASK,
		.FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
		.FilterID1 = identifier,
		.FilterID2 = mask};

	return HAL_FDCAN_ConfigFilter(&_hfdcan1, &filter) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32FD::subscribe(void (*onReceive)())
{
	CanBusStm32FD::_callbackFunction = onReceive;

	return HAL_FDCAN_ActivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus CanBusStm32FD::unsubscribe()
{
	CanBusStm32FD::_callbackFunction = nullptr;
	return HAL_FDCAN_DeactivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == HAL_OK ? CAN_OK : CAN_ERROR;
}

/* Implentaing STM32 weak function */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
	canBusPostInit(hfdcan);
}

void FDCAN1_IT0_IRQHandler(void)
{
	HAL_FDCAN_IRQHandler(&CanBusStm32FD::_hfdcan1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (CanBusStm32FD::_callbackFunction == nullptr)
		{
			return;
		}
		CanBusStm32FD::_callbackFunction();
	}
}

uint32_t CanBusStm32FD::available()
{
	return HAL_FDCAN_GetRxFifoFillLevel(&_hfdcan1, FDCAN_RX_FIFO0);
}

RxFrame CanBusStm32FD::readFrame()
{
	if (!available())
	{
#ifdef CAN_DEBUG
		Serial.print("GetRxMessage failed");
#endif
		return RxFrame{}; // return empty frame
	}

	if (HAL_FDCAN_GetRxMessage(&_hfdcan1, FDCAN_RX_FIFO0, &_rxHeader, _rxData) != HAL_OK)
	{
#ifdef CAN_DEBUG
		Serial.print("GetRxMessage failed");
#endif
		return RxFrame{}; // return empty frame
	}

	RxFrame frame = {
		.identifier = _rxHeader.Identifier,
		.isRemoteRequest = _rxHeader.RxFrameType == FDCAN_REMOTE_FRAME,
		.dataLength = _rxHeader.DataLength,
		.buffer = new uint8_t[_rxHeader.DataLength]};
	memcpy(frame.buffer, _rxData, dlcToLength(_rxHeader.DataLength));

	return frame;
}

uint32_t CanBusStm32FD::dlcToLength(uint32_t dlc)
{
	uint32_t length = dlc >> 16;
	if (length >= 13)
	{
		return 32 + (13 - length) * 16;
	}
	else if (length == 12)
	{
		return 24;
	}
	else if (length >= 9)
	{
		return 12 + (9 - length) * 4;
	}
	return length;
}

uint32_t CanBusStm32FD::lengthToDLC(uint32_t length)
{
	if (length <= 8)
	{
		return length << 16;
	}
	else if (length <= 12)
	{
		return FDCAN_DLC_BYTES_12;
	}
	else if (length <= 16)
	{
		return FDCAN_DLC_BYTES_16;
	}
	else if (length <= 20)
	{
		return FDCAN_DLC_BYTES_20;
	}
	else if (length <= 24)
	{
		return FDCAN_DLC_BYTES_24;
	}
	else if (length <= 32)
	{
		return FDCAN_DLC_BYTES_32;
	}
	else if (length <= 48)
	{
		return FDCAN_DLC_BYTES_48;
	}
	else if (length <= 64)
	{
		return FDCAN_DLC_BYTES_64;
	}
	else
	{
		// Error handling: Data length is out of range
		// You may want to handle this case appropriately for your application
		return 0;
	}
}

uint32_t toMode(CanMode mode)
{
	switch (mode)
	{
	case CAN_LOOPBACK:
		return FDCAN_MODE_INTERNAL_LOOPBACK;
	case CAN_LOOPBACK_EXTERNAL:
		return FDCAN_MODE_EXTERNAL_LOOPBACK;
	default:
		return FDCAN_MODE_NORMAL;
	}
}

WEAK CanStatus canBusInit(uint32_t bitrate, FDCAN_HandleTypeDef *hfdcan, CanMode mode)
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

#endif

	FDCAN_InitTypeDef *init = &(hfdcan->Init);
#if defined(STM32G4xx)
	// TODO: This is not availanle for H7, should we set another options?
	init->ClockDivider = FDCAN_CLOCK_DIV1; //<- this is on G4 but not H7
#endif

	init->FrameFormat = FDCAN_FRAME_CLASSIC; // TODO: We may want to support faster/longer FDCAN_FRAME_FD_BRS;
	init->Mode = toMode(mode);				 // FDCAN_MODE_NORMAL;
	init->AutoRetransmission = ENABLE;
	init->TransmitPause = DISABLE;
	init->ProtocolException = DISABLE;

	init->NominalPrescaler = (uint16_t)prescaler;
	init->NominalSyncJumpWidth = 1;
	init->NominalTimeSeg1 = nominalTimeSeg1;
	init->NominalTimeSeg2 = nominalTimeSeg2;

	// TODO: If we support proper FD frames then we'll need to set the following too
	// init->DataPrescaler = 1;
	// init->DataSyncJumpWidth = 4;
	// init->DataTimeSeg1 = 5;
	// init->DataTimeSeg2 = 4;
	init->StdFiltersNbr = 1;
	init->ExtFiltersNbr = 0;
	init->TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

#if defined(STM32H7xx)
	init->RxFifo0ElmtsNbr = 8;
	init->TxFifoQueueElmtsNbr = 8;
#endif

	if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
	{
#ifdef CAN_DEBUG
		Serial.println("CAN init failed");
#endif
		return CAN_ERROR;
	}

	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
	{
#ifdef CAN_DEBUG
		Serial.println("CAN configGlobalFilter failed");
		return CAN_ERROR;
#endif
	}
	return CAN_OK;
}

WEAK void canBusPostInit(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan == NULL || hfdcan->Instance != FDCAN1)
	{
		return;
	}

	RCC_PeriphCLKInitTypeDef periphClkInit = {};

	HAL_RCCEx_GetPeriphCLKConfig(&periphClkInit);

	// Initializes the peripherals clocks
	periphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_FDCAN;

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
	__HAL_RCC_FDCAN_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// FDCAN1 GPIO Configuration
	// PA11 ------> FDCAN1_RX
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = CanBusStm32FD::_rxPin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = CanBusStm32FD::_txPin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// FDCAN1 interrupt Init
	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}
#endif