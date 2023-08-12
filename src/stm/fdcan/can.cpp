#if defined(HAL_FDCAN_MODULE_ENABLED)

#include "can.h"
// #include <string.h>

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

// #if !defined(STM32H7)
void FDCAN1_IT0_IRQHandler();
// #endif

uint32_t Can::_shdnPin;
FDCAN_RxHeaderTypeDef _rxHeader{};
FDCAN_HandleTypeDef Can::_hfdcan1 = {};

void canBusPostInit(FDCAN_HandleTypeDef *hfdcan);
void (*Can::_callbackFunction)() = nullptr;

Can::Can(uint16_t rxPin, uint16_t txPin, uint16_t shdnPin) : BaseCan(rxPin, txPin, shdnPin)
{
  PinName rx_name = static_cast<PinName>(rxPin);
  PinName tx_name = static_cast<PinName>(txPin);

  pin_function(rx_name, pinmap_function(rx_name, PinMap_CAN_RD));
  pin_function(tx_name, pinmap_function(tx_name, PinMap_CAN_TD));

  Can::_shdnPin = shdnPin;
  _hfdcan1.Instance = FDCAN1;

  if (shdnPin != NC)
  {
    pinMode(shdnPin, OUTPUT);
  }
}

CanStatus Can::init(uint32_t bitrate, CanMode mode)
{

  if (bitrate > 1000000)
  {
#ifdef CAN_DEBUG
    Serial.println("bitrate > 1Mbit/s, failing");
#endif
    return CAN_ERROR;
  }

  // __HAL_RCC_SYSCFG_CLK_ENABLE();
  // this depends on how we set FdcanClockSelection
  // uint32_t clockFreq = 480000000;
  // uint32_t clockFreq = HAL_RCC_GetSysClockFreq();
  uint32_t clockFreq = HAL_RCC_GetPCLK1Freq(); // or use HAL_RCC_GetSysClockFreq();

  CanTiming timing = solveCanTiming(clockFreq, bitrate);
  FDCAN_InitTypeDef *init = &(_hfdcan1.Init);
#if defined(STM32G4xx)
  // TODO: This is not availanle for H7, should we set another options?
  init->ClockDivider = FDCAN_CLOCK_DIV1; //<- this is on G4 but not H7
#endif

  init->FrameFormat = FDCAN_FRAME_CLASSIC;   // TODO: We may want to support faster/longer FDCAN_FRAME_FD_BRS;
  init->Mode = FDCAN_MODE_INTERNAL_LOOPBACK; // toMode(mode);               // FDCAN_MODE_NORMAL;
  init->AutoRetransmission = DISABLE;
  init->TransmitPause = DISABLE;
  init->ProtocolException = DISABLE;

  init->NominalPrescaler = (uint16_t)timing.prescaler;
  init->NominalSyncJumpWidth = 1;
  init->NominalTimeSeg1 = timing.tseg1;
  init->NominalTimeSeg2 = timing.tseg2;

  // TODO: If we support proper FD frames then we'll need to set the following too
  init->DataPrescaler = (uint16_t)timing.prescaler; //<- max is 32
  init->DataSyncJumpWidth = 1;
  init->DataTimeSeg1 = timing.tseg1;
  init->DataTimeSeg2 = timing.tseg2;

  init->StdFiltersNbr = 1;
  init->ExtFiltersNbr = 1;
  init->TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

#if defined(STM32H7)
  init->MessageRAMOffset = 0;
  init->RxFifo0ElmtsNbr = 32;
  init->RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  // init->RxFifo1ElmtsNbr = 8;
  // init->RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  init->RxBuffersNbr = 8;
  init->RxBufferSize = FDCAN_DATA_BYTES_8;
  init->TxEventsNbr = 8;
  init->TxBuffersNbr = 8;
  init->TxFifoQueueElmtsNbr = 8;
  init->TxElmtSize = FDCAN_DATA_BYTES_8;
#endif

  if (HAL_FDCAN_Init(&_hfdcan1) != HAL_OK)
  {
#ifdef CAN_DEBUG
    Serial.println("CAN init failed");
#endif
    return CAN_ERROR;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
  {
#ifdef CAN_DEBUG
    Serial.println("CAN configGlobalFilter failed");
    return CAN_ERROR;
#endif
  }
  return CAN_OK;
}

CanStatus Can::deinit()
{
  return HAL_FDCAN_DeInit(&_hfdcan1) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::writeFrame(CanFrame *txFrame)
{
  uint32_t dataLength = txFrame->dataLength << 16;
  FDCAN_TxHeaderTypeDef TxHeader = {
      .Identifier = txFrame->identifier,
      .IdType = txFrame->isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
      .TxFrameType = txFrame->isRTR ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME,
      .DataLength = dataLength,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0};

  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&_hfdcan1, &TxHeader, txFrame->data);

#ifdef CAN_DEBUG
  Serial.print("tx: ");
  Serial.print(txFrame->identifier, HEX);
  Serial.print(" [");

  uint8_t length = dlcToLength(dataLength);
  Serial.print(length);
  Serial.print("] ");
  for (uint32_t byte_index = 0; byte_index < length; byte_index++)
  {
    Serial.print(txFrame->data[byte_index], HEX);
    Serial.print(" ");
  }
  Serial.println(status == HAL_OK ? "✅" : "❌");
#endif

  return status == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::start(void)
{
  if (Can::_shdnPin != NC)
  {
    digitalWrite(Can::_shdnPin, LOW);
  }
  return HAL_FDCAN_Start(&_hfdcan1) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::stop(void)
{
  if (Can::_shdnPin != NC)
  {
    digitalWrite(Can::_shdnPin, HIGH);
  }
  return HAL_FDCAN_Stop(&_hfdcan1) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::filter(FilterType filterType, uint32_t identifier, uint32_t mask, bool maskRtrBit, bool identifierRtrBit)
{
  FDCAN_FilterTypeDef filter = {
      .IdType = filterType == FILTER_MASK_EXTENDED_ID ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = identifier,
      .FilterID2 = mask};

  return HAL_FDCAN_ConfigFilter(&_hfdcan1, &filter) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::subscribe(void (*onReceive)())
{
  Can::_callbackFunction = onReceive;

  return HAL_FDCAN_ActivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::unsubscribe()
{
  Can::_callbackFunction = nullptr;
  return HAL_FDCAN_DeactivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == HAL_OK ? CAN_OK : CAN_ERROR;
}

/* Implentaing STM32 weak function */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  canBusPostInit(hfdcan);
}

void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&Can::_hfdcan1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if (Can::_callbackFunction == nullptr)
    {
      return;
    }
    Can::_callbackFunction();
  }
}

uint32_t Can::available()
{
  return HAL_FDCAN_GetRxFifoFillLevel(&_hfdcan1, FDCAN_RX_FIFO0);
}

CanStatus Can::readFrame(CanFrame *rxMessage)
{
  if (available() == 0)
  {
#ifdef CAN_DEBUG
    Serial.println("rx: no data 🚫");
#endif
    return CAN_NO_DATA;
  }

#ifdef CAN_DEBUG
  Serial.print("rx: ");
#endif
  if (HAL_FDCAN_GetRxMessage(&_hfdcan1, FDCAN_RX_FIFO0, &_rxHeader, rxMessage->data) != HAL_OK)
  {

#ifdef CAN_DEBUG
    Serial.println("❌");
    // Serial.print("GetRxMessage failed2");
#endif
    return CAN_ERROR;
  }
  else
  {
    rxMessage->identifier = _rxHeader.Identifier;
    rxMessage->isRTR = _rxHeader.RxFrameType == FDCAN_REMOTE_FRAME;
    rxMessage->dataLength = dlcToLength(_rxHeader.DataLength);
    rxMessage->isExtended = _rxHeader.IdType == FDCAN_EXTENDED_ID;

#ifdef CAN_DEBUG
    Serial.print(rxMessage->identifier, HEX);
    Serial.print(" [");

    // uint8_t length = dlcToLength(dataLength);
    Serial.print(rxMessage->dataLength);
    Serial.print("] ");
    for (uint32_t byte_index = 0; byte_index < rxMessage->dataLength; byte_index++)
    {
      Serial.print(rxMessage->data[byte_index], HEX);
      Serial.print(" ");
    }
    Serial.println("✅");
#endif
  }

  return CAN_OK;
}

uint32_t Can::dlcToLength(uint32_t dlc)
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

uint32_t Can::lengthToDLC(uint32_t length)
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
    return FDCAN_MODE_EXTERNAL_LOOPBACK;
  case CAN_LOOPBACK_EXTERNAL:
    return FDCAN_MODE_EXTERNAL_LOOPBACK;
  default:
    return FDCAN_MODE_NORMAL;
  }
}

WEAK void canBusPostInit(FDCAN_HandleTypeDef *hfdcan)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if (hfdcan->Instance == FDCAN1)
  {

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      // Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

#if defined(STM32H7)
    __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  }
}
#endif