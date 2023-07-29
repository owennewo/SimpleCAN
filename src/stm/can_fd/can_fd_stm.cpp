#if HAL_FDCAN_MODULE_ENABLED

#include "can_fd_stm.h"

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hcan);
extern "C" void FDCAN1_IT0_IRQHandler();
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs);

CanStatus canBusInit(int bitrate, FDCAN_HandleTypeDef *hcan, CanMode mode);
void canBusPostInit(FDCAN_HandleTypeDef *hcan);
void (*CanStm::receiveCallback)(CanFrame *rxMessage);

PinName CanStm::_pinSHDN = NC;

FDCAN_HandleTypeDef *CanStm::_hcan;

WEAK void SIMPLEFDCAN_STM32_PINMAP(PinName pinRX, PinName pinTX, PinName pinShdn);

WEAK void SIMPLEFDCAN_STM32_INIT(FDCAN_HandleTypeDef *hcan, uint32_t bitrate, CanMode mode);

static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan)
// {
//   CanStm::_messageReceive();
// }

CanStm::CanStm(PinName pinRX, PinName pinTX, PinName pinShdn)
{
  CanStm::_pinSHDN = pinShdn;
  SIMPLEFDCAN_STM32_PINMAP(pinRX, pinTX, pinShdn);
}

CanStatus CanStm::init(uint32_t bitrate, CanMode mode)
{

  SIMPLEFDCAN_STM32_INIT(_hcan, bitrate, mode);

  if (HAL_FDCAN_Init(_hcan) != HAL_OK)
  {
#ifdef CAN_DEBUG
    Serial.println("CAN init failed");
#endif
    return CAN_ERROR;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(_hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {

#ifdef CAN_DEBUG
    Serial.println("CAN configGlobalFilter failed");

#endif
    return CAN_ERROR;
  }
  return CAN_OK;
}
CanStatus CanStm::start()
{
  digitalWrite(CanStm::_pinSHDN, LOW);
  return static_cast<CanStatus>(HAL_FDCAN_Start(_hcan));
}
CanStatus CanStm::stop()
{
  digitalWrite(CanStm::_pinSHDN, HIGH);
  return static_cast<CanStatus>(HAL_FDCAN_Stop(_hcan));
}

CanStatus CanStm::configureFilter(FilterType filterType, uint32_t identifier, uint32_t mask, uint32_t filterIndex)
{

  // #ifdef CAN_DEBUG
  //   Serial.println("###### FILTER ######");
  //   Serial.print("filterType: ");
  //   Serial.print(filterType);
  //   Serial.print(" (identifier: ");
  //   Serial.print(identifier, HEX);
  //   Serial.print(",  mask: ");
  //   Serial.print(mask, HEX);
  //   Serial.println(")");
  //   Serial.print("registers (filterIdLow: ");
  //   Serial.print(filterIdLow, HEX);
  //   Serial.print(", filterIdHigh: ");
  //   Serial.print(filterIdHigh, HEX);
  //   Serial.print(", filterMaskLow: ");
  //   Serial.print(filterMaskLow, HEX);
  //   Serial.print(", filterMaskHigh: ");
  //   Serial.print(filterMaskHigh, HEX);
  //   Serial.println(")");
  // #endif

  uint32_t filterConfig = FDCAN_FILTER_TO_RXFIFO0;

  if (filterType == FILTER_DISABLE)
  {
    filterConfig = FDCAN_FILTER_DISABLE;
  }

  if (filterType == FILTER_ACCEPT_ALL)
  {
    mask = 0; // none of the bits need to be the same as mask to match
  }

  FDCAN_FilterTypeDef filter = {
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = filterIndex,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = identifier,
      .FilterID2 = mask};

  return static_cast<CanStatus>(HAL_FDCAN_ConfigFilter(_hcan, &filter));
}

CanStatus CanStm::writeFrame(uint32_t identifier, uint32_t frameType, uint32_t dataLength, uint8_t buffer[])
{

  FDCAN_TxHeaderTypeDef TxHeader = {
      .Identifier = identifier,
      .IdType = identifier <= 0b11111111111 ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID,
      .TxFrameType = frameType,
      .DataLength = dataLength,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0};

  uint32_t status = HAL_FDCAN_AddMessageToTxFifoQ(_hcan, &TxHeader, buffer);

#ifdef CAN_DEBUG
  // TODO: try serial write
  Serial.print("tx: ");
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

CanStatus CanStm::writeDataFrame(uint32_t identifier, byte buffer[], uint8_t length)
{
  return writeFrame(identifier, FDCAN_DATA_FRAME, length, buffer);
}

CanStatus CanStm::writeRemoteFrame(uint32_t identifier, uint8_t length)
{
  return writeFrame(identifier, FDCAN_REMOTE_FRAME, 0, nullptr);
}

uint32_t CanStm::available()
{
  return HAL_FDCAN_GetRxFifoFillLevel(_hcan, FDCAN_RX_FIFO0);
}

CanStatus CanStm::readFrame(CanFrame *rxMessage)
{

  return _readFrame(_hcan, rxMessage);
}

CanStatus CanStm::_readFrame(FDCAN_HandleTypeDef *hcan, CanFrame *rxMessage)
{

  static uint8_t buffer[8] = {0};
  static FDCAN_RxHeaderTypeDef rxHeader;

  CanStatus status = static_cast<CanStatus>(HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rxHeader, buffer));
  Serial.print("rx: ");
  if (status == CAN_OK)
  {
    rxMessage->dlc = rxHeader.DataLength;
    rxMessage->msgID = rxHeader.Identifier;
    rxMessage->isRTR = rxHeader.RxFrameType == FDCAN_REMOTE_FRAME ? true : false;
    rxMessage->isStandard = rxHeader.IdType == FDCAN_STANDARD_ID ? true : false;

    memcpy(rxMessage->data, buffer, DLCtoBytes[rxHeader.DataLength]);

    Serial.println(rxHeader.RxFrameType);
    Serial.print(rxMessage->msgID, HEX);
    Serial.print(" [");

    // uint8_t length = dlcToLength(dataLength);
    Serial.print(rxMessage->dlc);
    Serial.print("] ");
    for (uint32_t byte_index = 0; byte_index < rxMessage->dlc; byte_index++)
    {
      Serial.print(buffer[byte_index], HEX);
      Serial.print(" ");
    }
    Serial.println("✅");
  }
  else
  {
    Serial.println("❌");
  }

#ifdef CAN_DEBUG

#endif

  return status;
}

CanStatus CanStm::subscribe(void (*_messageReceiveCallback)(CanFrame *rxMessage))
{
  receiveCallback = _messageReceiveCallback;
  return static_cast<CanStatus>(HAL_FDCAN_ActivateNotification(_hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hcan)
{
  canBusPostInit(hcan);
}

void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(CanStm::_hcan);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if (CanStm::_messageReceive == nullptr)
    {
      return;
    }
    CanStm::_messageReceive();
  }
}

CanStatus CanStm::unsubscribe()
{
  return static_cast<CanStatus>(HAL_FDCAN_DeactivateNotification(_hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
}

void CanStm::_messageReceive()
{
  if (CanStm::receiveCallback != nullptr)
  {
    CanFrame rxMessage;
    if (CanStm::_readFrame(_hcan, &rxMessage) == CAN_OK)
    {
      CanStm::receiveCallback(&rxMessage);
    }
  }
}

void SIMPLEFDCAN_STM32_PINMAP(PinName pinRX, PinName pinTX, PinName pinShdn)
{
  // this is the arduino way, but it relies on a good PeripheralPins.c
  // pin_function(pinRX, pinmap_function(pinRX, PinMap_CAN_RD));
  // pin_function(pinTX, pinmap_function(pinTX, PinMap_CAN_TD));

  if (pinShdn != NC)
  {
    pinMode(pinShdn, OUTPUT);
  }

  // you may want to override this function with something lower level
  // GPIO_InitTypeDef GPIO_InitStruct = {0};
  // GPIO_InitStruct.Pin = GPIO_PIN_11;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  // GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // GPIO_InitStruct.Pin = GPIO_PIN_12;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  // GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SIMPLEFDCAN_STM32_INIT(FDCAN_HandleTypeDef *hcan, uint32_t bitrate, CanMode mode)
{
  uint32_t clockFreq = HAL_RCC_GetSysClockFreq(); // or SystemCoreClock;

  int quotient = clockFreq / bitrate;
  int remainder = clockFreq % bitrate;

  // Looking for a timeQuanta of between 8 and 25.
  // start at 16 and work outwards
  // this algo is a bit different to: http://www.bittiming.can-wiki.info/

  int baseQuanta = 16;
  int timeQuanta = baseQuanta;

  int offset = 0;
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
    Error_Handler();
  }

  int prescaler = clockFreq / (bitrate * timeQuanta);

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

  Serial.print("bitrate:");
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

  CanStm::_hcan = new FDCAN_HandleTypeDef{
      .Instance = FDCAN1,
      .Init = {
          .ClockDivider = FDCAN_CLOCK_DIV1,
          .FrameFormat = FDCAN_FRAME_CLASSIC,
          .Mode = mode == CAN_STANDARD ? FDCAN_MODE_NORMAL : FDCAN_MODE_INTERNAL_LOOPBACK,
          .AutoRetransmission = DISABLE,
          .TransmitPause = ENABLE,
          .ProtocolException = DISABLE,

          .NominalPrescaler = (uint16_t)prescaler,
          .NominalSyncJumpWidth = 1,
          .NominalTimeSeg1 = nominalTimeSeg1,
          .NominalTimeSeg2 = nominalTimeSeg2,

          .StdFiltersNbr = 8, // we can have up to 8 standard (11bit) filters
          .ExtFiltersNbr = 8, // we can have up to 8 extended (29bit) filters (max is 28)

          // As we have not set the following then we don't support flexible bitrate (date bitrate == nominal bitrate)
          // init->DataPrescaler = 1;
          // init->DataSyncJumpWidth = 4;
          // init->DataTimeSeg1 = 5;
          // init->DataTimeSeg2 = 4;

          .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION,
#if defined(STM32H7xx)
          .RxFifo0ElmtsNbr = 8,
          .TxFifoQueueElmtsNbr = 8,
#endif
      }};

#ifdef CAN_DEBUG

  uint32_t solvedBitrate = (HAL_RCC_GetPCLK1Freq() / CanStm::_hcan->Init.NominalPrescaler) / (1 + nominalTimeSeg1 + nominalTimeSeg2);

  Serial.println("###### TIMINGS ######");
  Serial.print("target bitrate:");
  Serial.print(bitrate);
  Serial.print(" (coreFreq:");
  Serial.print(HAL_RCC_GetSysClockFreq());
  Serial.print(", PCLK1: ");
  Serial.print(HAL_RCC_GetPCLK1Freq());
  Serial.println(")");

  Serial.print("solution bitrate:");
  Serial.print(bitrate);
  Serial.print(" (prescaler:");
  Serial.print(prescaler);
  Serial.print(", timeQuanta:");
  Serial.print(timeQuanta);
  Serial.print(", nominalTimeSeg1:");
  Serial.print(nominalTimeSeg1);
  Serial.print(", nominalTimeSeg2:");
  Serial.print(nominalTimeSeg2);
  Serial.print(", samplePoint:");
  Serial.print(samplePoint);
  Serial.println(")");
#endif
}

WEAK void canBusPostInit(FDCAN_HandleTypeDef *hcan)
{
  if (hcan == NULL || hcan->Instance != FDCAN1)
  {
    return;
  }

  RCC_PeriphCLKInitTypeDef periphClkInit = {};

  HAL_RCCEx_GetPeriphCLKConfig(&periphClkInit);

  // Initializes the peripherals clocks
  periphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_FDCAN;
  periphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&periphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  // Peripheral clock enable
  __HAL_RCC_FDCAN_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // FDCAN1 interrupt Init
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

#endif