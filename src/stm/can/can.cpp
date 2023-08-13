#if HAL_CAN_MODULE_ENABLED

#include "can.h"

extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
extern "C" void CAN1_RX0_IRQHandler(void);
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void (*Can::_callbackFunction)() = nullptr;
CAN_RxHeaderTypeDef _rxHeader = {};
CAN_TxHeaderTypeDef _txHeader = {};
CAN_HandleTypeDef Can::_hcan = {};

Can::Can(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN) : BaseCan(pinRX, pinTX, pinSHDN)
{
  PinName rx_name = static_cast<PinName>(pinRX);
  PinName tx_name = static_cast<PinName>(pinTX);

  pin_function(rx_name, pinmap_function(rx_name, PinMap_CAN_RD));
  pin_function(tx_name, pinmap_function(tx_name, PinMap_CAN_TD));

  Can::_pinSHDN = pinSHDN;
  _hcan.Instance = CAN1;
}

CanStatus Can::init(CanMode mode, uint32_t bitrate)
{
  if (bitrate > 1000000)
  {
    failAndBlink(CAN_ERROR_BITRATE_TOO_HIGH);
  }

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  // PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CAN;
  // PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    failAndBlink(CAN_ERROR_CLOCK);
  }

  __HAL_RCC_CAN1_CLK_ENABLE();
  // __HAL_RCC_GPIOA_CLK_ENABLE();
  // __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

  uint32_t clockFreq = HAL_RCC_GetPCLK1Freq(); // or use HAL_RCC_GetSysClockFreq();

  CanTiming timing = solveCanTiming(clockFreq, bitrate);
  CAN_InitTypeDef *init = &(_hcan.Init);

  init->Prescaler = (uint16_t)timing.prescaler;
  init->Mode = mode == CanMode::CAN_LOOPBACK ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
  init->SyncJumpWidth = CAN_SJW_1TQ;
  init->TimeSeg1 = (timing.tseg1 - 1) << CAN_BTR_TS1_Pos;
  init->TimeSeg2 = (timing.tseg2 - 1) << CAN_BTR_TS2_Pos;
  init->TimeTriggeredMode = DISABLE;
  init->AutoBusOff = DISABLE;
  init->AutoWakeUp = DISABLE;
  init->AutoRetransmission = DISABLE;
  init->ReceiveFifoLocked = DISABLE;
  init->TransmitFifoPriority = DISABLE;

  return (HAL_CAN_Init(&_hcan) == HAL_OK) ? CAN_OK : CAN_ERROR;
}

CanStatus Can::deinit()
{
  return HAL_CAN_DeInit(&_hcan) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::filter(FilterType filterType, uint32_t identifier, uint32_t mask, bool maskRtrBit, bool identifierRtrBit)
{
  // Unimplemented: the ability to filter on IDE or RTR bits (personally not that useful?!)

  static uint32_t filterIdHigh = 0xffff;
  static uint32_t filterIdLow = 0xffff; // <-- all digits must match
  static uint32_t filterMaskHigh = 0xffff;
  static uint32_t filterMaskLow = 0xffff;
  uint8_t filterIndex = 0;

  if (filterType == FilterType::FILTER_MASK_STANDARD)
  {
    filterIdHigh = identifier << 5; // make room for IDE, RTR bits (+ 3 unused)
    filterMaskHigh = mask << 5;
  }
  else if (filterType == FilterType::FILTER_MASK_EXTENDED)
  {
    filterIdLow = (identifier & 0x0000ffff) << 3; // make room for IDE, RTR bit (+ 1 unused bit)
    filterIdHigh = identifier >> 16;
    filterMaskLow = (mask & 0x0000ffff) << 3;
    filterMaskHigh = mask >> 16;
  }
  else if (filterType == FilterType::FILTER_ACCEPT_ALL_STANDARD || filterType == FilterType::FILTER_ACCEPT_ALL_EXTENDED)
  {
    filterIdLow = 0xffff;
    filterIdHigh = 0x0000; //<- no digits have to match
    filterMaskLow = 0xffff;
    filterMaskHigh = 0x0000;
  }

#ifdef CAN_DEBUG
  Serial.println("###### FILTER ######");
  Serial.print("filterType: ");
  Serial.print(filterType);
  Serial.print(" (identifier: ");
  Serial.print(identifier, HEX);
  Serial.print(",  mask: ");
  Serial.print(mask, HEX);
  Serial.println(")");
  Serial.print("registers (filterIdLow: ");
  Serial.print(filterIdLow, HEX);
  Serial.print(", filterIdHigh: ");
  Serial.print(filterIdHigh, HEX);
  Serial.print(", filterMaskLow: ");
  Serial.print(filterMaskLow, HEX);
  Serial.print(", filterMaskHigh: ");
  Serial.print(filterMaskHigh, HEX);
  Serial.println(")");
#endif

  CAN_FilterTypeDef filter = {
      .FilterIdHigh = filterIdHigh,
      .FilterIdLow = filterIdLow,
      .FilterMaskIdHigh = filterMaskHigh,
      .FilterMaskIdLow = filterMaskLow,
      .FilterFIFOAssignment = CAN_FILTER_FIFO0,
      .FilterBank = filterIndex,
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterScale = filterType == FilterType::FILTER_MASK_EXTENDED ? CAN_FILTERSCALE_32BIT : CAN_FILTERSCALE_16BIT,
      .FilterActivation = filterType == FilterType::FILTER_DISABLE ? DISABLE : ENABLE,
  };
  return HAL_CAN_ConfigFilter(&_hcan, &filter) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::subscribe(void (*_messageReceiveCallback)())
{
  Can::_callbackFunction = _messageReceiveCallback;
  return HAL_CAN_ActivateNotification(&_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::unsubscribe()
{
  _callbackFunction = nullptr;
  return HAL_CAN_DeactivateNotification(&_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::start()
{
  if (_pinSHDN != NC)
  {
    digitalWrite(_pinSHDN, LOW);
  }
  return (HAL_CAN_Start(&_hcan) == HAL_OK) ? CAN_OK : CAN_ERROR;
}

CanStatus Can::stop()
{
  if (_pinSHDN != NC)
  {
    digitalWrite(_pinSHDN, HIGH);
  }
  return (HAL_CAN_Stop(&_hcan) == HAL_OK) ? CAN_OK : CAN_ERROR;
}

CanStatus Can::writeFrame(CanFrame *txFrame)
{
#ifdef CAN_DEBUG
  Serial.print("tx: ");
  logFrame(txFrame);
#endif

  _txHeader = {
      .StdId = txFrame->isExtended ? 0 : txFrame->identifier,
      .ExtId = txFrame->isExtended ? txFrame->identifier : 0,
      .IDE = txFrame->isExtended ? CAN_ID_EXT : CAN_ID_STD,
      .RTR = txFrame->isRTR ? CAN_RTR_REMOTE : CAN_RTR_DATA,
      .DLC = txFrame->dataLength,
  };
  uint32_t usedMailbox;

  return HAL_CAN_AddTxMessage(&_hcan, &_txHeader, txFrame->data, &usedMailbox) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::readFrame(CanFrame *rxFrame)
{
  memset(&_rxHeader, 0, sizeof(_rxHeader)); // <-zero before reusing _rxHeader

  if (HAL_CAN_GetRxMessage(&_hcan, CAN_RX_FIFO0, &_rxHeader, rxFrame->data) != HAL_OK)
  {
    Serial.println("HAL_FDCAN_GetRxMessage failed");
    return CAN_ERROR;
  }
  else
  {
    rxFrame->identifier = _rxHeader.IDE == CAN_ID_STD ? _rxHeader.StdId : _rxHeader.ExtId;
    rxFrame->isRTR = _rxHeader.RTR;
    rxFrame->isExtended = _rxHeader.IDE == CAN_ID_STD ? false : true;
    rxFrame->dataLength = _rxHeader.DLC;

#ifdef CAN_DEBUG
    Serial.print("rx: ");
    logFrame(rxFrame);
#endif
    return CAN_OK;
  }
}

uint32_t Can::available()
{
  return HAL_CAN_GetRxFifoFillLevel(&_hcan, CAN_RX_FIFO0);
}

/*
#####################
HAL CALBACK FUNCTIONS
#####################
*/
void HAL_CAN_MspInit(CAN_HandleTypeDef *hfdcan)
{

  Serial.println("HAL_CAN_MspInit");
  __HAL_RCC_CAN1_CLK_ENABLE(); //<- this has to be enabled in this init callback
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&Can::_hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (Can::_callbackFunction == nullptr)
  {
    return;
  }
  Can::_callbackFunction();
}
#endif