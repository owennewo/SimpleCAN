#if HAL_CAN_MODULE_ENABLED

#include "can.h"

extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
extern "C" void CAN1_RX0_IRQHandler(void);
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void (*Can::_callbackFunction)() = nullptr;
CAN_RxHeaderTypeDef _rxHeader = {};
CAN_TxHeaderTypeDef _txHeader = {};
CAN_HandleTypeDef Can::_hcan = {};

uint16_t Can::_pinRX;
uint16_t Can::_pinTX;
uint16_t Can::_pinSHDN;

Can::Can(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN)
{
  Can::_pinRX = pinRX;
  Can::_pinTX = pinTX;
  Can::_pinSHDN = pinSHDN;
  _hcan.Instance = CAN1;
}

CanStatus Can::init(CanMode mode, uint32_t bitrate)
{
  if (bitrate > 1000000)
  {
    failAndBlink(CAN_ERROR_BITRATE_TOO_HIGH);
  }

  uint32_t clockFreq = HAL_RCC_GetPCLK1Freq(); // or use HAL_RCC_GetSysClockFreq();

  CanTiming timing = solveCanTiming(clockFreq, bitrate);
  CAN_InitTypeDef *init = &(_hcan.Init);

  init->Prescaler = (uint16_t)timing.prescaler;
  init->Mode = mode == CanMode::CAN_LOOPBACK ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
  init->SyncJumpWidth = CAN_SJW_3TQ; // <- Maybe we should make this configurable? 3 is fairly safe.
  init->TimeSeg1 = (timing.tseg1 - 1) << CAN_BTR_TS1_Pos;
  init->TimeSeg2 = (timing.tseg2 - 1) << CAN_BTR_TS2_Pos;
  init->TimeTriggeredMode = DISABLE;
  init->AutoBusOff = DISABLE;
  init->AutoWakeUp = DISABLE;
  init->AutoRetransmission = DISABLE;
  init->ReceiveFifoLocked = DISABLE;
  init->TransmitFifoPriority = DISABLE;

  return logStatus('i',
                   HAL_CAN_Init(&_hcan));
}

CanStatus Can::deinit()
{
  return logStatus('d',
                   HAL_CAN_DeInit(&_hcan));
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
  else if (filterType == FilterType::FILTER_ACCEPT_ALL)
  {
    filterIdLow = 0xffff;
    filterIdHigh = 0x0000; //<- no digits have to match
    filterMaskLow = 0xffff;
    filterMaskHigh = 0x0000;
  }

#ifdef CAN_DEBUG
  _Serial->println("###### FILTER ######");
  _Serial->print("filterType: ");
  _Serial->print(filterType);
  _Serial->print(" (identifier: ");
  _Serial->print(identifier, HEX);
  _Serial->print(",  mask: ");
  _Serial->print(mask, HEX);
  _Serial->println(")");
  _Serial->print("registers (filterIdLow: ");
  _Serial->print(filterIdLow, HEX);
  _Serial->print(", filterIdHigh: ");
  _Serial->print(filterIdHigh, HEX);
  _Serial->print(", filterMaskLow: ");
  _Serial->print(filterMaskLow, HEX);
  _Serial->print(", filterMaskHigh: ");
  _Serial->print(filterMaskHigh, HEX);
  _Serial->println(")");
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
  return logStatus('f',
                   HAL_CAN_ConfigFilter(&_hcan, &filter));
}

CanStatus Can::subscribe(void (*_messageReceiveCallback)())
{
  Can::_callbackFunction = _messageReceiveCallback;
  return logStatus('a',
                   HAL_CAN_ActivateNotification(&_hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
}

CanStatus Can::unsubscribe()
{
  _callbackFunction = nullptr;
  return logStatus('u',
                   HAL_CAN_DeactivateNotification(&_hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
}

CanStatus Can::start()
{
  if (_pinSHDN != NC)
  {
    digitalWrite(_pinSHDN, LOW);
  }

  return logStatus('s',
                   HAL_CAN_Start(&_hcan));
}

CanStatus Can::stop()
{
  if (_pinSHDN != NC)
  {
    digitalWrite(_pinSHDN, HIGH);
  }
  return logStatus('x',
                   HAL_CAN_Stop(&_hcan));
}

CanStatus Can::writeFrame(CanFrame *txFrame)
{
#ifdef CAN_DEBUG
  _Serial->print("tx: ");
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

  return logStatus('t',
                   HAL_CAN_AddTxMessage(&_hcan, &_txHeader, txFrame->data, &usedMailbox));
}

CanStatus Can::readFrame(CanFrame *rxFrame)
{
  memset(&_rxHeader, 0, sizeof(_rxHeader)); // <-zero before reusing _rxHeader

  if (logStatus('r',
                HAL_CAN_GetRxMessage(&_hcan, CAN_RX_FIFO0, &_rxHeader, rxFrame->data)) != CAN_OK)
  {
    return CAN_ERROR;
  }
  else
  {
    rxFrame->identifier = _rxHeader.IDE == CAN_ID_STD ? _rxHeader.StdId : _rxHeader.ExtId;
    rxFrame->isRTR = _rxHeader.RTR;
    rxFrame->isExtended = _rxHeader.IDE == CAN_ID_STD ? false : true;
    rxFrame->dataLength = _rxHeader.DLC;

#ifdef CAN_DEBUG
    _Serial->print("rx: ");
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

  __HAL_RCC_CAN1_CLK_ENABLE();
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

  pinMode(Can::_pinRX, INPUT_PULLUP); // <- on some boards like the stm32f403rg_disc1, this is required for CAN_LOOPBACK mode to work !?!

  // TWO APPROACHES
  // APPROACH 1: use pinmaps (this is the recommended approach and uses PeripheralPins.c)
  pin_function((PinName)Can::_pinRX, pinmap_function((PinName)Can::_pinRX, PinMap_CAN_RD));
  pin_function((PinName)Can::_pinTX, pinmap_function((PinName)Can::_pinTX, PinMap_CAN_TD));

  // APPROACH 2: use HAL and hardcoded settings - example for PD_0 and PD_1
  // GPIO_InitTypeDef GPIO_InitStruct = {0};
  // GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  // GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  // HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  // __HAL_RCC_GPIOD_CLK_ENABLE();
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

CanStatus Can::logStatus(char op, HAL_StatusTypeDef status)
{
#ifdef CAN_DEBUG
  if (status != HAL_OK)
  {
    _Serial->print("ERROR (");
    _Serial->print(op);
    _Serial->print(") ");
    _Serial->print(status);
    _Serial->print(", can_state: ");
    _Serial->print(HAL_CAN_GetState(&_hcan), HEX); // google HAL_CAN_StateTypeDef e.g. 5 = HAL_CAN_STATE_ERROR
    _Serial->print(", can_error: ");
    _Serial->println(HAL_CAN_GetError(&_hcan), HEX); // google CAN_HandleTypeDef::ErrorCode  e.g. 0x00020000U = HAL_CAN_ERROR_TIMEOUT
  }
#endif
  return status == HAL_OK ? CAN_OK : CAN_ERROR;
}

#endif