#if HAL_CAN_MODULE_ENABLED

#include "can.h"
#include <Arduino.h>

void (*Can::receiveCallback)();
CAN_HandleTypeDef *Can::_hcan;

Can::Can(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN) : BaseCan(pinRX, pinTX, pinSHDN)
{
  PinName rx_name = static_cast<PinName>(pinRX);
  PinName tx_name = static_cast<PinName>(pinTX);

  pin_function(rx_name, pinmap_function(rx_name, PinMap_CAN_RD));
  pin_function(tx_name, pinmap_function(tx_name, PinMap_CAN_TD));
}

CanStatus Can::init(CanMode mode, uint32_t bitrate)
{
  _hcan = new CAN_HandleTypeDef(
      {.Instance = CAN1,
       .Init = {}});

  // this depends on how we set FdcanClockSelection
  uint32_t clockFreq = HAL_RCC_GetPCLK1Freq(); // or use HAL_RCC_GetSysClockFreq();

  // Looking for a timeQuanta of between 8 and 25.
  // start at 16 and work outwards
  // this algo is inspired by: http://www.bittiming.can-wiki.info/

  CanTiming timing = solveCanTiming(clockFreq, bitrate);

  CAN_InitTypeDef *init = &(_hcan->Init);

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

  uint32_t solvedBitrate = (HAL_RCC_GetPCLK1Freq() / init->Prescaler) / (1 + timing.tseg1 + timing.tseg2);

  if (_hcan->Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
  else if (_hcan->Instance == CAN2)
  {
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  }

  return static_cast<CanStatus>(HAL_CAN_Init(_hcan));
}

CanStatus Can::deinit()
{
  return HAL_CAN_DeInit(_hcan) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::start()
{
  if (_pinSHDN != NC)
  {
    digitalWrite(Can::_pinSHDN, LOW);
  }
  return static_cast<CanStatus>(HAL_CAN_Start(_hcan));
}
CanStatus Can::stop()
{
  if (_pinSHDN != NC)
  {
    digitalWrite(Can::_pinSHDN, HIGH);
  }
  return static_cast<CanStatus>(HAL_CAN_Stop(_hcan));
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
  // else if (filterType == FilterType::FILTER_16BIT_2)
  // {
  //   filterIdLow = identifier << 5;
  //   filterMaskLow = mask << 5;
  // }
  if (filterType == FilterType::FILTER_MASK_EXTENDED)
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
  return static_cast<CanStatus>(HAL_CAN_ConfigFilter(_hcan, &filter));
}

CanStatus Can::writeFrame(CanFrame *txFrame)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  TxHeader.DLC = txFrame->dataLength;
  if (txFrame->isExtended)
  {
    TxHeader.ExtId = txFrame->identifier;
  }
  else
  {
    TxHeader.StdId = txFrame->identifier;
  }

  TxHeader.IDE = txFrame->isExtended ? CAN_ID_EXT : CAN_ID_STD;
  TxHeader.RTR = txFrame->isRTR ? CAN_RTR_REMOTE : CAN_RTR_DATA;

  uint32_t status = HAL_CAN_AddTxMessage(_hcan, &TxHeader, txFrame->data, &TxMailbox);

#ifdef CAN_DEBUG
  Serial.print("tx: ");
  logFrame(txFrame);
#endif

  return status == HAL_OK ? CAN_OK : CAN_ERROR;
}

uint32_t Can::available()
{
  return HAL_CAN_GetRxFifoFillLevel(_hcan, CAN_RX_FIFO0);
}

CanStatus Can::readFrame(CanFrame *rxMessage)
{

  static uint8_t buffer[8] = {0};
  static CAN_RxHeaderTypeDef rxHeader;

  CanStatus status = static_cast<CanStatus>(HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rxHeader, buffer));
  Serial.print("rx: ");
  if (status == CAN_OK)
  {
    rxMessage->dataLength = rxHeader.DLC;
    rxMessage->identifier = rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId;
    rxMessage->isRTR = rxHeader.RTR;
    rxMessage->isExtended = rxHeader.IDE == CAN_ID_STD ? false : true;

    memcpy(rxMessage->data, buffer, rxHeader.DLC);

#ifdef CAN_DEBUG
    logFrame(rxMessage);
#endif
  }

  return status;
}

CanStatus Can::subscribe(void (*_messageReceiveCallback)())
{
  Can::receiveCallback = _messageReceiveCallback;
  return static_cast<CanStatus>(HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
}

CanStatus Can::unsubscribe()
{
  return static_cast<CanStatus>(HAL_CAN_DeactivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
}

void Can::_messageReceive()
{

  if (Can::receiveCallback != nullptr)
  {
    Serial.print("+");
    // CanFrame rxMessage;
    // if (Can::_readFrame(_hcan, &rxMessage) == CAN_OK)
    // {
    //   Can::receiveCallback(&rxMessage);
    // }
  }
}

extern "C" void CAN1_RX0_IRQHandler(void)
{
  Serial.print("+");
  Serial.flush();

  HAL_CAN_IRQHandler(Can::_hcan);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  Serial.print("+");
  Serial.flush();
  Can::_messageReceive();
}
#endif