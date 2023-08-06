#if HAL_CAN_MODULE_ENABLED

#include "can_stm.h"
#include <Arduino.h>

void (*CanStm::receiveCallback)(CanFrame *rxMessage);
CAN_HandleTypeDef *CanStm::_hcan;

WEAK void SIMPLECAN_STM32_INIT(CAN_HandleTypeDef *hcan, uint32_t bitrate, CanMode mode);

CanStm::CanStm(uint32_t rx_pin, uint32_t tx_pin, uint32_t shdnPin)
{
  // Much of this function is equivalent to HAL_CAN_MspInit but dynamic using PinMap
  PinName rx_name = digitalPinToPinName(rx_pin);
  PinName tx_name = digitalPinToPinName(tx_pin);

  // these pin_functions enabe port clock and set correct alternative functions/speed
  pin_function(rx_name, pinmap_function(rx_name, PinMap_CAN_RD));
  pin_function(tx_name, pinmap_function(tx_name, PinMap_CAN_TD));

  if (pinSHDN != NC)
  {
    pinMode(pinSHDN, OUTPUT);
    digitalWrite(pinSHDN, HIGH);
  }
}

CanStatus CanStm::init(uint32_t bitrate, CanMode mode)
{
  _hcan = new CAN_HandleTypeDef(
      {.Instance = CAN1,
       .Init = {}});

  SIMPLECAN_STM32_INIT(_hcan, bitrate, mode);

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
CanStatus CanStm::start()
{
  if (_pinSHDN != NC)
  {
    digitalWrite(CanStm::_pinSHDN, LOW);
  }
  return static_cast<CanStatus>(HAL_CAN_Start(_hcan));
}
CanStatus CanStm::stop()
{
  if (_pinSHDN != NC)
  {
    digitalWrite(CanStm::_pinSHDN, HIGH);
  }
  return static_cast<CanStatus>(HAL_CAN_Stop(_hcan));
}

CanStatus CanStm::configureFilter(FilterType filterType, uint32_t identifier, uint32_t mask)
{
  // Unimplemented: the ability to filter on IDE or RTR bits (personally not that useful?!)

  static uint32_t filterIdHigh = 0xffff;
  static uint32_t filterIdLow = 0xffff; // <-- all digits must match
  static uint32_t filterMaskHigh = 0xffff;
  static uint32_t filterMaskLow = 0xffff;

  if (filterType == FilterType::FILTER_16BIT_1)
  {
    filterIdHigh = identifier << 5; // make room for IDE, RTR bits (+ 3 unused)
    filterMaskHigh = mask << 5;
  }
  else if (filterType == FilterType::FILTER_16BIT_2)
  {
    filterIdLow = identifier << 5;
    filterMaskLow = mask << 5;
  }
  else if (filterType == FilterType::FILTER_32BIT)
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
      .FilterBank = filterIndex,
      .FilterIdHigh = filterIdHigh,
      .FilterIdLow = filterIdLow,
      .FilterMaskIdHigh = filterMaskHigh,
      .FilterMaskIdLow = filterMaskLow,
      .FilterFIFOAssignment = CAN_FILTER_FIFO0,
      .FilterMode = CAN_FILTERMODE_IDMASK,
      .FilterScale = filterType == FilterType::FILTER_32BIT ? CAN_FILTERSCALE_32BIT : CAN_FILTERSCALE_16BIT,
      .FilterActivation = filterType == FilterType::FILTER_DISABLE ? DISABLE : ENABLE,
  };
  return static_cast<CanStatus>(HAL_CAN_ConfigFilter(_hcan, &filter));
}

CanStatus CanStm::writeFrame(uint32_t identifier, uint32_t frameType, uint32_t dataLength, uint8_t buffer[])
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  TxHeader.DLC = dataLength;
  TxHeader.StdId = identifier;
  TxHeader.IDE = identifier <= 0b11111111111 ? CAN_ID_STD : CAN_ID_EXT;
  TxHeader.RTR = frameType;

  uint32_t status = HAL_CAN_AddTxMessage(_hcan, &TxHeader, buffer, &TxMailbox);

#ifdef CAN_DEBUG
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
  return writeFrame(identifier, CAN_RTR_DATA, length, buffer);
}

CanStatus CanStm::writeRemoteFrame(uint32_t identifier, uint8_t length)
{
  return writeFrame(identifier, CAN_RTR_REMOTE, 0, nullptr);
}

uint32_t CanStm::available()
{
  return HAL_CAN_GetRxFifoFillLevel(_hcan, CAN_RX_FIFO0);
}

CanStatus CanStm::readFrame(CanFrame *rxMessage)
{

  return _readFrame(_hcan, rxMessage);
}

CanStatus CanStm::_readFrame(CAN_HandleTypeDef *hcan, CanFrame *rxMessage)
{

  static uint8_t buffer[8] = {0};
  static CAN_RxHeaderTypeDef rxHeader;

  CanStatus status = static_cast<CanStatus>(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, buffer));
  Serial.print("rx: ");
  if (status == CAN_OK)
  {
    rxMessage->dlc = rxHeader.DLC;
    rxMessage->msgID = rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId;
    rxMessage->isRTR = rxHeader.RTR;
    rxMessage->isStandard = rxHeader.IDE == CAN_ID_STD ? true : false;

    memcpy(rxMessage->data, buffer, rxHeader.DLC);

    Serial.println(rxHeader.IDE);
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
  return static_cast<CanStatus>(HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
}

CanStatus CanStm::unsubscribe()
{
  return static_cast<CanStatus>(HAL_CAN_DeactivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
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

void SIMPLECAN_STM32_INIT(CAN_HandleTypeDef *hcan, uint32_t bitrate, CanMode mode)
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
    Error_Handler();
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

  init->Prescaler = (uint16_t)prescaler;
  init->Mode = mode == CanMode::CAN_LOOPBACK ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
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

  uint32_t solvedBitrate = (HAL_RCC_GetPCLK1Freq() / init->Prescaler) / (1 + nominalTimeSeg1 + nominalTimeSeg2);

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

extern "C" void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(CanStm::_hcan);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CanStm::_messageReceive();
}
#endif