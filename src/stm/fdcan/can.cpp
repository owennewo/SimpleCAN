#if defined(HAL_FDCAN_MODULE_ENABLED)

#include "can.h"

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
#ifndef STM32H7
// H7 doesn't like this, avoiding compile 'multiple definition' error
extern "C" void FDCAN1_IT0_IRQHandler();
#endif
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

FDCAN_RxHeaderTypeDef _rxHeader{};
FDCAN_HandleTypeDef Can::_hfdcan1 = {};

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

CanStatus Can::init(CanMode mode, uint32_t bitrate)
{

  if (bitrate > 1000000)
  {
    failAndBlink(CAN_ERROR_BITRATE_TOO_HIGH);
  }

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    failAndBlink(CAN_ERROR_CLOCK);
  }

  __HAL_RCC_FDCAN_CLK_ENABLE();
  // __HAL_RCC_GPIOH_CLK_ENABLE();
  // __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  uint32_t clockFreq = HAL_RCC_GetPCLK1Freq(); // or use HAL_RCC_GetSysClockFreq();

  CanTiming timing = solveCanTiming(clockFreq, bitrate);
  FDCAN_InitTypeDef *init = &(_hfdcan1.Init);

  init->FrameFormat = FDCAN_FRAME_CLASSIC; // TODO: We may want to support faster FDCAN_FRAME_FD_BRS;
  init->Mode = mode == CAN_LOOPBACK ? FDCAN_MODE_INTERNAL_LOOPBACK : FDCAN_MODE_NORMAL;
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
  // not using RxFifo1
  // init->RxFifo1ElmtsNbr = 8;
  // init->RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  init->RxBuffersNbr = 8;
  init->RxBufferSize = FDCAN_DATA_BYTES_8;
  init->TxEventsNbr = 8;
  init->TxBuffersNbr = 8;
  init->TxFifoQueueElmtsNbr = 8;
  init->TxElmtSize = FDCAN_DATA_BYTES_8;
#endif

  Serial.println("init2");
  if (HAL_FDCAN_Init(&_hfdcan1) != HAL_OK)
  {
#ifdef CAN_DEBUG
    Serial.println("CAN init failed");
#endif
    return CAN_ERROR;
  }

  return CAN_OK;
}

CanStatus Can::deinit()
{
  return HAL_FDCAN_DeInit(&_hfdcan1) == HAL_OK ? CAN_OK : CAN_ERROR;
}

CanStatus Can::writeFrame(CanFrame *txFrame)
{
#ifdef CAN_DEBUG
  Serial.print("tx >> ");
  logFrame(txFrame);
#endif

  FDCAN_TxHeaderTypeDef TxHeader = {
      .Identifier = txFrame->identifier,
      .IdType = txFrame->isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
      .TxFrameType = txFrame->isRTR ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME,
      .DataLength = lengthToDLC(txFrame->dataLength),
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0};

  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&_hfdcan1, &TxHeader, txFrame->data);

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

  switch (filterType)
  {
  case FILTER_DISABLE:
    return (HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK) ? CAN_OK : CAN_ERROR;
  case FILTER_ACCEPT_ALL_STANDARD:
    return (HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) == HAL_OK) ? CAN_OK : CAN_ERROR;
  case FILTER_ACCEPT_ALL_EXTENDED:
    return (HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, FDCAN_REJECT, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) == HAL_OK) ? CAN_OK : CAN_ERROR;
  case FILTER_MASK_STANDARD:
    if (HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
      return CAN_ERROR;
    }
    break;
  case FILTER_MASK_EXTENDED:
    if (HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
      return CAN_ERROR;
    }
    break;
  }

  FDCAN_FilterTypeDef filter = {
      .IdType = (filterType == FILTER_MASK_EXTENDED || filterType == FILTER_ACCEPT_ALL_EXTENDED)
                    ? FDCAN_EXTENDED_ID
                    : FDCAN_STANDARD_ID,
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

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  __HAL_RCC_FDCAN_CLK_ENABLE(); //<- this has to be enabled in this init callback
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
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

CanStatus Can::readFrame(CanFrame *rxFrame)
{

  if (HAL_FDCAN_GetRxMessage(&_hfdcan1, FDCAN_RX_FIFO0, &_rxHeader, rxFrame->data) != HAL_OK)
  {
    return CAN_ERROR;
  }
  else
  {
    rxFrame->identifier = _rxHeader.Identifier;
    rxFrame->isRTR = _rxHeader.RxFrameType == FDCAN_REMOTE_FRAME;
    rxFrame->dataLength = dlcToLength(_rxHeader.DataLength);
    rxFrame->isExtended = _rxHeader.IdType == FDCAN_EXTENDED_ID;

#ifdef CAN_DEBUG
    Serial.print("rx << ");
    logFrame(rxFrame);
#endif

    return CAN_OK;
  }
}

uint32_t Can::dlcToLength(uint32_t dlc)
{
  switch (dlc)
  {
  case FDCAN_DLC_BYTES_0:
    return 0;
  case FDCAN_DLC_BYTES_1:
    return 1;
  case FDCAN_DLC_BYTES_2:
    return 2;
  case FDCAN_DLC_BYTES_3:
    return 3;
  case FDCAN_DLC_BYTES_4:
    return 4;
  case FDCAN_DLC_BYTES_5:
    return 5;
  case FDCAN_DLC_BYTES_6:
    return 6;
  case FDCAN_DLC_BYTES_7:
    return 7;
  case FDCAN_DLC_BYTES_8:
    return 8;
  case FDCAN_DLC_BYTES_12:
    return 12;
  case FDCAN_DLC_BYTES_16:
    return 16;
  case FDCAN_DLC_BYTES_20:
    return 20;
  case FDCAN_DLC_BYTES_24:
    return 24;
  case FDCAN_DLC_BYTES_32:
    return 32;
  case FDCAN_DLC_BYTES_48:
    return 48;
  case FDCAN_DLC_BYTES_64:
    return 64;
  default:
    return 0; // or handle as an error
  }
}

uint32_t Can::lengthToDLC(uint32_t length)
{
  switch (length)
  {
  case 0:
    return FDCAN_DLC_BYTES_0;
  case 1:
    return FDCAN_DLC_BYTES_1;
  case 2:
    return FDCAN_DLC_BYTES_2;
  case 3:
    return FDCAN_DLC_BYTES_3;
  case 4:
    return FDCAN_DLC_BYTES_4;
  case 5:
    return FDCAN_DLC_BYTES_5;
  case 6:
    return FDCAN_DLC_BYTES_6;
  case 7:
    return FDCAN_DLC_BYTES_7;
  case 8:
    return FDCAN_DLC_BYTES_8;
  case 12:
    return FDCAN_DLC_BYTES_12;
  case 16:
    return FDCAN_DLC_BYTES_16;
  case 20:
    return FDCAN_DLC_BYTES_20;
  case 24:
    return FDCAN_DLC_BYTES_24;
  case 32:
    return FDCAN_DLC_BYTES_32;
  case 48:
    return FDCAN_DLC_BYTES_48;
  case 64:
    return FDCAN_DLC_BYTES_64;
  default:
    return 0; // or handle as an error
  }
}

#endif