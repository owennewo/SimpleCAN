#if defined(HAL_FDCAN_MODULE_ENABLED)

#include "can.h"

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
#ifndef STM32H7
// H7 doesn't like this, avoiding compile 'multiple definition' error
extern "C" void FDCAN1_IT0_IRQHandler();
#endif
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

void (*Can::_callbackFunction)() = nullptr;
FDCAN_RxHeaderTypeDef _rxHeader = {};
FDCAN_HandleTypeDef Can::_hcan = {};

uint16_t Can::_pinRX;
uint16_t Can::_pinTX;
uint16_t Can::_pinSHDN;

Can::Can(uint16_t pinRX, uint16_t pinTX, uint16_t pinSHDN)
{
  PinName rx_name = static_cast<PinName>(pinRX);
  PinName tx_name = static_cast<PinName>(pinTX);

  pin_function(rx_name, pinmap_function(rx_name, PinMap_CAN_RD));
  pin_function(tx_name, pinmap_function(tx_name, PinMap_CAN_TD));

  Can::_pinRX = pinRX;
  Can::_pinTX = pinTX;
  Can::_pinSHDN = pinSHDN;

  _hcan.Instance = FDCAN1;
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
  FDCAN_InitTypeDef *init = &(_hcan.Init);

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

  return logStatus('i',
                   HAL_FDCAN_Init(&_hcan));
}

CanStatus Can::deinit()
{
  return logStatus('d',
                   HAL_FDCAN_DeInit(&_hcan));
}

CanStatus Can::filter(FilterType filterType, uint32_t identifier, uint32_t mask, bool maskRtrBit, bool identifierRtrBit)
{

  switch (filterType)
  {
  case FILTER_DISABLE:
    return logStatus('g',
                     HAL_FDCAN_ConfigGlobalFilter(&_hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE));
  case FILTER_ACCEPT_ALL:
    return logStatus('g',
                     HAL_FDCAN_ConfigGlobalFilter(&_hcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE));
  case FILTER_MASK_STANDARD:
    if (logStatus('g',
                  HAL_FDCAN_ConfigGlobalFilter(&_hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE)) != CAN_OK)
    {
      return CAN_ERROR;
    }
    break;
  case FILTER_MASK_EXTENDED:
    if (logStatus('g',
                  HAL_FDCAN_ConfigGlobalFilter(&_hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_FILTER_REMOTE)) != CAN_OK)
    {
      return CAN_ERROR;
    }
    break;
  }

  FDCAN_FilterTypeDef filter = {
      .IdType = filterType == FILTER_MASK_EXTENDED
                    ? FDCAN_EXTENDED_ID
                    : FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = identifier,
      .FilterID2 = mask};

  return logStatus('c',
                   HAL_FDCAN_ConfigFilter(&_hcan, &filter));
}

CanStatus Can::subscribe(void (*_messageReceiveCallback)())
{
  Can::_callbackFunction = _messageReceiveCallback;
  return logStatus('a',
                   HAL_FDCAN_ActivateNotification(&_hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
}

CanStatus Can::unsubscribe()
{
  Can::_callbackFunction = nullptr;
  return logStatus('u',
                   HAL_FDCAN_DeactivateNotification(&_hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
}

CanStatus Can::start(void)
{
  if (_pinSHDN != NC)
  {
    digitalWrite(_pinSHDN, LOW);
  }
  return logStatus('s',
                   HAL_FDCAN_Start(&_hcan));
}

CanStatus Can::stop(void)
{
  if (_pinSHDN != NC)
  {
    digitalWrite(_pinSHDN, HIGH);
  }
  return logStatus('x',
                   HAL_FDCAN_Stop(&_hcan));
}

CanStatus Can::writeFrame(CanFrame *txFrame)
{
#ifdef CAN_DEBUG
  Serial.print("tx >> ");
  logFrame(txFrame);
#endif

  _txHeader = {
      .Identifier = txFrame->identifier,
      .IdType = txFrame->isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
      .TxFrameType = txFrame->isRTR ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME,
      .DataLength = lengthToDLC(txFrame->dataLength),
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0};

  return logStatus('w',
                   HAL_FDCAN_AddMessageToTxFifoQ(&_hcan, &_txHeader, txFrame->data));
}

CanStatus Can::readFrame(CanFrame *rxFrame)
{
  memset(&_rxHeader, 0, sizeof(_rxHeader)); // <-zero before reusing _rxHeader

  if (logStatus('r',
                HAL_FDCAN_GetRxMessage(&_hcan, FDCAN_RX_FIFO0, &_rxHeader, rxFrame->data)) != CAN_OK)
  {
    Serial.println("HAL_FDCAN_GetRxMessage failed");
    return CAN_ERROR;
  }
  else
  {
    rxFrame->identifier = _rxHeader.Identifier;
    rxFrame->isRTR = _rxHeader.RxFrameType == FDCAN_REMOTE_FRAME;
    rxFrame->isExtended = _rxHeader.IdType == FDCAN_EXTENDED_ID;
    rxFrame->dataLength = dlcToLength(_rxHeader.DataLength);

#ifdef CAN_DEBUG
    Serial.print("rx << ");
    logFrame(rxFrame);
#endif

    return CAN_OK;
  }
}

uint32_t Can::available()
{
  return HAL_FDCAN_GetRxFifoFillLevel(&_hcan, FDCAN_RX_FIFO0);
}

/*
#####################
HAL CALBACK FUNCTIONS
#####################
*/
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  __HAL_RCC_FDCAN_CLK_ENABLE(); //<- this has to be enabled in this init callback
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&Can::_hcan);
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

/*
##########
HELPERS
##########
*/
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
    _Serial->print(HAL_FDCAN_GetState(&_hcan), HEX); // google HAL_CAN_StateTypeDef e.g. 5 = HAL_CAN_STATE_ERROR
    _Serial->print(", can_error: ");
    _Serial->println(HAL_FDCAN_GetError(&_hcan), HEX); // google CAN_HandleTypeDef::ErrorCode  e.g. 0x00020000U = HAL_CAN_ERROR_TIMEOUT
  }
#endif
  return status == HAL_OK ? CAN_OK : CAN_ERROR;
}

#endif