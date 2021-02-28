#include "SimpleCAN.h"
#include <Arduino.h>

// extern "C" HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
// extern "C" HAL_StatusTypeDef HAL_CAN_Stop (CAN_HandleTypeDef *);
// extern "C" HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *);
// extern "C" HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t );
// extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);

SimpleCan::RxHandler* SimpleCan::_rxHandler = NULL;

CAN_HandleTypeDef SimpleCan::hcan = {
  .Instance = CAN1,
  .Init = {
        .Prescaler = 21,
        .Mode = CAN_MODE_NORMAL,
        .SyncJumpWidth = CAN_SJW_1TQ,
        .TimeSeg1 = CAN_BS1_12TQ,
        .TimeSeg2 = CAN_BS2_4TQ,
        .TimeTriggeredMode = DISABLE,
        .AutoBusOff = DISABLE,
        .AutoWakeUp = DISABLE,
        .AutoRetransmission = DISABLE,
        .ReceiveFifoLocked = DISABLE,
        .TransmitFifoPriority = DISABLE,
  }
};

void messageCallback(CAN_HandleTypeDef *CanHandle) {
    Serial.println("message");
    digitalWrite(PC5, !digitalRead(PC5));
}

CanMessage createMessage(){
  CanMessage message;
  message.dlc = 8;
  message.msgID = 0x244;
  message.isRTR = false;
  message.isStandard = true;
  uint8_t messageLoadBuffer[8] ={0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x23};
  memcpy(message.data, messageLoadBuffer, 8);
  
  return message;
}

SimpleCan::SimpleCan(){
    return;
}
HAL_StatusTypeDef SimpleCan::init(CanSpeed speed, CanMode mode){

  return HAL_CAN_Init(&hcan);

}
HAL_StatusTypeDef SimpleCan::begin(){
    return HAL_CAN_Start(&hcan);

}
HAL_StatusTypeDef SimpleCan::stop(){
   	return HAL_CAN_Stop(&hcan);

}

HAL_StatusTypeDef SimpleCan::filterAcceptAll(){
  return filter(&FILTER_ACCEPT_ALL);
}

HAL_StatusTypeDef SimpleCan::filter(CAN_FilterTypeDef *filterDef){
  return HAL_CAN_ConfigFilter(&hcan, filterDef);
}

HAL_StatusTypeDef SimpleCan::send(CanMessage message){
	
    // char msg[50];
    CAN_TxHeaderTypeDef TxHeader;
    static uint8_t data = 0;
    data++;
    // // uint8_t data[5] = {'H','E','L','L','O'};
    uint32_t TxMailbox;
    TxHeader.DLC = 5;
    TxHeader.StdId = 0x244;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    return HAL_CAN_AddTxMessage(&hcan, &TxHeader, &data, &TxMailbox);    
}

HAL_StatusTypeDef SimpleCan::activateNotification( RxHandler *rxHandler)
{
	if (_rxHandler != NULL)
	{
		return HAL_ERROR;
	}


    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

	_rxHandler = rxHandler;
	return HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    // return HAL_OK;
}

HAL_StatusTypeDef SimpleCan::deactivateNotification(){
   // _rxHandler = NULL;
    return HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

SimpleCan::RxHandler::RxHandler(uint16_t dataLength, void (*callback)(CAN_RxHeaderTypeDef, uint8_t *))
{
	_rxData = new byte[dataLength];
	_callback = callback;
}

SimpleCan::RxHandler::~RxHandler()
{
	delete[] _rxData;
}

void SimpleCan::RxHandler::notify(CAN_HandleTypeDef *hcan1)
{
    if (SimpleCan::_rxHandler == NULL)
    {
        return;
    }
    SimpleCan::_rxHandler->notify(hcan1);
}
