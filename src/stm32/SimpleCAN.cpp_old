#include "SimpleCAN.h"
#include <Arduino.h>
#include <SimpleCAN_c.h>

can_callback_function_t SimpleCAN::receiveFunction;
// void(*SimpleCAN::receiveCallback)(can_message_t* message);
CAN_HandleTypeDef* SimpleCAN::_hcan;

SimpleCAN::SimpleCAN(){
    
}

void SimpleCAN::createCanHandle(CanSpeed speed, CanMode mode) {

  can_timing timing = can_timings[speed];

  uint32_t stm_mode = (mode == CanMode::LoopBackCan) ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;

  _hcan = new CAN_HandleTypeDef(
    {
      .Instance = CAN1,
      .Init = {
            .Prescaler = timing.Prescaler,
            .Mode = stm_mode,
            .SyncJumpWidth = CAN_SJW_1TQ,
            .TimeSeg1 = (timing.TimeSeg1 -1) << CAN_BTR_TS1_Pos,
            .TimeSeg2 = (timing.TimeSeg2 -1) << CAN_BTR_TS2_Pos,
            .TimeTriggeredMode = DISABLE,
            .AutoBusOff = DISABLE,
            .AutoWakeUp = DISABLE,
            .AutoRetransmission = DISABLE,
            .ReceiveFifoLocked = DISABLE,
            .TransmitFifoPriority = DISABLE,
      }
    }   
  );
  
}

CAN_Status SimpleCAN::init(int rx_pin, int tx_pin, CanSpeed speed, CanMode mode) {

    createCanHandle(speed, mode);

    // Much of this function is equivalent to HAL_CAN_MspInit but dynamic using PinMap
    PinName rx_name = digitalPinToPinName(rx_pin);
    PinName tx_name = digitalPinToPinName(tx_pin);

    // these pin_functions enabe port clock and set correct alternative functions/speed
    pin_function( rx_name ,pinmap_function(rx_name, PinMap_CAN_RD));
    pin_function( tx_name, pinmap_function(tx_name, PinMap_CAN_TD));
    
    if (_hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
        // HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    } else if (_hcan->Instance == CAN2) {
        __HAL_RCC_CAN2_CLK_ENABLE();
        HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    }

  return static_cast<CAN_Status>(HAL_CAN_Init(_hcan));

}
CAN_Status SimpleCAN::begin(){
    return static_cast<CAN_Status>(HAL_CAN_Start(_hcan));

}
CAN_Status SimpleCAN::stop(){
   	return static_cast<CAN_Status>(HAL_CAN_Stop(_hcan));
}

CAN_Status SimpleCAN::filterAcceptAll(){
  return filter(&FILTER_ACCEPT_ALL);
}

CAN_Status SimpleCAN::filter(CAN_FilterTypeDef *filterDef){
  return static_cast<CAN_Status>(HAL_CAN_ConfigFilter(_hcan, filterDef));
}

CAN_Status SimpleCAN::transmit(can_message_t * message){
	
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    TxHeader.DLC = message->dlc;
    TxHeader.StdId = message->id;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    return static_cast<CAN_Status>(HAL_CAN_AddTxMessage(_hcan, &TxHeader, message->data, &TxMailbox)); 
}

CAN_Status SimpleCAN::receive(can_message_t * rxMessage) {

  static uint8_t rxData[8]  = {0};
  static CAN_RxHeaderTypeDef rxHeader;

  CAN_Status status = static_cast<CAN_Status>(HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rxHeader, rxData));

  if (status == CAN_OK) {
    rxMessage->dlc = rxHeader.DLC;
    rxMessage->id = rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId;
    rxMessage->isRTR = rxHeader.RTR;
    rxMessage->isStandard = rxHeader.IDE == CAN_ID_STD ? true : false;

    memcpy(rxMessage->data, rxData, rxHeader.DLC);
  }
  return status;
}

CAN_Status SimpleCAN::subscribe(can_callback_function_t function)
{
  receiveFunction = function;
	return static_cast<CAN_Status>(HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
}

// CAN_Status SimpleCAN::subscribe2(void (*_receive) (can_message_t *message))
// {
//   receiveCallback = _receive;
// 	return static_cast<CAN_Status>(HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
// }

CAN_Status SimpleCAN::unsubscribe(){
    return static_cast<CAN_Status>(HAL_CAN_DeactivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING));
}

void SimpleCAN::_receive(can_message_t* message) {
  if (SimpleCAN::receiveFunction != nullptr) {
    SimpleCAN::receiveFunction(message);
  }
}



extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

  static can_message_t rxMessage;
  static uint8_t rxData[8]  = {0};
  static CAN_RxHeaderTypeDef rxHeader;

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

  rxMessage.dlc = rxHeader.DLC;
  rxMessage.id = rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId;
  rxMessage.isRTR = rxHeader.RTR;
  rxMessage.isStandard = rxHeader.IDE == CAN_ID_STD ? true : false;

  memcpy(rxMessage.data, rxData, rxHeader.DLC);

  SimpleCAN::_receive(&rxMessage);  

}


