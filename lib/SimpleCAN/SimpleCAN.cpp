#include "SimpleCAN.h"
#include <Arduino.h>

// SimpleCan::RxHandler* SimpleCan::_rxHandler = NULL;

// CAN_HandleTypeDef SimpleCan::hcan = {
//   .Instance = CAN1,
//   .Init = {
//         .Prescaler = 21,
//         .Mode = CAN_MODE_NORMAL,
//         .SyncJumpWidth = CAN_SJW_1TQ,
//         .TimeSeg1 = CAN_BS1_12TQ,
//         .TimeSeg2 = CAN_BS2_4TQ,
//         .TimeTriggeredMode = DISABLE,
//         .AutoBusOff = DISABLE,
//         .AutoWakeUp = DISABLE,
//         .AutoRetransmission = DISABLE,
//         .ReceiveFifoLocked = DISABLE,
//         .TransmitFifoPriority = DISABLE,
//   }
// };

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

SimpleCan::SimpleCan(CAN_HandleTypeDef* _hcan){

    hcan = _hcan;
}
HAL_StatusTypeDef SimpleCan::init(CanSpeed speed, CanMode mode, int rx_pin, int tx_pin){

    // Much of this function is equivalent to HAL_CAN_MspInit but dynamic using PinMap
    PinName rx_name = digitalPinToPinName(rx_pin);
    PinName tx_name = digitalPinToPinName(tx_pin);

    // these pin_functions enabe port clock and set correct alternative functions/speed
    pin_function( rx_name ,pinmap_function(rx_name, PinMap_CAN_RD));
    pin_function( tx_name, pinmap_function(tx_name, PinMap_CAN_TD));
    
    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
        // HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    } else if (hcan->Instance == CAN2) {
        __HAL_RCC_CAN2_CLK_ENABLE();
        HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    }

  return HAL_CAN_Init(hcan);

}
HAL_StatusTypeDef SimpleCan::begin(){
    return HAL_CAN_Start(hcan);

}
HAL_StatusTypeDef SimpleCan::stop(){
   	return HAL_CAN_Stop(hcan);

}

HAL_StatusTypeDef SimpleCan::filterAcceptAll(){
  return filter(&FILTER_ACCEPT_ALL);
}

HAL_StatusTypeDef SimpleCan::filter(CAN_FilterTypeDef *filterDef){
  return HAL_CAN_ConfigFilter(hcan, filterDef);
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

    return HAL_CAN_AddTxMessage(hcan, &TxHeader, &data, &TxMailbox);    
}

HAL_StatusTypeDef SimpleCan::activateNotification()
{

	return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

}

HAL_StatusTypeDef SimpleCan::deactivateNotification(){
   // _rxHandler = NULL;
    return HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

