#include "SimpleCAN.h"
#include <Arduino.h>

void(*SimpleCan::receiveCallback)(CanMessage* message);

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
	
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    TxHeader.DLC = message.dlc;
    TxHeader.StdId = message.msgID;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;

    return HAL_CAN_AddTxMessage(hcan, &TxHeader, message.data, &TxMailbox);    
}

HAL_StatusTypeDef SimpleCan::subscribe(void (*_receive) (CanMessage *message))
{
    receiveCallback = _receive;
	return HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

}

HAL_StatusTypeDef SimpleCan::unsubscribe(){
    return HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void SimpleCan::_receive(CanMessage* message) {
  if (SimpleCan::receiveCallback != nullptr) {
    SimpleCan::receiveCallback(message);
  } else {
    Serial.println("skipping");
  }

}
