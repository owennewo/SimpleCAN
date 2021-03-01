#include "SimpleCAN.h"
#include <Arduino.h>


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
HAL_StatusTypeDef SimpleCan::init(CanSpeed speed, CanMode mode, int rx_pin, int tx_pin){

    // GPIO_InitTypeDef GPIO_InitStruct = {0};
  // if(hcan->Instance==CAN1)
  // {
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */

   GPIO_TypeDef* port = digitalPinToPort(rx_pin);
    
    PinName rx_name = digitalPinToPinName(rx_pin);
    PinName tx_name = digitalPinToPinName(tx_pin);

    pin_function( rx_name ,pinmap_function(rx_name, PinMap_CAN_RD));
    pin_function( tx_name, pinmap_function(tx_name, PinMap_CAN_TD));
    

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  

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

// void CAN1_RX0_IRQHandler(void)
// {
//   HAL_CAN_IRQHandler(&SimpleCan::hcan);
// }

// void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
//   if(hcan->Instance==CAN1)
//   {
//     /* Peripheral clock enable */
//     __HAL_RCC_CAN1_CLK_ENABLE();
//     __HAL_RCC_GPIOB_CLK_ENABLE();
//     /**CAN1 GPIO Configuration
//     PB8     ------> CAN1_RX
//     PB9     ------> CAN1_TX
//     */
//     GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
//     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//     GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
//     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//     HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
//     HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
//   }
// }

// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//   digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
// //   Serial.println(".");
//   CAN_RxHeaderTypeDef RxHeader;
//   uint8_t RxData[8]  = {0};
//   HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
//   // Serial.println(RxData);
// }

