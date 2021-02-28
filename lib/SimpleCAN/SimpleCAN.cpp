#include "SimpleCAN.h"
#include <Arduino.h>

// extern "C" HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
// extern "C" HAL_StatusTypeDef HAL_CAN_Stop (CAN_HandleTypeDef *);
// extern "C" HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *);
// extern "C" HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t );
// extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);

SimpleCan::RxHandler* SimpleCan::_rxHandler = NULL;



void messageCallback(CAN_HandleTypeDef *CanHandle) {
    Serial.println("message");
    digitalWrite(PC5, !digitalRead(PC5));
}


// void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan1)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
//   if(hcan1->Instance==CAN1)
//   {
//   /* USER CODE BEGIN CAN1_MspInit 0 */

//   /* USER CODE END CAN1_MspInit 0 */
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

//     /* CAN1 interrupt Init */
//     HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
//     HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
//     /* USER CODE BEGIN CAN1_MspInit 1 */

//     /* USER CODE END CAN1_MspInit 1 */
//   }

// }

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
  hcan.Instance = CAN1;
//   hcan.Init.Prescaler = speed;
//   hcan.Init.Mode = mode;
//   hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
//   hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
//   hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
//   hcan.Init.TimeTriggeredMode = DISABLE;
//   hcan.Init.AutoBusOff = DISABLE;
//   hcan.Init.AutoWakeUp = DISABLE;
//   hcan.Init.AutoRetransmission = DISABLE;
//   hcan.Init.ReceiveFifoLocked = DISABLE;
//   hcan.Init.TransmitFifoPriority = DISABLE;
  
            // hcan.Init.Prescaler = 8;
            // hcan.Init.Mode = mode;
            // hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
            // hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
            // hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
            // hcan.Init.TimeTriggeredMode = DISABLE;
            // hcan.Init.AutoBusOff = ENABLE;
            // hcan.Init.AutoWakeUp = ENABLE;
            // hcan.Init.AutoRetransmission = ENABLE;
            // hcan.Init.ReceiveFifoLocked = DISABLE;
            // hcan.Init.TransmitFifoPriority = DISABLE;

hcan.Init.Prescaler = 21;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  

  return HAL_CAN_Init(&hcan);

}
HAL_StatusTypeDef SimpleCan::begin(){
    return HAL_CAN_Start(&hcan);

}
HAL_StatusTypeDef SimpleCan::stop(){
   	return HAL_CAN_Stop(&hcan);

}
HAL_StatusTypeDef SimpleCan::configFilter(CAN_FilterTypeDef *filterDef){

    // Default filter - accept all to CAN_FIFO*
    CAN_FilterTypeDef sFilterConfig;
    // sFilterConfig.FilterBank = 0;
    // sFilterConfig.FilterIdHigh = 0x00005;
    // sFilterConfig.FilterBank = 0x0000;
    // sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    // sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    // sFilterConfig.FilterIdHigh = 0x00 ;  // HE TOCADO ESTO !!!!!!!
    // sFilterConfig.FilterIdLow  = 0x0000;
    // sFilterConfig.FilterMaskIdHigh = 0x0000;
    // sFilterConfig.FilterMaskIdLow = 0x0000;
    // sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    // sFilterConfig.FilterActivation = ENABLE;

      sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0x244 << 5;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterScale= CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = ENABLE;

    return HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
}
HAL_StatusTypeDef SimpleCan::configSnifferFilter(){

    // Default filter - accept all to CAN_FIFO*
	  CAN_FilterTypeDef sFilterConfig;
	//   sFilterConfig.FilterBank = 0;
	//   sFilterConfig.FilterIdHigh = 0x00005;
	//   sFilterConfig.FilterBank = 0x0000;
	//   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	//   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	//   sFilterConfig.FilterIdHigh = 0x200 << 5;  //11-bit ID, in top bits
	//   sFilterConfig.FilterIdLow  = 0x0000;
	//   sFilterConfig.FilterMaskIdHigh = 0x0000;
	//   sFilterConfig.FilterMaskIdLow = 0x0000;
	//   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	//   sFilterConfig.FilterActivation = ENABLE;

  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterIdHigh = 0x244 << 5;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterScale= CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = ENABLE;


    return HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
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

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, &data, &TxMailbox);
    // HAL_StatusTypeDef status = HAL_OK;
    Serial.println(status);
    // Serial.println(HAL_OK);
    // delay(10); 
    if( status != HAL_OK){
        Serial.println("failed!");
        delay(10);
        Error_Handler();
    }
    return status;

    // while(HAL_CAN_IsTxMessagePending(&hcan, TxMailbox));
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // HAL_Delay(1000);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // HAL_Delay(1000);
    
    // uint32_t TxMailbox;
	// CAN_TxHeaderTypeDef pHeader;
    // pHeader.DLC= message.dlc;
    // message.isStandard ? pHeader.IDE=CAN_ID_STD : pHeader.IDE=CAN_ID_EXT;
    // message.isRTR ?  pHeader.RTR=CAN_RTR_REMOTE : pHeader.RTR=CAN_RTR_DATA;
    // message.isStandard ? pHeader.StdId=0x200 : pHeader.ExtId=0x200;




    // return HAL_CAN_AddTxMessage(&hcan, &pHeader, (uint8_t*)message.data, &TxMailbox);
}


HAL_StatusTypeDef SimpleCan::activateNotification( RxHandler *rxHandler)
{
	if (_rxHandler != NULL)
	{
		return HAL_ERROR;
	}

    // nvic.enable_with_prio(spi_rx_dma.get_irqn(), 4);
    // nvic.enable_with_prio(spi_tx_dma.get_irqn(), 3);

    // HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, &messageCallback);
    // HAL_CAN_RegisterCallback(&hcan, HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, HAL_CAN_TxMailbox0CompleteCallback);
    // HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, HAL_CAN_RxFifo0MsgPendingCallback);
    // HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_FULL_CB_ID, HAL_CAN_RxFifo0FullCallback);

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
//       RxHandler   Implementation

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

// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
// {
//    digitalToggle(PC13);

//     if (SimpleCan::_rxHandler == NULL)
//     {
//         return;
//     }
//     SimpleCan::_rxHandler->notify(hcan1);

// }
