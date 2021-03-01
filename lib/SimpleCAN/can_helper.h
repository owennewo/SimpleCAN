#pragma once

#include <Arduino.h>
#include "stm32f4xx_hal_can.h"
#include "SimpleCAN.h"

extern "C" void CAN1_RX0_IRQHandler(void); 
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

static CAN_HandleTypeDef hcan = {
  .Instance = CAN1,
  .Init = {
        .Prescaler = 21,
        // .Mode = CAN_MODE_LOOPBACK, // CAN_MODE_NORMAL,
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

CanMessage createStandardMessage(uint32_t id, uint8_t data[],uint8_t size){
  CanMessage message;
  message.dlc = size;
  message.msgID = id;
  message.isRTR = false;
  message.isStandard = true;
  // uint8_t messageLoadBuffer[8] ={0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x23};
  memcpy(message.data, data, size);
  
  return message;
}

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

CanMessage rxMessage;
uint8_t rxData[8]  = {0};
CAN_RxHeaderTypeDef rxHeader;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

  rxMessage.dlc = rxHeader.DLC;
  rxMessage.msgID = rxHeader.IDE == CAN_ID_STD ? rxHeader.StdId : rxHeader.ExtId;
  rxMessage.isRTR = rxHeader.RTR;
  rxMessage.isStandard = rxHeader.IDE == CAN_ID_STD ? true : false;

  memcpy(rxMessage.data, rxData, rxHeader.DLC);

  SimpleCan::_receive(&rxMessage);  

}