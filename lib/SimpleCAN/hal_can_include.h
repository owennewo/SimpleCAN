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

void(*receiveCallback)(CAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);

void attachReceiveCallback(void(*_receiveCallback)(CAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)) {
    receiveCallback = _receiveCallback;
}

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8]  = {0};
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    Serial.print("message received: "); Serial.println(RxData[0]);
  digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));

  // receiveCallback(RxHeader, RxData);

}