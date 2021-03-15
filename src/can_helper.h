#pragma once

#include <Arduino.h>
#include "stm32f4xx_hal_can.h"
#include "SimpleCAN.h"

extern "C" void CAN1_RX0_IRQHandler(void); 

can_message_t createStandardMessage(uint32_t id, uint8_t data[],uint8_t size){
  can_message_t message;
  message.dlc = size;
  message.id = id;
  message.isRTR = false;
  message.isStandard = true;
  memcpy(message.data, data, size);
  
  return message;
}

void printClockSpeed() {
  
  Serial.print("PCLK1: "); Serial.println(HAL_RCC_GetPCLK1Freq());
  Serial.print("SysClock: "); Serial.println(HAL_RCC_GetSysClockFreq());

}

void printCanSpeed() {
  CAN_InitTypeDef initType = SimpleCAN::_hcan->Init;
  uint32_t ts1 = (initType.TimeSeg1 >> CAN_BTR_TS1_Pos) +1;
  uint32_t ts2 = (initType.TimeSeg2 >> CAN_BTR_TS2_Pos) +1;
  
  uint32_t bitrate = (HAL_RCC_GetPCLK1Freq() / initType.Prescaler) / (1 + ts1 + ts2);
  
  Serial.print("PCKL1: "); Serial.println(HAL_RCC_GetPCLK1Freq());
  Serial.print("Prescaler: "); Serial.println(initType.Prescaler);
  Serial.print("TS1: "); Serial.println(ts1);
  Serial.print("TS2: "); Serial.println(ts2);
  Serial.print("CAN bitrate: "); Serial.println(bitrate);
}

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(SimpleCAN::_hcan);
}
