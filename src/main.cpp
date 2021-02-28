#include <Arduino.h>
#include "board_variant.h"
#include "stm32f4xx_hal_can.h"
#include <SimpleCAN.h>

static void handleCanMessage(CAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{
  Serial.println("message received");
}

extern "C" void CAN1_RX0_IRQHandler(void); 
extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan);
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&SimpleCan::hcan);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcan->Instance==CAN1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
  Serial.println(".");
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8]  = {0};
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  // Serial.println(RxData);
}

SimpleCan::RxHandler can1RxHandler(8, handleCanMessage);

SimpleCan can;



void setup() {

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Setup");

  pinMode(PC13,OUTPUT);
  can.init(Kbit250,CanMode::LoopBackCan);

  can.filterAcceptAll();
  can.activateNotification(&can1RxHandler);
  can.begin();
}

void loop() {
  delay(1000);
  digitalWrite(LED_RED, !digitalRead(LED_RED));
  
  Serial.println("Hola Can !!!");
  //digitalToggle(PC13);
  CanMessage  msg = createMessage();
  Serial.println("Send");
  
  can.send(msg);
  
}
