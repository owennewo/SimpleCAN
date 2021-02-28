#include <Arduino.h>
#include "board_variant.h"
#include "stm32f4xx_hal_can.h"
#include <SimpleCAN.h>

static void handleCanMessage(CAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{
  Serial.println("message received");
	// digitalToggle(PC13);
}

SimpleCan::RxHandler can1RxHandler(8, handleCanMessage);

SimpleCan can;

extern "C" {

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
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

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    // get_can(hcan).on_event_.invoke(MEMBER_CB(&get_can(hcan), on_rx_fifo_pending<CAN_RX_FIFO0>));
  digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
  Serial.println(".");
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8]  = {0};
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&can.hcan);
}
}


void setup() {

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
   GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  // pinMode(LED_BLUE, OUTPUT);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Setup");

  Serial.println(USE_HAL_CAN_REGISTER_CALLBACKS);

    // HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    // HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    // HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  
  pinMode(PC13,OUTPUT);
  can.init(Kbit250,CanMode::LoopBackCan);
  can.configSnifferFilter();
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
