#define USE_HAL_CAN_REGISTER_CALLBACKS 1

#include <Arduino.h>
#include "board_variant.h"
#include <SimpleCAN.h>
#include "hal_can_include.h"

void handleCanMessage(CAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{
  Serial.print("message received: "); Serial.println(rxData[0]);
  digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));

}

SimpleCan can(&hcan);

void setup() {

  // attachReceiveCallback(&handleCanMessage);
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup");
  Serial.println(USE_HAL_CAN_REGISTER_CALLBACKS);

  pinMode(PC13,OUTPUT);
  can.init(Kbit250,CanMode::LoopBackCan, CAN_RX, CAN_TX);

  can.filterAcceptAll();
  can.activateNotification(); //&can1RxHandler);
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
