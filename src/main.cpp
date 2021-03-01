#define USE_HAL_CAN_REGISTER_CALLBACKS 1

#include <Arduino.h>
#include "board_variant.h"
#include <SimpleCAN.h>
#include "can_helper.h"

void handleCanMessage(CanMessage *message)
{
  Serial.print("message can received!: "); Serial.println(message->data[0]);
  digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
}
CAN_HandleTypeDef* hcan = createCanHandle();
SimpleCan can = SimpleCan(hcan);

void setup() {
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  Serial.begin(115200);
  delay(2000);
  Serial.println("Setup");

  pinMode(PC13,OUTPUT);
  can.init(Kbit250, CanMode::LoopBackCan, CAN_RX, CAN_TX);

  can.filterAcceptAll();
  can.subscribe(&handleCanMessage);
  can.begin();
}

void loop() {
  delay(1000);
  digitalWrite(LED_RED, !digitalRead(LED_RED));
  
  static uint8_t data[2];
  data[0]++;
  data[0]++;
  
  CanMessage  msg = createStandardMessage(0x244, data, 2);
  Serial.println("Send");
  
  can.send(msg);
  
}
