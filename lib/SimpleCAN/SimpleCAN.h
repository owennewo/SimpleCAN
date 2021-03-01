#ifndef __CAN__H
#define __CAN__H

#include "Arduino.h"
#include "stm32f4xx_hal_can.h"
// #include "hal_can_include.h"
// #include "example_can.h"



// Value needed for prescaler. depends 
// on CLK configuration
enum CanSpeed
{
	Kbit500 = 5,
	Kbit250 = 10,
	Kbit125 = 20,
	Kbit100 = 25,
  Kbit50 =  50,
  Kbit20  = 125

};

enum CanMode
{
	NormalCAN = CAN_MODE_NORMAL,
	SilentCAN = CAN_MODE_SILENT,
	LoopBackCan = CAN_MODE_LOOPBACK,
	SilentLoopBackCAN = CAN_MODE_SILENT_LOOPBACK
};


typedef struct{
  uint8_t dlc;
  uint32_t msgID;
  bool isRTR;
  bool isStandard;
  uint8_t data[8];
}CanMessage;

static CAN_FilterTypeDef FILTER_ACCEPT_ALL = {
    .FilterMaskIdHigh = 0,
    .FilterMaskIdLow = 0,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale= CAN_FILTERSCALE_32BIT,
    .FilterActivation = ENABLE,
  };


CanMessage createMessage(void);

class SimpleCan
{
public:

	SimpleCan(CAN_HandleTypeDef* hcan);
	HAL_StatusTypeDef init(CanSpeed speed, CanMode mode, int rx_pin, int tx_pin);
	HAL_StatusTypeDef filter(CAN_FilterTypeDef *filter);
	HAL_StatusTypeDef filterAcceptAll();
	HAL_StatusTypeDef activateNotification();// RxHandler *rxHandler);
	HAL_StatusTypeDef deactivateNotification();
	HAL_StatusTypeDef begin();
	HAL_StatusTypeDef stop();
	HAL_StatusTypeDef send(CanMessage message);
	// static CAN_HandleTypeDef hcan;
private: 
	CAN_HandleTypeDef* hcan;
	

};
#endif